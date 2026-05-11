#include <ESP32Servo.h>
#include <Preferences.h>

// ============================================================
// PINNEOPPSETT
// ============================================================
const int SERVO1_PIN = 27;
const int SERVO2_PIN = 26;
const int SERVO3_PIN = 25;

const int IR_VENSTRE = 35;
const int IR_HOYRE   = 34;

// ============================================================
// PI UART-KOMMUNIKASJON
// ============================================================
const int UART2_RX_PIN = 13;  // Pi GPIO14 → ESP32 GPIO13 (GPIO16 er ustabil, unngå)
const int UART2_TX_PIN = 17;  // Pi GPIO15 ← ESP32 GPIO17

// ============================================================
// SERVOKONFIGURASJON
// ============================================================
const int STOPP    = 1500;
const int FART_INN = 1650;
const int FART_UT  = 1350;

const int STEG_MS         = 1000;
const int KOMPENSASJON_MS = 150;

// ============================================================
// IR-SENSOR KONFIGURASJON
// ============================================================
const int IR_TERSKEL = 2100;  // ~12 cm — juster etter kalibrering

const unsigned long IR_FORSINKELSE = 750;

// ============================================================
// VARIABLER
// ============================================================
Servo servo1, servo2, servo3;
Preferences prefs;

long tid1 = 0;
long tid2 = 0;
long tid3 = 0;

long pos0[3] = {0, 0, 0};
long pos1[3] = {0, 0, 0};
long pos2[3] = {0, 0, 0};
long pos3[3] = {0, 0, 0};
long pos4[3] = {0, 0, 0};
long pos5[3] = {0, 0, 0};

int gjeldendePosisjon = -1;
bool irAktiv = false;

unsigned long irDetektertTid = 0;
bool irDetektert = false;

// ============================================================
// HJELPEFUNKSJONER
// ============================================================
void stoppAlle() {
  servo1.writeMicroseconds(STOPP);
  servo2.writeMicroseconds(STOPP);
  servo3.writeMicroseconds(STOPP);
}

void skrivTider() {
  Serial.print("Pos (ms): S1=");
  Serial.print(tid1);
  Serial.print("  S2=");
  Serial.print(tid2);
  Serial.print("  S3=");
  Serial.println(tid3);
}

long* hentPosisjon(int nr) {
  switch (nr) {
    case 0: return pos0;
    case 1: return pos1;
    case 2: return pos2;
    case 3: return pos3;
    case 4: return pos4;
    case 5: return pos5;
    default: return pos0;
  }
}

void lagrePosisjon(int nr) {
  long *p = hentPosisjon(nr);
  p[0] = tid1;
  p[1] = tid2;
  p[2] = tid3;

  prefs.begin("armpos", false);
  String prefix = "p" + String(nr);
  prefs.putLong((prefix + "s1").c_str(), p[0]);
  prefs.putLong((prefix + "s2").c_str(), p[1]);
  prefs.putLong((prefix + "s3").c_str(), p[2]);
  prefs.end();

  gjeldendePosisjon = nr;
  irAktiv = (nr == 2);

  Serial.print("=> Posisjon ");
  Serial.print(nr);
  Serial.print(" lagret: S1=");
  Serial.print(p[0]);
  Serial.print("  S2=");
  Serial.print(p[1]);
  Serial.print("  S3=");
  Serial.println(p[2]);
}

void lastPosisjoner() {
  prefs.begin("armpos", true);
  for (int nr = 0; nr <= 5; nr++) {
    String prefix = "p" + String(nr);
    long *p = hentPosisjon(nr);
    p[0] = prefs.getLong((prefix + "s1").c_str(), 0);
    p[1] = prefs.getLong((prefix + "s2").c_str(), 0);
    p[2] = prefs.getLong((prefix + "s3").c_str(), 0);
  }
  prefs.end();
}

void kjorServo(Servo &servo, int retning, long &tid) {
  int fart = (retning == 1) ? FART_INN : FART_UT;
  servo.writeMicroseconds(fart);
  delay(STEG_MS);
  servo.writeMicroseconds(STOPP);
  tid += retning * STEG_MS;
  gjeldendePosisjon = -1;
  irAktiv = false;
  skrivTider();
}

void gaTilPosisjon(int malNr) {
  long *mal = hentPosisjon(malNr);

  Serial.print("=> Går til posisjon ");
  Serial.print(malNr);
  Serial.print(": S1=");
  Serial.print(mal[0]);
  Serial.print("  S2=");
  Serial.print(mal[1]);
  Serial.print("  S3=");
  Serial.println(mal[2]);

  long diff1 = mal[0] - tid1;
  long diff2 = mal[1] - tid2;
  long diff3 = mal[2] - tid3;

  long totalTid1 = abs(diff1);
  long totalTid2 = abs(diff2);
  long totalTid3 = abs(diff3);

  bool kompenserUt = (gjeldendePosisjon == 1 || gjeldendePosisjon == 3 || gjeldendePosisjon == 4);
  if (kompenserUt) {
    if (diff1 < 0) totalTid1 = max(0L, totalTid1 - KOMPENSASJON_MS);
    if (diff2 < 0) totalTid2 = max(0L, totalTid2 - KOMPENSASJON_MS);
    if (diff3 < 0) totalTid3 = max(0L, totalTid3 - KOMPENSASJON_MS);
    Serial.print("=> Kompenserer kortere ut (forlater posisjon ");
    Serial.print(gjeldendePosisjon);
    Serial.println(")");
  }

  if (diff1 != 0) servo1.writeMicroseconds((diff1 > 0) ? FART_INN : FART_UT);
  if (diff2 != 0) servo2.writeMicroseconds((diff2 > 0) ? FART_INN : FART_UT);
  if (diff3 != 0) servo3.writeMicroseconds((diff3 > 0) ? FART_INN : FART_UT);

  long maxTid = max(totalTid1, max(totalTid2, totalTid3));
  unsigned long startTid = millis();

  while (millis() - startTid < (unsigned long)maxTid) {
    unsigned long elapsed = millis() - startTid;

    if (elapsed >= (unsigned long)totalTid1 && diff1 != 0) {
      servo1.writeMicroseconds(STOPP);
      diff1 = 0;
    }
    if (elapsed >= (unsigned long)totalTid2 && diff2 != 0) {
      servo2.writeMicroseconds(STOPP);
      diff2 = 0;
    }
    if (elapsed >= (unsigned long)totalTid3 && diff3 != 0) {
      servo3.writeMicroseconds(STOPP);
      diff3 = 0;
    }

    // Nødstopp via Serial2 (Pi) under pågående bevegelse
    if (Serial2.available() > 0 && Serial2.peek() == 'q') {
      Serial2.read();
      Serial.println("STOPPET");
      Serial2.println("STOPPED");
      stoppAlle();
      return;
    }

    delay(10);
  }

  stoppAlle();
  tid1 = mal[0];
  tid2 = mal[1];
  tid3 = mal[2];
  gjeldendePosisjon = malNr;
  irAktiv = (malNr == 2);
  irDetektert = false;
  skrivTider();
  Serial.println("=> Posisjon nådd!");
  Serial.println("DONE:" + String(malNr));
  Serial2.println("DONE:" + String(malNr));

  if (irAktiv) {
    Serial.println("=> IR-overvåking aktiv");
  }
}

bool objektDetektert() {
  int verdi = analogRead(IR_VENSTRE);
  return (verdi >= IR_TERSKEL);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
  Serial2.println("ARM_READY");
  Serial.println("Serial2 aktiv: RX=GPIO13, TX=GPIO17");
  delay(500);

  pinMode(SERVO1_PIN, OUTPUT);
  pinMode(SERVO2_PIN, OUTPUT);
  pinMode(SERVO3_PIN, OUTPUT);
  digitalWrite(SERVO1_PIN, LOW);
  digitalWrite(SERVO2_PIN, LOW);
  digitalWrite(SERVO3_PIN, LOW);
  delay(100);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  stoppAlle();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  lastPosisjoner();

  Serial.println("=================================");
  Serial.println("  TIDSBASERT SERVOKONTROLL v66");
  Serial.println("=================================");
  Serial.println("  Kommandoer kun via Pi (Serial2)");
  Serial.println("  Serial = output-only (debug)");
  Serial.println("  0-5     -> Gå til posisjon");
  Serial.println("  t/g/y/h/u/j -> Servo-steg");
  Serial.println("  a/s/d   -> Hold inn 1/2/3");
  Serial.println("  A/S/D   -> Stopp hold");
  Serial.println("  w/e/r   -> Hold ut 1/2/3");
  Serial.println("  W/E/R   -> Stopp hold");
  Serial.println("  z/x/c/v -> Lagre pos 0/1/2/3");
  Serial.println("  q       -> Nødstopp");
  Serial.println("  IR aktiv i posisjon 2");
  Serial.println("=================================");
  skrivTider();
}

// ============================================================
// LOOP
// ============================================================
void loop() {

  // IR-overvåking — kun aktiv i posisjon 2
  if (irAktiv) {
    if (objektDetektert()) {
      if (!irDetektert) {
        irDetektert = true;
        irDetektertTid = millis();
        Serial.println("=> Objekt detektert! Venter...");
      } else if (millis() - irDetektertTid >= IR_FORSINKELSE) {
        Serial.println("=> Bekreftet! Går til posisjon 0, deretter 3...");
        irAktiv = false;
        irDetektert = false;
        gaTilPosisjon(0);
        gaTilPosisjon(3);
        Serial.println("GRABBED");
        Serial2.println("GRABBED");
      }
    } else {
      if (irDetektert) {
        Serial.println("=> Objekt mistet, venter...");
        irDetektert = false;
      }
    }
  }

  // Alle kommandoer kun via Serial2 (Pi).
  // Serial (USB) er output-only — løser floating RX-problem uten USB tilkoblet.
  if (Serial2.available() > 0) {
    char k = Serial2.read();
    switch (k) {
      case 't': kjorServo(servo1,  1, tid1); break;
      case 'g': kjorServo(servo1, -1, tid1); break;
      case 'y': kjorServo(servo2,  1, tid2); break;
      case 'h': kjorServo(servo2, -1, tid2); break;
      case 'u': kjorServo(servo3,  1, tid3); break;
      case 'j': kjorServo(servo3, -1, tid3); break;

      case 'a': servo1.writeMicroseconds(FART_INN); break;
      case 'A': servo1.writeMicroseconds(STOPP);    break;
      case 's': servo2.writeMicroseconds(FART_INN); break;
      case 'S': servo2.writeMicroseconds(STOPP);    break;
      case 'd': servo3.writeMicroseconds(FART_INN); break;
      case 'D': servo3.writeMicroseconds(STOPP);    break;

      case 'w': servo1.writeMicroseconds(FART_UT); break;
      case 'W': servo1.writeMicroseconds(STOPP);   break;
      case 'e': servo2.writeMicroseconds(FART_UT); break;
      case 'E': servo2.writeMicroseconds(STOPP);   break;
      case 'r': servo3.writeMicroseconds(FART_UT); break;
      case 'R': servo3.writeMicroseconds(STOPP);   break;

      case '0': gaTilPosisjon(0); break;
      case '1': gaTilPosisjon(1); break;
      case '2': gaTilPosisjon(2); break;
      case '3': gaTilPosisjon(3); break;
      case '4': gaTilPosisjon(4); break;
      case '5': gaTilPosisjon(5); break;

      case 'z': case 'Z': lagrePosisjon(0); break;
      case 'x': case 'X': lagrePosisjon(1); break;
      case 'c': case 'C': lagrePosisjon(2); break;
      case 'v': case 'V': lagrePosisjon(3); break;
      case 'b': case 'B': lagrePosisjon(4); break;
      case 'n': case 'N': lagrePosisjon(5); break;

      case 'q': case 'Q':
        stoppAlle();
        irAktiv = false;
        irDetektert = false;
        Serial.println("=> Nødstopp!");
        Serial.println("STOPPED");
        Serial2.println("STOPPED");
        break;
    }
  }
}
