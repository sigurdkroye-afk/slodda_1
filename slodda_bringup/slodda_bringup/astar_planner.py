import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import heapq
import math

ARENA_W = 3.6
ARENA_H = 2.4
CELL = 0.05
COLS = int(ARENA_W / CELL)
ROWS = int(ARENA_H / CELL)

OBSTACLES = [
    (0.5,  0.0, 0.12),
    (0.9,  0.3, 0.12),
]


def build_grid():
    import numpy as np
    grid = np.zeros((ROWS, COLS), dtype=np.uint8)
    grid[0, :] = 1
    grid[-1, :] = 1
    grid[:, 0] = 1
    grid[:, -1] = 1
    margin = 3
    for ox, oy, hw in OBSTACLES:
        c_center = int((ox + ARENA_W / 2) / CELL)
        r_center = int((oy + ARENA_H / 2) / CELL)
        half = int(hw / CELL) + margin
        for r in range(r_center - half, r_center + half + 1):
            for c in range(c_center - half, c_center + half + 1):
                if 0 <= r < ROWS and 0 <= c < COLS:
                    grid[r, c] = 1
    return grid


def world_to_grid(x, y):
    col = int((x + ARENA_W / 2) / CELL)
    row = int((y + ARENA_H / 2) / CELL)
    col = max(1, min(COLS - 2, col))
    row = max(1, min(ROWS - 2, row))
    return (row, col)


def grid_to_world(row, col):
    x = col * CELL - ARENA_W / 2 + CELL / 2
    y = row * CELL - ARENA_H / 2 + CELL / 2
    return x, y


def heuristic(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def astar(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0.0, start))
    came_from = {}
    g_score = {start: 0.0}
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),
                       (-1,-1),(-1,1),(1,-1),(1,1)]:
            nb = (current[0] + dr, current[1] + dc)
            if not (0 <= nb[0] < ROWS and 0 <= nb[1] < COLS):
                continue
            if grid[nb] == 1:
                continue
            step = 1.414 if dr != 0 and dc != 0 else 1.0
            tentative_g = g_score[current] + step
            if tentative_g < g_score.get(nb, float("inf")):
                came_from[nb] = current
                g_score[nb] = tentative_g
                f = tentative_g + heuristic(nb, goal)
                heapq.heappush(open_set, (f, nb))
    return []


def smooth_path(path, grid):
    if len(path) < 3:
        return path
    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if line_free(path[i], path[j], grid):
                break
            j -= 1
        smoothed.append(path[j])
        i = j
    return smoothed


def line_free(a, b, grid):
    r0, c0 = a
    r1, c1 = b
    steps = max(abs(r1 - r0), abs(c1 - c0))
    if steps == 0:
        return True
    for k in range(steps + 1):
        r = int(round(r0 + (r1 - r0) * k / steps))
        c = int(round(c0 + (c1 - c0) * k / steps))
        if grid[r, c] == 1:
            return False
    return True


class AstarPlanner(Node):
    def __init__(self):
        super().__init__("astar_planner")
        self.grid = build_grid()
        self.current_pos = (0.0, 0.0)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.path_pub = self.create_publisher(Path, "/planned_path", qos)
        self.goal_world = (1.0, 0.0)
        self.get_logger().info("A* planner klar. Planlegger om 2 sekunder...")
        self.timer = self.create_timer(2.0, self.plan_once)

    def odom_cb(self, msg):
        self.current_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y)

    def plan_once(self):
        self.timer.cancel()
        start = world_to_grid(*self.current_pos)
        goal  = world_to_grid(*self.goal_world)
        self.get_logger().info(f"Start: {start}  Maal: {goal}")
        raw = astar(self.grid, start, goal)
        if not raw:
            self.get_logger().error("Ingen sti funnet!")
            return
        path = smooth_path(raw, self.grid)
        self.get_logger().info(
            f"Ra sti: {len(raw)} noder  ->  Glatt sti: {len(path)} noder")
        msg = Path()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        for row, col in path:
            wx, wy = grid_to_world(row, col)
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)
        self.get_logger().info("Sti publisert pa /planned_path")


def main(args=None):
    rclpy.init(args=args)
    node = AstarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
