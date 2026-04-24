from setuptools import setup

package_name = 'slodda_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='slodda1',
    maintainer_email='sigurkoy@stud.ntnu.no',
    description='YOLO-basert objektdeteksjon for Slødda 1',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_detector = slodda_vision.yolo_detector:main',
        ],
    },
)
