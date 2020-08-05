from setuptools import setup

import os
from glob import glob


package_name = 'nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Liu',
    maintainer_email='lucas.liu@duke.edu',
    description='Autobot navigational package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motors = nav.motors.motors:main',
            'imu = nav.imu.imu:main',
            'dwm = nav.dwm.dwm:main',
            'mux = nav.mux:main',
            'posefusion = nav.pose_fusion:main',
            'turtlepid = nav.turtlesim_controllers.turtle_position_pid:main',
            'turtleturn = nav.turtlesim_controllers.turtle_heading_pid:main',
            'pos = nav.vehicle_controllers.position_pid:main',
            'imupid = nav.vehicle_controllers.imu_pid:main',
            'bturn = nav.vehicle_controllers.bturn:main',
            'bang = nav.vehicle_controllers.bang:main',
            'bang2 = nav.vehicle_controllers.bang2:main',
            'bang3 = nav.vehicle_controllers.bang3:main',
        ],
    },
)
