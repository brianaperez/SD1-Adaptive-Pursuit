from setuptools import find_packages, setup

package_name = 'pure_pursuit_pid'
csv_file = 'trajectory.csv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['share/' + package_name + '/' + csv_file]),
        ('share/' + package_name + '/launch', ['launch/pure_pursuit_pid_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'velocity_logger = pure_pursuit_pid.velocity_logger:main',
            'waypoint_publisher = pure_pursuit_pid.newway:main',
            'pure_pursuit_controller = pure_pursuit_pid.newppc:main',
            'pid_controller = pure_pursuit_pid.newpid:main',
            'cmd_vel_combiner = pure_pursuit_pid.combine:main',
            'path_recorder = pure_pursuit_pid.recorder:main',
            #'velocity_plotter = pure_pursuit_pid.graph:main',
        ],
    },
)