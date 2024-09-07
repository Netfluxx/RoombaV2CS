from setuptools import find_packages, setup

package_name = 'GUI'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/control_station.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/joystick_params.yaml']),
    ],
    install_requires=['setuptools', 'rclpy', 'customtkinter', 'matplotlib', 'Pillow', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='arno',
    maintainer_email='arno.blan334@gmail.com',
    description='ROS2 Node GUI that subscribes to the rover data and displays it using customtkinter',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = GUI.gui:main',
            'fake_map_pub = GUI.fake_occupancy_grid_publisher:main',
        ],
    },
)
