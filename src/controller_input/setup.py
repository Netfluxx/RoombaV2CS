from setuptools import find_packages, setup

package_name = 'controller_input'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/joystick.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/joystick_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arno',
    maintainer_email='arno.blan334@gmail.com',
    description='package to read and publish ps4 controller inputs for rover control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
