from setuptools import find_packages, setup

package_name = 'RoverInput'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arno',
    maintainer_email='arno.blan334@gmail.com',
    description='A package to capture input from the keyboard and publish it to a topic',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'input_node = RoverInput.input_node:main'
        ],
    },
)
