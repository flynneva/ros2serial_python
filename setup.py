from setuptools import setup

package_name = 'ros2serial_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evan Flynn',
    maintainer_email='evanflynn.msu@gmail.com',
    description='ROS2 port of the original rosserial_python package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = ros2serial_python.serial_node:main'
        ],
    },
)
