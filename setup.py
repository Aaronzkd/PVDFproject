from setuptools import find_packages, setup

package_name = 'arduino_data_collector'

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
    maintainer='aaron',
    maintainer_email='aaron@163.com',
    description='ROS 2 package to collect data from Arduino and write to a CSV file',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_collector = arduino_data_collector.data_collector:main'
        ],
    },
)
