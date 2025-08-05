import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'temperature_monitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmedosama07',
    maintainer_email='ahmed.osama8282@gmail.com',
    description='Temperature monitoring system - Python publisher node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temperature_publisher = temperature_monitor.temperature_publisher:main',
        ],
    },
)