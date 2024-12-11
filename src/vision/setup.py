from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],

    install_requires=[
        'setuptools',
        'rclpy',
    ],

    zip_safe=True,
    maintainer='Hiromichi123',
    maintainer_email='2271612727@qq.com',
    description='vision package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_calibration = vision.ball_calibration:main',
            'ball_depth = vision.ball_depth:main',
        ],
    },
)
