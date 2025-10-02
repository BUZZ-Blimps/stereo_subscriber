import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stereo_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'calibration'), glob('calibration/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laptop-1',
    maintainer_email='buzzblimps@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_subscriber_node = stereo_subscriber.stereo_subscriber_node:main'
        ],
    },
)
