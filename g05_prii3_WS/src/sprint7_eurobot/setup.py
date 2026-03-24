from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sprint7_eurobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rodrigo',
    maintainer_email='rbriram@upv.edu.es',
    description='Sprint 7 Eurobot - Siguelineas',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower = sprint7_eurobot.line_follower:main',
        ],
    },
)
