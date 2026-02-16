from setuptools import setup
import os
from glob import glob

package_name = 'sprint5_eurobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rodrigo',
    maintainer_email='rodrigo@upv.es',
    description='Paquete Sprint 5 Real Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'real_navigator = sprint5_eurobot.real_navigator:main',
        ],
    },
)
