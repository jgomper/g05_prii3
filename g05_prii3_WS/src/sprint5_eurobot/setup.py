import os
from glob import glob
from setuptools import setup

package_name = 'sprint5_eurobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include world files (si las copias del Sprint 4)
        (os.path.join('share', package_name, 'world'), glob('world/*.world')),
        # Include RViz configs
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rodrigo',
    maintainer_email='rbriram@upv.edu.es',
    description='Sprint 5: Integración con hardware real y Pattern Matching',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'string_navigator = sprint5_eurobot.string_navigator:main',
        ],
    },
)
