from setuptools import setup
import os
from glob import glob

package_name = 'sprint4_eurobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Archivos Launch y World
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'world'), glob('world/*.world')),
        
        # --- NUEVO: Instalar texturas y scripts de materiales ---
        (os.path.join('share', package_name, 'materials/scripts'), glob('materials/scripts/*')),
        (os.path.join('share', package_name, 'materials/textures'), glob('materials/textures/*')),
        # --------------------------------------------------------

        # Install models
        (os.path.join('share', package_name, 'models/waffle_aruco'), glob('models/waffle_aruco/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rodrigo',
    maintainer_email='rbriram@upv.edu.es',
    description='Sprint 4 Eurobot - Recreación y Visión',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)