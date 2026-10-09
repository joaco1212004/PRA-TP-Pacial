from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'tp_parcial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        # glob de *.py y no de *.launch.py: la consigna de la seccion 3 pide
        # lanzar `tu_launch.py`, cuyo nombre no termina en .launch.py y quedaba
        # fuera del install sin ningun error de build.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='udesa',
    maintainer_email='tadeo.casiraghi@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "likelihood = tp_parcial.likelihood:main",
            "particle_localization = tp_parcial.particles:main",
            "ekf_localization = tp_parcial.ekf:main",
            "ground_truth_path = tp_parcial.ground_truth_path:main",
            "doctor = tp_parcial.doctor:main",
        ],
    },
)
