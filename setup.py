from setuptools import setup
from glob import glob
import os

package_name = 'rih0012_smartoffice'
pkg = package_name

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{pkg}']),
        (f'share/{pkg}', ['package.xml']),
        (f'share/{pkg}/config', [f'config/temperature_dashboard.rviz.yaml']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='primordia@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'smart.py = rih0012_smartoffice.smart:main',
            'schedule.py = rih0012_smartoffice.schedule:main',
            'visualise.py = rih0012_smartoffice.visualise:main'
        ],
    },
)
