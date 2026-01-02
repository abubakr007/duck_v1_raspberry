from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'duck_localization'

def files_under(pattern):
    return [f for f in glob(pattern, recursive=True) if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', files_under('launch/*.launch.py')),
        (f'share/{package_name}/config', files_under('config/*.yaml')),
        (f'share/{package_name}/maps', files_under('maps/*')),  # yaml + pgm/png
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='altkamul',
    maintainer_email='altkamul@todo.todo',
    description='Nav2 AMCL + map_server launch for localization',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'set_initial_pose = duck_localization.set_initial_pose:main',
        ],
    },
)
