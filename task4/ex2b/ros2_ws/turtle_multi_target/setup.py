from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_multi_target'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='valerie',
    maintainer_email='b_val4@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_switcher = turtle_multi_target.target_switcher:main',
            'turtle_controller = turtle_multi_target.turtle_controller:main',
            'carrot = two_turtles_one_carrot.carrot:main',
            'broadcaster = two_turtles_one_carrot.broadcaster:main',
            'listener = two_turtles_one_carrot.listener:main',
        ],
    },
)
