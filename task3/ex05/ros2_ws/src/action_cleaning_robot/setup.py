from setuptools import find_packages, setup

package_name = 'action_cleaning_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('action', ['action/CleaningTask.action']),
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
            'cleaning_action_server = action_cleaning_robot.cleaning_action_server:main',
            'cleaning_action_client = action_cleaning_robot.cleaning_action_client:main'
        ],
    },
)
