from setuptools import find_packages, setup

package_name = 'wpi_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vehicle_position.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joisie',
    maintainer_email='digimonlord2@gmail.com',
    description='PX4 vehicle position listener',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_position_listener = wpi_drone.vehicle_position_listener:main',
        ],
    },
)
