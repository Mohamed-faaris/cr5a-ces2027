from setuptools import find_packages, setup

package_name = 'pgea_gripper'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/pgea_gripper.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FTX',
    maintainer_email='futingxing@dobot-robot.com',
    description='DH-Robotics PGEA gripper bridge (GripperCommand action + joint states) for the CR5A',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pgea_gripper_node = pgea_gripper.pgea_gripper_node:main',
        ],
    },
)
