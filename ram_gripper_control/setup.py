import os
from glob import glob
from setuptools import setup

package_name = 'ram_gripper_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join(os.path.join('share', package_name), "config"), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='George Dwyer',
    maintainer_email='george.dwyer@ucl.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'implant_handler = ram_gripper_control.implant_handler_tf_pub:main',
            'sim_gripper_controller = ram_gripper_control.sim_gripper_controller:main',
            'gripper_controller = ram_gripper_control.gripper_controller:main'
        ],
    },
)
