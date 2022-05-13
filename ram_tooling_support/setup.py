from setuptools import setup
import os
from glob import glob

package_name = 'ram_tooling_support'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ("share/" + package_name + "/config", glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='george',
    maintainer_email='george.dwyer@ucl.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stock_handler = ram_tooling_support.moveit_stock_handler:main',
            'toolpath_handler = ram_tooling_support.toolpath_handler:main',
            'us_cutter_controller = ram_tooling_support.us_cutter_io:main',
            'sim_us_cutter_controller = ram_tooling_support.sim_us_cutter:main',
            'get_acm = ram_tooling_support.get_acm:main'
        ],
    },
)
