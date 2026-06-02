import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={'arm_pkg': ['*.urdf']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='xiangcong',
    maintainer_email='2182845713@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_joint_node = arm_pkg.arm_joint_node:main',
            'arm_catch_box_node = arm_pkg.arm_catch_box_node:main',
        ],
    },
)
