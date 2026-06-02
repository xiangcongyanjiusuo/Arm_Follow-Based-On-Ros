from setuptools import find_packages, setup

package_name = 'vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv_bridge', 'tf2_ros', 'tf_transformations', 'numpy', 'pyyaml'],
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
            'camera_native_node = vision_pkg.camera_native_node:main',
            'hsv_image_node = vision_pkg.hsv_image_node:main',
            'tf_node = vision_pkg.tf_node:main',
        ],
    },
)
