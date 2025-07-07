from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_detect_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'),glob('models/*.bin'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='YOLOv11 detection package for RDK X5',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo11_detect = yolo_detect_pkg.yolo11_detect:main',
            'yolo11_detect_lifecycle = yolo_detect_pkg.yolo11_detect_lifecycle:main',
        ],
    },
)
