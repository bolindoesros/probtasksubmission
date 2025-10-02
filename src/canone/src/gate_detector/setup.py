from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gate_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 👇 This installs all launch files into share/gate_detector/launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bolin',
    maintainer_email='iwanttobeaquant@gmail.com',
    description='Gate detector and navigation nodes',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'gate_aligner = gate_detector.gate_aligner:main',
            'gate_committer = gate_detector.gate_commiter:main',
            'guided_mode = guided_mode.guided_mode:main',
            'mode_switcher_key = guided_mode.mode_switcher_key:main',
            'depth_controller = dive.depth_controller:main',
        ],
    },
)
