from setuptools import setup
import os
from glob import glob

package_name = 'drone_sim'
submodules = "drone_sim/submodules"

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            full_path = os.path.join(path, filename)
            paths.append((os.path.join('share', package_name, path), [full_path]))
    return paths

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]

data_files += package_files('models')
data_files += package_files('worlds')
data_files += package_files('urdf')
data_files += package_files('launch')
data_files += package_files('scripts')
data_files += package_files('resource')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mateusz',
    maintainer_email='',
    description='Drone simulation with thermal camera',
    license='MIT',
    entry_points={
        'console_scripts': [
            'offboard = drone_sim.offboard:main',
            'pose_to_tf = drone_sim.pose_to_tf:main',
            'camera_stabilizer = drone_sim.camera_stabilizer:main',
            'move_image_path = drone_sim.move_image_path:main',
            'human_detector = drone_sim.human_detector:main',
            'human_position_publisher = drone_sim.human_position_publisher:main',
        ],
    },
)
