import os

from glob import glob
from setuptools import setup

package_name = 'turtlebot2_airobotbook'


def model_data_files(p_name, m_name):
    return [
        (os.path.join('share', p_name, f'models/{m_name}'),
            glob(f'models/{m_name}/*.config')),
        (os.path.join('share', p_name, f'models/{m_name}'),
            glob(f'models/{m_name}/*.sdf')),
        (os.path.join('share', p_name, f'models/{m_name}/meshes'),
            glob(f'models/{m_name}/meshes/*')),
        (os.path.join('share', p_name, f'models/{m_name}/materials/textures'),
            glob(f'models/{m_name}/materials/textures/*')),
    ]


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*.dae')),
        (os.path.join('share', package_name, 'meshes', 'images'),
            glob('meshes/images/*.jpg')),
        (os.path.join('share', package_name, 'param'),
            glob('param/*.yaml')),
        (os.path.join('share', package_name, 'map'),
            glob('map/*.yaml')),
        (os.path.join('share', package_name, 'map'),
            glob('map/*.pgm')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ]
    + model_data_files(package_name, 'airobotbook_world')
    + model_data_files(package_name, 'coke_can_airobotbook'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MASUTANI Yasuhiro',
    maintainer_email='ai-robot-book@googlegroups.com',
    description='Turtlebot2 model for AI Robot Book',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
