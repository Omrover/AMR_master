from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Omrover',
    maintainer_email='sanduich13@gmail.com',
    description='Gemelo digital en Gazebo Harmonic del Mobile Manipulator Omrover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'gz_mcu = amr_gazebo.gz_mcu:main',
    ]},
)
