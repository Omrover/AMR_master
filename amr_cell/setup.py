from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_cell'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'CELL_CONTRACT.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'paho-mqtt>=1.6,<3'],
    zip_safe=True,
    maintainer='Omrover',
    maintainer_email='sanduich13@gmail.com',
    description='Comunicación MQTT con el PLC de la celda (aviso de pieza colocada).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'cell_client = amr_cell.cell_client:main',
        'cell_sim = amr_cell.cell_sim:main',
    ]},
)
