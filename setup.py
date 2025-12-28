from setuptools import find_packages, setup

package_name = 'speed_lim_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aki Moto',
    maintainer_email='aki.robosys2025@gmail.com',
    description='Limits the speed status sent to cmd_vel',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lim_node = speed_lim_pkg.lim_node:main',
        ],
    },
)
