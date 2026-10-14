from setuptools import find_packages, setup

package_name = 'nav_stack'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Colin Rousseau',
    maintainer_email='colinc131@gmail.com',
    description='Shared navigation nodes for Zenith mission repos: init and takeoff, GPS to local conversion, laps streaming, traffic deconfliction',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'init = nav_stack.init:main',
            'convert = nav_stack.convert:main',
            'lap = nav_stack.lap:main',
            'deconfliction = nav_stack.deconfliction:main',
        ],
    },
)
