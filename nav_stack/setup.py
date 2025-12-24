from setuptools import find_packages, setup

package_name = 'nav_stack'

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
    maintainer='Colin Rousseau',
    maintainer_email='colin.rousseau@etud.polymtl.ca',
    description='Navigation stack for aeac 2026',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lap = nav_stack.lap:main',
            'init = nav_stack.init:main',
            'convert = nav_stack.convert:main',
        ],
    },
)
