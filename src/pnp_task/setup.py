from setuptools import find_packages, setup

package_name = 'pnp_task'

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
    maintainer='nast41',
    maintainer_email='subrahmanyam.kodela33@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
               'gz_grasp_bridge = pnp_task.gz_grasp_bridge:main',
               'controller_activator = pnp_task.controller_activator:main',
               'sorter = pnp_task.sorter_node:main',
        ],
    },
)
