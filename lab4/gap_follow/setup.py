from glob import glob

from setuptools import setup

package_name = 'gap_follow'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marcantonio Povolo',
    maintainer_email='povolo.ma@gmail.com',
    description='RoboRacer Lab 4: reactive follow the gap, and obstacle avoidance.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'reactive_node = gap_follow.reactive_node:main',
            'obstacle_avoidance = gap_follow.obstacle_avoidance:main',
            'make_racing_line = gap_follow.racing_line:main',
        ],
    },
)
