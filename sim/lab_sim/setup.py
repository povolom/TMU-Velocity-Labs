from glob import glob

from setuptools import setup

package_name = 'lab_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marcantonio Povolo',
    maintainer_email='povolo.ma@gmail.com',
    description='Launches the RoboRacer simulator on a chosen map and start pose.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'steady_clock = lab_sim.steady_clock:main',
            'crash_reset = lab_sim.crash_reset:main',
        ],
    },
)
