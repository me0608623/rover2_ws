from setuptools import setup
import os
from glob import glob

package_name = 'campusrover_rl_policy'

setup(
    name=package_name,
    version='0.1.0',
    packages=['scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aa',
    maintainer_email='aa@todo.todo',
    description='RL policy inference node for VLP-16 navigation',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'rl_policy_node = scripts.rl_policy_node:main',
        ],
    },
)
