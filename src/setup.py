from setuptools import setup
from glob import glob

package_name = 'wall_following_diff_robot'

setup(
    name=package_name,
    version='0.0.1',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch/", glob("launch/*launch*")),
        ('share/' + package_name + "/rviz/", glob("rviz/*")),
        ('share/' + package_name + "/world/" , glob('world/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    # Declare the enry point for your python scripts
    entry_points={
        'console_scripts': [
            'wall_following_diff_robot = wall_following_diff_robot.__init__:main'
        ],
    },
)
