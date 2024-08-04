from setuptools import find_packages, setup

package_name = 'rossi_eeg_stuff'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/start.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='develop',
    maintainer_email='sebastian.rossi@elkiro.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = rossi_eeg_stuff.better_publisher:main',
        ],
    },
)
