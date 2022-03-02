from setuptools import setup

package_name = 'robot_mock'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kálley Wilkerson',
    maintainer_email='kalleywra@gmail.com',
    description='Libraries for mocking robots',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_pub = robot_mock.battery:main'
        ],
    },
)
