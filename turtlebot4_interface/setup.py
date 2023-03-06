from setuptools import setup

package_name = 'turtlebot4_interface'

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
    maintainer='tianmuw',
    maintainer_email='tianmuwang99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_client = turtlebot_interface.nav_client:main',
            'nav_server = turtlebot_interface.nav_server:main',
        ],
    },
)
