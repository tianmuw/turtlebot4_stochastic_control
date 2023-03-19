from setuptools import setup

package_name = 'stochastic_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml.xml']),
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
            'nav_action_client = stochastic_control.nav_action_client:main',
            'nav_action_server = stochastic_control.nav_action_server:main',
            'stochastic_control = stochastic_control.stochastic_control:main',
        ],
    },
)
