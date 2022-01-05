from setuptools import setup

package_name = 'jetbot_motors'

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
    maintainer='eric',
    maintainer_email='epiraux@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pioled = jetbot_motors.pioled_node:main',
            'pioled_stats = jetbot_motors.pioled_stats:main',
            'motors = jetbot_motors.motors_node:main'
        ],
    },
)
