from setuptools import setup

package_name = 'dynamic_transform_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/dynamic_transform_publisher_launch.py']),
    ('share/' + package_name + '/config', ['config/params.yaml']),  
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quangluong',
    maintainer_email='quangluong@todo.todo',
    description='Publish dynamic transforms based on input',
    license='Apache License 2.0',  
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_transform = dynamic_transform_publisher.robot_tf:main',
            'odom_back = dynamic_transform_publisher.odom:main',
             

        ],
    },
)
