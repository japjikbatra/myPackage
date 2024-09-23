from setuptools import find_packages, setup

package_name = 'myPackage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                #'talker = myPackage.publisher_member_function:main',
                #'listener = myPackage.subscriber_member_function:main',
                'publish_map = myPackage.publisher_nodes:main',
                'publish_goal_coordinate = myPackage.publisher_nodes:main',
                'publish_start_coordinate = myPackage.publisher_nodes:main',
                'publish_path = myPackage.path_publisher:main',
        ],
    },
)
