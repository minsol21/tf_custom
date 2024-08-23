from setuptools import find_packages, setup

package_name = 'tf_custom'

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
    maintainer='mkim',
    maintainer_email='mkim@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broadcaster_node = tf_custom.broadcaster_node:main',
            'central_node = tf_custom.central_node:main',
            'turtlebotmover = tf_custom.turtlebotmover:main',
            'turtlebotmover2 = tf_custom.turtlebotmover2:main',
        ],
    },
)
