from setuptools import find_packages, setup

package_name = 'processing_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to include your YAML files
        ('share/' + package_name + '/test/test_config', ['test/test_config/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Processing node to subscribe to ROS topics to give out processed data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    },
)
