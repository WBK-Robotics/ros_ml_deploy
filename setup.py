from setuptools import find_packages, setup

package_name = 'processing_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # Add this line to include your YAML files
        ('share/' + package_name + '/test/test_config', ['src/processing_node/test/test_config/config.yaml', 'src/processing_node/test/test_config/integration_config.yaml']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Processing node to subscribe to ROS topics to give out processed data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['recorder=processing_node.recorder_node:main'],
    },
)
