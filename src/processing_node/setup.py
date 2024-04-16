from setuptools import find_packages, setup

package_name = 'processing_node'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # Add this line to include your YAML files
        ('share/' + package_name + '/test/test_config', ['test/test_config/config.yaml', 'test/test_config/integration_config.yaml', 'test/test_config/recorder_test_config.yaml']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marvin Frisch',
    maintainer_email='marvin.frisch@kit.edu',
    description='ROSMLDeploy enables building ML pipelines based on ROS2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['recorder=processing_node.recorder_node:main'],
    },
)
