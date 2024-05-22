# ROSMLDeploy
ROSMLDeploy enables versatile and scalable implementation of processing functionality in ROS2 contexts. It is aimed towards enabling deployment of machine learning models, but developers are free in the processes they want to implement. It includes automatic setup of ROS2 topics, publishers, and subscribers from config files for both processing and recording of data.

## Documentation

ToDo

## Getting Started

There is currently only one way to get started with the library: Use the source version from the repository.

### Using the source version

To install the source version from the repository, run the following commands:

```
git clone https://github.com/WBK-Robotics/ros_ml_deploy.git
cd ros_ml_deploy
pip install src/processing_node
```

#### Importing the ProcessingNode

Starting from this, you can import the ProcessingNode into your python project using

```python
from processing_node.processing_node import ProcessingNode
```

#### Using the RecorderNode

To use the RecorderNode, you need to additionally run
```
source install/setup.bash
```

before running 

```
ros2 run processing_node recorder --out_folder /path/to/output/folder --config_path /path/to/config --num 150
```

with the following arguments:

- out_folder: Path to output folder. Mandatory.

- config_path: Path to config.yaml specifying the relevant Inputs. Mandatory.

- num: Number of input points to record. Optional, defaults to "-1", i.e. record until manual stoppage.