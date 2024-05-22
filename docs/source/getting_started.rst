###############
Getting started
###############

There is currently only one way to get started with the library: Use the source version from the repository.

------------------------
Using the source version
------------------------

To install the source version from the repository, run the following commands:

::
    
    git clone https://github.com/WBK-Robotics/ros_ml_deploy.git
    cd ros_ml_deploy
    pip install src/processing_node

..

Starting from this, you can import the :class:`ProcessingNode` into your python project using

.. code-block:: python

    from processing_node.processing_node import ProcessingNode

To use the :class:`RecorderNode`, you need to additionally run
::
    source install/setup.bash
::

before running the command specified in "How to use the package"