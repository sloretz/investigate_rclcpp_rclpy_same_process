# rclcpp_rclpy_same_process
This repo explored using ROS 2 rclcpp and rclpy in the same process using ROS Rolling Ridley in October/November 2020.
The goal was to see if any unexpected issues would come up when using rclcpp and rclpy in the same process.

# Results

Rclpy and rclcpp appear to happily coexist in the same process (using different `rcl_context_t` instances), and can publish/subscribe to each other topics and send/receive messages in both directions.
This worked using the rmw implementations `rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp`, `rmw_connext_cpp`

# What's here

Rclcpp was put into in a CPython extension using Pybind11.
The extension creates to classes using rclcpp.
One initializes the ROS context, and the other uses that to create a `PingPong` node with a publish and subscriber on different topics.
This is contained in `CMakeLists.txt` and `use_ros.cpp`.

A similar `PingPongPy` class is created in the file `both.py`.
This file imports the C++ CPython extension and declares a similar python class.
The python class has a publisher and subscriber on the same (but opposite) topics as the C++ one.

Both `rclpy` and `rclcpp` nodes are spun in different python threads.
Both threads must of course release the GIL, which `rclpy` already does but I had to do manually for `rclcpp` in the CPython extension.

Calling `PingPongPy.ping()` sends a message from Python to the C++ subscriber, and calling `PingPong.ping()` sends a message from C++ to Python.
This is how python -> c++ and c++ -> python over ROS topics in the same process is tested.

# Not tested

`rmw_gurumdds_cpp` was not tested because I don't have a license.
I did not try to see if `rclcpp` and `rclpy` could use the same `rcl_context_t`, mostly because it was not obvious to me how I would pass that context from one client library to the other.
I did not test services or actions.

# Interesting future work

Since the two client libraries work in the same process, it might be interesting to try to make a composable node container that could load both `rclcpp` and `rclcpy` composable nodes. 
