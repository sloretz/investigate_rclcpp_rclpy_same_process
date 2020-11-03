#!/usr/bin/env python

import threading

import rccl
import rclpy
import rclpy.node
from std_msgs.msg import String as StringMsg

print('hey, able to import both :)')

def rccl_setup():
    r = rccl.ROS()
    pp = rccl.PingPong(r)

    def spin():
        print('spin_cpp start')
        r.spin()
        print('spin_cpp stop')

    t = threading.Thread(target=spin, daemon=True)
    t.start()
    return r, pp, t


def rclpy_setup():

    class PingPongPy(rclpy.node.Node):
        def __init__(self):
            super().__init__('PingPongPy')
            self._publisher = self.create_publisher(StringMsg, 'py_to_cpp', 10)
            self._subscription = self.create_subscription(StringMsg, 'cpp_to_py', self._pong, 10)

        def ping(self, msg='Python ping'):
            message = StringMsg()
            message.data = msg
            self._publisher.publish(message)

        def _pong(self, message):
            print("python:", message.data)

    rclpy.init()
    py_node = PingPongPy()

    def spin():
        print('spin_python start')
        rclpy.spin(py_node)
        print('spin_python stop')

    t = threading.Thread(target=spin, daemon=True)
    t.start()
    return py_node, t


if __name__ == '__main__':
    ros_context, cpp_node, t_cpp = rccl_setup()
    print('able to start rclcpp via python')

    py_node, t_py = rclpy_setup()
    print('able to start rclpy and rclcpp nodes :)')

    import code
    code.interact(local=locals())
