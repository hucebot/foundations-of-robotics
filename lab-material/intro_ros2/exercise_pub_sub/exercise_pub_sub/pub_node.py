import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # TODO: declare and load params 'topic_name' and 'timer_period'
        # ...
        # TODO: define a String publisher over 'chatter'
        # ...
        # TODO: define  the timer
        # ...
        # TODO: init the counter variable
        # ...

    def timer_callback(self):
        # TODO: fill the String message and publish it
        # ...
        # TODO: print the message (using log INFO)
        # ...
        # TODO: increase the counter variable
        # ...
        pass


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()