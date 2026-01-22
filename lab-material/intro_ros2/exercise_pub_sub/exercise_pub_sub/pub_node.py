import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # TODO: declare and load params 'topic_name' and 'timer_period'
        self.declare_parameter("topic_name", "chatter")
        self.declare_parameter("timer_period", 1.0)
        topic_name = self.get_parameter("topic_name").value
        timer_period = self.get_parameter("timer_period").value
        # TODO: define a String publisher over 'chatter'
        self.publisher = self.create_publisher(String, topic_name, 10)
        # TODO: define  the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # TODO: init the counter variable
        self.i = 0

    def timer_callback(self):
        # TODO: fill the String message and publish it
        msg = String()
        msg.data = f"Counter: {self.i}"
        self.publisher.publish(msg)
        # TODO: print the message (using log INFO)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        # TODO: increase the counter variable
        self.i += 1


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