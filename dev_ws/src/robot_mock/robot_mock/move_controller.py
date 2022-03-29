import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('move_controler')
        self.publisher_ = self.create_publisher(String, 'movebase/robot', 10)
        timer_period = 1.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        if self.i == 1:
            msg.data = 'goto 150 160'
        elif self.i >= 20:
            msg.data = "exit"
        else:
            msg.data = 'idle %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    move_controler = MinimalPublisher()

    rclpy.spin(move_controler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move_controler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()