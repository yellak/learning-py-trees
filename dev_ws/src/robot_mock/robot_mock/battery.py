import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import sensor_msgs.msg as sensor_msgs


class BatteryPublisher(Node):
    """
    This is a mock for a publisher that publishes battery status
    """

    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher_ = self.create_publisher(sensor_msgs.BatteryState, '/battery/state', 10)
        timer_period = 1.5  # seconds

        # initialisations
        self.battery = sensor_msgs.BatteryState()
        self.battery.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.battery.voltage = float('nan')
        self.battery.current = float('nan')
        self.battery.charge = float('nan')
        self.battery.capacity = float('nan')
        self.battery.design_capacity = float('nan')
        self.battery.percentage = 100.0
        self.battery.power_supply_health = sensor_msgs.BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self.battery.power_supply_technology = sensor_msgs.BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_FULL
        self.battery.present = True
        self.battery.location = ""
        self.battery.serial_number = ""

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.battery.percentage -= 1
        self.publisher_.publish(self.battery)
        self.get_logger().info('Publishing: "%s"' % self.battery.percentage)


def main(args=None):
    rclpy.init(args=args)

    battery_publisher = BatteryPublisher()

    rclpy.spin(battery_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    battery_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()