#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import subprocess

class ClockSync(Node):
    def __init__(self):
        super().__init__('ida_clock_sync')
        self.subscription = self.create_subscription(
            Clock,
            '/ap/clock',
            self.clock_callback,
            10
        )
        self.synced = False
        self.get_logger().info("[CLOCK SYNC] /ap/clock bekleniyor (Pixhawk zaman senkronizasyonu)...")

    def clock_callback(self, msg):
        if self.synced:
            return  # Bir kez senkronize olmak yeterlidir

        sec = msg.clock.sec
        nanosec = msg.clock.nanosec

        if sec > 1000000000:  # Mantıklı bir tarih kontrolü (1970'den büyük)
            total_seconds = sec + (nanosec / 1e9)
            date_str = f"@{total_seconds}"

            try:
                subprocess.run(["date", "-u", "-s", date_str], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self.get_logger().info(f"[CLOCK SYNC] BASARILI! Jetson saati Pixhawk ile esitlendi: {total_seconds}")
                self.synced = True
            except Exception as e:
                self.get_logger().error(f"[CLOCK SYNC] Saat ayarlanamadi: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ClockSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()