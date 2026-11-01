import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty


class TrailNode(Node):
    """/odom を記録して /trail（Path）として配信する"""

    def __init__(self):
        super().__init__('trail_node')
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.last_xy = None
        self.min_step = 0.03  # 1cm以上動いたら点を追加

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Empty, '/trail_clear', self.on_clear, 10)
        self.pub = self.create_publisher(Path, '/trail', 10)
        self.create_timer(0.1, self.publish_trail)

    def on_odom(self, msg):
        self.path.header.frame_id = msg.header.frame_id
        self.path.header.stamp = msg.header.stamp
        p = msg.pose.pose.position
        if self.last_xy is not None:
            dx, dy = p.x - self.last_xy[0], p.y - self.last_xy[1]
            if dx * dx + dy * dy < self.min_step ** 2:
                return
        self.last_xy = (p.x, p.y)
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.path.poses.append(ps)

    def on_clear(self, _msg):
        self.path.poses.clear()
        self.last_xy = None
        self.get_logger().info('軌跡を消去しました')

    def publish_trail(self):
        self.pub.publish(self.path)


def main():
    rclpy.init()
    rclpy.spin(TrailNode())


if __name__ == '__main__':
    main()
