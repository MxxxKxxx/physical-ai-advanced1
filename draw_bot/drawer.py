import json
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def star(r=0.8):
    """一筆書きの星（五芒星）"""
    order = [0, 2, 4, 1, 3, 0]
    return [[r * math.cos(math.radians(90 + 72 * k)),
             r * math.sin(math.radians(90 + 72 * k))] for k in order]


SHAPES = {
    'square': [[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]],
    'triangle': [[0, 0], [1.2, 0], [0.6, 1.0], [0, 0]],
    'star': star(),
}


def load_points(arg):
    if arg in SHAPES:
        return SHAPES[arg]
    with open(arg) as f:          # JSONファイル {"points": [[x, y], ...]}
        return json.load(f)['points']


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Drawer(Node):
    """点の列を順番にたどって走る（今の位置を図形の始点に合わせる）"""

    def __init__(self, raw_points):
        super().__init__('drawer')
        self.raw = raw_points
        self.pts = None
        self.i = 0
        self.pose = None
        self.done = False
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.step)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def step(self):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        if self.pts is None:
            ox, oy = x - self.raw[0][0], y - self.raw[0][1]
            self.pts = [(px + ox, py + oy) for px, py in self.raw]
            self.i = 1
            self.get_logger().info(f'描画開始：{len(self.pts)}点')
        cmd = Twist()
        if self.i >= len(self.pts):
            self.pub.publish(cmd)  # 停止
            if not self.done:
                self.get_logger().info('描画完了')
                self.done = True
            return
        tx, ty = self.pts[self.i]
        dist = math.hypot(tx - x, ty - y)
        if dist < 0.03:
            self.i += 1
            return
        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        if abs(err) > 0.25:        # 向きが大きくずれていたら、その場で回転
            cmd.angular.z = max(-2.5, min(2.5, 3.0 * err))
        else:                      # 向きが合っていたら、進みながら微調整
            cmd.linear.x = min(0.22, 1.0 * dist)
            cmd.angular.z = max(-2.5, min(2.5, 3.0 * err))
        self.pub.publish(cmd)


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else 'star'
    rclpy.init()
    rclpy.spin(Drawer(load_points(arg)))


if __name__ == '__main__':
    main()
