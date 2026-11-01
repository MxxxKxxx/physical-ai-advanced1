import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from drawer import load_points, wrap  # 図形の読み込みは drawer.py のものを再利用

V_MAX = 0.22        # 前進の最高速度 [m/s]
W_MAX = 2.5         # 回転の最高速度 [rad/s]
LOOKAHEAD = 0.15    # 先読み距離 [m]（大きいほど滑らか、角は丸くなる）
STEP = 0.02         # 経路を細かく分ける間隔 [m]
V_MIN_RATIO = 0.15  # 急な角でも止まらず、この割合の速度は保つ


def densify(pts, step):
    """折れ線を細かい点の列に分ける"""
    out = [tuple(pts[0])]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        for k in range(1, n + 1):
            out.append((x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n))
    return out


class SmoothDrawer(Node):
    def __init__(self, raw_points):
        super().__init__('smooth_drawer')
        self.raw = raw_points
        self.path = None
        self.idx = 0
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
        cmd = Twist()

        if self.path is None:
            ox, oy = x - self.raw[0][0], y - self.raw[0][1]
            self.path = densify([(px + ox, py + oy) for px, py in self.raw], STEP)
            self.get_logger().info(f'描画開始：{len(self.raw)}点（細分化後 {len(self.path)}点）')

        if self.done:
            self.pub.publish(cmd)
            return

        # 進捗：今の位置にいちばん近い経路上の点（後戻りしない）
        best, best_d = self.idx, float('inf')
        for j in range(self.idx, min(self.idx + 30, len(self.path))):
            d = math.hypot(self.path[j][0] - x, self.path[j][1] - y)
            if d < best_d:
                best, best_d = j, d
        self.idx = best

        # ゴール判定
        gx, gy = self.path[-1]
        d_goal = math.hypot(gx - x, gy - y)
        if self.idx >= len(self.path) - 2 and d_goal < 0.03:
            self.done = True
            self.pub.publish(cmd)
            self.get_logger().info('描画完了')
            return

        # 先読み点を目標にする
        t = self.idx
        while t < len(self.path) - 1 and \
                math.hypot(self.path[t][0] - x, self.path[t][1] - y) < LOOKAHEAD:
            t += 1
        tx, ty = self.path[t]
        alpha = wrap(math.atan2(ty - y, tx - x) - yaw)

        # 向きのずれが大きいほど減速（ただし止まらない）
        v = V_MAX * max(V_MIN_RATIO, math.cos(alpha))
        if t == len(self.path) - 1:          # ゴール手前だけ減速
            v = max(0.03, v * min(1.0, d_goal / LOOKAHEAD))
        cmd.linear.x = v
        cmd.angular.z = max(-W_MAX, min(W_MAX, 3.0 * alpha))
        self.pub.publish(cmd)


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else 'star'
    rclpy.init()
    rclpy.spin(SmoothDrawer(load_points(arg)))


if __name__ == '__main__':
    main()
