import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from drawer import load_points, wrap

V_MAX = 0.22                 # 前進の最高速度 [m/s]
W_MAX = 2.5                  # 回転の最高速度 [rad/s]
SHARP = math.radians(35)     # これより急な角は、止まって向きを変える
ALIGN_TOL = math.radians(5)  # その場回転で向きを合わせる精度
REACH_SHARP = 0.03           # 鋭い角への到達判定 [m]
REACH_SMOOTH = 0.06          # ゆるい角は、この距離まで来たら次へ（止まらない）


def clamp(v, m):
    return max(-m, min(m, v))


class PlotterDrawer(Node):
    """鋭い角だけ止まって回転し、それ以外は走り抜ける"""

    def __init__(self, raw_points):
        super().__init__('plotter_drawer')
        self.raw = raw_points
        self.pts = None
        self.i = 1
        self.mode = 'align'
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

    def turn_at(self, k):
        """点kでの曲がる角度（最後の点は鋭い角として扱う）"""
        if k >= len(self.pts) - 1:
            return math.pi
        (x0, y0), (x1, y1), (x2, y2) = self.pts[k - 1], self.pts[k], self.pts[k + 1]
        a1 = math.atan2(y1 - y0, x1 - x0)
        a2 = math.atan2(y2 - y1, x2 - x1)
        return abs(wrap(a2 - a1))

    def step(self):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        cmd = Twist()

        if self.pts is None:
            ox, oy = x - self.raw[0][0], y - self.raw[0][1]
            self.pts = [(px + ox, py + oy) for px, py in self.raw]
            self.get_logger().info(f'描画開始：{len(self.pts)}点')

        if self.done:
            self.pub.publish(cmd)
            return

        tx, ty = self.pts[self.i]
        dist = math.hypot(tx - x, ty - y)
        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        sharp = self.turn_at(self.i) > SHARP

        # 次の点へ進むか判定
        if dist < (REACH_SHARP if sharp else REACH_SMOOTH):
            self.i += 1
            if self.i >= len(self.pts):
                self.done = True
                self.pub.publish(cmd)
                self.get_logger().info('描画完了')
                return
            if sharp:
                self.mode = 'align'   # 鋭い角を曲がるときだけ、止まって向きを合わせる
            return

        # その場回転（辺の走り始めだけ）
        if self.mode == 'align':
            if abs(err) < ALIGN_TOL:
                self.mode = 'drive'
            else:
                cmd.angular.z = clamp(3.0 * err, W_MAX)
                self.pub.publish(cmd)
                return

        # 大きくずれたときだけ向きを合わせ直す
        if abs(err) > math.radians(60):
            self.mode = 'align'
            return

        # 走行：鋭い角の手前だけ減速
        v = V_MAX
        if sharp:
            v = min(V_MAX, max(0.05, 1.2 * dist))
        cmd.linear.x = v
        cmd.angular.z = clamp(3.0 * err, W_MAX)
        self.pub.publish(cmd)


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else 'star'
    rclpy.init()
    rclpy.spin(PlotterDrawer(load_points(arg)))


if __name__ == '__main__':
    main()
