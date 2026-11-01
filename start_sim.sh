#!/bin/bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_DATABASE_URI=""

# 古いGazeboを終了
pkill -9 -f gzserver; pkill -9 -f gzclient
sleep 1

# Gazebo本体を起動（裏で動かす）
gzserver -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
  /opt/ros/humble/share/turtlebot3_gazebo/worlds/empty_world.world &

echo "Gazeboの起動を待っています..."
until ros2 service list --no-daemon 2>/dev/null | grep -q /spawn_entity; do sleep 2; done

# ロボットを出現させる（返事は待たない）
ros2 run gazebo_ros spawn_entity.py -entity burger \
  -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
  -x 0 -y 0 -z 0.01 -timeout 120 &

echo "ロボットの出現を待っています..."
until ros2 topic list --no-daemon 2>/dev/null | grep -qx /odom; do sleep 2; done
echo "準備完了。Ctrl+Cで終了します。"
wait
