# 1. 启动 PX4
cd ~/PX4-Autopilot
#git checkout exp/rate-pid-eso
make px4_sitl none_iris

# 2. 启动 ROS HIL 闭环
source ~/catkin_ws/devel/setup.bash
roslaunch quad_sim pure_hil.launch rviz:=true monitor:=true

# 3. 启动高度循环实验
source ~/catkin_ws/devel/setup.bash
rosrun offboard_py offb_height_cycle_node.py \
  _target_delta_z:=1.0 \
  _num_cycles:=5 \
  _z_ramp_rate_up:=0.02 \
  _z_ramp_rate_down:=0.02 \
  _hover_top_sec:=10.0 \
  _hover_base_sec:=4.0 \
  _start_hold_sec:=2.0 \
  _start_settle_tol:=0.08 \
  _return_home_delta_z:=0.35 \
  _return_xy_tol:=0.12 \
  _return_z_tol:=0.10 \
  _return_home_stable_sec:=1.0 \
  _base_xy_tol:=0.12 \
  _base_z_tol:=0.08 \
  _base_stable_sec:=2.0 \
  _z_reach_tol:=0.06 \
  _estimator_timeout_sec:=5.0
