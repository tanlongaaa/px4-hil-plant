# PX4 HIL Plant — 六自由度可编程物理引擎 (v9)

**对标 Gazebo Iris 模型的 HIL 仿真框架**

> ROS1 Noetic | PX4 v1.13.3 | MAVLink HIL | RViz 3D 可视化

---

## 项目概述

`px4-hil-plant` 是一个 **PX4 Hardware-in-the-Loop (HIL) 仿真框架**，核心是自建的 6-DOF 可编程物理引擎 `plant_6dof.py`，通过 MAVLink 与 PX4 飞控对接，替代 Gazebo 进行高速、可编程的物理仿真。

**设计目标：** 动力学行为对标 Gazebo Iris SDF 模型，同时保持代码完全可控（参数可调、物理可扩展、无 Gazebo 依赖）。

### 两条仿真通路

| 通路 | 启动文件 | 物理引擎 | 用途 |
|------|---------|---------|------|
| **HIL** (推荐) | `hil_backend.launch` | plant_6dof | PX4 真飞控 + 可编程物理 |
| 备用 | `pure_hil.launch` | plant_6dof | 同上，含更多节点 |

---

## 目录结构

```
px4-hil-plant/
├── README.md
│
├── quad_sim/                          # ★ 核心仿真包
│   ├── config/
│   │   └── sim_default.yaml           # 全局参数 (plant + backend + sensors)
│   │
│   ├── scripts/
│   │   ├── plant_6dof.py              # ★★★ 6-DOF 可编程物理引擎
│   │   ├── backend_main.py            # HIL Backend 主循环 (MAVLink ↔ Plant)
│   │   ├── mavlink_backend.py         # MAVLink HIL 接口 (TCP 4560)
│   │   ├── sensor_models.py           # IMU / GPS / 气压计模型
│   │   ├── sim_bridge_odom.py         # 备用通路 (Odom 直驱，不依赖 PX4)
│   │   ├── drone_6dof_rviz_demo.py    # RViz 3D 可视化节点
│   │   ├── flight_state_monitor.py    # 飞行状态监控
│   │   ├── px4_bridge.py              # PX4 MAVLink 桥接
│   │   ├── px4_6dof_node.py           # PX4 6-DOF 控制节点
│   │   ├── px4_style_rate_controller.py # PX4 风格角速率控制器
│   │   ├── simple_hover_controller.py # 简易悬停控制器
│   │   ├── truth_odom_to_mavros.py    # 真值 Odometry → MAVROS
│   │   └── test_cmd_pub.py            # 测试命令发布器
│   │
│   ├── launch/
│   │   ├── hil_backend.launch         # ★ HIL 主启动 (支持 rviz:=true)
│   │   ├── pure_hil.launch            # HIL 全节点启动
│   │   ├── demo.launch                # 简易演示
│   │   ├── mavros_odom_loop.launch    # MAVROS + Odom 闭环
│   │   └── truth_odom_bridge.launch   # 真值 Odom 桥接
│   │
│   ├── rviz/
│   │   ├── quad_sim.rviz              # RViz 配置 (3D 四旋翼 + 轨迹)
│   │   └── demo.rviz                  # 简化 RViz 配置
│   │
│   └── urdf/
│       └── quadrotor.urdf             # 四旋翼 URDF 模型
│
├── offboard_py/                        # Offboard 控制包
│   ├── scripts/
│   │   ├── offb_node.py               # 简单 offboard 测试
│   │   ├── offb_pos_node.py           # 位置控制
│   │   ├── offb_vel_node.py           # 速度控制
│   │   ├── offb_rate_node.py          # 角速率控制
│   │   ├── offb_raw_test.py           # Raw 控制测试
│   │   └── offb_height_cycle_node.py  # 高度循环实验
│   └── offb_hold.py                   # 悬停节点
│
└── .gitignore
```

---

## v9 重构：原始版本 vs 当前版本

### 核心差异对比

| 模块 | 原始版本 (3171dab) | v9 当前版本 (4f32292) |
|------|-------------------|----------------------|
| **惯量参数** | J=diag(0.02,0.02,0.04) ❌ | J=diag(0.029,0.029,0.055) ✅ iris.sdf |
| **推力常数** | k_thrust=9.1 | k_thrust=7.36 (标定到 hover=0.50) |
| **悬停油门** | hover_throttle=0.42 | hover_throttle=0.50 |
| **线性阻尼** | kV=4.0 (简单标量) | base_drag=3.5 + 各向异性修正 (x/y/z) |
| **角阻尼** | kw=0.45 | kw=0.3 |
| **力矩 Mixer** | motor_*_moment=0.08/0.08/0.02 ❌ | 1.30/1.30/0.12 ✅ 对标 Gazebo |
| **转子侧向阻力** | ❌ 无 | ✅ Martin & Salaün (2010) |
| **Blade Flapping** | ❌ 无 | ✅ τ=-Σ\|ω\|·k_flap·v_perp |
| **陀螺力矩** | ❌ 无 | ✅ τ=I_rotor·Σεω·(ez×ω_body) |
| **动态入流** | ❌ 无 | ✅ Pitt-Peters 一阶滤波 |
| **各向异性 drag** | ❌ 无 | ✅ body_drag_x/y/z=0.10/0.30/0.1 |
| **二次方风阻** | ❌ 无 | ✅ body_CdA=0.12 |
| **风力注入** | 静态力参数 | ✅ 双路径 (rotor-level + body-level) |
| **控制模式** | actuator_controls | motor_outputs (du-based mixer) |

### plant_6dof.py 关键改动

```
原始 (3171dab)                    v9 当前 (4f32292)
══════════════                    ════════════════
控制: Roll/Pitch/Yaw/Throttle     控制: du-based mixer (motor_outputs)
      → 简单比例映射                   → k_thrust·sum(u) = 推力
                                       → motor_moment·sum(du·rs/ps/ys) = 力矩
阻尼: F=-kV*v                     阻尼: F_body=-base_drag*v_body  (各向异性)
                                       + F_rotor=-Σ|ω|·k_d·v_perp (Martin 2010)
风:   静态 wind_force_enu         风:   set_wind_vel_enu → rotor drag + flap
                                       set_ext_force_enu → 机身 CdA
无被动效应                         blade flapping + gyro + inflow

运动学: 欧拉角 (直积)             运动学: 欧拉角 (clamp 保护)
                                      + 欧拉方程 Jω̇=ω×Jω+τ, coriolis damping
```

### backend_main.py 关键改动

```
原始                             v9 当前
════                             ══════
无风场订阅                       订阅 /wind_field/velocity
无 wind_fresh 超时                wind_fresh() 超时保护 (无风数据→风=0)
无风力注入                       双路径: set_wind_vel_enu + set_ext_force_enu
                                 风力仅在 DYNAMIC 阶段注入
```

---

## 快速启动

### Step 1: 环境准备

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### Step 2: Backend + RViz 3D 可视化

```bash
export DISPLAY=:0
roslaunch quad_sim hil_backend.launch rviz:=true
```

> Backend 监听 `tcp://0.0.0.0:4560`，等待 PX4 SITL 连接。

### Step 3: PX4 SITL（无 Gazebo 守护进程模式）

```bash
cd ~/Desktop/px4rl/PX4-Autopilot
NO_PXH=1 no_sim=1 make px4_sitl none_iris
```

> ⚠️ 必须是这个 PX4 路径 (v1.13.3)。`NO_PXH=1` 是守护进程模式，`no_sim=1` 跳过 Gazebo。

### Step 4: MAVROS

```bash
rosrun mavros mavros_node _fcu_url:=udp://:14540@127.0.0.1:14580
```

### Step 5: 设置 PX4 参数并起飞

```bash
# PX4 出厂 PID 参数
rosservice call /mavros/param/set '{param_id: "SYS_HITL", value: {integer: 1}}'
rosservice call /mavros/param/set '{param_id: "MPC_XY_VEL_P_ACC", value: {real: 1.8}}'
rosservice call /mavros/param/set '{param_id: "MPC_XY_VEL_I_ACC", value: {real: 0.4}}'
rosservice call /mavros/param/set '{param_id: "MPC_XY_VEL_D_ACC", value: {real: 0.2}}'
rosservice call /mavros/param/set '{param_id: "MPC_XY_P", value: {real: 0.95}}'
rosservice call /mavros/param/set '{param_id: "MPC_TILTMAX_AIR", value: {real: 45.0}}'
rosservice call /mavros/param/set '{param_id: "MPC_Z_VEL_P_ACC", value: {real: 4.0}}'
rosservice call /mavros/param/set '{param_id: "MPC_Z_VEL_I_ACC", value: {real: 2.0}}'

# 启动 PID 控制器 (来自 tcw-mpc 项目)
rosrun offboard_test pid_baseline.py
```

---

## 阶跃风场测试

### 方法：通过 ROS topic 直接注入风数据

plant_6dof 的风力通过 `backend_main.py` 的 `/wind_field/velocity` 话题注入：

```
step_wind.py  ──Vector3Stamped──▶  /wind_field/velocity
                                        │
                              backend_main._wind_cb()
                                        │
                        plant.set_wind_vel_enu(wind_3d)
                            (rotor drag + blade flapping)
                        plant.set_ext_force_enu(f_body_drag)
                            (0.5·ρ·CdA·|Vrel|·Vrel)
```

### 阶跃风脚本示例

```python
#!/usr/bin/env python3
"""发布阶跃风: 15s 无风 → 30s 5m/s → 45s 恢复"""
import rospy
from geometry_msgs.msg import Vector3Stamped

rospy.init_node("step_wind")
pub = rospy.Publisher("/wind_field/velocity", Vector3Stamped, queue_size=10)
rate = rospy.Rate(10)  # 10 Hz

# 第一阶段: 无风 (0-15s)
for i in range(150):
    pub.publish(Vector3Stamped(
        header=rospy.Header(stamp=rospy.Time.now()),
        vector=Vector3(0, 0, 0)))
    rate.sleep()

rospy.loginfo("Wind ON: 5 m/s +Y")
# 第二阶段: 阶跃风 (15-45s)
for i in range(300):
    pub.publish(Vector3Stamped(
        header=rospy.Header(stamp=rospy.Time.now()),
        vector=Vector3(0, 5, 0)))  # x=0, y=5, z=0
    rate.sleep()

rospy.loginfo("Wind OFF — observing recovery")
# 第三阶段: 恢复 (45-90s)
for i in range(450):
    pub.publish(Vector3Stamped(
        header=rospy.Header(stamp=rospy.Time.now()),
        vector=Vector3(0, 0, 0)))
    rate.sleep()
```

### 风力注入的物理路径

| 路径 | 物理效应 | 注入方式 |
|------|---------|---------|
| `set_wind_vel_enu()` | 转子侧向阻力 + Blade flapping | `F_drag = -Σ|ω|·k_d·(v_body-v_wind)` |
| `set_ext_force_enu()` | 机身二次方风阻 | `F = 0.5·ρ·CdA·|Vrel|·Vrel` |
| 线性阻尼 | 仅用 v_body (不含风) | `F = -(base_drag+body_drag)·v_body` |

> ⚠️ 风只从 `set_wind_vel_enu` 一条路径进入，不再有三重计数问题。

---

## 验证结果 (2026-06-19)

| 测试 | 条件 | 结果 |
|------|------|------|
| 无风悬停 | v9 全效应, PX4 出厂 PID | err_xy ±6 mm |
| Blade Flapping | flap=5e-5 单独开 | err_xy ±6 mm |
| Gyroscopic | I_rotor=4e-5 单独开 | err_xy ±6 mm |
| Dynamic Inflow | inflow_tau=0.05 单独开 | err_xy ±5 mm |
| 五效应全开 | flap+gyro+inflow+drag+CdA | err_xy ±10 mm |
| 阶跃风 5 m/s | 全开, 风停后恢复 | 最大漂移 0.86 m, 15s 拉回 |

---

## 关键教训

### motor_roll_moment 16 倍 mismatch

原始版本 `motor_roll_moment=0.08` 比正确值 1.30 小 16 倍，导致 PX4 出厂 PID 严重欠驱动——plant 的力矩响应只有 Gazebo 的 1/16，飞控角度指令到实际角速度的映射弱 16 倍。

**修复:** `motor_roll_moment=1.30`, `motor_pitch_moment=1.30`, `motor_yaw_moment=0.12`

### 风力三重计数

原始版本 wind 同时从三条路径影响动力学（线性阻尼含风 + 外力风 + 转子风），导致 3-6 倍超量风阻。

**修复:** 风统一从 `set_wind_vel_enu` 进入，线性阻尼只用 v_body，`set_ext_force_enu` 单独计算二次方风阻。

---

## 相关项目

| 项目 | 仓库 | 说明 |
|------|------|------|
| TCW-MPC | `tanlongaaa/tcw-mpc` | MPC 控制器 (mpc_node.py, wind_field.py) |
| PX4 Autopilot | `PX4/PX4-Autopilot` | v1.13.3 fork, 不可 push |
| ACMPC | `uzh-rpg/acmpc_public` | 参考项目 (TRO 2025) |
