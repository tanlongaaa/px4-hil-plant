#!/bin/bash
# ==============================================================================
# start_hil_full.sh — 六自由度无人机 HIL 全链路一键脚本
# ==============================================================================
# 用法:
#   bash start_hil_full.sh              # 默认: 悬停 20s → 5m/s 阶跃风 30s → 恢复 45s
#   bash start_hil_full.sh --no-wind    # 仅悬停, 不打风
#   bash start_hil_full.sh --wind 7.0   # 自定义风速 (m/s, ENU +Y 即正北)
#
# 架构:
#   Backend(plant_6dof) ──TCP──▶ PX4 SITL ──UDP──▶ MAVROS ──ROS──▶ pid_baseline
#        ▲                                                             │
#        └──────── /wind_field/velocity (阶跃风) ──────────────────────┘
#
# 依赖:
#   - ROS Noetic + catkin_ws
#   - PX4-Autopilot @ ~/Desktop/px4rl/PX4-Autopilot
#   - quad_sim (plant_6dof + backend_main + sensor_models)
# ==============================================================================

set -euo pipefail

# ── 默认参数 ─────────────────────────────────────────────────────────────────
WIND_SPEED=5.0        # 阶跃风速 (m/s, ENU +Y)
HOVER_WARMUP=20       # 悬停稳定等待时间 (秒)
WIND_DURATION=30      # 阶跃风持续时间 (秒)
RECOVERY_DURATION=45  # 风停后恢复观察时间 (秒)
NO_WIND=false

# ── 解析命令行 ───────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-wind) NO_WIND=true; shift ;;
        --wind) WIND_SPEED="$2"; shift 2 ;;
        *) echo "未知参数: $1"; echo "用法: bash start_hil_full.sh [--no-wind] [--wind SPEED]"; exit 1 ;;
    esac
done

# ── 路径 ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
QUAD_SIM_DIR="$PROJECT_DIR/quad_sim"
ROS_SETUP="/opt/ros/noetic/setup.bash"
CATKIN_SETUP="$HOME/catkin_ws/devel/setup.bash"
PX4_DIR="$HOME/Desktop/px4rl/PX4-Autopilot"
CSV_DIR="$QUAD_SIM_DIR/scripts"

# ── 颜色 ─────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BLUE='\033[0;34m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $(date '+%H:%M:%S')  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $(date '+%H:%M:%S')  $*"; }
err()   { echo -e "${RED}[ERR]${NC}   $(date '+%H:%M:%S')  $*" >&2; }
step()  {
    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN}  $*${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

# ── 全局 PID ─────────────────────────────────────────────────────────────────
BACKEND_PID=""; PX4_PID=""; MAVROS_PID=""; PIDCTL_PID=""; WIND_PID=""

# ── 清理函数 ─────────────────────────────────────────────────────────────────
cleanup() {
    echo ""
    warn "正在清理所有进程..."
    for pid in $WIND_PID $PIDCTL_PID $MAVROS_PID $PX4_PID $BACKEND_PID; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    pkill -f "step_wind_hil" 2>/dev/null || true
    pkill -f "pid_baseline" 2>/dev/null || true
    pkill -f "mavros_node" 2>/dev/null || true
    sleep 1
    info "清理完成"
}
trap cleanup EXIT INT TERM

# ── 工具函数 ─────────────────────────────────────────────────────────────────
get_pose() {
    # 返回 "px py pz" 三个值
    rostopic echo /mavros/local_position/pose -n1 2>/dev/null \
        | grep -E '^\s+[xyz]:' | head -3 | awk '{print $2}'
}

get_state() {
    rostopic echo /mavros/state -n1 2>/dev/null
}

# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                           主流程                                            ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

echo ""
echo -e "${BLUE}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║    六自由度无人机 HIL 全链路一键脚本                      ║${NC}"
echo -e "${BLUE}║    Backend → PX4 SITL → MAVROS → PID 悬停 + 阶跃阵风     ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""
info "项目路径:  $PROJECT_DIR"
info "PX4 路径:  $PX4_DIR"
if $NO_WIND; then
    info "阶跃风:    关闭 (纯悬停)"
else
    info "阶跃风:    ${WIND_SPEED} m/s +Y (正北), ${WIND_DURATION}s"
fi
info "悬停热身:  ${HOVER_WARMUP}s"
echo ""

# ══════════════════════════════════════════════════════════════════════════════
# Step 1: 环境准备
# ══════════════════════════════════════════════════════════════════════════════
step "Step 1/8  环境准备"

# Source ROS
if [[ -f "$ROS_SETUP" ]]; then
    source "$ROS_SETUP"
else
    err "找不到 ROS 环境: $ROS_SETUP"; exit 1
fi
# Source catkin
if [[ -f "$CATKIN_SETUP" ]]; then
    source "$CATKIN_SETUP"
else
    err "找不到 Catkin 环境: $CATKIN_SETUP"; exit 1
fi

export DISPLAY="${DISPLAY:-:0}"
info "ROS_ROOT=$ROS_ROOT"
info "DISPLAY=$DISPLAY"

# 检查 roscore
if ! rostopic list &>/dev/null; then
    err "roscore 未运行! 请先执行 roscore"; exit 1
fi
info "roscore 已运行 ✓"

# 修复 use_sim_time
rosparam set /use_sim_time false 2>/dev/null || true
SIM_VAL=$(rosparam get /use_sim_time 2>/dev/null || echo "unset")
info "use_sim_time=$SIM_VAL"

# 检查必要文件
for f in "$QUAD_SIM_DIR/scripts/plant_6dof.py" \
         "$QUAD_SIM_DIR/scripts/backend_main.py" \
         "$QUAD_SIM_DIR/scripts/pid_baseline.py" \
         "$QUAD_SIM_DIR/config/sim_default.yaml"; do
    if [[ ! -f "$f" ]]; then
        err "缺少文件: $f"; exit 1
    fi
done

if [[ ! -d "$PX4_DIR" ]]; then
    err "PX4 目录不存在: $PX4_DIR"; exit 1
fi

# 预清理残留
pkill -f "backend_main.py" 2>/dev/null || true
pkill -f "mavros_node" 2>/dev/null || true
pkill -f "pid_baseline" 2>/dev/null || true
sleep 1

info "环境准备完成 ✓"

# ══════════════════════════════════════════════════════════════════════════════
# Step 2: 启动 HIL Backend + RViz
# ══════════════════════════════════════════════════════════════════════════════
step "Step 2/8  启动 HIL Backend (plant_6dof) + RViz"

roslaunch quad_sim hil_backend.launch rviz:=true &
BACKEND_PID=$!
info "Backend PID=$BACKEND_PID"

# 等 4560 端口
for i in $(seq 1 20); do
    if lsof -i :4560 2>/dev/null | grep -q LISTEN; then
        info "Backend 就绪 (port 4560) ✓"; break
    fi
    sleep 1
done
sleep 2

# ══════════════════════════════════════════════════════════════════════════════
# Step 3: 启动 PX4 SITL
# ══════════════════════════════════════════════════════════════════════════════
step "Step 3/8  启动 PX4 SITL (守护进程)"

cd "$PX4_DIR"
PX4_SIM_HOST_ADDR=127.0.0.1 NO_PXH=1 no_sim=1 make px4_sitl none_iris &
PX4_PID=$!
info "PX4 PID=$PX4_PID"

# 等 PX4 启动完成 (进程存活 + 有 MAVLink 输出)
info "等待 PX4 启动 (约 12-15s)..."
for i in $(seq 1 25); do
    sleep 1
    if ! kill -0 "$PX4_PID" 2>/dev/null; then
        err "PX4 进程意外退出"; exit 1
    fi
    # 检查是否有 px4 子进程在跑 (说明 make 已完成编译, PX4 已启动)
    if pgrep -f "^.*px4.*none_iris" >/dev/null 2>&1; then
        info "PX4 SITL 启动完成 ✓ (${i}s)"; break
    fi
done
sleep 2

# ══════════════════════════════════════════════════════════════════════════════
# Step 4: 启动 MAVROS
# ══════════════════════════════════════════════════════════════════════════════
step "Step 4/8  启动 MAVROS"

rosrun mavros mavros_node _fcu_url:=udp://:14540@127.0.0.1:14580 &
MAVROS_PID=$!
info "MAVROS PID=$MAVROS_PID"

# 等连接
for i in $(seq 1 30); do
    sleep 1
    if get_state | grep -q "connected: True"; then
        info "MAVROS 已连接 PX4 ✓ (${i}s)"; break
    fi
done
sleep 2

# ══════════════════════════════════════════════════════════════════════════════
# Step 5: 设置 PX4 参数
# ══════════════════════════════════════════════════════════════════════════════
step "Step 5/8  设置 PX4 出厂默认 PID 参数"

rosservice call /mavros/param/pull '{}' >/dev/null 2>&1
sleep 2

declare -A PARAMS=(
    ["SYS_HITL"]="int:1"
    ["MPC_XY_P"]="real:0.95"
    ["MPC_XY_VEL_P_ACC"]="real:1.8"
    ["MPC_XY_VEL_I_ACC"]="real:0.4"
    ["MPC_XY_VEL_D_ACC"]="real:0.2"
    ["MPC_Z_VEL_P_ACC"]="real:4.0"
    ["MPC_Z_VEL_I_ACC"]="real:2.0"
    ["MPC_TILTMAX_AIR"]="real:45.0"
)

for param_id in "${!PARAMS[@]}"; do
    val_spec="${PARAMS[$param_id]}"
    type="${val_spec%%:*}"
    val="${val_spec##*:}"
    if [[ "$type" == "int" ]]; then
        result=$(rosservice call /mavros/param/set "{param_id: \"$param_id\", value: {integer: $val}}" 2>&1)
    else
        result=$(rosservice call /mavros/param/set "{param_id: \"$param_id\", value: {real: $val}}" 2>&1)
    fi
    if echo "$result" | grep -q "success: True"; then
        info "  $param_id = $val ✓"
    else
        warn "  $param_id 设置失败"
    fi
done

info "PX4 参数设置完成 ✓"

# ══════════════════════════════════════════════════════════════════════════════
# Step 6: 启动 PID 悬停控制
# ══════════════════════════════════════════════════════════════════════════════
step "Step 6/8  启动 PID 悬停控制 (目标: 0, 0, 2.5)"

cd "$CSV_DIR"
PYTHONUNBUFFERED=1 python3 -u pid_baseline.py &
PIDCTL_PID=$!
info "PID 控制 PID=$PIDCTL_PID"

# 等起飞
info "等待解锁起飞..."
for i in $(seq 1 30); do
    sleep 1
    if get_state | grep -q "armed: True"; then
        info "已解锁起飞 ✓ (${i}s)"; break
    fi
done

# ══════════════════════════════════════════════════════════════════════════════
# Step 7: 悬停热身
# ══════════════════════════════════════════════════════════════════════════════
step "Step 7/8  悬停滞稳 (${HOVER_WARMUP}s)"

for ((t=5; t<=HOVER_WARMUP; t+=5)); do
    sleep 5
    read -r px py pz <<< "$(get_pose)"
    info "  悬停 t=${t}s | pos=(${px:-?}, ${py:-?}, ${pz:-?})"
done

read -r fx fy fz <<< "$(get_pose)"
info "悬停滞稳完成 ✓ | 最终=(${fx:-?}, ${fy:-?}, ${fz:-?})"

# ══════════════════════════════════════════════════════════════════════════════
# Step 8: 阶跃阵风 / 纯悬停
# ══════════════════════════════════════════════════════════════════════════════
if $NO_WIND; then
    step "Step 8/8  纯悬停 — 按 Ctrl+C 停止"

    info "无人机在 (0,0,2.5) 悬停中, RViz 可观察六自由度姿态"
    info "按 Ctrl+C 停止所有进程"
    while true; do sleep 5; done
else
    step "Step 8/8  阶跃阵风: ${WIND_SPEED} m/s +Y (正北)"

    echo ""
    info "风场: /wind_field/velocity → plant_6dof (转子层+机身层)"
    info "阶段: 15s 基线 → ${WIND_DURATION}s 风 → ${RECOVERY_DURATION}s 恢复"
    echo ""

    # 内嵌 Python 阶跃风注入 (无外部依赖)
    python3 -u -c "
import rospy
from geometry_msgs.msg import Vector3Stamped

rospy.init_node('step_wind', anonymous=True)
pub = rospy.Publisher('/wind_field/velocity', Vector3Stamped, queue_size=10)
rate = rospy.Rate(10)

W = $WIND_SPEED
dur = $WIND_DURATION
rec = $RECOVERY_DURATION

# 基线无风
print('[WIND] 基线 15s (无风)')
for _ in range(150):
    msg = Vector3Stamped(); msg.header.stamp = rospy.Time.now()
    pub.publish(msg); rate.sleep()

# 阶跃风
print('[WIND] 🌬️  风起! {:.1f} m/s +Y'.format(W))
for _ in range(dur * 10):
    msg = Vector3Stamped(); msg.header.stamp = rospy.Time.now()
    msg.vector.y = W
    pub.publish(msg); rate.sleep()

# 恢复
print('[WIND] ☀️  风停 — 观察恢复')
for _ in range(rec * 10):
    msg = Vector3Stamped(); msg.header.stamp = rospy.Time.now()
    pub.publish(msg); rate.sleep()

print('[WIND] ✅ 测试完成')
" &
    WIND_PID=$!

    # 实时监控位置
    START_TIME=$(date +%s)
    PHASE1_END=$((15))
    PHASE2_END=$((15 + WIND_DURATION))

    while kill -0 "$WIND_PID" 2>/dev/null; do
        sleep 3
        ELAPSED=$(($(date +%s) - START_TIME))
        read -r px py pz <<< "$(get_pose)"

        if [[ $ELAPSED -lt $PHASE1_END ]]; then
            echo -e "  ${CYAN}[基线]${NC}  +${ELAPSED}s | (${px:-?}, ${py:-?}, ${pz:-?})"
        elif [[ $ELAPSED -lt $PHASE2_END ]]; then
            WT=$((ELAPSED - PHASE1_END))
            echo -e "  ${YELLOW}[🌬️  ${WIND_SPEED}m/s +Y]${NC} +${ELAPSED}s 风+${WT}s | (${px:-?}, ${py:-?}, ${pz:-?})"
        else
            RT=$((ELAPSED - PHASE2_END))
            echo -e "  ${GREEN}[☀️ 恢复]${NC} +${ELAPSED}s 风停+${RT}s | (${px:-?}, ${py:-?}, ${pz:-?})"
        fi
    done
    wait "$WIND_PID" 2>/dev/null || true

    echo ""
    read -r ex ey ez <<< "$(get_pose)"
    info "最终位置: ($ex, $ey, $ez)"
    info "阶跃阵风测试完成 ✓"
fi

# ══════════════════════════════════════════════════════════════════════════════
# 收尾
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo -e "${BLUE}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║           全链路测试完成!                                 ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""

LATEST_CSV=$(ls -t "$CSV_DIR"/pid_log_*.csv 2>/dev/null | head -1)
if [[ -n "$LATEST_CSV" ]]; then
    info "CSV 日志: $LATEST_CSV"
    CSV_LINES=$(wc -l < "$LATEST_CSV")
    info "数据行数: $CSV_LINES"
fi
info "5s 后自动清理, 或按 Ctrl+C 立即停止"
sleep 5
