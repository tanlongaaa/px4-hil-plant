#!/bin/bash
# ==============================================================================
# start_hil_full.sh — 六自由度无人机 HIL 全链路一键脚本
# ==============================================================================
# 用法:
#   bash start_hil_full.sh                     # 默认: 悬停 20s → 5m/s 阶跃风 30s → 恢复 45s
#   bash start_hil_full.sh --no-wind           # 仅悬停, 不打风
#   bash start_hil_full.sh --wind 7.0          # 自定义阶跃风速 (m/s, ENU +Y 即正北)
#   bash start_hil_full.sh --turbulent         # 连续湍流风场 (Dryden + 幂律均风 + 阵风) + RViz风场可视化
#   bash start_hil_full.sh --turbulent --wind-intensity 12.0 --wind-seed 123
#   bash start_hil_full.sh --px4-path /path/to/PX4-Autopilot
#
# 环境变量:
#   PX4_DIR   — PX4-Autopilot 源码目录 (优先级: 命令行 > 环境变量 > 自动探测)
#   CATKIN_WS — catkin 工作空间 (默认 ~/catkin_ws)
#
# 风场模式:
#   默认 (--wind N):    阶跃风, 15s基线 → N m/s 持续 → 恢复
#   --turbulent:       连续湍流风 (wind_field.py), Dryden湍流+幂律均风+1-cos阵风
#                       全程通过 RViz 可视化风场箭头+网格+历史轨迹
#
# 架构:
#   wind_field ──/wind_field/velocity──▶ Backend(plant_6dof) ──TCP──▶ PX4 SITL ──UDP──▶ MAVROS ──ROS──▶ pid_baseline
#                                             │
#                                        wind_visualizer ──/wind_viz/markers──▶ RViz
#
# 依赖:
#   - ROS Noetic + catkin_ws
#   - PX4-Autopilot (v1.13+), 通过 --px4-path 或环境变量 PX4_DIR 指定
#   - quad_sim (plant_6dof + backend_main + sensor_models)
# ==============================================================================

set -eo pipefail

# 某些 ROS 环境脚本需要 ROS_DISTRO 变量
ROS_DISTRO="${ROS_DISTRO:-noetic}"

# ── 默认参数 ─────────────────────────────────────────────────────────────────
WIND_SPEED=5.0              # 阶跃风速 (m/s, ENU +Y)
TURBULENT=false             # 连续湍流风场模式
WIND_INTENSITY=8.0          # 湍流模式: 10m 参考风速 (m/s)
WIND_SEED=42                # 湍流模式: 随机种子 (可复现)
WIND_SIGMA_SCALE=0.25       # 湍流强度缩放 (0.25=轻度验证, 1.0=极端论文工况)
WIND_GUST_MAX=2.0           # 垂向阵风峰值 (m/s, 极端可设 8.0)
WIND_EXTREME=false          # 极端档: sigma_scale=1.0 + gust=8.0
HOVER_WARMUP=20             # 悬停稳定等待时间 (秒)
WIND_DURATION=30            # 阶跃风持续时间 (秒)
RECOVERY_DURATION=45        # 风停后恢复观察时间 (秒)
NO_WIND=false
PX4_DIR_USER=""              # 用户通过 --px4-path 指定的路径
CATKIN_WS_USER=""            # 用户通过 --catkin-ws 指定的路径

# ── 解析命令行 ───────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-wind) NO_WIND=true; shift ;;
        --wind) WIND_SPEED="$2"; shift 2 ;;
        --turbulent) TURBULENT=true; shift ;;
        --wind-intensity) WIND_INTENSITY="$2"; shift 2 ;;
        --wind-seed) WIND_SEED="$2"; shift 2 ;;
        --wind-sigma) WIND_SIGMA_SCALE="$2"; shift 2 ;;
        --wind-gust) WIND_GUST_MAX="$2"; shift 2 ;;
        --extreme) WIND_EXTREME=true; shift ;;
        --px4-path) PX4_DIR_USER="$2"; shift 2 ;;
        --catkin-ws) CATKIN_WS_USER="$2"; shift 2 ;;
        -h|--help)
            echo "用法: bash start_hil_full.sh [选项]"
            echo ""
            echo "风场选项:"
            echo "  --no-wind              仅悬停, 不注入任何风"
            echo "  --wind SPEED           阶跃风 (默认 5.0 m/s, +Y 正北)"
            echo "  --turbulent            连续湍流风场 (Dryden + 幂律均风 + 1-cos 阵风)"
            echo "  --wind-intensity SPEED 湍流模式参考风速 (默认 8.0 m/s @10m)"
            echo "  --wind-seed SEED       湍流模式随机种子 (默认 42, 可复现)"
            echo "  --wind-sigma SCALE     湍流强度缩放 (默认 0.25=轻度; 1.0=极端)"
            echo "  --wind-gust MAX        垂向阵风峰值 m/s (默认 2.0; 极端 8.0)"
            echo "  --extreme              极端档快捷: sigma=1.0 + gust=8.0 (论文工况)"
            echo ""
            echo "路径选项:"
            echo "  --px4-path PATH        PX4-Autopilot 源码路径"
            echo "  --catkin-ws PATH       Catkin 工作空间 (默认 ~/catkin_ws)"
            echo "  -h, --help             显示此帮助"
            echo ""
            echo "环境变量:"
            echo "  PX4_DIR                PX4 源码路径 (优先级: --px4-path > \$PX4_DIR > 自动探测)"
            echo ""
            echo "PX4 自动探测顺序:"
            echo "  1. ~/PX4-Autopilot"
            echo "  2. ~/px4/PX4-Autopilot"
            echo "  3. ~/Desktop/px4rl/PX4-Autopilot"
            echo "  4. ~/src/PX4-Autopilot"
            exit 0
            ;;
        *) echo "未知参数: $1"; echo "用法: bash start_hil_full.sh [-h] [--no-wind] [--wind SPEED] [--turbulent]"; exit 1 ;;
    esac
done

# 冲突检查
if $TURBULENT && [[ "$WIND_SPEED" != "5.0" ]]; then
    warn "--turbulent 与 --wind 冲突, 使用 --turbulent --wind-intensity $WIND_INTENSITY"
fi
if $TURBULENT && $NO_WIND; then
    warn "--turbulent 与 --no-wind 冲突, 使用 --turbulent"
    NO_WIND=false
fi

# 极端档覆盖
WIND_EXTREME_FLAG=""
if $WIND_EXTREME; then
    WIND_SIGMA_SCALE=1.0
    WIND_GUST_MAX=8.0
    WIND_EXTREME_FLAG="--extreme"
fi

# ── 路径 ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
QUAD_SIM_DIR="$PROJECT_DIR/quad_sim"
ROS_SETUP="/opt/ros/noetic/setup.bash"

# Catkin workspace
if [[ -n "$CATKIN_WS_USER" ]]; then
    CATKIN_SETUP="$CATKIN_WS_USER/devel/setup.bash"
else
    CATKIN_SETUP="$HOME/catkin_ws/devel/setup.bash"
fi

# PX4 路径解析 (优先级: --px4-path > $PX4_DIR > 自动探测)
PX4_SOURCE=""   # 记录路径来源
find_px4_dir() {
    # 1. 命令行参数
    if [[ -n "$PX4_DIR_USER" ]]; then
        PX4_SOURCE="--px4-path"
        echo "$PX4_DIR_USER"
        return
    fi
    # 2. 环境变量
    if [[ -n "${PX4_DIR:-}" ]]; then
        PX4_SOURCE="\$PX4_DIR"
        echo "$PX4_DIR"
        return
    fi
    # 3. 自动探测常见路径
    local candidates=(
        "$HOME/PX4-Autopilot"
        "$HOME/px4/PX4-Autopilot"
        "$HOME/Desktop/px4rl/PX4-Autopilot"
        "$HOME/src/PX4-Autopilot"
    )
    for dir in "${candidates[@]}"; do
        if [[ -d "$dir" ]] && [[ -f "$dir/Makefile" ]]; then
            PX4_SOURCE="自动探测"
            echo "$dir"
            return
        fi
    done
    echo ""
}

PX4_DIR=$(find_px4_dir)
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
BACKEND_PID=""; PX4_PID=""; MAVROS_PID=""; PIDCTL_PID=""; WIND_PID=""; VIZ_PID=""

# ── 清理函数 ─────────────────────────────────────────────────────────────────
cleanup() {
    echo ""
    warn "正在清理所有进程..."
    for pid in $VIZ_PID $WIND_PID $PIDCTL_PID $MAVROS_PID $PX4_PID $BACKEND_PID; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    pkill -f "wind_visualizer" 2>/dev/null || true
    pkill -f "wind_field" 2>/dev/null || true
    pkill -f "wind_field_node" 2>/dev/null || true
    pkill -f "step_wind_hil" 2>/dev/null || true
    pkill -f "pid_baseline" 2>/dev/null || true
    pkill -f "mavros_node" 2>/dev/null || true
    pkill -f "backend_main" 2>/dev/null || true
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
echo -e "${BLUE}║    Backend → PX4 SITL → MAVROS → PID + 风场 + RViz       ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""
info "项目路径:  $PROJECT_DIR"
info "PX4 路径:  $PX4_DIR  (来源: $PX4_SOURCE)"
if $TURBULENT; then
    info "风场模式:  连续湍流 (Dryden ${WIND_INTENSITY} m/s + 幂律均风 + 1-cos 阵风)"
    info "随机种子:  ${WIND_SEED}"
elif $NO_WIND; then
    info "风场模式:  关闭 (纯悬停)"
else
    info "风场模式:  阶跃风 ${WIND_SPEED} m/s +Y (正北), ${WIND_DURATION}s"
fi
info "悬停热身:  ${HOVER_WARMUP}s"
info "RViz 可视化: ✅ (无人机模型 + 风场箭头 + 网格 + 历史轨迹)"
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

if [[ ! -d "$PX4_DIR" ]] || [[ -z "$PX4_DIR" ]]; then
    err "找不到 PX4-Autopilot!"
    err ""
    err "请通过以下方式之一指定 PX4 源码路径:"
    err "  1. 命令行:  --px4-path /path/to/PX4-Autopilot"
    err "  2. 环境变量: export PX4_DIR=/path/to/PX4-Autopilot"
    err "  3. 将 PX4 放在以下自动探测路径之一:"
    err "     ~/PX4-Autopilot"
    err "     ~/px4/PX4-Autopilot"
    err "     ~/Desktop/px4rl/PX4-Autopilot"
    err "     ~/src/PX4-Autopilot"
    exit 1
fi

# 预清理残留
pkill -f "backend_main.py" 2>/dev/null || true
pkill -f "mavros_node" 2>/dev/null || true
pkill -f "pid_baseline" 2>/dev/null || true
sleep 1

info "环境准备完成 ✓"

# ══════════════════════════════════════════════════════════════════════════════
# Step 2: 启动 HIL Backend + 风场 + RViz
# ══════════════════════════════════════════════════════════════════════════════
if $TURBULENT; then
    step "Step 2/8  启动 HIL Backend + RViz (风场将在悬停稳定后启动)"

    # 注意: 湍流模式下, wind_field/wind_visualizer 在 Step 8 才启动
    # 否则风在无人机起飞前就开始吹, 导致 PX4 lockdown
    roslaunch quad_sim hil_backend.launch rviz:=true &
    BACKEND_PID=$!
    info "Backend + RViz PID=$BACKEND_PID"
else
    step "Step 2/8  启动 HIL Backend (plant_6dof) + RViz"

    roslaunch quad_sim hil_backend.launch rviz:=true &
    BACKEND_PID=$!
    info "Backend PID=$BACKEND_PID"
fi

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
    ["MPC_XY_VEL_I_ACC"]="real:1.2"
    ["MPC_XY_VEL_D_ACC"]="real:0.2"
    ["MPC_Z_VEL_P_ACC"]="real:4.0"
    ["MPC_Z_VEL_I_ACC"]="real:2.0"
    ["MPC_TILTMAX_AIR"]="real:45.0"
)
# 2026-06-28 调参: MPC_XY_VEL_I_ACC 出厂 0.4 → 1.2 (抑风核心)
#   恒风 5m/s: exy_rms 0.545 → 0.079m;  湍流: 0.192 → 0.079m;  无风悬停 9mm
#   选 1.2 而非 1.6 (几同精度但更大稳定裕度/抗积分饱和)

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
# Step 8: 风场测试 / 纯悬停
# ══════════════════════════════════════════════════════════════════════════════
if $NO_WIND; then
    step "Step 8/8  纯悬停 — 按 Ctrl+C 停止"

    info "无人机在 (0,0,2.5) 悬停中"
    info "RViz: 无人机模型 (RobotModel) + Odometry + TF 可见"
    info "按 Ctrl+C 停止所有进程"
    while true; do sleep 5; done

elif $TURBULENT; then
    step "Step 8/8  启动风场 + 风场可视化 + 连续湍流测试"

    echo ""
    info "🌪️  风场配置:"
    info "  风源:       wind_field.py (连续运行, Dryden + 幂律均风 + 1-cos 阵风)"
    info "  参考风速:   ${WIND_INTENSITY} m/s @10m (幂律剖面 α=0.35)"
    info "  风向:       45° (东北风 → 吹向西南)"
    info "  湍流:       Dryden σ_u,v=4.0×${WIND_SIGMA_SCALE}, σ_w=2.5×${WIND_SIGMA_SCALE} m/s"
    info "  阵风:       1-cos 垂向 ±${WIND_GUST_MAX} m/s, 间隔 ~20s"
    info "  注入路径:   /wind_field/velocity → plant_6dof (转子侧向阻力 + 叶片挥舞)"
    info "  RViz 可视化: 风矢量箭头 + 5×5 网格 + 历史彩色轨迹 + 风速标签"
    echo ""

    info "🚀 启动 wind_field.py (无风基线 10s 后开始湍流)..."

    # 启动 wind_field (no-wrench, 只发风场数据)
    # 需要 remap odom 来自 backend 发布的 /sim/odom
    rosrun offboard_test wind_field.py \
        --no-wrench --rate 20 --u-ref ${WIND_INTENSITY} --seed ${WIND_SEED} \
        --sigma-scale ${WIND_SIGMA_SCALE} --gust-w-max ${WIND_GUST_MAX} \
        ${WIND_EXTREME_FLAG} \
        /mavros/local_position/odom:=/sim/odom &
    WIND_PID=$!
    sleep 2

    # 启动 wind_visualizer (RViz 风场可视化)
    rosrun quad_sim wind_visualizer.py \
        --rate 10 --grid-size 5 --grid-spacing 2.0 \
        --u-ref ${WIND_INTENSITY} --wind-dir 45 &
    VIZ_PID=$!
    sleep 2

    info "🌪️  连续湍流测试中 (wind_field.py PID=$WIND_PID, viz PID=$VIZ_PID)"
    info "按 Ctrl+C 停止所有进程"
    echo ""

    # 实时监控位置 + 风况 (每 5s 打印一次)
    while true; do
        sleep 5
        read -r px py pz <<< "$(get_pose)"
        # 取最近的风速值
        wind_now=$(rostopic echo /wind_field/velocity -n1 2>/dev/null \
            | grep -E '^[[:space:]]+[xyz]:' \
            | awk '{printf "%.1f", sqrt($2^2+$4^2+$6^2)}' 2>/dev/null || echo "?")
        echo -e "  ${YELLOW}[🌪️  连续湍流 ${WIND_INTENSITY}m/s]${NC} |w|=${wind_now}  pos=(${px:-?}, ${py:-?}, ${pz:-?})"
    done

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
