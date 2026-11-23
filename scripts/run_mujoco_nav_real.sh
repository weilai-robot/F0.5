#!/bin/bash
# ============================================================
# real2sim 导航一键启动 (real_lab_env 场景) — run_mujoco_nav_real.sh
# 基于 run_mujoco_nav.sh 的 P2 变体 (2026-11-23), 与真机同世界观:
#   - 场景: xyber_x1_nav_real.xml → real_lab_env.xml (PCD→体素box 771 box)
#   - 出生点: (0.14, 0.51) bag 常态站位 (sim_x1_nav_real.yaml init_pos)
#   - nav2 2D 图: robot_lab_20261030.pgm (真机同图, world xy ≡ pgm xy)
#   - ICP 图: mujoco_real_lab.pcd (须先 CAPTURE=1 从 env 自采, 见下)
#
# 两种模式:
#   ./run_mujoco_nav_real.sh            # 回放模式: 全栈 + 真机同图 (ICP 图须已就位)
#   CAPTURE=1 ./run_mujoco_nav_real.sh  # 采图模式: 只起 sim+lidar+fastlio(存图)+odom_bridge
#                                       #   驱动机器人走覆盖路径后 Ctrl+C 停 fastlio,
#                                       #   mujoco_real_lab.pcd 落在 fastlio 窗口 cwd
#
# 可覆盖变量: SESSION_NAME / DISPLAY (81 服务器: DISPLAY=:99) / ROS_DOMAIN_ID
# AI 并发纪律: 勿占人类默认域 0 —— 共享服务器先 export ROS_DOMAIN_ID=<navsim 分配的 41-50>
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
NAV_DIR="${ROOT_DIR}/navigation"
BUILD_DIR="${ROOT_DIR}/build"
SESSION_NAME="${SESSION_NAME:-f1_sim_nav_real}"

AIMRT_CFG="x1_cfg_sim_nav_real.yaml"
MODEL_NAME="xyber_x1_nav_real.xml"
NAV2_MAP_YAML="robot_lab_20261030.yaml"          # 与真机同一张 GIMP 2D 图
ICP_MAP_PCD="mujoco_real_lab.pcd"                # sim 自采 ICP 图 (open3d_loc/maps/)
FASTLIO_CONFIG="sim_module_mid360_custom.yaml"

CAPTURE="${CAPTURE:-0}"
if [ "${CAPTURE}" = "1" ]; then
  # 采图模式: fastlio 开 pcd_save_en, 停止时整图写 map_file_path
  FASTLIO_CONFIG="sim_module_mid360_capture.yaml"
fi

# --- AI 并发隔离检查（勿占人类默认域 0 / 物理实显） ---
if [ -z "${ROS_DOMAIN_ID:-}" ]; then
    echo -e "${YELLOW}[WARN] ROS_DOMAIN_ID 未设置 —— 将占用默认域 0（人类世界）。${NC}"
    echo -e "${YELLOW}  共享服务器请先: export ROS_DOMAIN_ID=<navsim 分配的 41-50>${NC}"
fi
if [ -z "${DISPLAY:-}" ]; then
    echo -e "${RED}[ERROR] DISPLAY 未设置 —— MuJoCo 渲染窗口无处显示.${NC}"
    echo -e "  81 服务器用虚拟显示（勿占 :0 实显）:"
    echo -e "    pgrep Xvfb >/dev/null 2>&1 || Xvfb :99 -screen 0 1280x1024x24 -ac &"
    echo -e "    export DISPLAY=:99"
    exit 1
fi

# --- 颜色 ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

SESSION_CREATED=0
launch_cleanup() {
  local rc=$?
  if [ "${SESSION_CREATED}" = 1 ] && [ "${rc}" -ne 0 ]; then
    echo ""
    echo -e "${RED}[FAIL] 启动中断 (rc=${rc}).${NC}" >&2
    echo -e "${YELLOW}  tmux session ${SESSION_NAME} 可能仍在运行，可先排查现场:${NC}" >&2
    echo -e "${YELLOW}    tmux attach -t ${SESSION_NAME}${NC}" >&2
    echo -e "${YELLOW}  排查完后清理:${NC}" >&2
    echo -e "${YELLOW}    tmux kill-session -t ${SESSION_NAME}${NC}" >&2
  fi
}
trap launch_cleanup EXIT

echo -e "${GREEN}[run_sim_nav_real] 启动 real2sim 导航链路 (CAPTURE=${CAPTURE})...${NC}"

# ── 自动检测 ROS2 setup.bash 路径 ──────────────────────────
if [ -z "${ROS_SETUP_BASH}" ]; then
    if [ -f "${CONDA_PREFIX:-}/ros_humble/setup.bash" ]; then
        ROS_SETUP_BASH="${CONDA_PREFIX}/ros_humble/setup.bash"
    elif [ -f "${CONDA_PREFIX:-}/setup.bash" ]; then
        ROS_SETUP_BASH="${CONDA_PREFIX}/setup.bash"
    elif [ -f /opt/ros/humble/setup.bash ]; then
        ROS_SETUP_BASH="/opt/ros/humble/setup.bash"
    elif [ -n "${AMENT_PREFIX_PATH:-}" ]; then
        ROS_BASE=$(echo "${AMENT_PREFIX_PATH}" | tr ':' '\n' | tail -1)
        if [ -f "${ROS_BASE}/setup.bash" ]; then
            ROS_SETUP_BASH="${ROS_BASE}/setup.bash"
        fi
    fi
fi

if [ -z "${ROS_SETUP_BASH}" ] || [ ! -f "${ROS_SETUP_BASH}" ]; then
    echo -e "${RED}[ERROR] 未找到 ROS2 setup.bash${NC}"
    echo -e "  尝试过的路径:"
    echo -e "  /opt/ros/humble/setup.bash  \${CONDA_PREFIX}/setup.bash  \${CONDA_PREFIX}/ros_humble/setup.bash"
    echo -e "  请手动设置: export ROS_SETUP_BASH=/path/to/setup.bash"
    exit 1
fi

echo -e "${GREEN}  ROS2: ${ROS_SETUP_BASH}${NC}"

# --- source ---
source "${ROS_SETUP_BASH}"
source "${NAV_DIR}/install/setup.bash"

ros_pkg_exists() {
    ros2 pkg prefix "$1" >/dev/null 2>&1
}

print_missing_ros_pkg_help() {
    local pkg="$1"
    echo -e "${RED}[ERROR] 缺少 ROS2 package: ${pkg}${NC}" >&2
    echo -e "  当前 ROS2: ${ROS_SETUP_BASH}"
    echo -e "  如果使用 apt ROS Humble:"
    echo -e "    sudo apt install ros-humble-nav2-bringup ros-humble-octomap-server"
    echo -e "  如果使用 conda/robostack ROS Humble:"
    echo -e "    mamba install -c robostack-staging -c conda-forge ros-humble-nav2-bringup ros-humble-octomap-server"
}

for pkg in fast_lio humanoid_sim open3d_loc; do
    if ! ros_pkg_exists "${pkg}"; then
        print_missing_ros_pkg_help "${pkg}"
        echo -e "  请先运行: ./build_nav.sh"
        exit 1
    fi
done

# --- CustomMsg 依赖检查 (output_type:=custom 需要 livox_ros_driver2) ---
if ! python3 -c "from livox_ros_driver2.msg import CustomMsg, CustomPoint" >/dev/null 2>&1; then
    echo -e "${RED}[ERROR] 未检测到 livox_ros_driver2 Python 消息: livox_ros_driver2.msg.CustomMsg${NC}"
    echo -e "  当前配置会启动 output_type:=custom，因此必须可导入 livox_ros_driver2。"
    echo -e "  请重新构建 navigation（不要跳过 livox）:"
    echo -e "    ./build_nav.sh"
    exit 1
fi

# --- FastLIO2 配置文件检查 (必须安装到 share/fast_lio/config) ---
FAST_LIO_PREFIX="$(ros2 pkg prefix fast_lio)"
FAST_LIO_CFG="${FAST_LIO_PREFIX}/share/fast_lio/config/${FASTLIO_CONFIG}"
if [ ! -f "${FAST_LIO_CFG}" ]; then
    echo -e "${RED}[ERROR] FastLIO2 配置未安装: ${FAST_LIO_CFG}${NC}"
    echo -e "  请重新构建 fast_lio（确保 config 被 install）:"
    echo -e "    ./build_nav.sh --packages-select fast_lio"
    exit 1
fi

HUMANOID_SIM_PREFIX="$(ros2 pkg prefix humanoid_sim)"
ODOM_BRIDGE_EXE="${HUMANOID_SIM_PREFIX}/lib/humanoid_sim/odom_bridge.py"
if [ ! -x "${ODOM_BRIDGE_EXE}" ]; then
    echo -e "${RED}[ERROR] odom_bridge.py 未安装或不可执行: ${ODOM_BRIDGE_EXE}${NC}"
    echo -e "  请重新构建 navigation:"
    echo -e "    ./build_nav.sh"
    exit 1
fi

if [ "${CAPTURE}" != "1" ]; then
    if ! ros_pkg_exists nav2_bringup; then
        print_missing_ros_pkg_help "nav2_bringup"
        exit 1
    fi
    # 回放模式硬门槛: ICP 图必须已自采到位
    ICP_MAP_PATH="${NAV_DIR}/localization/open3d_loc/maps/${ICP_MAP_PCD}"
    if [ ! -f "${ICP_MAP_PATH}" ]; then
        echo -e "${RED}[ERROR] ICP 图未就位: ${ICP_MAP_PATH}${NC}"
        echo -e "  先采图:  CAPTURE=1 $0   (驱动机器人走覆盖路径后停 fastlio)"
        echo -e "  然后:    cp <fastlio窗口cwd>/mujoco_real_lab.pcd ${ICP_MAP_PATH} && ./build_nav.sh --packages-select open3d_loc"
        exit 1
    fi
fi

# --- 检查构建产物 ---
if [ ! -f "${BUILD_DIR}/aimrt_main" ]; then
    echo -e "${RED}[ERROR] motion_control 未构建: ${BUILD_DIR}/aimrt_main 不存在${NC}"
    echo -e "  请先运行: ./build.sh"
    exit 1
fi
if [ ! -f "${NAV_DIR}/install/setup.bash" ]; then
    echo -e "${RED}[ERROR] navigation workspace 未构建${NC}"
    echo -e "  请先运行: ./build_nav.sh"
    exit 1
fi

# --- 检查导航场景模型 ---
MODEL_PATH="${BUILD_DIR}/cfg/sim_module/model/mjcf/${MODEL_NAME}"
if [ ! -f "$MODEL_PATH" ]; then
    echo -e "${RED}[ERROR] 未找到场景模型: $MODEL_PATH${NC}"
    echo -e "  请重新运行 ./build.sh (需要 sim_x1_nav_real.yaml + xyber_x1_nav_real.xml + real_lab_env.xml)"
    exit 1
fi

# --- 检查是否已有同名 session ---
if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    echo -e "${YELLOW}[WARN] 已存在 tmux session: ${SESSION_NAME}${NC}"
    echo -e "  关闭: tmux kill-session -t ${SESSION_NAME}"
    exit 1
fi

# --- 每个窗口的 source 前缀 ---
TMUX_SOURCE="source /home/robot/miniconda/etc/profile.d/conda.sh && conda deactivate >/dev/null 2>&1 || true && conda activate nav && source ${ROS_SETUP_BASH} && source ${NAV_DIR}/install/setup.bash"

# 域前缀（ROS_DOMAIN_ID 设置时注入每个 tmux 窗口，勿串人类默认域 0；navsim 补丁锚点依赖上一行 TMUX_SOURCE 原样）
DOMAIN_EXPORT=""
if [ -n "${ROS_DOMAIN_ID:-}" ]; then
    DOMAIN_EXPORT="export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && "
fi

# --- [窗口 0] aimrt_main (运动控制 + 物理仿真) ---
tmux new-session -d -s "${SESSION_NAME}" -n "aimrt"
SESSION_CREATED=1
echo -e "${GREEN}  [0] aimrt_main (${AIMRT_CFG}, 出生点 0.14,0.51)${NC}"
tmux send-keys -t "${SESSION_NAME}:0" \
    "${DOMAIN_EXPORT}cd ${BUILD_DIR} && source install/share/ros2_plugin_proto/local_setup.bash 2>/dev/null; ./aimrt_main --cfg_file_path=./cfg/${AIMRT_CFG}" Enter

echo -e "${YELLOW}  等待 aimrt_main + MuJoCo 渲染窗口启动 (5s)...${NC}"
sleep 5

# --- [窗口 1] MuJoCo LiDAR Bridge ---
tmux new-window -t "${SESSION_NAME}" -n "lidar_bridge"
echo -e "${GREEN}  [1] MuJoCo LiDAR Bridge${NC}"
tmux send-keys -t "${SESSION_NAME}:1" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:1" \
    "export MUJOCO_LIDAR_SRC=${NAV_DIR}/sim/MuJoCo-LiDAR/src" Enter
tmux send-keys -t "${SESSION_NAME}:1" \
        "${DOMAIN_EXPORT}python3 ${NAV_DIR}/planning/humanoid_sim/scripts/mujoco_lidar_bridge.py --ros-args -p model_path:='${MODEL_PATH}' -p output_type:=custom -p downsample:=5 -p lidar_hz:=10" Enter

sleep 2

# --- [窗口 2] livox IMU 200Hz 转发 ---
tmux new-window -t "${SESSION_NAME}" -n "imu"
echo -e "${GREEN}  [2] livox_imu_throttle (/livox/imu -> /livox/imu_200)${NC}"
tmux send-keys -t "${SESSION_NAME}:imu" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:imu" \
    "${DOMAIN_EXPORT}ros2 run humanoid_sim livox_imu_throttle.py" Enter

sleep 1

# --- [窗口 3] FastLIO2 ---
tmux new-window -t "${SESSION_NAME}" -n "fastlio"
if [ "${CAPTURE}" = "1" ]; then
    echo -e "${GREEN}  [3] FastLIO2 (${FASTLIO_CONFIG}, 采图: pcd_save_en=true)${NC}"
else
    echo -e "${GREEN}  [3] FastLIO2 (${FASTLIO_CONFIG}, CustomMsg)${NC}"
fi
tmux send-keys -t "${SESSION_NAME}:3" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:3" \
    "${DOMAIN_EXPORT}ros2 launch fast_lio mapping_sim_module.launch.py config_file:=${FASTLIO_CONFIG}" Enter

sleep 2

if [ "${CAPTURE}" = "1" ]; then
    # --- [窗口 4] odom_bridge (采图模式也要: /cmd_vel → /cmd_vel_limiter 才能驱动机器人) ---
    tmux new-window -t "${SESSION_NAME}" -n "odom_bridge"
    echo -e "${GREEN}  [4] odom_bridge (cmd_vel 通路)${NC}"
    tmux send-keys -t "${SESSION_NAME}:odom_bridge" "${TMUX_SOURCE}" Enter
    tmux send-keys -t "${SESSION_NAME}:odom_bridge" \
        "${DOMAIN_EXPORT}ros2 run humanoid_sim odom_bridge.py --ros-args -p use_sim_time:=false" Enter

    sleep 1
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN} 采图模式就绪 (CAPTURE=1)${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo " 驱动覆盖 (cmd_vel → odom_bridge 限幅 → /cmd_vel_limiter):"
    echo "   [先带域] export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-(未设置! 勿串默认域0)}"
    echo "   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}'"
    echo "   建议沿 bag 已验证自由走廊 (goal A/B/C 往返) + 补充横扫;"
    echo "   盲开 lawnmower 会撞进 env 实体 box, 优先复用已验证路径。"
    echo " 采完后: 在 fastlio 窗口 Ctrl+C 停止 → mujoco_real_lab.pcd 落在窗口 cwd"
    echo "         cp 到 ${NAV_DIR}/localization/open3d_loc/maps/ 后重建 open3d_loc"
    echo " 关闭全部: tmux kill-session -t ${SESSION_NAME}"
    exit 0
fi

# --- [窗口 4] open3d_loc (ICP) — 用 sim 自采图 ---
tmux new-window -t "${SESSION_NAME}" -n "icp"
echo -e "${GREEN}  [4] open3d_loc (ICP, map=${ICP_MAP_PCD})${NC}"
tmux send-keys -t "${SESSION_NAME}:4" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:4" \
    "${DOMAIN_EXPORT}ros2 launch open3d_loc open3d_loc_x1.launch.py use_sim_time:=true map_file:=${NAV_DIR}/localization/open3d_loc/maps/${ICP_MAP_PCD}" Enter

sleep 2

# --- [窗口 5] Nav2 — 与真机同图 ---
tmux new-window -t "${SESSION_NAME}" -n "nav2"
echo -e "${GREEN}  [5] Nav2 导航 (map=${NAV2_MAP_YAML}, 真机同图)${NC}"
tmux send-keys -t "${SESSION_NAME}:5" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:5" \
    "${DOMAIN_EXPORT}ros2 launch humanoid_sim navigation.launch.py map_yaml:=${NAV2_MAP_YAML}" Enter

sleep 2

# --- [窗口 6] 测试数据采集 (rosbag + pidstat) ---
tmux new-window -t "${SESSION_NAME}" -n "record"
echo -e "${GREEN}  [record] 测试数据采集 (手动启动)${NC}"
tmux send-keys -t "${SESSION_NAME}:record" "${DOMAIN_EXPORT}source ${ROS_SETUP_BASH}" Enter
tmux send-keys -t "${SESSION_NAME}:record" \
    "echo '=== real2sim 导航测试数据采集 (域 ${ROS_DOMAIN_ID:-0}) ===\n  录制 bag: ${DOMAIN_EXPORT}ros2 bag record /mujoco/ground_truth /cmd_vel /cmd_vel_limiter /Odometry /tf -o real2sim_run_NNN\n  实时监控: ros2 topic echo /mujoco/ground_truth --once'" Enter

# --- 完成 ---
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} real2sim 导航链路已启动 (session: ${SESSION_NAME})${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo " tmux 窗口布局:"
echo "   [0] aimrt        - aimrt_main (ONNX RL + MuJoCo, real_lab_env)"
echo "   [1] lidar_bridge - MuJoCo LiDAR 射线追踪 + /clock"
echo "   [2] imu          - /livox/imu -> /livox/imu_200 (200Hz)"
echo "   [3] fastlio      - FastLIO2 里程计"
echo "   [4] icp          - open3d_loc (ICP, mujoco_real_lab.pcd)"
echo "   [5] nav2         - Nav2 导航栈 (robot_lab_20261030.pgm 真机同图)"
echo "   record           - 测试数据采集 (rosbag + pidstat)"
echo ""
echo " 出生点: (0.14, 0.51) yaw=0 — 发目标前先手柄 stand_mode → walk_mode"
echo " goal A/B/C 坐标见 docs/sim2real/real2sim-P0-盘点-2026-11-20.md"
echo " 关闭全部: tmux kill-session -t ${SESSION_NAME}"
