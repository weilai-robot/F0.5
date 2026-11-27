#!/bin/bash
# ============================================================
# X1 模式切换 — switch_x1_mode.sh
# 对齐 sim_control/set_*_mode.sh：必须先 source AimRT 侧 ros2_setup，
# 否则真机上 topic 发得出、小脑却收不到（DOMAIN / plugin_proto 环境不一致）。
#
# 用法:
#   ./switch_x1_mode.sh zero     # 阻尼 / 复位
#   ./switch_x1_mode.sh stand    # 站立（导航结束也用这个，先于 cmd_vel 归零）
#   ./switch_x1_mode.sh walk     # 进入行走（可收 /cmd_vel_limiter）
#   ./switch_x1_mode.sh ready    # zero → stand → walk（带 hold，首次联调）
#   ./switch_x1_mode.sh plan     # 先切 stand 再挥手（组合入口，不确定当前态时用）
#   ./switch_x1_mode.sh wave     # 直接挥手（裸发 /plan_mode 短窗，等价手柄 RB，需已在 stand）
#
# 前提:
#   1. AimRT 小脑已用 run.sh 启动
#   2. x1_cfg.yaml 已打开 mode topic 的 ros2 后端
#
# 注意:
#   不用 ros2 topic pub --once（DDS 发现未完成消息常丢）。
#   与 send_nav_goal.sh 一样持续发 ~2s。
#   wave/plan: plan 态自包含（重复触发 = 轨迹从头重启），故 /plan_mode 只发 0.8s
#   短窗——全落控制端 1s 节流窗内必然只触发一次。wave 裸发需已在 stand 态
#   （idle/zero 下静默拒、无任何反馈）；不确定当前态时用 plan（先 stand 再 wave）。
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

MODE="${1:-}"
HOLD_ZERO="${ZERO_HOLD_S:-2.5}"
HOLD_STAND="${STAND_HOLD_S:-3.0}"
PUB_SEC="${MODE_PUB_S:-2.0}"
PUB_RATE="${MODE_PUB_RATE:-5}"

usage() {
  sed -n '2,25p' "$0"
  echo ""
  echo "环境变量可选: ZERO_HOLD_S STAND_HOLD_S MODE_PUB_S MODE_PUB_RATE PLAN_PUB_S AIMRT_DIR"
  exit "${1:-0}"
}

case "${MODE}" in
  -h|--help) usage 0 ;;
  zero|stand|walk|ready|plan|wave) ;;
  "" )
    echo -e "${RED}[ERROR] 缺少模式参数${NC}"
    usage 1
    ;;
  * )
    echo -e "${RED}[ERROR] 未知模式: ${MODE}${NC}"
    usage 1
    ;;
esac

# ── 定位 AimRT 目录（含 ros2_setup.sh / aimrt_main）──
AIMRT_CANDIDATES=(
  "${AIMRT_DIR:-}"
  "${ROOT_DIR}/build"
  "${ROOT_DIR}/motion_control/install/linux/bin"
)

AIMRT_BIN=""
for d in "${AIMRT_CANDIDATES[@]}"; do
  [ -z "${d}" ] && continue
  if [ -f "${d}/ros2_setup.sh" ] || [ -x "${d}/aimrt_main" ]; then
    if [ -f "${d}/cfg/x1_cfg.yaml" ] || [ -f "${d}/ros2_setup.sh" ]; then
      AIMRT_BIN="${d}"
      break
    fi
  fi
done

if [ -z "${AIMRT_BIN}" ]; then
  echo -e "${RED}[ERROR] 未找到 AimRT 目录（ros2_setup.sh / aimrt_main）${NC}"
  echo -e "  试过: ${ROOT_DIR}/build , ${ROOT_DIR}/motion_control/install/linux/bin"
  echo -e "  可: export AIMRT_DIR=/path/to/aimrt_bin"
  exit 1
fi

# ── ROS2 + AimRT ros2_plugin_proto（与 sim_control 一致）──
# ros2_setup.sh 内部用相对路径 ./install/...，必须先 cd 到 AimRT 目录再 source
if [ -f "${SCRIPT_DIR}/ros2_source.sh" ]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/ros2_source.sh"
elif [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
else
  echo -e "${RED}[ERROR] 未找到 ROS2 setup.bash${NC}"
  exit 1
fi

pushd "${AIMRT_BIN}" >/dev/null
if [ -f ./ros2_setup.sh ]; then
  # shellcheck source=/dev/null
  source ./ros2_setup.sh
  echo -e "${GREEN}  AimRT ros2_setup: ${AIMRT_BIN}/ros2_setup.sh${NC}"
else
  # 回退：绝对路径拉 plugin_proto
  for proto in \
    "${AIMRT_BIN}/install/share/ros2_plugin_proto/local_setup.bash" \
    "${AIMRT_BIN}/build/install/share/ros2_plugin_proto/local_setup.bash" \
    "${AIMRT_BIN}/../share/ros2_plugin_proto/local_setup.bash"
  do
    if [ -f "${proto}" ]; then
      # shellcheck source=/dev/null
      source "${proto}"
      echo -e "${GREEN}  AimRT proto: ${proto}${NC}"
      break
    fi
  done
fi
popd >/dev/null

# ── 持续发布模式 topic（避免 --once 丢包）──
# topic 与 Float32 data 数值与 sim_control 对齐；控制端只看 topic 触发，data 0/1 均可。
pub_mode() {
  local topic="$1"
  local data="$2"
  local label="$3"
  local dur="${4:-${PUB_SEC}}"   # 可选发布时长，缺省 PUB_SEC（旧模式行为不变）

  echo -e "${GREEN}→ ${label}  (/ ${topic} data=${data}, ${PUB_RATE}Hz × ${dur}s)${NC}"

  # 可选：提示是否有订阅者（小脑未起时明显）
  if command -v ros2 >/dev/null 2>&1; then
    if ! ros2 topic info "${topic}" 2>/dev/null | grep -q 'Subscription count: [1-9]'; then
      echo -e "${YELLOW}  [WARN] ${topic} 暂无订阅者——确认 AimRT 已启动且 x1_cfg 打开了 ros2 后端${NC}"
    fi
  fi

  ros2 topic pub -r "${PUB_RATE}" "${topic}" std_msgs/msg/Float32 "{data: ${data}}" >/dev/null 2>&1 &
  local pub_pid=$!
  sleep "${dur}"
  kill "${pub_pid}" 2>/dev/null || true
  wait "${pub_pid}" 2>/dev/null || true
}

do_zero()  { pub_mode /zero_mode  1.0 "zero_mode"; }
do_stand() { pub_mode /stand_mode 1.0 "stand_mode"; }
do_walk()  { pub_mode /walk_mode  0.0 "walk_mode"; }
# wave 裸发 /plan_mode 短窗（等价手柄 RB 键）：需已在 stand 系态（idle/zero 下
# 被状态机静默拒、无任何反馈）。plan 态自包含（重复触发 = 轨迹重启；2s 窗会在
# 1s 节流放行处重触发、总长拖到 ~8.2s），故只发 0.8s——全落节流窗内必触发一次。
do_wave()  { pub_mode /plan_mode  1.0 "plan_mode" "${PLAN_PUB_S:-0.8}"; }
# plan 组合 = 先 stand（幂等：已站时重发被静默拒）→ sleep 1（do_stand 2s 发布窗
# 结束后再等 1s，保证 wave 首帧距上次节流推进 ≥1s、不被吞）→ wave。
# 不确定当前态时的稳妥入口（从 zero 也能一步到位）；idle/keep 下两步都被静默拒。
# 挥手动作约 7.2s 播完自回站立位，无需收尾指令。
do_plan()  { do_stand; sleep 1; do_wave; }

case "${MODE}" in
  zero)
    do_zero
    echo -e "${GREEN}完成: zero_mode${NC}"
    ;;
  stand)
    do_stand
    echo -e "${GREEN}完成: stand_mode（导航结束停步也用此命令）${NC}"
    echo -e "${YELLOW}提示: 先 stand，再让 /cmd_vel 归零，勿反序${NC}"
    ;;
  walk)
    do_walk
    echo -e "${GREEN}完成: walk_mode（应开始原地踏步，可收 cmd_vel_limiter）${NC}"
    ;;
  plan)
    echo -e "${YELLOW}序列: stand → wave（不确定当前态时的稳妥入口）${NC}"
    do_plan
    echo -e "${GREEN}完成: plan 组合已发送（挥手约 7.2s 异步播放，无需收尾）${NC}"
    ;;
  wave)
    do_wave
    echo -e "${GREEN}完成: wave 指令已发送（挥手约 7.2s 异步播放，无需收尾）${NC}"
    ;;
  ready)
    echo -e "${YELLOW}序列: zero → stand → walk（请确认机器人已支撑/安全）${NC}"
    do_zero
    echo -e "${YELLOW}  hold zero ${HOLD_ZERO}s...${NC}"
    sleep "${HOLD_ZERO}"
    do_stand
    echo -e "${YELLOW}  hold stand ${HOLD_STAND}s（确认站稳再继续）...${NC}"
    sleep "${HOLD_STAND}"
    do_walk
    echo -e "${GREEN}完成: ready（已 walk）。可发: ./send_nav_goal_real.sh <x> <y>${NC}"
    ;;
esac
