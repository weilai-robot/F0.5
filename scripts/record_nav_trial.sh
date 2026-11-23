#!/bin/bash
# ============================================================
# record_nav_trial.sh — 单试次录包，与 send_nav_goal_real.sh 原子同步起停
#
# 背景: 2026-11-18 换下半身前最后一个真机窗口，采「改前」基线:
#   P6 停止轮廓 / real 刹车需求 / 旧下半身 cmd→响应跟踪 / 摔倒签名。
#   开录 → 发目标(可人工 Ctrl+C 取消) → 续录 tail 秒(抓 NSM 兜底/摔倒余波) → 停录。
#
#   2026-11-23 起 send_nav_goal_real.sh 录包改显式参数：带 --record 才 exec 进本
#   脚本（RECORD_NAV=1 防递归），label 自动 auto_<HHMMSS>；本脚本也可直调
#   （可带人工 label，如 T3_goal_01）。
#
# topic 集（单 bag；刻意不录点云/costmap）:
#   导航分析集 = record_nav_test.sh --nav 同款（11-06 goalmix045 真机实证可录）:
#     /tf /tf_static /Odometry /odom /odom2map /localization_3d_confidence
#     /cmd_vel /cmd_vel_limiter /plan
#     /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback
#     /stand_mode /walk_mode
#   下半身（换件前唯一机会）: /joint_states /imu/data /actuator_states /actuator_cmd
#   原始 IMU: /livox/imu      事件: /trial_event（start/end 打点，对齐手机视频）
#
# 用法:
#   record_nav_trial.sh <label> -- <send_nav_goal_real.sh 参数...>
#     record_nav_trial.sh T3_goal_01 -- 2.0 0.0           # 到点自然停（温柔停基线）
#     record_nav_trial.sh T1_cancel_01 -- 2.0 0.0         # 直行巡航中人工 Ctrl+C 取消
#     record_nav_trial.sh T2_turn_01 -- 2.0 2.0 90 120    # 转弯中取消（摔窗口姿态）
#   record_nav_trial.sh --no-snapshot -- 2.0 0.0        # label 省略 → 自动 auto_<HHMMSS>
#     （语音线形态：voice_agent ssh 调 send_nav_goal_real.sh → 旁路 exec 进本脚本）
#   选项:
#     --no-snapshot   跳过开录前快照（省 ~5s）
#     --tail <sec>    结束后续录秒数（默认 10）
#     --max <sec>     录包硬上限（默认 600，防孤儿包/磁盘耗尽）
#
# 前提: run_nav_real.sh 栈已起 + 小脑 aimrt_main 已起。在你平时跑 send_nav_goal_real.sh
#   的终端直接跑即可；建议 tmux（防 ssh 断线）但不强制——断线时 send_nav_goal_real
#   自身 HUP→safe_stand 护机器人，录包孤儿由 --max 兜底优雅收尾，仅损失 tail/notes/台账。
# 输出: F1/reports/nav_test_record/<ts>_<label>/
#     bag/  recorder.log  qos_override.yaml  pre_snapshot.txt  bag_info.txt  notes.md
#     并向 reports/nav_test_record/trial_log.csv 追加一行（人工列留空待填）
# 离线: python3 scripts/bag_analyze.py <bag目录> --goal X Y YAW
#
# Ctrl+C 语义: 一次 = 取消导航（send_nav_goal_real 内部 → CANCEL → safe_stand），
#   录包不停，+tail 秒后自动收尾；tail 期间再按一次 = 跳过 tail 立即停录。
# ============================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPORT_ROOT="${ROOT_DIR}/reports/nav_test_record"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

usage() { sed -n '2,43p' "$0"; exit "${1:-0}"; }

LABEL=""
SNAPSHOT=true
TAIL=10
MAX_DUR=600
GOAL_ARGS=()

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage 0 ;;
    --no-snapshot) SNAPSHOT=false; shift ;;
    --tail) TAIL="${2:?--tail 缺秒数}"; shift 2 ;;
    --max) MAX_DUR="${2:?--max 缺秒数}"; shift 2 ;;
    --) shift; GOAL_ARGS=("$@"); break ;;
    *) if [ -z "${LABEL}" ]; then LABEL="$1"; else
         echo -e "${RED}[ERROR] 多余位置参数: $1（label 之后请用 -- 分隔目标参数）${NC}" >&2; usage 1
       fi; shift ;;
  esac
done
if [ ${#GOAL_ARGS[@]} -eq 0 ]; then
  echo -e "${RED}[ERROR] 用法: record_nav_trial.sh [label] -- <goal x y [yaw] [timeout]>${NC}" >&2
  usage 1
fi
if [ -z "${LABEL}" ]; then
  LABEL="auto_$(date +%H%M%S)"   # 语音线经旁路进来不带 label：自动名，台账仍可对 trial_event/视频
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
SAFE_LABEL="${LABEL//[^a-zA-Z0-9_-]/_}"
RUN_DIR="${REPORT_ROOT}/${STAMP}_${SAFE_LABEL}"
BAG_PATH="${RUN_DIR}/bag"          # ros2 bag record -o 目标目录不能预建（goalmix045 教训）
mkdir -p "${RUN_DIR}"

# ── 磁盘门禁（盘点 P0-4：三个录包入口都无 df 检查，这里先补上）──
AVAIL_MB="$(df -Pm "${HOME}" | awk 'NR==2{print $4}')"
if [ "${AVAIL_MB}" -lt 10240 ]; then
  echo -e "${RED}[ERROR] 磁盘剩余 ${AVAIL_MB}MB < 10GB，拒绝开录（先清理 ~/bags 旧包）${NC}" >&2
  exit 1
fi
echo -e "${GREEN}[trial] 磁盘剩余 ${AVAIL_MB}MB${NC}"

# ── ROS2 环境（照抄 record_nav_test.sh，含 11-06 minipc 的 set -u 坑注释）──
set +u
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/ros2_source.sh"
if [ -f "${ROOT_DIR}/navigation/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "${ROOT_DIR}/navigation/install/setup.bash"
fi
set -u
if ! command -v ros2 >/dev/null 2>&1; then
  echo -e "${RED}[ERROR] ros2 未找到，请先 source ROS2 环境${NC}" >&2
  exit 1
fi

# ── topic 存在性检查（缺只 WARN 不阻断——与 record_nav_test.sh 同口径；
#    单次 list 缓存后本地 grep：语音线直调时省 ~5-20s 串行 ros2 调用延迟）──
TOPICS=(
  /tf /tf_static /Odometry /odom /odom2map /localization_3d_confidence
  /cmd_vel /cmd_vel_limiter /plan
  /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback
  /stand_mode /walk_mode
  /joint_states /imu/data /actuator_states /actuator_cmd
  /livox/imu /trial_event
)
TOPIC_LIST_OUT="$(ros2 topic list 2>/dev/null || true)"
MISSING=()
for t in "${TOPICS[@]}"; do
  grep -qx "${t}" <<<"${TOPIC_LIST_OUT}" || MISSING+=("${t}")
done
if [ ${#MISSING[@]} -gt 0 ]; then
  echo -e "${YELLOW}[WARN] 以下 topic 当前不存在（/actuator_* 常见=小脑未起或 actuator_debug 关；仍会尝试）:${NC}" >&2
  printf '  %s\n' "${MISSING[@]}" >&2
fi

# ── QoS 覆盖：best_effort 订阅端对 reliable/best_effort 发布端都兼容——
#    下半身/传感器话题的发布端 QoS 未实证（导航集已实证 RELIABLE），这里保底录上。
QOS_YAML="${RUN_DIR}/qos_override.yaml"
cat > "${QOS_YAML}" <<'EOF'
/Odometry:
  history: keep_last
  depth: 50
  reliability: best_effort
  durability: volatile
/livox/imu:
  history: keep_last
  depth: 200
  reliability: best_effort
  durability: volatile
/joint_states:
  history: keep_last
  depth: 200
  reliability: best_effort
  durability: volatile
/imu/data:
  history: keep_last
  depth: 200
  reliability: best_effort
  durability: volatile
/actuator_states:
  history: keep_last
  depth: 200
  reliability: best_effort
  durability: volatile
/actuator_cmd:
  history: keep_last
  depth: 200
  reliability: best_effort
  durability: volatile
EOF

echo -e "${GREEN}[trial] 开录 → ${BAG_PATH}${NC}"
# setsid：录包进程独立进程组/会话——试次中 Ctrl+C 取消导航时，终端 INT
# 不会误杀录包（cancel 试次恰恰最需要录到取消瞬间）；孤儿由 --max 硬上限兜底，
# timeout 发 INT → rosbag2 优雅收尾（metadata.yaml 完整）。
setsid timeout --signal=INT --kill-after=10 "${MAX_DUR}" \
  ros2 bag record -o "${BAG_PATH}" --qos-profile-overrides-path "${QOS_YAML}" "${TOPICS[@]}" \
  > "${RUN_DIR}/recorder.log" 2>&1 &
REC_PID=$!
sleep 1.5   # 等订阅发现完成（trial_event 是 VOLATILE，首发早了会丢）

if ! kill -0 "${REC_PID}" 2>/dev/null; then
  echo -e "${RED}[ERROR] 录包进程启动即退（静默无包是历史坑，看 recorder.log）：${NC}" >&2
  cat "${RUN_DIR}/recorder.log" >&2
  exit 1
fi

# 事件打点：手机视频开头口报同一 label，即可对齐 bag（wall 时间戳）与视频。
# 后台化：pub --times 5 --rate 2 要 ~2.5s，语音线直调时不拖导航启动延迟（孤儿随脚本退出自然结束）
pub_event() {
  ros2 topic pub --times 5 --rate 2 /trial_event std_msgs/String \
    "{data: '$1'}" >/dev/null 2>&1 || true
}
pub_event "start ${SAFE_LABEL}" &

if [ "${SNAPSHOT}" = true ]; then
  {
    echo "# time: $(date --iso-8601=seconds)"
    echo "# df: $(df -Pm "${HOME}" | tail -1)"
    ros2 topic list -t
    timeout 5 ros2 run tf2_ros tf2_echo map odom
  } > "${RUN_DIR}/pre_snapshot.txt" 2>&1 || true
fi

# ── 跑目标（子脚本原样执行；wrapper 捕获 INT/TERM/HUP 但不转发——
#    注意不能用 trap ''（SIG_IGN 会被子进程继承 → Ctrl+C 取消彻底失效）。
#    HUP 是语音取消路径：executor 杀本地 ssh → 远端前台进程组收 HUP →
#    send_nav_goal_real 自己的 HUP trap 走 safe_stand 护机器人；wrapper 吞 HUP
#    存活到收尾（end 事件/tail/notes/台账完整），不信令即超时兜底同旧路径）──
trap 'true' INT TERM HUP
echo -e "${GREEN}[trial] ── 开跑: send_nav_goal_real.sh ${GOAL_ARGS[*]} ──（人在旁护着）${NC}"
set +e
# RECORD_NAV=1：告诉 send_nav_goal_real 已在 wrapper 内层，别再 exec 回来（防递归）
RECORD_NAV=1 bash "${SCRIPT_DIR}/send_nav_goal_real.sh" "${GOAL_ARGS[@]}"
RC=$?
set +e   # 本脚本全程无 -e：取消/超时 rc≠0 是正常路径，后续收尾不许中断

pub_event "end ${SAFE_LABEL} rc=${RC}"
echo -e "${YELLOW}[trial] 目标结束(rc=${RC})，续录 ${TAIL}s 抓 NSM 兜底/摔倒余波…${NC}"
sleep "${TAIL}"

# ── 停录 + 元数据 ──
kill -INT "${REC_PID}" 2>/dev/null || true
wait "${REC_PID}" 2>/dev/null || true
ros2 bag info "${BAG_PATH}" > "${RUN_DIR}/bag_info.txt" 2>&1 || true

cat > "${RUN_DIR}/notes.md" <<EOF
# 试次记录（换下半身前基线 2026-11-18）

- **label**: ${SAFE_LABEL}
- **goal args**: ${GOAL_ARGS[*]}
- **结束 rc**: ${RC}（0=到达/服务端完成，124=超时，其他=人工取消/中断）
- **bag**: ${BAG_PATH}
- **视频编号**: ___（口报 label 对齐）
- **摔**: 否 / 是（方向/程度：___）
- **皮尺距墙/障碍 min 距离**: ___m
- **备注**: ___（姿态异常/NSM 行为/声音等）
EOF

LOG_CSV="${REPORT_ROOT}/trial_log.csv"
[ -f "${LOG_CSV}" ] || echo "ts,label,goal_args,rc,fall,dist_min_m,video,note" >> "${LOG_CSV}"
printf '%s,%s,"%s",%s,,,,\n' "$(date +%Y%m%d_%H%M%S)" "${SAFE_LABEL}" "${GOAL_ARGS[*]}" "${RC}" >> "${LOG_CSV}"

echo -e "${GREEN}[trial] 完成${NC}"
echo "  目录: ${RUN_DIR}"
echo "  请填: ${RUN_DIR}/notes.md（摔/距墙/视频编号）+ 台账 ${LOG_CSV}"
# 语音线直调依赖 RC 语义（0=ok/124=timeout/其他=fail）：透传 send_nav_goal_real 的 RC
exit "${RC}"
