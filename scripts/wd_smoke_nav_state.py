#!/usr/bin/env python3
"""wd_smoke v3 — nav_state_manager 纯看门狗冒烟回归（P5 线②分级版，2026-11-17）

用法（WSL / 机器人侧均可，需 ROS2 环境）：
    source /opt/ros/humble/setup.bash && python3 F1/scripts/wd_smoke_nav_state.py
    # 可选：python3 F1/scripts/wd_smoke_nav_state.py <nav_state_manager.py 绝对路径>
    # 耗时 ~40s；节点子进程日志落 /tmp/wd_smoke_node.log（不入仓的工件）

⚠️ 纪律：**每次改 nav_state_manager.py 之后必须重跑本脚本**（P0 校验 V0 的
教训——2026-10-16 实施者留档冒烟跑的是上一次编辑的代码，最后一轮文案级
编辑未被留档覆盖，靠校验方重跑才闭合证明链）。改判据/触发逻辑不重跑 =
回归结论无效。

v2 → v3（P5 看门狗线②分级，方案 docs/nav/改进TODO/P5-遇堵先停-BT门控方案.md
§3.2）：判据从「沉默即死」改为「按心跳源末帧值分级」，v2 相位 1/3 的期待值
与旧契约一起作废，重排三相位：
  A) 零末帧豁免 + 零速档兜底：双路零值 10Hz 流动期不触发（零值=心跳，P1
     等待语义不误杀）；双路沉默 >3s **不触发**（P5 门控停等豁免——v2 旧
     规则在 ~8.3s 会误杀，本相位专防这件事回归）；backstop 参数压到 8s
     （spawn 时 --ros-args -p zero_silence_backstop_s:=8.0，顺带验证参数
     接线）→ ~13s burst1（判定源=limiter，末帧零速档）
  B) 线① + F1 re-arm：第二 goal SUCCEEDED → 0.5s 后 burst2（无任何心跳
     消息，验旧一次性缺陷已修：第二个 goal 到点必须再发 stand）
  C) 链路死档保留：双路非零流动 → limiter 沉默（cmd_vel 活）→ 3s burst3
     ——odom_bridge 死盲走保护（末帧非零档，原行为不变）

断言：共 30 条 /stand_mode = 3 burst ×10；零末帧沉默期（5.0s 起）到 burst1
前零触发（3s 豁免）；burst1 ~[12.7,14.5] / burst2 ~[20.8,22.0] / burst3
~[32.4,34.5]；节点日志零速档/链路死文案各 ≥1、指名 limiter ≥2。
"""
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# 默认相对本文件解析（F1/scripts/ → 仓内 navigation 子仓）；argv[1] 可覆盖
NODE_SCRIPT = (
    sys.argv[1] if len(sys.argv) > 1 else
    str(Path(__file__).resolve().parent.parent
        / 'navigation' / 'planning' / 'humanoid_sim' / 'scripts'
        / 'nav_state_manager.py'))
NODE_LOG = "/tmp/wd_smoke_node.log"
T0 = time.monotonic()
events = []
lock = threading.Lock()


class Smoke(Node):
    def __init__(self):
        super().__init__('wd_smoke')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lim_pub = self.create_publisher(Twist, '/cmd_vel_limiter', 10)
        self.walk_pub = self.create_publisher(Float32, '/walk_mode', 10)
        self.sta_pub = self.create_publisher(
            GoalStatusArray, '/navigate_to_pose/_action/status', 10)
        self.sub = self.create_subscription(
            Float32, '/stand_mode', self.on_stand, 10)

    def on_stand(self, msg):
        with lock:
            events.append(time.monotonic() - T0)

    def pub_walk(self):
        # 真机口径 data=0.0（事件推断忽略 data）
        m = Float32()
        m.data = 0.0
        self.walk_pub.publish(m)

    def pub_walk_burst(self):
        # 5Hz×2s 语义的浓缩：3 连发盖 DDS 发现窗
        for _ in range(3):
            self.pub_walk()
            time.sleep(0.2)

    def pub_status(self, status):
        m = GoalStatusArray()
        gs = GoalStatus()
        gs.status = status
        m.status_list = [gs]
        self.sta_pub.publish(m)

    def pub_status_twice(self, status):
        self.pub_status(status)
        time.sleep(0.25)
        self.pub_status(status)

    def pub_twist(self, pub, vx):
        m = Twist()
        m.linear.x = vx
        pub.publish(m)


def wait_to(t):
    d = t - (time.monotonic() - T0)
    if d > 0:
        time.sleep(d)


def drive(node, done):
    # ── 相位 A：零末帧豁免 + 零速档兜底（P5 分级，backstop 参数压到 8s）──
    wait_to(1.0)
    node.pub_walk_burst()                       # walk 事件 → in_walk + re-arm
    wait_to(2.0)
    node.pub_status_twice(GoalStatus.STATUS_EXECUTING)
    for i in range(29):                         # 2.1→4.9 双路零值 10Hz
        wait_to(2.1 + i * 0.1)
        node.pub_twist(node.cmd_pub, 0.0)
        node.pub_twist(node.lim_pub, 0.0)
    # 5.0 起双路沉默（末帧=零）：>3s 不得触发（P5 门控停等豁免，v2 旧规则
    # 会在 ~8.3s 误杀）→ backstop=8s 到点 ~13.x burst1

    # ── 相位 B：线① + F1 re-arm（无心跳消息）──
    wait_to(19.0)                               # burst1 停止序列 ~17.5 收尾后再 re-arm
    node.pub_walk_burst()
    wait_to(20.0)
    node.pub_status_twice(GoalStatus.STATUS_EXECUTING)
    wait_to(20.5)
    node.pub_status_twice(GoalStatus.STATUS_SUCCEEDED)   # → burst2 ~21.0

    # ── 相位 C：链路死档保留（limiter 末帧非零后沉默=odom_bridge 死盲走）──
    wait_to(26.5)                               # burst2 停止序列 ~25.5 收尾后再 re-arm
    node.pub_walk_burst()
    wait_to(27.5)
    node.pub_status_twice(GoalStatus.STATUS_EXECUTING)
    for i in range(21):                         # 27.6→29.6 双路非零 10Hz
        wait_to(27.6 + i * 0.1)
        node.pub_twist(node.cmd_pub, 0.3)
        node.pub_twist(node.lim_pub, 0.3)
    for i in range(19):                         # 29.7→31.5 仅 cmd_vel 活（limiter 死）
        wait_to(29.7 + i * 0.1)
        node.pub_twist(node.cmd_pub, 0.3)
    # 期待 ~32.7 burst3（limiter 末帧非零 + 沉默 >3s；cmd_vel 31.5 即停，
    # 防停止序列收尾后被非零回声 re-arm 出第 4 个 burst）
    done.set()


def main():
    if not Path(NODE_SCRIPT).is_file():
        print(f"[smoke] FAIL: 找不到被测节点 {NODE_SCRIPT}")
        sys.exit(2)

    log = open(NODE_LOG, 'w')
    # P5 分级：backstop 参数压到 8s（零速档兜底在 40s 冒烟窗内可观测，
    # 顺带验证 zero_silence_backstop_s 参数接线；CLI '8.0' 带小数点→float，
    # 写 '8' 会以 int 落参、与 declare 的 double 缺省类型冲突）
    proc = subprocess.Popen(
        [sys.executable, NODE_SCRIPT,
         '--ros-args', '-p', 'zero_silence_backstop_s:=8.0'],
        stdout=log, stderr=subprocess.STDOUT)
    time.sleep(1.0)                             # 节点先起（订阅就位）
    rclpy.init()
    node = Smoke()
    done = threading.Event()
    threading.Thread(target=drive, args=(node, done), daemon=True).start()

    deadline = time.monotonic() + 38.5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
    log.close()

    # ── 断言 ──
    with lock:
        ev = sorted(events)
    fails = []
    print(f"[smoke] /stand_mode 共 {len(ev)} 条: {[round(t,1) for t in ev]}")

    if len(ev) != 30:
        fails.append(f"消息数 {len(ev)} != 30（每 burst 10×3）")

    # 按 0.5s 间隙聚簇
    bursts = []
    for t in ev:
        if bursts and t - bursts[-1][-1] < 0.5:
            bursts[-1].append(t)
        else:
            bursts.append([t])
    print(f"[smoke] burst 起点数={len(bursts)}: "
          f"{[ (round(b[0],1), round(b[-1],1)) for b in bursts ]}")

    if len(bursts) != 3:
        fails.append(f"burst 数 {len(bursts)} != 3")
    else:
        b1, b2, b3 = bursts
        if not (12.7 <= b1[0] <= 14.5):
            fails.append(f"burst1 起点 {b1[0]:.1f} 不在 [12.7,14.5]"
                         "（零速档兜底 backstop=8s）")
        if not (20.8 <= b2[0] <= 22.0):
            fails.append(f"burst2 起点 {b2[0]:.1f} 不在 [20.8,22.0]（线①+re-arm）")
        if not (32.4 <= b3[0] <= 34.5):
            fails.append(f"burst3 起点 {b3[0]:.1f} 不在 [32.4,34.5]（链路死 3s 档）")

    # P5 豁免核心断言：零末帧沉默期（5.0 起）到 burst1 前不得出现任何触发
    # ——v2 旧「沉默即死」规则会在 ~8.3s 误杀，本断言专防分级退化回归
    if ev and ev[0] < 12.7:
        fails.append(f"零末帧沉默期出现早期触发（{ev[0]:.1f}）"
                     "——P5 分级失效（3s 规则误杀门控停等）")

    # 节点日志：burst1/burst3 应分别走零速档/链路死分级文案，且判定源=limiter
    logtxt = open(NODE_LOG, errors='replace').read()
    n_zero = logtxt.count('末帧零速后沉默')
    n_nonzero = logtxt.count('末帧非零后沉默')
    n_limiter = logtxt.count('cmd_vel_limiter 末帧')
    print(f"[smoke] 节点日志：零速档 ×{n_zero}（期待 ≥1）、链路死 ×{n_nonzero}"
          f"（期待 ≥1）、指名 limiter ×{n_limiter}（期待 ≥2：burst1+burst3）")
    if n_zero < 1:
        fails.append(f"日志 '末帧零速后沉默' 出现 {n_zero} 次 <1（零速档未走分级文案）")
    if n_nonzero < 1:
        fails.append(f"日志 '末帧非零后沉默' 出现 {n_nonzero} 次 <1（链路死档未走分级文案）")
    if n_limiter < 2:
        fails.append(f"日志 'cmd_vel_limiter 末帧' 出现 {n_limiter} 次 <2（判定源应为 limiter）")

    if fails:
        print("[smoke] FAIL:")
        for f in fails:
            print(f"  - {f}")
        sys.exit(1)
    print("[smoke] PASS（P5 线②分级：零末帧 3s 豁免 + backstop 兜底 / 非零末帧"
          " 3s 链路死保留 + 线①re-arm 二次触发）")


if __name__ == '__main__':
    main()
