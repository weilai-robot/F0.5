#!/usr/bin/env python3
# ============================================================
# real2sim 链式导航跑 — r2s_chain.py（人 / agent 双用，2026-11-24）
#
# 把 P3 链式协议固化为一个脚本：preflight 体检 → 链式腿(walk 接力、
# 腿间零空闲、防 P6 慢性站立熔断) → 每腿自动指标(落点误差/朝向残差/
# col/路径) → results.json + 退出码。等价于 /tmp/r2s_p3{,b}.sh 的升级版。
#
# ── 人手动用法（栈已起: tmux attach 看 pane）──
#   python3 r2s_chain.py preflight                 # 只体检（load/TF/GT 三门）
#   python3 r2s_chain.py leg A --x -3.22 --y 3.24  # 单腿冒烟（yaw 默认 0）
#   python3 r2s_chain.py leg B --x -2.68 --y -0.50 --yaw -90
#   python3 r2s_chain.py pass1                     # 全链 A→S1→B→S2→C (yaw=0)
#   python3 r2s_chain.py pass2                     # yaw 对齐真机协议链 6 腿
#   python3 r2s_chain.py goals '[{"name":"D","x":1.0,"y":2.0,"yaw_deg":90}]'
#
# ── agent 用法（同一条路，多三个机器接口）──
#   python3 r2s_chain.py pass2 --boot --kill-session --json /tmp/out.json
#   退出码: 0=全绿  2=preflight 拒跑(含 boot 未就绪)  3=有腿失败
#   results.json 默认落 <bagroot>/results.json（人跑也会写，无害）
#   --boot 会自己起 launcher(tmux 7 窗) + 等 TF 就绪；不带 --boot 则要求栈已在
#
# 纪律内置:
#   - 域未设置直接拒跑（防占人类域 0）；显式 --domain 0 会大声警告后放行
#   - load 1min ≥ max-load(默认 20, 81 40 核) 拒跑（11-24 fastlio 饥饿教训）
#   - TF camera_init→body stamp 落后 /clock >2s 拒跑（假死/饥饿签名）
#   - /mujoco/ground_truth 无数据拒跑（栈没起/域不对）
#   - 链中某腿失败不中断后续腿（抢占自愈），最后汇总+退出码 3
#   - 全程结束自动 stand_mode（walk 零令蠕漂教训）
# ============================================================
import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

DEFAULT_WS = '/home/robot/hmj_projects/F1_r2s'   # 81; WSL 用 --ws /home/hmj/humanoid_ws/F1
DEFAULT_BAG_BASE = '/home/robot/bags'
DEFAULT_SESSION = 'f1_sim_nav_real'
DEFAULT_LOG = '/tmp/r2s_chain.log'
MAX_LOAD = 20.0          # 81 40 核机门槛（11-24 教训）
TF_LAG_GREEN = 0.5       # 绿：stamp 与 clock 差 ≤0.5s
TF_LAG_GATE = 2.0        # 闸：>2s 拒跑
GOAL_TIMEOUT = 200

TOPICS = ['/mujoco/ground_truth', '/cmd_vel', '/cmd_vel_limiter', '/Odometry', '/tf']

# P3 定版目标（真机同协议）; S*=spawn 备用点
PASS1 = [('A', -3.22, 3.24, 0.0), ('S1', 0.14, 0.51, 0.0),
         ('B', -2.68, -0.50, 0.0), ('S2', 0.14, 0.51, 0.0),
         ('C', 1.15, 3.23, 0.0)]
PASS2 = [('S0', 0.14, 0.51, 0.0), ('A', -3.22, 3.24, 180.0),
         ('S1', 0.14, 0.51, 0.0), ('B', -2.68, -0.50, -90.0),
         ('S2', 0.14, 0.51, 0.0), ('C', 1.15, 3.23, 0.0)]

_LOGFH = None


def log(msg):
    global _LOGFH
    line = '[%s] %s' % (time.strftime('%H:%M:%S'), msg)
    print(line, flush=True)
    if _LOGFH:
        _LOGFH.write(line + '\n')
        _LOGFH.flush()


# ---------- 纯函数（无 ROS 依赖，可单测） ----------

def parse_gt_text(text):
    """GT 快照文本 → [simt,x,y,z,roll,pitch,yaw,rtf,col,dist] 或 None。
    容忍 DDS 'A message was lost' 噪声行。"""
    m = re.search(r"array\('d',\s*\[([^\]]*)\]", text)
    if not m:
        return None
    try:
        vals = [float(v) for v in m.group(1).replace(',', ' ').split()]
    except ValueError:
        return None
    return vals if len(vals) >= 10 else None


def status_from_output(out, rc):
    """ros2 action send_goal 输出 → 状态串。"""
    st = None
    for m in re.finditer(r'Goal finished with status: (\w+)', out):
        st = m.group(1)
    if st:
        return st
    if rc == 124:
        return 'TIMEOUT'   # 客户端盲等/孤儿 goal 家族
    return 'UNKNOWN(rc=%d)' % rc


def wrap_deg(d):
    """归一到 [-180,180)（取 abs 后无符号歧义）。"""
    return (d + 180.0) % 360.0 - 180.0


def yaw_err_deg(goal_yaw_rad, actual_yaw_rad):
    return wrap_deg(math.degrees(goal_yaw_rad - actual_yaw_rad))


# ---------- ROS 封装 ----------

def build_env(ws):
    """source ROS + 工作区后抓完整 env（PATH/AUCTION 等），后续子进程全用它。"""
    setup = Path(ws) / 'navigation' / 'install' / 'setup.bash'
    if not setup.is_file():
        sys.exit('[FATAL] 找不到 %s\n'
                 '  81 默认工作区不对？手动指定: --ws /path/to/F1_r2s' % setup)
    cmd = ('source /opt/ros/humble/setup.bash && source %s && env -0' % setup)
    r = subprocess.run(['bash', '-c', cmd], capture_output=True)
    env = {}
    for chunk in r.stdout.split(b'\0'):
        if b'=' in chunk:
            k, _, v = chunk.partition(b'=')
            env[k.decode()] = v.decode()
    return env


def sh(env, args, timeout=None):
    return subprocess.run(args, env=env, capture_output=True,
                          text=True, timeout=timeout)


def snap(env, bagroot, tag):
    """GT 单帧快照：追加落盘并返回解析值。"""
    r = sh(env, ['timeout', '4', 'ros2', 'topic', 'echo',
                 '/mujoco/ground_truth', '--once', '--field', 'data'], timeout=8)
    f = bagroot / ('%s_gt.txt' % tag)
    with f.open('a') as fh:
        fh.write(r.stdout)
    return parse_gt_text(r.stdout)


def tf_stamp_of(env, parent, child, tries=5):
    """从 /tf 抓 parent→child 最新 stamp（epoch 秒, sim 域）。"""
    pat = re.compile(
        r'sec:\s*(\d+)\s+nanosec:\s*(\d+)\s+frame_id:\s*%s\s+child_frame_id:\s*%s'
        % (re.escape(parent), re.escape(child)))
    for _ in range(tries):
        r = sh(env, ['timeout', '5', 'ros2', 'topic', 'echo', '/tf', '--once'],
               timeout=8)
        m = pat.search(r.stdout)
        if m:
            return int(m.group(1)) + int(m.group(2)) * 1e-9
    return None


def clock_now(env):
    r = sh(env, ['timeout', '5', 'ros2', 'topic', 'echo', '/clock', '--once'],
           timeout=8)
    m = re.search(r'sec:\s*(\d+)\s+nanosec:\s*(\d+)', r.stdout)
    return int(m.group(1)) + int(m.group(2)) * 1e-9 if m else None


def nav2_ready(env):
    r = sh(env, ['timeout', '6', 'ros2', 'action', 'list'], timeout=9)
    return '/navigate_to_pose' in r.stdout


# ---------- preflight ----------

def preflight(env, max_load, boot_wait=False, need_nav2=True):
    """四门体检（load/TF/GT/nav2）。返回 (pass, detail)。
    boot_wait=True 时轮询等栈就绪；need_nav2=False 供采图等无 nav2 场景。"""
    load1 = os.getloadavg()[0]
    if load1 >= max_load:
        return False, {'load1': load1, 'reason': 'load 1min %.1f ≥ %.0f（11-24 fastlio 饥饿教训，拒跑）' % (load1, max_load)}
    detail = {'load1': load1}

    deadline = time.time() + (150 if boot_wait else 0)
    while True:
        clock = clock_now(env)
        body = tf_stamp_of(env, 'camera_init', 'body')
        gt = snap(env, Path('/tmp'), 'preflight_probe') if clock else None
        nav2 = nav2_ready(env) if need_nav2 else True
        if clock and body and gt and nav2:
            lag = clock - body
            detail.update(tf_lag_s=round(lag, 2), clock=round(clock, 1))
            if lag > TF_LAG_GATE:
                if time.time() < deadline:
                    log('  TF lag %.1fs 未就绪，等栈起（preflight 轮询）...' % lag)
                    time.sleep(5)
                    continue
                return False, dict(detail, reason='camera_init→body stamp 落后 /clock %.1fs（fastlio 假死签名）' % lag)
            detail['tf_green'] = lag <= TF_LAG_GREEN
            return True, detail
        if time.time() < deadline:
            miss = [n for n, v in (('clock', clock), ('tf', body), ('gt', gt),
                                   ('nav2', nav2)) if not v]
            log('  栈未就绪缺 %s，等 5s ...' % '/'.join(miss))
            time.sleep(5)
            continue
        miss = [n for n, v in (('clock', clock), ('tf', body), ('gt', gt),
                               ('nav2', nav2)) if not v]
        return False, dict(detail, reason='栈未就绪: 缺 %s（栈没起或域不对）' % '/'.join(miss))


# ---------- 腿执行 ----------

def next_bag_dir(bagroot, name):
    d = bagroot / name
    i = 1
    while d.exists():          # record -o 不能指向已存在目录（rosbag2 坑）
        i += 1
        d = bagroot / ('%s_%d' % (name, i))
    return d


def run_leg(env, bagroot, name, x, y, yaw_deg, goal_timeout):
    yaw = math.radians(yaw_deg)
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    log('LEG %s begin (%.2f, %.2f) yaw=%.0f°' % (name, x, y, yaw_deg))
    t0 = time.time()
    gt0 = snap(env, bagroot, name + '_start')

    bag = next_bag_dir(bagroot, name)
    rec = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(bag)] + TOPICS,
                           env=env, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL,
                           start_new_session=True)
    time.sleep(1)
    # walk_mode: 控制端 1s 节流, 发 2.5s 保险（防零令蠕漂）
    sh(env, ['timeout', '2.5', 'ros2', 'topic', 'pub', '-r', '5',
             '/walk_mode', 'std_msgs/msg/Float32', 'data: 0.0'])
    goal_yaml = ('{pose: {header: {frame_id: map}, pose: {position: '
                 '{x: %s, y: %s, z: 0.0}, orientation: {x: 0.0, y: 0.0, '
                 'z: %.6f, w: %.6f}}}}' % (x, y, qz, qw))
    resfile = bagroot / ('result_%s.txt' % name)
    try:
        r = subprocess.run(
            ['timeout', str(goal_timeout), 'ros2', 'action', 'send_goal',
             '/navigate_to_pose', 'nav2_msgs/action/NavigateToPose', goal_yaml],
            env=env, capture_output=True, text=True, timeout=goal_timeout + 15)
        out, rc = r.stdout + r.stderr, r.returncode
    except subprocess.TimeoutExpired as e:
        out, rc = (e.stdout or b''), 124
        if isinstance(out, bytes):
            out = out.decode(errors='replace')
    resfile.write_text(out)
    dur = time.time() - t0

    rec.send_signal(signal.SIGINT)
    try:
        rec.wait(timeout=15)
    except subprocess.TimeoutExpired:
        rec.kill()
    gt1 = snap(env, bagroot, name + '_end')

    status = status_from_output(out, rc)
    leg = {'name': name, 'goal': {'x': x, 'y': y, 'yaw_deg': yaw_deg},
           'status': status, 'ok': status == 'SUCCEEDED',
           'dur_s': round(dur, 1), 'bag': str(bag)}
    if gt0 and gt1:
        err = math.hypot(x - gt1[1], y - gt1[2])
        leg.update(landing=[round(gt1[1], 3), round(gt1[2], 3)],
                   err_m=round(err, 2),
                   yaw_resid_deg=round(abs(yaw_err_deg(yaw, gt1[6])), 1),
                   col=gt1[8], path_m=round(gt1[9] - gt0[9], 2),
                   sim_t=round(gt1[0], 1))
        log('LEG %s %s %.1fs err=%.2fm yawresid=%.1f° col=%d path=%.2fm'
            % (name, status, dur, err, leg['yaw_resid_deg'], gt1[8],
               leg['path_m']))
    else:
        leg['warn'] = 'GT 快照缺失(start=%s end=%s)' % (gt0 is not None, gt1 is not None)
        log('LEG %s %s %.1fs （GT 快照缺失，无指标）' % (name, status, dur))
    return leg


def stand_down(env):
    log('standing down (stand_mode 3s)')
    sh(env, ['timeout', '3', 'ros2', 'topic', 'pub', '-r', '5',
             '/stand_mode', 'std_msgs/msg/Float32', 'data: 0.0'])


# ---------- boot（--boot 模式: 自己起栈） ----------

def ensure_xvfb():
    r = subprocess.run(['pgrep', '-x', 'Xvfb'], capture_output=True, text=True)
    if r.returncode == 0:
        log('Xvfb 已在跑 pid=%s（沿用 :99 惯例）' % r.stdout.split())
        return None
    p = subprocess.Popen(['Xvfb', ':99', '-screen', '0', '1280x1024x24', '-ac'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
    log('Xvfb :99 已启动 pid=%d' % p.pid)
    time.sleep(2)
    return p.pid


def boot_stack(ws, session, domain, logpath):
    env = dict(os.environ, SESSION_NAME=session, ROS_DOMAIN_ID=str(domain),
               DISPLAY=':99', CAPTURE='0')
    ensure_xvfb()
    launcher = Path(ws) / 'scripts' / 'run_mujoco_nav_real.sh'
    fh = open(logpath, 'a')
    p = subprocess.Popen(['bash', str(launcher)], env=env, cwd=ws,
                         stdout=fh, stderr=subprocess.STDOUT,
                         start_new_session=True)
    log('launcher 已起 pid=%d session=%s 域=%d (pane: tmux attach -t %s)'
        % (p.pid, session, domain, session))
    return p


def kill_session(session):
    r = subprocess.run(['tmux', 'has-session', '-t', '=' + session],
                       capture_output=True)
    if r.returncode != 0:
        log('tmux session %s 不存在，跳过 kill' % session)
        return
    subprocess.run(['tmux', 'kill-session', '-t', '=' + session])
    log('tmux session %s 已收摊（收尾纪律: kill 前证据已落盘）' % session)


# ---------- 汇总 ----------

def print_table(legs):
    print('\n%-4s %-14s %-9s %6s %6s %7s %4s %7s' %
          ('leg', 'goal', 'status', 'dur', 'err', 'yawresid', 'col', 'path'))
    for l in legs:
        g = l['goal']
        print('%-4s (%5.2f,%5.2f) %-9s %5.1fs %5.2fm %6.1f° %4d %6.2fm' %
              (l['name'], g['x'], g['y'], l['status'], l.get('dur_s', 0),
               l.get('err_m', float('nan')), l.get('yaw_resid_deg', float('nan')),
               l.get('col', -1), l.get('path_m', float('nan'))))


def write_results(path, meta, pf, legs):
    ok_n = sum(1 for l in legs if l.get('ok'))
    doc = dict(meta, preflight=pf, legs=legs,
               summary={'total': len(legs), 'ok': ok_n,
                        'col_max': max([l.get('col', 0) for l in legs] or [0])})
    path.write_text(json.dumps(doc, ensure_ascii=False, indent=1))
    return doc


# ---------- main ----------

def main():
    ap = argparse.ArgumentParser(
        description='real2sim 链式导航跑（人/agent 双用）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='示例:\n'
               '  人:  python3 r2s_chain.py preflight && python3 r2s_chain.py pass2\n'
               '  人:  python3 r2s_chain.py leg A --x -3.22 --y 3.24 --yaw 180\n'
               '  agent: python3 r2s_chain.py pass2 --boot --kill-session --json /tmp/o.json\n'
               '退出码: 0=全绿 2=preflight 拒跑 3=有腿失败')
    ap.add_argument('cmd', choices=['preflight', 'leg', 'pass1', 'pass2', 'goals'])
    ap.add_argument('operand', nargs='?', help='leg 模式: 腿名; goals 模式: JSON 数组')
    ap.add_argument('--x', type=float, help='leg 模式: 目标 x')
    ap.add_argument('--y', type=float, help='leg 模式: 目标 y')
    ap.add_argument('--yaw', type=float, default=0.0, help='leg/goals: 朝向角(度, 默认 0)')
    ap.add_argument('--ws', default=DEFAULT_WS, help='工作区根（含 navigation/install）')
    ap.add_argument('--domain', type=int, default=None, help='ROS_DOMAIN_ID（必填或 env 已设）')
    ap.add_argument('--bagroot', default=None, help='录包根目录（默认 /home/robot/bags/r2s_<cmd>）')
    ap.add_argument('--json', default=None, help='results.json 输出路径（默认 <bagroot>/results.json）')
    ap.add_argument('--session', default=None, help='--boot: tmux session 名(默认 $SESSION_NAME 或 f1_sim_nav_real)')
    ap.add_argument('--boot', action='store_true', help='自己起 launcher 栈（默认要求栈已在）')
    ap.add_argument('--kill-session', action='store_true', help='跑完收摊 tmux session（--boot 才有意义）')
    ap.add_argument('--max-load', type=float, default=MAX_LOAD)
    ap.add_argument('--goal-timeout', type=float, default=GOAL_TIMEOUT)
    ap.add_argument('--log', default=DEFAULT_LOG)
    a = ap.parse_args()
    # session 解析: CLI > env(navsim env 隔离契约) > 人手默认; 仅 --boot 真正消费
    a.session = a.session or os.environ.get('SESSION_NAME') or DEFAULT_SESSION

    global _LOGFH
    _LOGFH = open(a.log, 'a', buffering=1)

    # 域纪律: 未设置拒跑（防占人类域 0）
    domain = a.domain if a.domain is not None else (
        int(os.environ['ROS_DOMAIN_ID']) if 'ROS_DOMAIN_ID' in os.environ else None)
    if domain is None:
        sys.exit('[FATAL] ROS_DOMAIN_ID 未设置 —— 拒跑（防占人类域 0）。\n'
                 '  手动: export ROS_DOMAIN_ID=44 或加 --domain 44')
    if domain == 0:
        log('⚠️  --domain 0 显式指定：这是人类默认域，AI 永不占（人自担）')

    env = build_env(a.ws)
    env['ROS_DOMAIN_ID'] = str(domain)
    log('=== r2s_chain %s 域=%d ws=%s log=%s ===' % (a.cmd, domain, a.ws, a.log))

    bagroot = Path(a.bagroot or (DEFAULT_BAG_BASE + '/r2s_' + a.cmd))
    bagroot.mkdir(parents=True, exist_ok=True)

    proc = None
    if a.boot:
        proc = boot_stack(a.ws, a.session, domain, a.log)
        pf_pass, pf = preflight(env, a.max_load, boot_wait=True,
                                need_nav2=(a.cmd != 'preflight'))
    else:
        pf_pass, pf = preflight(env, a.max_load,
                                need_nav2=(a.cmd != 'preflight'))

    if not pf_pass:
        log('PREFLIGHT FAIL: %s' % pf.get('reason'))
        if a.json:
            write_results(Path(a.json), {'cmd': a.cmd, 'domain': domain},
                          pf, [])
        if proc is not None:
            log('boot 后体检不过: tmux session %s 保留（pane 证据勿焚, 排查后手动收摊）' % a.session)
        sys.exit(2)
    log('PREFLIGHT PASS: %s' % pf)

    if a.cmd == 'preflight':
        sys.exit(0)

    if a.cmd == 'leg':
        if a.operand is None or a.x is None or a.y is None:
            sys.exit('[FATAL] leg 模式需 腿名 --x --y（--yaw 可选），如: leg A --x -3.22 --y 3.24')
        legs_spec = [(a.operand, a.x, a.y, a.yaw)]
    elif a.cmd == 'pass1':
        legs_spec = PASS1
    elif a.cmd == 'pass2':
        legs_spec = PASS2
    else:
        try:
            arr = json.loads(a.operand or '')
            legs_spec = [(d['name'], float(d['x']), float(d['y']),
                          float(d.get('yaw_deg', 0))) for d in arr]
        except Exception as e:
            sys.exit('[FATAL] goals JSON 解析失败: %s' % e)
    log('计划 %d 腿: %s' % (len(legs_spec), [s[0] for s in legs_spec]))

    legs = []
    for (name, x, y, yaw) in legs_spec:   # 链式: 腿间仅快照 ~6s, 零长空闲
        legs.append(run_leg(env, bagroot, name, x, y, yaw, a.goal_timeout))

    stand_down(env)
    snap(env, bagroot, 'final')
    print_table(legs)
    out_json = Path(a.json) if a.json else bagroot / 'results.json'
    write_results(out_json, {'cmd': a.cmd, 'domain': domain,
                             'session': a.session if a.boot else None,
                             'bagroot': str(bagroot)}, pf, legs)
    log('results → %s' % out_json)
    if a.boot and a.kill_session:
        kill_session(a.session)
    ok_n = sum(1 for l in legs if l.get('ok'))
    sys.exit(0 if ok_n == len(legs) else 3)


if __name__ == '__main__':
    main()
