#!/usr/bin/env python3
"""P1 real2sim: 真机实验室 PCD → 体素占据 → 贪心 box 分解 → MJCF environment

路线（2026-11-20 拍板：体素/box 分解）:
  PCD(z-down, 地面 z≈1.25) → 物理高度 h = floor_z - z
  → h∈[hmin,hmax] 带内体素占据(0.05m, 对齐 2D 图栅格)
  → 贪心长方体合并(z→y→x 逐维扩张) → MJCF <geom type=box>
  → 栅格化回 2D 与 PGM occupied 对照(IoU/精确率/召回率)

坐标系约定(产出的 env xml):
  world xy ≡ robot_lab_20261030 pgm map xy (world 原点=map 原点)
  world z  = 物理高度(地面 z=0); 天花板按 PCD 实测 h 顶加装
用法:
  python3 pcd2mjcf_boxes.py [--voxel 0.05] [--min-voxels 8] [--min-count 1]
      [--out /tmp/real_lab_env.xml] [--report]
只读 PCD/PGM, 不动仓库其他文件。
"""
import argparse, math
import numpy as np

PCD_DEF = "/home/hmj/humanoid_ws/F1/navigation/localization/open3d_loc/maps/robot_lab_20261030.pcd"
PGM_DEF = "/home/hmj/humanoid_ws/F1/navigation/planning/humanoid_sim/maps/robot_lab_20261030.pgm"
ORIGIN_DEF = (-4.849999809265137, -1.7500000238418578)
RES_DEF = 0.05
# bag 实测自由区探针点位(P0 盘点: 4/4 单簇 1.17~1.31 → 地面标定用)
PROBE_SPOTS = [(0.14, 0.51), (0.0, 1.2), (1.5, 1.0), (-2.0, 1.5)]


def load_pcd(path):
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if line.startswith(b"DATA"):
                break
        pts = np.frombuffer(f.read(), dtype=np.float32).reshape(-1, 8)
    return pts[:, :3].astype(np.float64)


def calibrate_floor(xyz):
    """取已知自由区点位邻域 z 中位数 = 地面 z(z-down)"""
    zs = []
    for x, y in PROBE_SPOTS:
        d = np.linalg.norm(xyz[:, :2] - [x, y], axis=1)
        sel = xyz[d < 0.25]
        if len(sel):
            zs.append(np.median(sel[:, 2]))
    floor_z = float(np.median(zs))
    return floor_z, zs


def load_pgm(path):
    with open(path, "rb") as f:
        f.readline()
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    occ = data < (1 - 0.65) * 255  # ROS map_server occupied
    free = data > (1 - 0.196) * 255
    return occ, free, w, h


def voxelize(xyz_h, ox, oy, res, w, h, hmin, hmax, min_count):
    """PCD(物理高度 h) → 3D 占据 bool[w, h, nz]  (x 列/y 行对齐 pgm 栅格)"""
    nz = int(math.ceil((hmax - hmin) / res)) + 1
    cnt = np.zeros((w, h, nz), dtype=np.int32)
    cols = ((xyz_h[:, 0] - ox) / res).astype(int)
    rows = ((xyz_h[:, 1] - oy) / res).astype(int)
    ks = ((xyz_h[:, 2] - hmin) / res).astype(int)
    ok = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h) & (ks >= 0) & (ks < nz)
    np.add.at(cnt, (cols[ok], rows[ok], ks[ok]), 1)
    occ = cnt >= max(1, min_count)
    return occ, ok.sum(), len(xyz_h)


def fill_vertical_gaps_exact(occ, max_gap=3):
    """逐列扫描空洞长度, 仅闭合 ≤max_gap 的空洞段(精确版)"""
    w, hh, nz = occ.shape
    out = occ.copy()
    cols, rows = np.nonzero(occ.any(axis=2))
    for i, j in zip(cols, rows):
        col = occ[i, j]
        ks = np.nonzero(col)[0]
        if len(ks) < 2:
            continue
        for a, b in zip(ks[:-1], ks[1:]):
            gap = b - a - 1
            if 0 < gap <= max_gap:
                out[i, j, a + 1:b] = True
    return out


def complete_walls_from_pgm(occ, pgm_occ, pgm_free, res, hmin, hmax, wall_h=1.85, high=1.5):
    """墙体竖向补全(稀疏墙段): pgm occ 列且 3×3 邻域有高(>high)返回 → 补到 wall_h。
    依据: PCD 对远距/掠射墙面竖向采样稀疏(单列空洞大), 但 2D 手绘图按 nav 世界观
    把边界结构画成墙(真机 nav 亦然); pgm_free(门洞)列不补 → 不会封门;
    家具列邻域无高返回 → 不误抬。
    注: 触发与玻璃无关——实验室玻璃仅两处(玻璃正面柜子 + 一面带窗帘的墙上玻璃),
    补全列沿整面远墙成片分布与之无对应(2026-11-20 用户指认后修订)。"""
    w, hh, nz = occ.shape
    # pgm 掩码 [row_from_top, col] → 对齐 occ 的 [col, row_from_bottom]
    p_occ = np.flip(pgm_occ.T, axis=1)
    p_free = np.flip(pgm_free.T, axis=1)
    k_wall = min(nz - 1, int((wall_h - hmin) / res))
    k_high = max(0, int((high - hmin) / res))
    any_high = occ[:, :, k_high:].any(axis=2)
    # 3×3 膨胀(8 位移或)
    hi = any_high.copy()
    for sh in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
        src = np.roll(any_high, sh, axis=(0, 1))
        if sh[0] > 0: src[:sh[0], :] = False
        if sh[0] < 0: src[sh[0]:, :] = False
        if sh[1] > 0: src[:, :sh[1]] = False
        if sh[1] < 0: src[:, sh[1]:] = False
        hi |= src
    fill = p_occ & ~p_free & hi
    occ[fill, :k_wall + 1] = True
    return int(fill.sum())


def greedy_boxes(occ, res, hmin):
    """贪心长方体合并: 种子格→z 扩张→y 扩张→x 扩张, 返回 [(i0,i1, j0,j1, k0,k1)]"""
    w, hh, nz = occ.shape
    visited = np.zeros_like(occ, dtype=bool)
    boxes = []
    cols, rows, ks = np.nonzero(occ)
    for i, j, k in zip(cols, rows, ks):
        if visited[i, j, k]:
            continue
        # z 扩张
        k1 = k
        while k1 + 1 < nz and occ[i, j, k1 + 1] and not visited[i, j, k1 + 1]:
            k1 += 1
        # y 扩张(要求当前 z 段整列占据)
        j1 = j
        while j1 + 1 < hh and all(occ[i, jj, kk] and not visited[i, jj, kk]
                                  for jj in [j1 + 1] for kk in range(k, k1 + 1)):
            j1 += 1
        # x 扩张(要求整个 y×z 面占据)
        i1 = i
        while i1 + 1 < w and all(occ[ii, jj, kk] and not visited[ii, jj, kk]
                                 for ii in [i1 + 1]
                                 for jj in range(j, j1 + 1)
                                 for kk in range(k, k1 + 1)):
            i1 += 1
        visited[i:i1 + 1, j:j1 + 1, k:k1 + 1] = occ[i:i1 + 1, j:j1 + 1, k:k1 + 1] = False
        visited[i:i1 + 1, j:j1 + 1, k:k1 + 1] = True
        boxes.append((i, i1, j, j1, k, k1))
    return boxes


def box_geoms(boxes, ox, oy, res, hmin):
    """栅格 box → MJCF geom 参数(map xy 米制, z=物理高度)"""
    out = []
    for i0, i1, j0, j1, k0, k1 in boxes:
        cx = ox + (i0 + i1 + 1) * res / 2
        cy = oy + (j0 + j1 + 1) * res / 2
        cz = hmin + (k0 + k1 + 1) * res / 2
        sx = (i1 - i0 + 1) * res / 2
        sy = (j1 - j0 + 1) * res / 2
        sz = (k1 - k0 + 1) * res / 2
        nvox = (i1 - i0 + 1) * (j1 - j0 + 1) * (k1 - k0 + 1)
        out.append((cx, cy, cz, sx, sy, sz, nvox))
    return out


def rasterize_2d(geoms, w, hh, ox, oy, res, hmin, hmax):
    """box 组合 → 2D 占据; 返回 pgm 同构 [row_from_top, col]"""
    m = np.zeros((w, hh), dtype=bool)  # [col, row_from_bottom]
    for cx, cy, cz, sx, sy, sz, _ in geoms:
        zlo, zhi = cz - sz, cz + sz
        if zlo > hmax or zhi < hmin:
            continue
        i0 = max(0, int((cx - sx - ox) / res)); i1 = min(w - 1, int((cx + sx - ox) / res))
        j0 = max(0, int((cy - sy - oy) / res)); j1 = min(hh - 1, int((cy + sy - oy) / res))
        m[i0:i1 + 1, j0:j1 + 1] = True
    return np.flipud(m.T)  # → [row_from_top, col], 与 pgm_occ 同构


START = (0.14, 0.51)  # bag 实测常态站位
GOALS = [(-3.22, 3.24), (-2.68, -0.50), (1.15, 3.23)]  # 三次真机任务目标


def check_reachability(geoms, w, hh, ox, oy, res, hmin, hmax):
    """box 栅格上从起点 BFS 到三 goal 的自由区连通性(通道是否被堵死)"""
    occ2 = rasterize_2d(geoms, w, hh, ox, oy, res, 0.15, 1.5)  # 机器人身体带
    free = (~occ2).T  # → [col, row_from_top], 与 cell() 返回序一致
    free = np.ascontiguousarray(free)
    def cell(x, y):
        return int((x - ox) / res), hh - 1 - int((y - oy) / res)
    from collections import deque
    si, sj = cell(*START)
    seen = np.zeros_like(free, dtype=bool)
    if not free[si, sj]:
        return None, "起点 (0.14,0.51) 本身被 box 占据!"
    dq = deque([(si, sj)]); seen[si, sj] = True
    while dq:
        i, j = dq.popleft()
        for di, dj in ((1,0),(-1,0),(0,1),(0,-1)):
            a, b = i+di, j+dj
            if 0 <= a < w and 0 <= b < hh and free[a, b] and not seen[a, b]:
                seen[a, b] = True; dq.append((a, b))
    res_ = {}
    for g in GOALS:
        gi, gj = cell(*g)
        res_[g] = bool(seen[gi, gj])
    return res_, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pcd", default=PCD_DEF)
    ap.add_argument("--pgm", default=PGM_DEF)
    ap.add_argument("--origin", nargs=2, type=float, default=ORIGIN_DEF)
    ap.add_argument("--res", type=float, default=RES_DEF)
    ap.add_argument("--voxel", type=float, default=0.05, help="竖向体素分辨率(m)")
    ap.add_argument("--hmin", type=float, default=0.12)
    ap.add_argument("--hmax", type=float, default=2.0)
    ap.add_argument("--min-count", type=int, default=1, help="体素最少点数")
    ap.add_argument("--min-voxels", type=int, default=8, help="box 最少体素数(噪声过滤)")
    ap.add_argument("--gap", type=int, default=3, help="竖向闭合空洞上限(体素数); 扫描时从 8 起试")
    ap.add_argument("--no-gap-sweep", action="store_true", help="只扫 min_count×min_voxels(用 --gap)")
    ap.add_argument("--wall-h", type=float, default=1.85, help="墙体竖向补全目标高(m)")
    ap.add_argument("--no-wall-complete", action="store_true", help="关闭 pgm 引导的墙体补全")
    ap.add_argument("--out", default="/tmp/p0_real2sim/real_lab_env.xml")
    args = ap.parse_args()

    xyz = load_pcd(args.pcd)
    floor_z, probes = calibrate_floor(xyz)
    hphys = floor_z - xyz[:, 2]                      # 物理高度(地面=0, 向上为正)
    print("地面标定(z-down): %s → floor_z=%.3f" % (["%.3f" % p for p in probes], floor_z))
    print("物理高度 h 范围: %.2f ~ %.2f" % (hphys.min(), hphys.max()))

    pgm_occ, pgm_free, w, hh = load_pgm(args.pgm)
    ox, oy = args.origin
    res = args.res

    pts_h = np.column_stack([xyz[:, 0], xyz[:, 1], hphys])

    # ── 参数扫描: gap × min_count × min_voxels → 指标表 ──
    rows = []
    occ_cache = {}
    nfill_cache = {}
    for gap in (args.gap,) if args.no_gap_sweep else (8, 12, 16, 24):
        for mc in (1, 2):
            occ, used, total = voxelize(pts_h, ox, oy, args.voxel, w, hh,
                                        args.hmin, args.hmax, mc)
            occ = fill_vertical_gaps_exact(occ, max_gap=gap)
            nfill = 0 if args.no_wall_complete else \
                complete_walls_from_pgm(occ, pgm_occ, pgm_free, args.voxel,
                                        args.hmin, args.hmax, args.wall_h)
            boxes_all = greedy_boxes(occ, args.voxel, args.hmin)
            occ_cache[(gap, mc)] = boxes_all
            nfill_cache[(gap, mc)] = nfill
            for mv in (8, 20, 40):
                big = [b for b in boxes_all
                       if (b[1]-b[0]+1)*(b[3]-b[2]+1)*(b[5]-b[4]+1) >= mv]
                geoms = box_geoms(big, ox, oy, args.voxel, args.hmin)
                rast = rasterize_2d(geoms, w, hh, ox, oy, res, args.hmin, args.hmax)
                inter = (rast & pgm_occ).sum(); union = (rast | pgm_occ).sum()
                intr = (rast & pgm_free).sum()
                reach, err = check_reachability(geoms, w, hh, ox, oy, res, args.hmin, args.hmax)
                tall = sum(1 for b in big if (b[5]-b[4]+1)*args.voxel/2 > 0.8)
                ok = "-" if reach is None else ("Y" if all(reach.values()) else
                                                "N(" + ",".join(str(k) for k, v in reach.items() if not v) + ")")
                rows.append((gap, mc, mv, len(big), tall, inter/max(1, union), inter/max(1, rast.sum()),
                             inter/max(1, pgm_occ.sum()), intr, ok, err or ""))
    print("\n%-5s %-9s %-10s %-6s %-5s %-6s %-7s %-7s %-7s %-8s"
          % ("gap", "min_count", "min_voxels", "boxes", "高箱", "IoU", "精确率", "召回率", "free侵入", "三goal可达"))
    for r in rows:
        print("%-5d %-9d %-10d %-6d %-5d %-6.3f %-7.3f %-7.3f %-8d %s %s"
              % (r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10]))

    # 选型: 保真优先——可达为硬门槛, 偏好 mv=20(墙+大家具, 噪声块已滤);
    # mv=8 保留散噪, mv=40 只剩墙(家具全丢, 保真不足); gap 大→墙连续(高箱多)
    pref = [(1, 20), (1, 8), (2, 8), (1, 40), (2, 20), (2, 40)]
    cand = [r for r in rows if r[9] == "Y" and (r[1], r[2]) in pref]
    order = {p: n for n, p in enumerate(pref)}
    best = sorted(cand, key=lambda r: (order[(r[1], r[2])], -r[0]))[0] if cand \
        else sorted(rows, key=lambda r: r[3])[0]
    gap, mc, mv = best[0], best[1], best[2]
    big = [b for b in occ_cache[(gap, mc)]
           if (b[1]-b[0]+1)*(b[3]-b[2]+1)*(b[5]-b[4]+1) >= mv]
    geoms = box_geoms(big, ox, oy, args.voxel, args.hmin)
    rast = rasterize_2d(geoms, w, hh, ox, oy, res, args.hmin, args.hmax)
    inter = (rast & pgm_occ).sum(); union = (rast | pgm_occ).sum()
    prec = inter / max(1, rast.sum()); rec = inter / max(1, pgm_occ.sum())
    print("\n选型 gap=%d min_count=%d min_voxels=%d → %d boxes (pgm 补全列 %d)"
          % (gap, mc, mv, len(big), nfill_cache[(gap, mc)]))

    # 最大不匹配簇粗报(供目视核对现场)
    miss = pgm_occ & ~rast   # 图有 box 无
    extra = rast & ~pgm_occ  # box 有图无
    for name, m in (("pgm有box无(漏)", miss), ("box有pgm无(多)", extra)):
        if not m.any():
            print("  %s: 0 px" % name); continue
        ys, xs = np.nonzero(m)
        print("  %s: %d px, 重心示例: " % (name, len(m.nonzero()[0])) +
              ", ".join("(%.2f,%.2f)" % (ox + x * res, oy + (hh - 1 - y) * res)
                        for x, y in list(zip(xs, ys))[::max(1, len(xs)//5)][:6]))

    # 天花板高度 = h 物理上限(99.5 分位, 掐噪声)
    hceil = float(np.quantile(hphys, 0.995))

    mj = ['<?xml version="1.0" encoding="utf-8"?>',
          '<!-- real_lab_env.xml: 真机实验室 PCD→体素box 自动生成 (%s)' % __import__('datetime').date.today(),
          '     工具: scripts/pcd2mjcf_boxes.py  PCD: %s' % args.pcd,
          '     坐标: world xy ≡ pgm map xy (origin %.3f,%.3f res %.2f);' % (ox, oy, res),
          '           world z = 物理高度(地面 z=0, PCD 地面标定 z=%.3f z-down);' % floor_z,
          '     体素 %.2fm  h带 [%.2f,%.2f]  gap=%d  pgm补墙(%.2fm)=%d列  min_count=%d  min_voxels=%d  boxes=%d;' %
          (args.voxel, args.hmin, args.hmax, gap, args.wall_h, nfill_cache[(gap, mc)], mc, mv, len(big)),
          '     PGM 对照 IoU=%.3f 精确率=%.3f 召回率=%.3f -->' % (inter / max(1, union), prec, rec),
          '<mujoco model="real_lab">',
          '  <worldbody>',
          '    <light pos="0 0 3" dir="0 0 -1" directional="true"/>',
          '    <!-- 地面: 盖满 pgm 覆盖范围 -->',
          '    <geom name="floor" type="plane" size="%.2f %.2f 0.05" pos="%.3f %.3f 0" conaffinity="7" condim="3" friction="1" rgba="0.85 0.85 0.85 1"/>'
          % (w * res / 2 + 2, hh * res / 2 + 2, ox + w * res / 2, oy + hh * res / 2),
          '    <!-- 天花板: PCD 实测 h99.5=%.2fm -->' % hceil,
          '    <geom name="ceiling" type="plane" size="%.2f %.2f 0.05" pos="%.3f %.3f %.3f" rgba="0.75 0.75 0.78 1" contype="1" conaffinity="0"/>'
          % (w * res / 2 + 2, hh * res / 2 + 2, ox + w * res / 2, oy + hh * res / 2, hceil)]
    for n, (cx, cy, cz, sx, sy, sz, nv) in enumerate(geoms):
        tall = sz > 0.8
        mj.append('    <geom name="obs%03d" type="box" pos="%.3f %.3f %.3f" size="%.3f %.3f %.3f" conaffinity="7" rgba="%s"/>'
                  % (n, cx, cy, cz, sx, sy, sz,
                     "0.35 0.35 0.38 1" if tall else "0.55 0.45 0.3 1"))
    mj += ['  </worldbody>', '</mujoco>']
    with open(args.out, "w") as f:
        f.write("\n".join(mj) + "\n")
    print("MJCF → %s  (boxes=%d, 天花板 %.2fm)" % (args.out, len(big), hceil))


if __name__ == "__main__":
    main()
