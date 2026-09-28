#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bag_scavenger v3 — 损坏 rosbag 打捞（模式定位版）。

不依赖索引、不假设记录头字段顺序（本组 bag 的字段顺序与标准 rosbag 不同）：
  1. 全文件搜 op=\\x05 / op=\\x07 的字段命中，从命中点【向前回溯】候选记录起点
     （header_len 合法 + 字段严格解析 + 内容校验），chunk 解压后块内严格顺序遍历；
  2. 块外裸消息记录按 b'\\x26\\x00\\x00\\x00\\t\\x00\\x00\\x00conn=' 特征收割
     （header_len=38: conn+op+time 三字段，总长与顺序无关）；
  3. 连接表齐全后统一反序列化，按时间排序重写成干净 bag。

用法: python3 bag_scavenger3.py <坏bag> <输出bag>
"""
import bz2
import struct
import sys
import time as _time

import rosbag
import rospy
from roslib.message import get_message_class

try:
    import lz4.frame as lz4f
except ImportError:
    lz4f = None

RAWMSG_SIG = b'\x26\x00\x00\x00\t\x00\x00\x00conn='
RAWMSG_HL = 38
DECOMP = {'none': lambda d: d, 'lz4': (lz4f.decompress if lz4f else None),
          'bz2': bz2.decompress}


def fix_time(t):
    """双写文件里一种写法把时间存成 [秒 u32][纳秒 u32]，重拼成 ns。"""
    lo = t & 0xFFFFFFFF
    hi = t >> 32
    if 1_600_000_000 <= lo <= 1_900_000_000 and hi < 1_000_000_000:
        return lo * 1_000_000_000 + hi
    return t


def parse_fields(buf):
    """严格解析头字段（键保留 bytes）；任何越界/未耗尽 → None。"""
    out, i, n = {}, 0, len(buf)
    while i < n:
        if i + 4 > n:
            return None
        (l,) = struct.unpack_from('<I', buf, i)
        i += 4
        if l > n - i:
            return None
        s = buf[i:i + l]
        i += l
        k, _, v = s.partition(b'=')
        out[k] = v
    return out


def find_record_start(raw, val_pos, op_byte, max_header=640):
    """给定 op 字段【值字节】位置 val_pos，向前回溯记录起点。

    返回 (start, hl, h)：记录起点、头长、已解析字段；失败 None。
    """
    op_field_start = val_pos - 3
    for s in range(max(0, op_field_start - max_header), op_field_start):
        if s + 8 > len(raw):
            continue
        (hl,) = struct.unpack_from('<I', raw, s)
        if not (8 <= hl <= max_header):
            continue
        hend = s + 4 + hl
        if not (s + 4 <= op_field_start and hend >= val_pos + 1):
            continue
        h = parse_fields(raw[s + 4:hend])
        if h is None or h.get(b'op') != op_byte:
            continue
        return s, hl, h
    return None


def walk_records(buf, conns, msgs, stats):
    """严格顺序遍历一段对齐可靠的记录区（chunk 内部）。"""
    pos, n = 0, len(buf)
    while pos + 8 <= n:
        (hl,) = struct.unpack_from('<I', buf, pos)
        if not (4 <= hl <= 640):
            stats['inner_bad'] += 1
            return
        h = parse_fields(buf[pos + 4:pos + 4 + hl])
        if h is None or b'op' not in h or len(h[b'op']) != 1:
            stats['inner_bad'] += 1
            return
        op = h[b'op'][0]
        (dl,) = struct.unpack_from('<I', buf, pos + 4 + hl)
        dstart = pos + 4 + hl + 4
        end = dstart + dl
        if end > n:
            stats['inner_bad'] += 1
            return
        if op == 7:
            cid = struct.unpack('<I', h[b'conn'])[0]
            ch = parse_fields(buf[dstart:end])
            if ch and b'topic' in ch and b'type' in ch:
                conns[cid] = (ch[b'topic'].decode('ascii', 'replace'),
                              ch[b'type'].decode('ascii', 'replace'),
                              ch.get(b'md5sum', b'?').decode('ascii', 'replace'))
                stats['conn'] += 1
        elif op == 2:
            cid = struct.unpack('<I', h[b'conn'])[0]
            t_ns = fix_time(struct.unpack('<Q', h[b'time'])[0])
            msgs.append((cid, t_ns, bytes(buf[dstart:end])))
            stats['msg'] += 1
        pos = end


def main():
    import argparse as _ap
    _p = _ap.ArgumentParser()
    _p.add_argument('src'); _p.add_argument('dst')
    _p.add_argument('--map', default='')
    _a = _p.parse_args()
    src, dst, args = _a.src, _a.dst, _a
    raw = open(src, 'rb').read()
    print('%s: %.0f MB' % (src, len(raw) / 1e6))
    try:
        (hl,) = struct.unpack_from('<I', raw, 13)
        hh = parse_fields(raw[17:17 + hl])
        print('bag头: chunk_count=%s conn_count=%s index_pos=%s' %
              (struct.unpack('<I', hh[b'chunk_count'])[0],
               struct.unpack('<I', hh[b'conn_count'])[0],
               struct.unpack('<Q', hh[b'index_pos'])[0]))
    except Exception as e:
        print('bag 头解析失败（不影响强扫）: %s: %s' % (type(e).__name__, e))

    conns, msgs = {}, []
    stats = {'chunk_ok': 0, 'chunk_fail': 0, 'inner_bad': 0, 'conn': 0,
             'conn_file': 0, 'msg': 0, 'raw_msg': 0, 'dup': 0}
    t0 = _time.time()

    # --- 1. conn 记录（文件级 + 块内都由后面的遍历收；这里先收文件级可达的）---
    pos = 0
    while True:
        v = raw.find(b'op=\x07', pos)          # 值字节位置
        if v < 0:
            break
        pos = v + 1
        r = find_record_start(raw, v, b'\x07', max_header=640)
        if r is None:
            continue
        s, hl, h = r
        if b'conn' not in h:
            continue
        try:
            (dl,) = struct.unpack_from('<I', raw, s + 4 + hl)
            dstart = s + 4 + hl + 4
            if dl > 4 << 20 or dstart + dl > len(raw):
                continue
            ch = parse_fields(raw[dstart:dstart + dl])
        except Exception:
            continue
        if ch and b'topic' in ch and b'type' in ch and b'md5sum' in ch:
            cid = struct.unpack('<I', h[b'conn'])[0]
            conns[cid] = (ch[b'topic'].decode('ascii', 'replace'),
                          ch[b'type'].decode('ascii', 'replace'),
                          ch[b'md5sum'].decode('ascii', 'replace'))
            stats['conn'] += 1
            stats['conn_file'] += 1

    # --- 2. chunk 记录 ---
    pos = 0
    while True:
        v = raw.find(b'op=\x05', pos)
        if v < 0:
            break
        pos = v + 1
        r = find_record_start(raw, v, b'\x05', max_header=64)
        if r is None:
            continue
        s, hl, h = r
        if b'size' not in h:
            continue
        try:
            comp = h.get(b'compression', b'none').decode('ascii', 'replace')
            size = struct.unpack('<I', h[b'size'])[0]
            if comp not in DECOMP or DECOMP[comp] is None or size > 200 << 20:
                continue
            (dl,) = struct.unpack_from('<I', raw, s + 4 + hl)
            dstart = s + 4 + hl + 4
            if dl > 200 << 20 or dstart + dl > len(raw):
                continue
            dec = DECOMP[comp](raw[dstart:dstart + dl])
        except Exception:
            continue
        if len(dec) != size:
            stats['chunk_fail'] += 1
            continue
        before = (stats['conn'], stats['msg'])
        walk_records(dec, conns, msgs, stats)
        if (stats['conn'], stats['msg']) == before:
            stats['chunk_fail'] += 1
        else:
            stats['chunk_ok'] += 1
            pos = dstart + dl               # 跳过数据区（防数据内伪命中）

    # --- 3. 块外裸消息记录 ---
    pos = 0
    while True:
        p = raw.find(RAWMSG_SIG, pos)
        if p < 0:
            break
        pos = p + 4
        h = parse_fields(raw[p + 4:p + 4 + RAWMSG_HL])
        if h is None or b'conn' not in h or b'time' not in h:
            continue
        (dl,) = struct.unpack_from('<I', raw, p + 4 + RAWMSG_HL)
        if dl > 8 << 20 or p + 4 + RAWMSG_HL + 4 + dl > len(raw):
            continue
        msgs.append((struct.unpack('<I', h[b'conn'])[0],
                     fix_time(struct.unpack('<Q', h[b'time'])[0]),
                     raw[p + 4 + RAWMSG_HL + 4: p + 4 + RAWMSG_HL + 4 + dl]))
        stats['msg'] += 1
        stats['raw_msg'] += 1

    print('扫描 %.1f s: %s' % (_time.time() - t0, stats))
    uniq, seen = [], set()
    for cid, t_ns, d in msgs:
        k = (cid, t_ns, len(d), hash(d) & 0xffffffff)
        if k in seen:
            stats['dup'] += 1
            continue
        seen.add(k)
        uniq.append((cid, t_ns, d))
    msgs = sorted(uniq, key=lambda m: m[1])
    # --- 连接表缺失时：按消息内容指纹反推 conn → topic ---
    manual = dict(kv.split('=', 1) for kv in args.map.split(',') if '=' in kv) if args.map else {}
    if manual and msgs:
        print('使用手工映射:', manual)
        conns.update({int(k): (v, '?', '?') for k, v in manual.items()})
    if not conns and msgs:
        print('连接表为空，启用内容指纹推断...')
        import math as _m
        from sensor_msgs.msg import PointCloud2 as _PC2
        CANDS = [
            ('nav_msgs/Odometry',
             lambda m: (abs(m.pose.pose.position.x) < 2000 and
                        abs(m.pose.pose.position.y) < 2000 and
                        abs(_qn(m.pose.pose.orientation) - 1) < 0.1)),
            ('sensor_msgs/PointCloud2',
             lambda m: (m.height == 1 and 100 <= m.width <= 200000 and
                        8 <= m.point_step <= 64 and m.is_dense and
                        any(f.name == 'x' for f in m.fields) and
                        len(m.data) == m.width * m.height * m.point_step)),
            ('tf2_msgs/TFMessage',
             lambda m: len(m.transforms) > 0 and
                       all(len(tr.child_frame_id) > 0 for tr in m.transforms)),
            ('sensor_msgs/NavSatFix',
             lambda m: 0 < m.latitude < 90 and 70 < m.longitude < 140),
            ('nav_msgs/Path',
             lambda m: len(m.poses) < 100000),
            ('thruster_manager/Thruster',
             lambda m: all(1100 <= v <= 1900 for v in
                           (m.left_thruster_pwm, m.right_thruster_pwm,
                            m.left_sideThruster_pwm, m.right_sideThruster_pwm))),
            ('std_msgs/String',
             lambda m: m.data.startswith('OK') or m.data.startswith('ERROR')),
            ('std_msgs/Float32MultiArray',
             lambda m: 8 <= len(m.data) <= 16),
            ('rl_local_planner/NavCmd',
             lambda m: -3 <= m.desired_speed <= 3 and -7 <= m.desired_course <= 7),
        ]

        def _qn(q):
            return _m.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)

        samples = {}
        for cid, t_ns, d in msgs:
            samples.setdefault(cid, []).append(d)
        ambs = []
        for cid in sorted(samples):
            got = None
            for ty, check in CANDS:
                cls = get_message_class(ty)
                if cls is None:
                    continue
                ok_n = 0
                for d in samples[cid][:5]:
                    try:
                        m = cls()
                        m.deserialize(bytes(d))
                        if check(m):
                            ok_n += 1
                    except Exception:
                        pass
                if ok_n >= max(1, len(samples[cid][:5]) // 2):
                    got = (ty, cls._md5sum if hasattr(cls, '_md5sum') else '?')
                    break
            if got is None:
                ambs.append(cid)
                print('  conn %3d: 无法识别' % cid)
            else:
                conns[cid] = ('FINGERPRINT', got[0], got[1])
        # 按消息尺寸/频率细分同类
        by_type = {}
        for cid, (tp, ty, md5) in conns.items():
            by_type.setdefault(ty, []).append(cid)
        for ty, cids in by_type.items():
            if ty == 'tf2_msgs/TFMessage' and len(cids) > 1:
                pass  # 多发布者正常
        # tiny Thruster 双连接区分：看哪条更常伴随非 1500 值
        tiny = by_type.get('thruster_manager/Thruster', [])
        if len(tiny) == 2:
            def activeness(cid):
                vs = []
                for cid2, t_ns, d in msgs:
                    if cid2 != cid:
                        continue
                    try:
                        m = get_message_class('thruster_manager/Thruster')()
                        m.deserialize(bytes(d))
                        vs.append(abs(m.left_thruster_pwm - 1500) +
                                  abs(m.right_thruster_pwm - 1500))
                    except Exception:
                        pass
                vs.sort(reverse=True)
                top = vs[:max(1, len(vs) // 5)]
                return sum(top) / max(1, len(top))
            def span(cid):
                ts = [t for c, t, d in msgs if c == cid]
                return (max(ts) - min(ts)) if ts else 0
            a, b = sorted(tiny, key=span, reverse=True)
            conns[a] = ('/controller', 'thruster_manager/Thruster', '?')      # 跨度宽=定时器常发
            conns[b] = ('/auto_thruster_pwm', 'thruster_manager/Thruster', '?')
            print('  双 Thruster 连接按时间跨度区分: conn %d→controller(跨度宽), conn %d→auto_thruster_pwm'
                  % (a, b))
        # 其余类型 → 默认话题名
        DEF_TOPIC = {
            'nav_msgs/Odometry': '/Odometry',
            'sensor_msgs/PointCloud2': '/cloud_registered_body',
            'tf2_msgs/TFMessage': '/tf',
            'sensor_msgs/NavSatFix': '/fix',
            'std_msgs/String': '/planner_status',
            'std_msgs/Float32MultiArray': '/guidance/rl_status',
            'rl_local_planner/NavCmd': '/guidance/rl_cmd',
        }
        for cid, (tp, ty, md5) in list(conns.items()):
            if tp == 'FINGERPRINT' and ty in DEF_TOPIC:
                conns[cid] = (DEF_TOPIC[ty], ty, md5)
        # Path 双连接：消息条数多的 = /LOS_path（连续发布），极少 = /waypoints_path
        pids = [cid for cid, (tp, ty, md5) in conns.items() if ty == 'nav_msgs/Path']
        if pids:
            cnt = {cid: sum(1 for c, _, _ in msgs if c == cid) for cid in pids}
            more = max(pids, key=lambda c: cnt[c])
            conns[more] = ('/LOS_path', 'nav_msgs/Path', '?')
            for c in pids:
                if c != more:
                    conns[c] = ('/waypoints_path' if cnt[c] <= 5 else '/LOS_path',
                                'nav_msgs/Path', '?')
        # 单个 tiny Thruster：默认当 /controller（定时器常发）
        tiny1 = [cid for cid, (tp, ty, md5) in conns.items()
                 if tp == 'FINGERPRINT' and ty == 'thruster_manager/Thruster']
        for c in tiny1:
            conns[c] = ('/controller', 'thruster_manager/Thruster', '?')
        for cid in sorted(conns):
            tp, ty, md5 = conns[cid]
            print('  conn %3d → %-38s %s' % (cid, tp, ty))

    print('连接表 %d 个:' % len(conns))
    for cid, (tp, ty, md5) in sorted(conns.items()):
        print('  conn %3d → %-38s %s' % (cid, tp, ty))

    TOPIC2TYPE = {
        '/Odometry': 'nav_msgs/Odometry',
        '/tf': 'tf2_msgs/TFMessage',
        '/cloud_registered_body': 'sensor_msgs/PointCloud2',
        '/fix': 'sensor_msgs/NavSatFix',
        '/ship/ship_info': 'tcp/ship_info',
        '/controller': 'thruster_manager/Thruster',
        '/auto_thruster_pwm': 'thruster_manager/Thruster',
        '/LOS_path': 'nav_msgs/Path',
        '/waypoints_path': 'nav_msgs/Path',
        '/guidance/rl_cmd': 'rl_local_planner/NavCmd',
        '/guidance/rl_status': 'std_msgs/Float32MultiArray',
        '/planner_status': 'std_msgs/String',
    }
    out = rosbag.Bag(dst, 'w')
    ok = bad_cls = bad_des = bad_time = 0
    ok_n, fail_n = {}, {}
    cls_cache = {}
    t0 = _time.time()
    for cid, t_ns, raw_msg in msgs:
        if not (1.735e18 <= t_ns <= 1.80e18):     # 2025-01 ~ 2027-01 之外 = 垃圾戳
            bad_time += 1
            continue
        info = conns.get(cid)
        if info is None:
            bad_cls += 1
            continue
        tp, ty, md5 = info
        if ty == '?':
            ty = TOPIC2TYPE.get(tp, '?')
            if ty == '?':
                bad_cls += 1
                continue
        cls = cls_cache.get(ty)
        if cls is None:
            cls = get_message_class(ty)
            cls_cache[ty] = cls
            if cls is None:
                print('!! 类型无法加载: %s (%s)' % (ty, tp))
        if cls is None:
            bad_cls += 1
            continue
        try:
            m = cls()
            m.deserialize(raw_msg)
            out.write(tp, m, rospy.Time(t_ns // 1000000000, t_ns % 1000000000))
            ok += 1
            ok_n[tp] = ok_n.get(tp, 0) + 1
        except Exception:
            bad_des += 1
            fail_n[tp] = fail_n.get(tp, 0) + 1
    out.close()
    print('写出 %s: 成功 %d 条, 无连接表 %d, 反序列化失败 %d, 垃圾时间 %d, 用时 %.1f s'
          % (dst, ok, bad_cls, bad_des, bad_time, _time.time() - t0))
    print('各话题: 成功 / 失败')
    for tp in sorted(set(ok_n) | set(fail_n)):
        print('   %-38s %6d / %6d' % (tp, ok_n.get(tp, 0), fail_n.get(tp, 0)))


if __name__ == '__main__':
    main()
