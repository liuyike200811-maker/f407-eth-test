#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
传感器平台 - 数据桥接与监控台 (Windows端)
纯桥接 + 监控，无控制、无3D。控制主体在板子+HMI。

三条数据路（详见 板子端串口协议-Win对接手册.md）：
  服务1 传感器→板子 (COM6/USART6, 双向):
     - 写: UDP:1399 收WT传感器帧 → 原样写COM6
     - 读: COM6 读裸 '0'/'1' 接收状态流 → 数 '1' 与已发送帧数比对(丢帧检测)
  服务2 板子→局域网 (micro-USB CDC, 读):
     - 读混流 → 0xAA55+32B数字孪生帧(校验后转发局域网) + 可读文本(→日志面板)

启动: python server.py   然后浏览器开 http://localhost:8080
"""

import json
import socket
import struct
import threading
import time
from collections import deque
from queue import Queue, Empty

import serial
import serial.tools.list_ports
from flask import Flask, Response, request, jsonify

app = Flask(__name__)

# ─────────────────────────── 全局状态 ───────────────────────────
CFG = {
    "com_sensor": "",          # COM6 / USART6，传感器双向口
    "sensor_udp_port": 1399,   # 监听传感器WiFi/UDP的端口
    "com_twin": "",            # micro-USB CDC，数字孪生帧+文本 混流口
    "lan_ip": "192.168.32.161",# 局域网数字孪生消费端 IP（借鉴旧UI默认，可改）
    "lan_port": 9000,          # 局域网数字孪生消费端 端口
    "forward_lan": False,      # 是否把解析出的孪生帧转发到局域网（可选，独立于监测）
    "baud": 115200,
}

ST = {
    "sensor": {"running": False, "sent": 0, "recv_ones": 0, "last_rx": 0.0, "err": "",
               "char_counts": {}},   # 板子回传诊断字符统计 {'1':n,'0':n,'L':n,...}
    # twin = 板子监测(micro-USB)：必备的板子状态源；forwarded 只在开了局域网转发时增长
    "twin":   {"running": False, "parsed": 0, "forwarded": 0, "bad": 0,
               "phase": -1, "last_frame": 0.0, "err": ""},
}

# 板子回传字符诊断：最近的原始字符流（滚动显示）
_sensor_recent = deque(maxlen=200)

# 诊断字符含义（板子固件约定）
CHAR_MEAN = {
    "1": "有效帧·校验全过", "0": "本周期无帧",
    "L": "帧长≠54(粘/断帧)", "H": "头≠WT",
    "T": "尾≠0D0A", "V": "版本≠13032",
}

_svc = {"sensor": None, "twin": None}   # 每个服务保存 {stop: Event, threads: [...], handles:[...]}
_log = deque(maxlen=600)
_subscribers = []                        # SSE 客户端队列列表
_sub_lock = threading.Lock()

PHASE_NAME = {0: "康复0", 1: "康复1", 2: "康复2", 3: "康复3", 4: "康复4",
              6: "传感器跟随", 99: "待机", -1: "—"}


def broadcast(event, data):
    msg = f"event: {event}\ndata: {json.dumps(data, ensure_ascii=False)}\n\n"
    with _sub_lock:
        dead = []
        for q in _subscribers:
            try:
                q.put_nowait(msg)
            except Exception:
                dead.append(q)
        for q in dead:
            _subscribers.remove(q)


def add_log(text, cls="info"):
    line = {"t": time.strftime("%H:%M:%S"), "msg": text, "cls": cls}
    _log.append(line)
    broadcast("log", line)


def classify(line):
    if any(k in line for k in ["★", "完成", "OK", "到位", "✓", "使能"]):
        return "ok"
    if any(k in line for k in ["报警", "失败", "ERROR", "错误", "!!"]):
        return "err"
    if any(k in line for k in ["警告", "WARN", "停止"]):
        return "warn"
    if any(k in line for k in ["mm", "RPM", "WKC", "从站", "Modbus"]):
        return "data"
    return "info"


# ─────────────────────────── 服务1: 传感器双向 ───────────────────────────
def sensor_write_loop(ser, sock, stop):
    """UDP:1399 收WT帧 → 原样写COM6"""
    while not stop.is_set():
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except Exception:
            break
        # 基础过滤：54字节、WT开头、0D0A结尾（真正校验交给板子）
        if len(data) != 54 or not data.startswith(b"WT") or not data.endswith(b"\r\n"):
            continue
        try:
            ser.write(data)
            ST["sensor"]["sent"] += 1
        except Exception as e:
            ST["sensor"]["err"] = f"写COM失败:{e}"
            break


def sensor_read_loop(ser, stop):
    """COM6 读板子回传的诊断字符流 → 数 '1' + 各诊断字符统计 + 保留最近原始流"""
    cc = ST["sensor"]["char_counts"]
    while not stop.is_set():
        try:
            chunk = ser.read(256)
        except Exception as e:
            ST["sensor"]["err"] = f"读COM失败:{e}"
            break
        if chunk:
            ST["sensor"]["recv_ones"] += chunk.count(b"1")
            ST["sensor"]["last_rx"] = time.time()
            for b in chunk:
                ch = chr(b) if 32 <= b < 127 else f"\\x{b:02x}"
                cc[ch] = cc.get(ch, 0) + 1
                _sensor_recent.append(ch)


def start_sensor():
    if ST["sensor"]["running"]:
        return False, "服务已在运行"
    if not CFG["com_sensor"]:
        return False, "未选择传感器COM口"
    stop = threading.Event()
    try:
        ser = serial.Serial(CFG["com_sensor"], CFG["baud"], timeout=0.3)
    except Exception as e:
        return False, f"打开{CFG['com_sensor']}失败: {e}"
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(0.5)
        sock.bind(("0.0.0.0", int(CFG["sensor_udp_port"])))
    except Exception as e:
        ser.close()
        return False, f"绑定UDP:{CFG['sensor_udp_port']}失败: {e}"

    ST["sensor"].update({"running": True, "sent": 0, "recv_ones": 0, "err": "", "char_counts": {}})
    _sensor_recent.clear()
    tw = threading.Thread(target=sensor_write_loop, args=(ser, sock, stop), daemon=True)
    tr = threading.Thread(target=sensor_read_loop, args=(ser, stop), daemon=True)
    tw.start()
    tr.start()
    _svc["sensor"] = {"stop": stop, "threads": [tw, tr], "handles": [ser, sock]}
    add_log(f"▶ 传感器转发启动: UDP:{CFG['sensor_udp_port']} → {CFG['com_sensor']}", "ok")
    return True, "ok"


# ─────────────────────────── 服务2: 板子→局域网 ───────────────────────────
def twin_loop(ser, out_sock, stop):
    buf = bytearray()
    text = bytearray()

    def flush_text():
        # 按 \n 切行，交给日志
        while b"\n" in text:
            i = text.index(b"\n")
            raw = bytes(text[:i]).rstrip(b"\r")
            del text[:i + 1]
            if raw:
                try:
                    s = raw.decode("utf-8", errors="replace")
                except Exception:
                    s = repr(raw)
                add_log(s, classify(s))

    while not stop.is_set():
        try:
            chunk = ser.read(512)
        except Exception as e:
            ST["twin"]["err"] = f"读micro-USB失败:{e}"
            break
        if not chunk:
            continue
        buf.extend(chunk)

        while True:
            i = buf.find(b"\xaa\x55")
            if i < 0:
                # 没有帧头：除末尾1字节(可能是半个帧头)外全当文本
                if len(buf) > 1:
                    text.extend(buf[:-1])
                    del buf[:-1]
                flush_text()
                break
            # 帧头前的字节 → 文本
            if i > 0:
                text.extend(buf[:i])
                del buf[:i]
                flush_text()
            # 此时 buf 以 0xAA55 开头
            if len(buf) < 32:
                break  # 帧没到齐
            frame = bytes(buf[:32])
            x = 0
            for b in frame[2:31]:
                x ^= b
            if x == frame[31]:
                # 真帧：始终解析(板子监测必备)；仅在开了局域网转发时才发出去
                ST["twin"]["parsed"] += 1
                ST["twin"]["phase"] = frame[6]
                ST["twin"]["last_frame"] = time.time()
                if CFG["forward_lan"]:
                    payload = frame[2:31]  # 剥头去尾的29字节载荷
                    try:
                        out_sock.sendto(payload, (CFG["lan_ip"], int(CFG["lan_port"])))
                        ST["twin"]["forwarded"] += 1
                    except Exception as e:
                        ST["twin"]["err"] = f"转发失败:{e}"
                del buf[:32]
            else:
                # 假同步：把首字节0xAA当文本，从下一位重找
                ST["twin"]["bad"] += 1
                text.extend(buf[:1])
                del buf[:1]


def start_twin():
    if ST["twin"]["running"]:
        return False, "服务已在运行"
    if not CFG["com_twin"]:
        return False, "未选择micro-USB COM口"
    stop = threading.Event()
    try:
        ser = serial.Serial(CFG["com_twin"], CFG["baud"], timeout=0.3)
    except Exception as e:
        return False, f"打开{CFG['com_twin']}失败: {e}"
    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        out_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except Exception:
        pass

    ST["twin"].update({"running": True, "parsed": 0, "forwarded": 0, "bad": 0, "err": ""})
    t = threading.Thread(target=twin_loop, args=(ser, out_sock, stop), daemon=True)
    t.start()
    _svc["twin"] = {"stop": stop, "threads": [t], "handles": [ser, out_sock]}
    fwd = f"（并转发→{CFG['lan_ip']}:{CFG['lan_port']}）" if CFG["forward_lan"] else ""
    add_log(f"▶ 板子监测启动: {CFG['com_twin']} {fwd}", "ok")
    return True, "ok"


def stop_service(name):
    svc = _svc.get(name)
    if not svc:
        ST[name]["running"] = False
        return
    svc["stop"].set()
    time.sleep(0.4)
    for h in svc["handles"]:
        try:
            h.close()
        except Exception:
            pass
    ST[name]["running"] = False
    _svc[name] = None
    add_log(f"■ {'传感器转发' if name=='sensor' else '板子监测'} 已停止", "warn")


# ─────────────────────────── HTTP / SSE ───────────────────────────
@app.route("/")
def index():
    return Response(HTML, mimetype="text/html")


@app.route("/api/ports")
def api_ports():
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append({"device": p.device, "desc": p.description or ""})
    return jsonify({"ports": ports})


@app.route("/api/config", methods=["POST"])
def api_config():
    d = request.get_json(force=True, silent=True) or {}
    for k in ("com_sensor", "com_twin", "lan_ip"):
        if k in d:
            CFG[k] = str(d[k])
    for k in ("sensor_udp_port", "lan_port"):
        if k in d:
            try:
                CFG[k] = int(d[k])
            except Exception:
                pass
    if "forward_lan" in d:
        CFG["forward_lan"] = bool(d["forward_lan"])
    return jsonify({"ok": True, "cfg": CFG})


@app.route("/api/forward_lan", methods=["POST"])
def api_forward_lan():
    """实时开/关局域网转发，不需要重启监测服务"""
    d = request.get_json(force=True, silent=True) or {}
    CFG["forward_lan"] = bool(d.get("on", False))
    if ST["twin"]["running"]:
        add_log(f"局域网转发 {'开启 → ' + CFG['lan_ip'] + ':' + str(CFG['lan_port']) if CFG['forward_lan'] else '关闭'}",
                "ok" if CFG["forward_lan"] else "warn")
    return jsonify({"ok": True, "forward_lan": CFG["forward_lan"]})


@app.route("/api/start/<name>", methods=["POST"])
def api_start(name):
    ok, msg = (start_sensor() if name == "sensor" else
               start_twin() if name == "twin" else (False, "未知服务"))
    if not ok:
        add_log(f"启动失败: {msg}", "err")
    return jsonify({"ok": ok, "msg": msg})


@app.route("/api/stop/<name>", methods=["POST"])
def api_stop(name):
    stop_service(name)
    return jsonify({"ok": True})


def stats_snapshot():
    now = time.time()
    s, t = ST["sensor"], ST["twin"]
    sensor_alive = s["running"] and (now - s["last_rx"] < 1.5) if s["last_rx"] else False
    twin_alive = t["running"] and (now - t["last_frame"] < 1.5) if t["last_frame"] else False
    return {
        "sensor": {"running": s["running"], "sent": s["sent"], "recv_ones": s["recv_ones"],
                   "alive": sensor_alive, "err": s["err"],
                   "char_counts": dict(s["char_counts"]),
                   "recent": "".join(_sensor_recent)},
        "twin": {"running": t["running"], "parsed": t["parsed"], "forwarded": t["forwarded"],
                 "bad": t["bad"], "phase": t["phase"],
                 "phase_name": PHASE_NAME.get(t["phase"], str(t["phase"])),
                 "alive": twin_alive, "err": t["err"], "forward_lan": CFG["forward_lan"]},
        "cfg": CFG,
    }


@app.route("/stream")
def stream():
    q = Queue(maxsize=200)
    with _sub_lock:
        _subscribers.append(q)
    # 先补发历史日志
    for line in list(_log):
        try:
            q.put_nowait(f"event: log\ndata: {json.dumps(line, ensure_ascii=False)}\n\n")
        except Exception:
            break

    def gen():
        try:
            while True:
                try:
                    yield q.get(timeout=1.0)
                except Empty:
                    yield ": ping\n\n"
        finally:
            with _sub_lock:
                if q in _subscribers:
                    _subscribers.remove(q)
    return Response(gen(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


def stats_ticker():
    while True:
        broadcast("stats", stats_snapshot())
        time.sleep(0.5)


# ─────────────────────────── 前端页面 ───────────────────────────
HTML = r"""<!DOCTYPE html><html lang="zh"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>传感器桥接监控台</title>
<style>
:root{
  --bg0:#1a1d23; --bg1:linear-gradient(160deg,#20242b 0%,#16191f 100%); --bg2:#242830; --bg3:#2d333e;
  --b1:rgba(255,255,255,.06); --b2:rgba(255,255,255,.1); --bl:rgba(255,255,255,.12);
  --ac:#00C896; --ac-glow:rgba(0,200,150,.4); --ac-dim:rgba(0,200,150,.12);
  --ok:#00D29D; --warn:#FFB020; --err:#F05050;
  --txt:#F8FAFC; --txt2:#94A3B8; --txt3:#475569;
  --r:6px; --r2:12px; --font-num:'Consolas',monospace; --font-ui:'Inter','Microsoft YaHei',system-ui,sans-serif;
  --shadow:0 16px 32px -8px rgba(0,0,0,.6),0 4px 12px rgba(0,0,0,.4);
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg0);color:var(--txt);font-family:var(--font-ui);min-height:100vh;
     background-image:radial-gradient(circle at 50% -20%,#20252e 0%,var(--bg0) 80%);padding-bottom:2rem}
::-webkit-scrollbar{width:6px;height:6px}::-webkit-scrollbar-thumb{background:var(--b2);border-radius:3px}
.hdr{height:56px;background:linear-gradient(180deg,#1d2128,#15181d);border-bottom:1px solid #000;
     box-shadow:0 2px 20px rgba(0,0,0,.5);display:flex;align-items:center;padding:0 1.2rem;gap:.75rem;position:sticky;top:0;z-index:20}
.hdr-t{font-size:.9rem;font-weight:700;letter-spacing:.15em;text-transform:uppercase}
.hdr-t small{display:block;font-size:.5rem;color:var(--ac);letter-spacing:.2em;text-shadow:0 0 8px var(--ac-glow)}
.hdr-conn{margin-left:auto;display:flex;align-items:center;gap:.4rem;font-size:.65rem;color:var(--txt2);
          padding:.3rem .7rem;border-radius:4px;background:var(--bg2);border:1px solid var(--b1)}
.wrap{max-width:1080px;margin:1.2rem auto;padding:0 1.2rem;display:flex;flex-direction:column;gap:1rem}
.card{background:var(--bg1);border:1px solid #111;border-top:1px solid var(--bl);border-radius:var(--r2);
      box-shadow:var(--shadow);overflow:hidden}
.card-h{display:flex;align-items:center;gap:.6rem;padding:.7rem 1rem;background:rgba(255,255,255,.02);
        border-bottom:1px solid #111;font-size:.72rem;font-weight:700;letter-spacing:.08em;text-transform:uppercase}
.card-h .ci{color:var(--ac);text-shadow:0 0 8px var(--ac-glow)}
.card-b{padding:1rem}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:1rem}
@media(max-width:720px){.grid2{grid-template-columns:1fr}}
.fi{font-size:.6rem;color:var(--txt2);letter-spacing:.05em;text-transform:uppercase;font-weight:600;margin-bottom:.25rem;display:block}
.finput,select.finput{width:100%;background:rgba(0,0,0,.3);border:1px solid var(--b1);border-radius:var(--r);
       color:var(--txt);padding:.45rem .6rem;font-size:.78rem;outline:none;margin-bottom:.6rem;font-family:var(--font-ui)}
.finput:focus{border-color:var(--ac);box-shadow:0 0 0 2px var(--ac-dim)}
.row{display:flex;gap:.6rem}.row>*{flex:1}
.dot{width:9px;height:9px;border-radius:50%;background:var(--txt3);flex-shrink:0}
.dot.run{background:var(--ok);box-shadow:0 0 8px var(--ok);animation:pulse 2s infinite}
.dot.err{background:var(--err);box-shadow:0 0 10px var(--err)}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}
.svc-top{display:flex;align-items:center;gap:.5rem;margin-bottom:.8rem}
.svc-state{font-size:.7rem;color:var(--txt2);font-weight:600}
.btn{padding:.5rem 1.1rem;border-radius:var(--r);border:none;font-size:.72rem;font-weight:800;letter-spacing:.08em;
     cursor:pointer;transition:.15s;text-transform:uppercase;margin-left:auto}
.btn-on{background:linear-gradient(135deg,#00E5AA,#009e76);color:#000;box-shadow:0 4px 12px var(--ac-glow)}
.btn-on:hover{transform:translateY(-1px)}
.btn-off{background:linear-gradient(135deg,#a02828,#5a1212);color:#fff}
.btn-off:hover{background:linear-gradient(135deg,#cc3333,#8a1a1a)}
.stat-row{display:grid;grid-template-columns:1fr 1fr;gap:.6rem;margin-top:.3rem}
.stat{background:rgba(0,0,0,.25);border:1px solid var(--b1);border-radius:var(--r);padding:.5rem .7rem;text-align:center}
.stat .sv{font-size:1.4rem;font-weight:700;color:var(--ac);font-family:var(--font-num);text-shadow:0 0 8px var(--ac-glow)}
.stat .sv.warn{color:var(--warn);text-shadow:0 0 8px rgba(255,176,32,.4)}
.stat .sl{font-size:.55rem;color:var(--txt3);text-transform:uppercase;font-weight:600;margin-top:.15rem}
.cmp{font-size:.7rem;color:var(--txt2);margin-top:.6rem;text-align:center;padding:.4rem;border-radius:var(--r);background:rgba(0,0,0,.2)}
.cmp b{font-family:var(--font-num)}
.cmp.good{color:var(--ok)}.cmp.bad{color:var(--warn)}
.phase-badge{display:inline-block;padding:.2rem .6rem;border-radius:20px;font-size:.7rem;font-weight:700;
             background:var(--ac-dim);color:var(--ac);border:1px solid var(--ac-glow)}
.log-h{display:flex;align-items:center;gap:.5rem}
.lcnt{margin-left:auto;font-size:.6rem;background:var(--bg3);padding:.15rem .5rem;border-radius:8px;color:var(--txt2);font-family:var(--font-num)}
.lclr{background:none;border:1px solid var(--b1);border-radius:var(--r);color:var(--txt3);font-size:.6rem;
      padding:.2rem .5rem;cursor:pointer}.lclr:hover{border-color:var(--warn);color:var(--warn)}
.log-body{height:240px;overflow-y:auto;padding:.6rem 1rem;background:rgba(0,0,0,.3);
          font-family:var(--font-num);font-size:.72rem}
.ll{line-height:1.7;display:flex;gap:.6rem;border-bottom:1px dashed rgba(255,255,255,.02);padding:.1rem 0}
.lt{color:var(--txt3);flex-shrink:0}
.lm.ok{color:var(--ok)}.lm.err{color:var(--err)}.lm.warn{color:var(--warn)}.lm.data{color:var(--ac)}.lm.info{color:var(--txt2)}
.hint{font-size:.58rem;color:var(--txt3);line-height:1.5;margin-top:-.3rem;margin-bottom:.5rem}
.fwd-box{margin-top:.7rem;padding:.7rem;border:1px solid var(--b1);border-radius:var(--r);background:rgba(0,0,0,.2)}
.fwd-toggle{display:flex;align-items:center;gap:.5rem;cursor:pointer;font-size:.72rem;color:var(--txt);font-weight:600}
.fwd-toggle input{width:15px;height:15px;accent-color:var(--ac);cursor:pointer}
.fwd-detail{display:flex;justify-content:space-between;font-size:.65rem;color:var(--txt2);margin:.45rem 0 .35rem}
.fwd-detail b{font-family:var(--font-num);color:var(--ac)}
.fwd-box.off{opacity:.55}
.diag-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:.6rem}
.dc{background:rgba(0,0,0,.25);border:1px solid var(--b1);border-radius:var(--r);padding:.5rem .7rem;
    display:flex;align-items:center;gap:.6rem}
.dc-ch{font-family:var(--font-num);font-size:1.5rem;font-weight:700;width:1.6rem;text-align:center;flex-shrink:0}
.dc-mid{flex:1;min-width:0}
.dc-mean{font-size:.6rem;color:var(--txt2);line-height:1.3}
.dc-n{font-family:var(--font-num);font-size:1.1rem;font-weight:700}
.dc.ok .dc-ch,.dc.ok .dc-n{color:var(--ok)}
.dc.zero .dc-ch,.dc.zero .dc-n{color:var(--txt3)}
.dc.bad .dc-ch,.dc.bad .dc-n{color:var(--err)}
.dc.hot{border-color:var(--err);box-shadow:0 0 10px rgba(240,80,80,.3)}
.diag-stream{font-family:var(--font-num);font-size:.85rem;letter-spacing:.12em;line-height:1.6;
    background:rgba(0,0,0,.35);border:1px solid var(--b1);border-radius:var(--r);padding:.6rem .7rem;
    word-break:break-all;color:var(--txt2);max-height:70px;overflow-y:auto}
.diag-stream .c1{color:var(--ok)}.diag-stream .c0{color:var(--txt3)}
.diag-stream .cx{color:var(--err);font-weight:700}
</style></head><body>
<div class="hdr">
  <div class="hdr-t">传感器桥接监控台<small>SENSOR BRIDGE · WINDOWS</small></div>
  <div class="hdr-conn"><span class="dot" id="sseDot"></span><span id="sseTxt">连接中…</span></div>
</div>
<div class="wrap">

  <!-- 配置 -->
  <div class="card"><div class="card-h"><span class="ci">⚙</span>端口与转发配置
     <button class="lclr" style="margin-left:auto" onclick="loadPorts()">↻ 刷新COM口</button></div>
  <div class="card-b grid2">
    <div>
      <label class="fi">传感器 COM 口 (USART6 双向)</label>
      <select class="finput" id="com_sensor"></select>
      <label class="fi">传感器 UDP 监听端口</label>
      <input class="finput" id="sensor_udp_port" value="1399">
    </div>
    <div>
      <label class="fi">micro-USB COM 口 (数字孪生帧+文本)</label>
      <select class="finput" id="com_twin"></select>
      <label class="fi">局域网数字孪生 目标 IP : 端口</label>
      <div class="row"><input class="finput" id="lan_ip" value="192.168.32.161">
           <input class="finput" id="lan_port" value="9000" style="flex:.4"></div>
    </div>
  </div></div>

  <div class="grid2">
    <!-- 服务1 -->
    <div class="card"><div class="card-h"><span class="ci">➊</span>传感器 → 板子 转发</div>
    <div class="card-b">
      <div class="svc-top"><span class="dot" id="dot_sensor"></span>
        <span class="svc-state" id="state_sensor">已停止</span>
        <button class="btn btn-on" id="btn_sensor" onclick="toggle('sensor')">启动</button></div>
      <div class="stat-row">
        <div class="stat"><div class="sv" id="s_sent">0</div><div class="sl">电脑已发送帧</div></div>
        <div class="stat"><div class="sv" id="s_ones">0</div><div class="sl">板子回报收到('1')</div></div>
      </div>
      <div class="cmp" id="cmp">—</div>
      <div class="hint">UDP:1399 收传感器帧原样写COM6；同时读板子回传的 0/1 流，数 '1' 与发送数比对判断丢帧。</div>
    </div></div>

    <!-- 服务2：板子监测（必备） -->
    <div class="card"><div class="card-h"><span class="ci">➋</span>板子监测 (micro-USB)</div>
    <div class="card-b">
      <div class="svc-top"><span class="dot" id="dot_twin"></span>
        <span class="svc-state" id="state_twin">已停止</span>
        <button class="btn btn-on" id="btn_twin" onclick="toggle('twin')">启动</button></div>
      <div class="stat-row">
        <div class="stat"><div class="sv" id="t_parsed">0</div><div class="sl">收到孪生帧</div></div>
        <div class="stat"><div class="sv" id="t_phasebig" style="font-size:1rem;line-height:2.05">—</div><div class="sl">板子当前模式</div></div>
      </div>
      <div class="hint">读micro-USB混流：文本→右侧板子状态日志（必备）；0xAA55+32B帧校验后解析。开此监测即可看板子状态，与下面局域网转发无关。</div>

      <div class="fwd-box">
        <label class="fwd-toggle"><input type="checkbox" id="fwd_chk" onchange="toggleForward()">
          <span>把孪生帧转发到局域网数字孪生设备</span></label>
        <div class="fwd-detail">
          <span>目标 <b id="fwd_target">192.168.32.161:9000</b></span>
          <span>本机已发出 <b id="t_fwd">0</b> 帧</span>
        </div>
        <div class="hint" style="margin:0">UDP单向发出，无对端反馈——计数仅表示本机发送量。丢弃(假同步) <b id="t_bad">0</b>。</div>
      </div>
    </div></div>
  </div>

  <!-- 板子回传字符诊断（排查用） -->
  <div class="card"><div class="card-h"><span class="ci">🔍</span>板子回传字符诊断 (USART6 回传流)
     <span style="margin-left:auto;font-size:.55rem;color:var(--txt3);font-weight:500;text-transform:none">
       固件把校验结果编码成ASCII字符回传，看哪个字符占多数即知卡在哪一关</span></div>
  <div class="card-b">
    <div class="diag-grid" id="diag_grid"></div>
    <div class="fi" style="margin-top:.6rem">最近回传原始字符流（滚动）</div>
    <div class="diag-stream" id="diag_stream">—</div>
  </div></div>

  <!-- 日志 -->
  <div class="card"><div class="card-h log-h"><span class="ci">▤</span>板子实时状态 / 日志
     <span class="lcnt" id="lcnt">0</span>
     <button class="lclr" onclick="clearLog()">清空</button></div>
  <div class="log-body" id="log"></div></div>

</div>
<script>
const $=id=>document.getElementById(id);
let logCount=0;

function fillSelect(sel,ports,keepDesc){
  const cur=sel.value;
  sel.innerHTML='<option value="">— 选择 —</option>'+ports.map(p=>
    `<option value="${p.device}">${p.device}　${p.desc}</option>`).join('');
  if(cur)sel.value=cur;
}
async function loadPorts(){
  const r=await fetch('/api/ports');const d=await r.json();
  fillSelect($('com_sensor'),d.ports);
  fillSelect($('com_twin'),d.ports);
}
async function saveCfg(){
  await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({com_sensor:$('com_sensor').value,com_twin:$('com_twin').value,
      sensor_udp_port:$('sensor_udp_port').value,lan_ip:$('lan_ip').value,lan_port:$('lan_port').value})});
}
async function toggle(name){
  const running=$('btn_'+name).classList.contains('btn-off');
  if(!running){await saveCfg();await fetch('/api/start/'+name,{method:'POST'});}
  else{await fetch('/api/stop/'+name,{method:'POST'});}
}
function setSvc(name,st){
  const dot=$('dot_'+name),btn=$('btn_'+name),lbl=$('state_'+name);
  if(st.running){
    btn.textContent='停止';btn.className='btn btn-off';
    if(st.alive){dot.className='dot run';lbl.textContent='运行中·数据流正常';}
    else{dot.className='dot run';lbl.textContent='运行中·等待数据';}
  }else{
    btn.textContent='启动';btn.className='btn btn-on';dot.className='dot';lbl.textContent='已停止';
  }
  if(st.err){dot.className='dot err';lbl.textContent=st.err;}
}
let fwdUserTouched=false;
function onStats(s){
  setSvc('sensor',s.sensor);setSvc('twin',s.twin);
  $('s_sent').textContent=s.sensor.sent;
  $('s_ones').textContent=s.sensor.recv_ones;
  $('t_parsed').textContent=s.twin.parsed;
  $('t_phasebig').textContent=s.twin.phase_name;
  $('t_fwd').textContent=s.twin.forwarded;
  $('t_bad').textContent=s.twin.bad;
  // 局域网转发勾选状态（跟随后端，除非用户正在操作）
  if(!fwdUserTouched){$('fwd_chk').checked=s.twin.forward_lan;}
  $('fwd_target').textContent=$('lan_ip').value+':'+$('lan_port').value;
  document.querySelector('.fwd-box').classList.toggle('off',!s.twin.forward_lan);
  // 板子回传字符诊断
  renderDiag(s.sensor.char_counts||{}, s.sensor.recent||'');
  // 传感器丢帧比对
  const sent=s.sensor.sent,ones=s.sensor.recv_ones,cmp=$('cmp');
  if(!s.sensor.running||sent===0){cmp.textContent='—';cmp.className='cmp';}
  else{const diff=sent-ones;
    if(Math.abs(diff)<=Math.max(3,sent*0.05)){cmp.innerHTML=`链路健康 ✓ 发<b>${sent}</b> / 板收<b>${ones}</b>`;cmp.className='cmp good';}
    else{cmp.innerHTML=`疑似丢帧 ⚠ 发<b>${sent}</b> / 板收<b>${ones}</b> (差${diff})`;cmp.className='cmp bad';}
  }
}
const DIAG_MEAN={'1':'有效帧·校验全过','0':'本周期无帧','L':'帧长≠54(粘/断帧)',
                 'H':'头≠WT','T':'尾≠0D0A','V':'版本≠13032'};
const DIAG_ORDER=['1','0','L','H','T','V'];
function renderDiag(counts,recent){
  // 已知诊断字符 + 出现过的其他字符
  const keys=DIAG_ORDER.slice();
  for(const k in counts){if(!keys.includes(k))keys.push(k);}
  // 找出错误类里最多的那个（用于高亮）
  let hotKey=null,hotN=0;
  for(const k of keys){if(k!=='1'&&k!=='0'&&(counts[k]||0)>hotN){hotN=counts[k];hotKey=k;}}
  $('diag_grid').innerHTML=keys.map(k=>{
    const n=counts[k]||0;
    const mean=DIAG_MEAN[k]||'未知字符';
    let cls=n===0?'zero':(k==='1'?'ok':(k==='0'?'':'bad'));
    if(k===hotKey&&n>0)cls+=' hot';
    return `<div class="dc ${cls}"><div class="dc-ch">${escapeHtml(k)}</div>
      <div class="dc-mid"><div class="dc-mean">${mean}</div><div class="dc-n">${n}</div></div></div>`;
  }).join('');
  // 原始流着色
  const box=$('diag_stream');
  if(!recent){box.textContent='—';}
  else{box.innerHTML=recent.split('').map(c=>{
    if(c==='1')return '<span class="c1">1</span>';
    if(c==='0')return '<span class="c0">0</span>';
    return `<span class="cx">${escapeHtml(c)}</span>`;
  }).join('');box.scrollTop=box.scrollHeight;}
}
async function toggleForward(){
  fwdUserTouched=true;
  await saveCfg();
  await fetch('/api/forward_lan',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({on:$('fwd_chk').checked})});
  setTimeout(()=>fwdUserTouched=false,1500);
}
function addLog(l){
  const el=document.createElement('div');el.className='ll';
  el.innerHTML=`<span class="lt">${l.t}</span><span class="lm ${l.cls}">${escapeHtml(l.msg)}</span>`;
  const box=$('log');const atBottom=box.scrollHeight-box.scrollTop-box.clientHeight<40;
  box.appendChild(el);logCount++;$('lcnt').textContent=logCount;
  while(box.children.length>800)box.removeChild(box.firstChild);
  if(atBottom)box.scrollTop=box.scrollHeight;
}
function clearLog(){$('log').innerHTML='';logCount=0;$('lcnt').textContent=0;}
function escapeHtml(s){return s.replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));}

const es=new EventSource('/stream');
es.addEventListener('open',()=>{$('sseDot').className='dot run';$('sseTxt').textContent='已连接';});
es.addEventListener('error',()=>{$('sseDot').className='dot err';$('sseTxt').textContent='断开·重连中';});
es.addEventListener('stats',e=>onStats(JSON.parse(e.data)));
es.addEventListener('log',e=>addLog(JSON.parse(e.data)));
loadPorts();
</script></body></html>"""


if __name__ == "__main__":
    threading.Thread(target=stats_ticker, daemon=True).start()
    print("=" * 50)
    print("  传感器桥接监控台")
    print("  浏览器打开:  http://localhost:8080")
    print("=" * 50)
    app.run(host="0.0.0.0", port=8080, threaded=True, debug=False)
