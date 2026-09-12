#!/usr/bin/env python3
# 【讲解】臂内 TCP 服务，跑在机械臂里的树莓派上（舵机接在它的串口上，所以只能在这里驱动）。
# 把 arm_common.py 的功能包成"每行一条 JSON"的协议：ping / get_angles / goto / gripper / stop / soft / hold / record / points。
# Jetson 上的 real_driver 与 teach 都通过它操作臂，自己不碰串口。
# 结构：FakeArm 假臂（--fake 时用，无臂联调）→ LockedArm 给串口加锁（运动中允许穿插读角度/急停）→ handle 分发命令 → serve_conn 收发行。
# 讲解要点：goto 到位才回复；gripper 发完读回开合值，没动就重发最多 3 次（这台固件偶尔丢帧）。
# 臂内 TCP 服务：把 arm_common.py 的功能包成「每行一条 JSON」的协议，
# 供 Jetson 上的 ROS 2 驱动（mecharm_real/real_driver）远程调用。
# 臂的限位、直发角度、示教存点全部沿用 arm_common，本文件只做转发。
#
# 用法（在臂内树莓派）:
#   python3 /home/er/arm_server.py                 # 监听 0.0.0.0:9001，接真实串口
#   python3 /home/er/arm_server.py --fake          # 不接串口，用假臂模拟（Mac/Jetson 联调用）
#   python3 /home/er/arm_server.py --port 9001 --points /home/er/taught_points.json
#
# 协议：客户端发一行 JSON {"cmd": "...", ...}，服务端回一行 JSON {"ok": true/false, ...}
#   ping                          -> {"ok":true,"fake":bool}
#   get_angles                    -> {"ok":true,"angles":[6],"gripper":v}
#   goto angles speed timeout tol -> {"ok":true,"angles":[6],"err":[6]}   到位后才返回
#   gripper state(0开/1合)        -> {"ok":true,"value":v}
#   gripper_value value(0-100)    -> {"ok":true,"value":v}
#   stop / soft / hold            -> {"ok":true}
#   record name                   -> {"ok":true,"name":..,"angles":[6]}  存到 taught_points.json
#   points                        -> {"ok":true,"points":{...}}
import argparse, json, os, socket, sys, threading, time, types

# 终端不是 UTF-8（如 nohup / systemd 下 LANG=C）时中文日志也不能把进程打死
def _utf8_stdout():
    import io, sys
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except AttributeError:  # Python < 3.7
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace", line_buffering=True)
_utf8_stdout()

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


# 【讲解】假臂：角度按速度线性逼近目标，夹爪/力矩只记状态；接口与 MechArm270 用到的子集一致
class FakeArm:
    """假臂：角度按速度线性逼近目标；夹爪、力矩只记状态。接口与 MechArm270 用到的子集一致。"""
    MAX_DPS = 120.0  # 官方规格最大关节速度 120°/s，speed 为百分比

    def __init__(self, *a, **k):
        self.q = [0.0] * 6
        self.target = None
        self.q0 = None
        self.t0 = 0.0
        self.speed = 15
        self.grip = 89
        self.torque = [True] * 6

    def _angle2int(self, a):
        return int(round(a * 100))

    def _mesg(self, genre, ints, speed, has_reply=False):
        self.target = [i / 100.0 for i in ints]
        self.q0 = list(self.q)
        self.t0 = time.time()
        self.speed = max(1, min(100, int(speed)))
        return 1

    def get_angles(self):
        if self.target is not None:
            step = self.MAX_DPS * self.speed / 100.0 * (time.time() - self.t0)
            done = True
            for i in range(6):
                d = self.target[i] - self.q0[i]
                if abs(d) <= step:
                    self.q[i] = self.target[i]
                else:
                    self.q[i] = self.q0[i] + step * (1 if d > 0 else -1)
                    done = False
            if done:
                self.target = None
        return [round(v, 2) for v in self.q]

    def get_coords(self):
        return [0.0] * 6

    def stop(self):
        self.target = None
        return 1

    def set_gripper_state(self, s, sp):
        self.grip = 45 if s else 89   # 假臂：合爪当作夹住 25mm 方块
        return 1

    def set_gripper_value(self, v, sp):
        self.grip = int(v)
        return 1

    def get_gripper_value(self):
        return self.grip

    def release_servo(self, j):
        self.torque[j - 1] = False
        return 1

    def focus_servo(self, j):
        self.torque[j - 1] = True
        return 1


# 【讲解】没装 pymycobot 的机器上注入桩模块，让 arm_common 能被 import（只在 --fake 时用）
def install_fake_pymycobot():
    """没装 pymycobot 的机器上（Mac）注入桩模块，让 arm_common 能被 import。"""
    class ProtocolCode:
        SEND_ANGLES = 0x22
    common = types.ModuleType("pymycobot.common")
    common.ProtocolCode = ProtocolCode
    pkg = types.ModuleType("pymycobot")
    pkg.common = common
    pkg.MechArm270 = FakeArm
    sys.modules["pymycobot"] = pkg
    sys.modules["pymycobot.common"] = common


# 【讲解】给 pymycobot 对象的每次方法调用加锁：串口不能并发，但允许一个连接在运动、另一个连接读角度
class LockedArm:
    """给 pymycobot 对象每个方法调用加锁：串口不能并发，但允许「运动中」穿插「读角度」。"""

    def __init__(self, mc):
        self._mc = mc
        self._lock = threading.Lock()

    def __getattr__(self, name):
        attr = getattr(self._mc, name)
        if not callable(attr):
            return attr

        def wrapped(*a, **k):
            with self._lock:
                return attr(*a, **k)
        return wrapped


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def err(msg):
    return {"ok": False, "error": msg}


# 【讲解】起服务：解析参数 → 连臂（或假臂）→ 监听端口 → 每个连接一个线程
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=9001)
    ap.add_argument("--fake", action="store_true", help="不接串口，用假臂模拟")
    ap.add_argument("--points", default=None, help="示教点文件，默认 arm_common.POINTS_FILE")
    args = ap.parse_args()

    if args.fake:
        install_fake_pymycobot()
    import arm_common as ac
    points_file = args.points or (os.path.join(HERE, "taught_points_fake.json") if args.fake else ac.POINTS_FILE)
    ac.POINTS_FILE = points_file

    mc = LockedArm(FakeArm() if args.fake else ac.connect())
    file_lock = threading.Lock()
    log("臂 %s 就绪，示教点文件 %s" % ("(假臂)" if args.fake else "(串口 /dev/ttyAMA0)", points_file))

    def safe(fn, default=None):
        try:
            return fn()
        except Exception:
            return default

    def load_points():
        if os.path.exists(points_file):
            with open(points_file) as f:
                return json.load(f)
        return {}

    def handle(req):
        cmd = req.get("cmd")
        if cmd == "ping":
            return {"ok": True, "fake": bool(args.fake), "t": time.time()}
        if cmd == "get_angles":
            a = ac.read6(mc.get_angles)
            return {"ok": a is not None, "angles": a, "gripper": safe(mc.get_gripper_value)}
        if cmd == "goto":
            tgt = [float(v) for v in req.get("angles", [])]
            if len(tgt) != 6:
                return err("需要 6 个角度")
            for k, (v, lo, hi) in enumerate(zip(tgt, ac.LIMIT_MIN, ac.LIMIT_MAX)):
                if not lo <= v <= hi:
                    return err("J%d=%.2f 超出限位 %d~%d，未发送" % (k + 1, v, lo, hi))
            speed = max(1, min(100, int(req.get("speed", 15))))
            timeout = float(req.get("timeout", 20))
            tol = float(req.get("tol", 1.5))
            try:
                a, e = ac.goto(mc, tgt, speed, timeout=timeout, tol=tol)
            except SystemExit as ex:      # arm_common.goto 超限时会 sys.exit，这里拦成错误回复
                return err(str(ex))
            if a is None:
                return err("运动后读不到角度")
            return {"ok": True, "angles": a, "err": e}
        if cmd == "gripper":
            # 新固件偶尔会丢夹爪帧：发完读回开合值，没动就重发，最多 3 次；
            # 判"动了"用相对量（比发指令前变化 >=10）或绝对量（合 <=40 / 开 >=60），
            # 这样夹住物体停在中间值也算合上了
            st = 1 if int(req.get("state", 0)) else 0
            before = safe(mc.get_gripper_value)
            def reached(v):
                if v is None:
                    return False
                if st:
                    return v <= 40 or (before is not None and v <= before - 10)
                return v >= 60 or (before is not None and v >= before + 10)
            tries = 0
            v = None
            while tries < 4:
                tries += 1
                ac.gripper(mc, st, ("合" if st else "开") + ("" if tries == 1 else "(重发%d)" % (tries - 1)))
                v = safe(mc.get_gripper_value)
                if reached(v):
                    break
            return {"ok": True, "value": v, "before": before, "reached": bool(reached(v)), "tries": tries}
        if cmd == "gripper_value":
            v = max(0, min(100, int(req.get("value", 50))))
            mc.set_gripper_value(v, 50)
            time.sleep(2)
            return {"ok": True, "value": safe(mc.get_gripper_value)}
        if cmd == "stop":
            mc.stop()
            return {"ok": True}
        if cmd == "soft":
            ac.soft(mc)
            return {"ok": True}
        if cmd == "hold":
            ac.hold(mc)
            return {"ok": True}
        if cmd == "record":
            name = str(req.get("name", "")).strip()
            if not name:
                return err("record 需要 name")
            a = ac.read6(mc.get_angles)
            if not a:
                return err("读不到角度")
            with file_lock:
                pts = load_points()
                pts[name] = a
                with open(points_file, "w") as f:
                    json.dump(pts, f, indent=1)
            return {"ok": True, "name": name, "angles": a}
        if cmd == "points":
            with file_lock:
                return {"ok": True, "points": load_points()}
        return err("未知命令 %r" % cmd)

    def serve_conn(conn, addr):
        log("连接 %s:%d" % addr)
        buf = b""
        try:
            with conn:
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if not line.strip():
                            continue
                        try:
                            req = json.loads(line.decode("utf-8"))
                            resp = handle(req)
                        except Exception as ex:
                            resp = err("%s: %s" % (type(ex).__name__, ex))
                        conn.sendall((json.dumps(resp, ensure_ascii=False) + "\n").encode("utf-8"))
        finally:
            log("断开 %s:%d" % addr)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(4)
    log("监听 %s:%d（Ctrl+C 退出）" % (args.host, args.port))
    try:
        while True:
            conn, addr = srv.accept()
            threading.Thread(target=serve_conn, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        log("退出，停止运动")
        safe(mc.stop)


if __name__ == "__main__":
    main()
