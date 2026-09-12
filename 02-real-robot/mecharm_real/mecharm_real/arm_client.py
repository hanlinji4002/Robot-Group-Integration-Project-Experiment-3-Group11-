# 【讲解】arm_server.py 的客户端：每次发一行 JSON、收一行 JSON。
# call() 负责连接、超时、断线重连一次；下面的 goto/gripper/soft/hold/record 等只是把命令名和参数包一下。
# 讲解要点：goto 是同步的，臂到位服务端才回复，所以套接字超时要比运动时间长；soft/hold 因固件回执慢给了 45 s。
"""arm_server.py 的客户端（纯标准库）。每行一条 JSON，阻塞等回复。"""
import json
import socket
import threading


class ArmError(RuntimeError):
    pass


# 【讲解】一条 TCP 连接对应一个实例；驱动里用两个实例（运动 / 状态）避免互相阻塞
class ArmClient:
    def __init__(self, host, port=9001, timeout=10.0):
        self.host, self.port, self.timeout = host, int(port), float(timeout)
        self._sock = None
        self._buf = b""
        self._lock = threading.Lock()

    def _connect(self):
        s = socket.create_connection((self.host, self.port), timeout=self.timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock, self._buf = s, b""

    def close(self):
        if self._sock:
            try:
                self._sock.close()
            finally:
                self._sock = None

    # 【讲解】发送一条命令并等回复；ok=false 时抛 ArmError，调用方据此判失败
    def call(self, cmd, sock_timeout=None, **kw):
        """发一条命令，返回服务端 JSON；ok 为 false 时抛 ArmError。
        sock_timeout 是本端套接字等待上限；协议里的 timeout 字段原样放在 kw 里传给服务端。"""
        req = dict(kw, cmd=cmd)
        line = (json.dumps(req) + "\n").encode("utf-8")
        with self._lock:
            for attempt in (1, 2):
                try:
                    if self._sock is None:
                        self._connect()
                    self._sock.settimeout(sock_timeout if sock_timeout is not None else self.timeout)
                    self._sock.sendall(line)
                    while b"\n" not in self._buf:
                        chunk = self._sock.recv(4096)
                        if not chunk:
                            raise ConnectionError("连接被服务端关闭")
                        self._buf += chunk
                    resp_line, self._buf = self._buf.split(b"\n", 1)
                    break
                except (OSError, ConnectionError) as e:
                    self.close()
                    if attempt == 2:
                        raise ArmError("与臂内服务通信失败: %s" % e)
        resp = json.loads(resp_line.decode("utf-8"))
        if not resp.get("ok", False):
            raise ArmError(resp.get("error", "未知错误"))
        return resp

    # 便捷封装
    def ping(self):
        return self.call("ping")

    def get_angles(self):
        return self.call("get_angles")

    # 【讲解】同步移动到六个角度，服务端到位（或超时）才返回角度与残差
    def goto(self, angles, speed, timeout=20.0, tol=1.5):
        """同步移动：服务端到位（或超时）才回复，套接字超时留出余量。"""
        req = {"angles": [float(a) for a in angles], "speed": int(speed),
               "timeout": float(timeout), "tol": float(tol)}
        return self.call("goto", sock_timeout=float(timeout) + 5.0, **req)

    def gripper(self, state):
        return self.call("gripper", sock_timeout=25.0, state=int(state))

    def gripper_value(self, value):
        return self.call("gripper_value", sock_timeout=8.0, value=int(value))

    def stop(self):
        return self.call("stop", sock_timeout=15.0)

    # 松/锁力矩每个舵机一条指令，新固件回执慢，臂内可能要十几秒，套接字超时放宽
    def soft(self):
        return self.call("soft", sock_timeout=45.0)

    def hold(self):
        return self.call("hold", sock_timeout=45.0)

    def record(self, name):
        return self.call("record", sock_timeout=20.0, name=name)

    def points(self):
        return self.call("points")["points"]
