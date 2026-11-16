"""Minimal MQTT 3.1.1 broker (QoS 0/1, retained messages, last will).

Only used so the demo runs without installing Mosquitto. If a broker is
already listening on the port, start() returns False and nothing happens.

    python3 -m confluence_sim.vendors.mini_broker [port]
"""
import socket
import socketserver
import struct
import sys
import threading


def _match(filt, topic):
    f, t = filt.split("/"), topic.split("/")
    for i, part in enumerate(f):
        if part == "#":
            return True
        if i >= len(t) or (part != "+" and part != t[i]):
            return False
    return len(f) == len(t)


def _enc_len(n):
    out = bytearray()
    while True:
        b = n % 128
        n //= 128
        out.append(b | (0x80 if n else 0))
        if not n:
            return bytes(out)


def _str(b, i):
    n = struct.unpack_from("!H", b, i)[0]
    return b[i + 2:i + 2 + n].decode(), i + 2 + n


def _bytes(b, i):
    n = struct.unpack_from("!H", b, i)[0]
    return b[i + 2:i + 2 + n], i + 2 + n


def _publish_packet(topic, payload, retain=False):
    t = topic.encode()
    body = struct.pack("!H", len(t)) + t + payload
    return bytes([0x30 | (1 if retain else 0)]) + _enc_len(len(body)) + body


class Broker:
    def __init__(self):
        self.lock = threading.Lock()
        self.clients = {}        # handler -> set(filters)
        self.retained = {}

    def route(self, topic, payload, retain):
        if retain:
            with self.lock:
                if payload:
                    self.retained[topic] = payload
                else:
                    self.retained.pop(topic, None)
        pkt = _publish_packet(topic, payload)
        with self.lock:
            targets = [h for h, fs in self.clients.items() if any(_match(f, topic) for f in fs)]
        for h in targets:
            h.send(pkt)


class _Handler(socketserver.BaseRequestHandler):
    broker = None

    def send(self, data):
        try:
            with self.wlock:
                self.request.sendall(data)
        except OSError:
            pass

    def _read_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.request.recv(n - len(buf))
            if not chunk:
                raise ConnectionError
            buf += chunk
        return buf

    def _read_packet(self):
        h = self._read_exact(1)[0]
        mult, length = 1, 0
        while True:
            b = self._read_exact(1)[0]
            length += (b & 0x7F) * mult
            mult *= 128
            if not b & 0x80:
                break
        return h, self._read_exact(length) if length else b""

    def handle(self):
        self.wlock = threading.Lock()
        br = self.broker
        will = None
        clean = False
        with br.lock:
            br.clients[self] = set()
        try:
            while True:
                h, body = self._read_packet()
                ptype = h >> 4
                if ptype == 1:                                   # CONNECT
                    _, i = _str(body, 0)
                    flags = body[i + 1]
                    i += 4
                    _, i = _str(body, i)                         # client id
                    if flags & 0x04:
                        wt, i = _str(body, i)
                        wm, i = _bytes(body, i)
                        will = (wt, wm, bool(flags & 0x20))
                    self.send(b"\x20\x02\x00\x00")
                elif ptype == 3:                                 # PUBLISH
                    qos = (h >> 1) & 3
                    topic, i = _str(body, 0)
                    if qos:
                        pid = body[i:i + 2]
                        i += 2
                        self.send(b"\x40\x02" + pid)
                    br.route(topic, body[i:], bool(h & 1))
                elif ptype == 8:                                 # SUBSCRIBE
                    pid, i, granted, new = body[:2], 2, bytearray(), []
                    while i < len(body):
                        f, i = _str(body, i)
                        i += 1
                        new.append(f)
                        granted.append(0)
                    with br.lock:
                        br.clients[self].update(new)
                        retained = [(t, p) for t, p in br.retained.items() if any(_match(f, t) for f in new)]
                    self.send(bytes([0x90]) + _enc_len(2 + len(granted)) + pid + bytes(granted))
                    for t, p in retained:
                        self.send(_publish_packet(t, p, retain=True))
                elif ptype == 10:                                # UNSUBSCRIBE
                    pid, i, gone = body[:2], 2, []
                    while i < len(body):
                        f, i = _str(body, i)
                        gone.append(f)
                    with br.lock:
                        br.clients[self].difference_update(gone)
                    self.send(b"\xb0\x02" + pid)
                elif ptype == 12:                                # PINGREQ
                    self.send(b"\xd0\x00")
                elif ptype == 14:                                # DISCONNECT
                    clean = True
                    break
        except (ConnectionError, OSError, IndexError, struct.error):
            pass
        finally:
            with br.lock:
                br.clients.pop(self, None)
            if will and not clean:
                br.route(will[0], will[1], will[2])


def port_in_use(port, host="127.0.0.1"):
    with socket.socket() as s:
        s.settimeout(0.3)
        return s.connect_ex((host, port)) == 0


def start(port=1883, host="0.0.0.0"):
    """Start in a background thread. Returns False if something already listens."""
    if port_in_use(port):
        return False
    _Handler.broker = Broker()
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    srv = socketserver.ThreadingTCPServer((host, port), _Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return True


def main_cli():
    p = int(sys.argv[1]) if len(sys.argv) > 1 else 1883
    if start(p):
        print(f"mini MQTT broker listening on :{p}")
        threading.Event().wait()
    else:
        print(f"port {p} already in use (a broker is probably running) - nothing to do")


if __name__ == "__main__":
    main_cli()
