import serial, time
def chk(b): return (~sum(b)) & 0xFF

class Bus:
    def __init__(self, port="/dev/ttyACM0", baud=1_000_000):
        self.ser = serial.Serial(port, baud, timeout=0, write_timeout=1)
        time.sleep(0.3); self.ser.reset_input_buffer()
    def _txn(self, pkt, nparams, timeout=0.2):
        sid = pkt[2]; need = 6 + nparams
        self.ser.reset_input_buffer()
        self.ser.write(pkt); self.ser.flush()
        buf = bytearray(); dl = time.time() + timeout
        while time.time() < dl:
            d = self.ser.read(64)
            if d: buf += d
            for i in range(0, max(0, len(buf) - need + 1)):
                if buf[i]==0xFF and buf[i+1]==0xFF and buf[i+2]==sid and buf[i+3]==nparams+2:
                    f = bytes(buf[i:i+need])
                    if chk(f[2:-1]) == f[-1]: return f
            time.sleep(0.002)
        return None
    def ping(self, sid, tries=8):
        p = bytes([0xFF,0xFF,sid,0x02,0x01]); p += bytes([chk(p[2:])])
        for _ in range(tries):
            if self._txn(p, 0) is not None: return True
        return False
    def read(self, sid, addr, ln, tries=8):
        p = bytes([0xFF,0xFF,sid,0x04,0x02,addr,ln]); p += bytes([chk(p[2:])])
        for _ in range(tries):
            r = self._txn(p, ln)
            if r: return int.from_bytes(r[5:5+ln], 'little')
        return None
    def write(self, sid, addr, val, ln, tries=8):
        data = int(val).to_bytes(ln, 'little')
        p = bytes([0xFF,0xFF,sid,3+ln,0x03,addr]) + data
        p = bytes([0xFF,0xFF]) + p[2:] + bytes([chk(p[2:])])
        for _ in range(tries):
            if self._txn(p, 0) is not None: return True
        return False
    def close(self): self.ser.close()
