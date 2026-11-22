import socket, sys, time
port = int(sys.argv[1]); dur = float(sys.argv[2])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('0.0.0.0', port)); s.settimeout(0.5)
t0 = time.time(); n = 0; sample = None
while time.time() - t0 < dur:
    try:
        d, _ = s.recvfrom(4096)
        n += 1
        if n == 80: sample = d.decode('ascii', 'ignore')
    except socket.timeout:
        pass
print('收到 %d 帧' % n)
if sample:
    v = sample.split()
    print('通道数 %d' % len(v))
    print('accel_x/z/y = %s / %s / %s' % (v[0], v[1], v[2]))
    print('rot_y/x/z   = %s / %s / %s' % (v[6], v[7], v[8]))
    print('speed_x/z   = %s / %s' % (v[3], v[4]))
    print('time        = %s' % v[24])
s.close()
