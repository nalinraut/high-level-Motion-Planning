from hokuyolx import HokuyoLX           # For UST10LX

laser = HokuyoLX(addr=('192.168.0.10', 10940))

timestamps, scan = laser.get_filtered_dist()

scan = scan.tolist()

keys = [0]*len(scan)

for i in range(len(scan)):
    keys[i] = scan[i][0]

print keys
