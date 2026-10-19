#!/usr/bin/env python3
"""Check the GP12 at home clears a workpiece placement footprint.
Usage: check_home_clear.py X Y Z   (plate origin; part local bbox hard-coded
from workpiece_sample: x[-0.16,0] y[-0.26,0] z[0,0.115], yaw applied by caller)
"""
import sys
import numpy as np
import struct

def load_stl(path):
    with open(path, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        data = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
        tris = data[:, 12:48].copy().view('<f4').reshape(n, 3, 3)
    return tris.reshape(-1, 3)

def rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def T(R, p):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = p
    return M

q = dict(j1=0.0, j2=0.3, j3=-0.6, j4=0.0, j5=0.4, j6=0.0)  # home
W = {}
W['base_link'] = np.eye(4)
W['link1'] = W['base_link'] @ T(rx(0) @ np.eye(3), [0, 0, 0])
W['link1'] = T(np.eye(3), [0, 0, 0])  # j1 angle 0
W['link2'] = W['link1'] @ T(ry(q['j2']), [0.155, 0, 0.450])
W['link3'] = W['link2'] @ T(ry(-q['j3']), [0, 0, 0.614])
W['link4'] = W['link3'] @ T(rx(-q['j4']), [0.640, 0, 0.200])
W['link5'] = W['link4'] @ T(ry(-q['j5']), [0, 0, 0])
W['flange'] = W['link5'] @ T(rx(-q['j6']), [0, 0, 0])

# workpiece footprint in world (yaw 90 deg rotates local x->-y, local y->x)
ox, oy, oz = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
# local part bbox rel origin (empirical from imported edge extents):
# x [0, 0.26], y [-0.16, 0], z [0, 0.115]; yaw90: world x = ox - ly, y = oy + lx
xs = [ox - ly for ly in (0, -0.16)]
ys = [oy + lx for lx in (0, 0.26)]
box = dict(x=(min(xs), max(xs)), y=(min(ys), max(ys)), z=(oz, oz + 0.115))
print(f"part world box: x[{box['x'][0]:.3f},{box['x'][1]:.3f}] "
      f"y[{box['y'][0]:.3f},{box['y'][1]:.3f}] z[{box['z'][0]:.3f},{box['z'][1]:.3f}]")

worst = []
for name in ['link2', 'link3', 'link4', 'link5', 'flange']:
    v = load_stl(f'/home/xz/rosim_ws/src/rosim_description/meshes/gp12/{name}.stl')
    v = (W[name][:3, :3] @ v.T).T + W[name][:3, 3]
    m = ((v[:, 0] >= box['x'][0]) & (v[:, 0] <= box['x'][1]) &
         (v[:, 1] >= box['y'][0]) & (v[:, 1] <= box['y'][1]))
    if m.any():
        zmin = v[m][:, 2].min()
        margin = zmin - box['z'][1]
        worst.append((margin, name, zmin, int(m.sum())))
        print(f"  {name:6s} zmin over box = {zmin:.3f}  vs part top {box['z'][1]:.3f}"
              f"  -> margin {margin:+.3f}")
    else:
        print(f"  {name:6s} no mesh over box")
if worst:
    worst.sort()
    print("VERDICT:", "COLLISION" if worst[0][0] <= 0.01 else f"clear (worst margin {worst[0][0]:+.3f} m)")
