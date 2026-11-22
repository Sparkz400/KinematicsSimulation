# Independent reference for the T-gantry's modes (pulleys held), from the physics, not from the crate's code.
# Coordinates: x (carriage on the frame), z (Z axis on its belt), q (frame sway), u (Z axis's lean at its COM).
import numpy as np

G = 9.80665
EA, P, mu = 5.0e5, 300.0, 0.1
s, sx, sz = 0.15, 1.0, 0.5
mc, mzaxis, mpay = 4.0, 5.0, 2.0
zcom, ztool = 0.3, 0.7
lat, tilt = 2.0e7, 1.2e4
mf, kf = 30.0, 7.4e5


def modes(x, z, frame=True, lean=True, mu=mu, g=G):
    mz = mzaxis + mpay
    # Geometric section lengths (no teeth): 2s + x + z, 4s + sx + 2sz - 2z, 2s + sx - x + z.
    geo = np.array([2 * s + x + z, 4 * s + sx + 2 * sz - 2 * z, 2 * s + sx - x + z])
    # Unstretched belt at rest: tensions T0, T0 + w / 2, T0, total = that at pretension at x = z = 0; w the Z axis's
    # weight and that of the belt's vertical runs, mu (4 s + 2 sz) g (their potential's rate in z).
    geo0 = np.array([2 * s, 4 * s + sx + 2 * sz, 2 * s + sx])
    total = (geo0 / (1 + P / EA)).sum()
    off = np.array([0.0, (mz + mu * (4 * s + 2 * sz)) * g / 2, 0.0])
    lo, hi = -1e4, 1e4
    for _ in range(200):
        t0 = 0.5 * (lo + hi)
        mat = geo / (1 + (t0 + off) / EA)
        if mat.sum() > total:
            lo = t0
        else:
            hi = t0
    T = t0 + off
    k = EA / mat
    gvec = [np.array([1, 1]), np.array([0, -2]), np.array([-1, 1])]
    idx = ['x', 'z'] + (['q'] if frame else []) + (['u'] if lean else [])
    n = len(idx)
    i = {c: j for j, c in enumerate(idx)}
    K = np.zeros((n, n))
    M = np.zeros((n, n))
    for ks, gv in zip(k, gvec):
        K[:2, :2] += ks * np.outer(gv, gv)
    # Kinetic energy: carriage at x' + q', Z axis at (x' + q' + u', z'), frame q', belt runs (table of `belt_mass`).
    def add(vec, w):
        v = np.zeros(n)
        for c, a in vec:
            if c in i:
                v[i[c]] += a
        M[:, :] += w * np.outer(v, v)
    add([('x', 1), ('q', 1)], mc)
    add([('x', 1), ('q', 1), ('u', 1)], mz)
    add([('z', 1)], mz)
    if frame:
        add([('q', 1)], mf)
        K[i['q'], i['q']] += kf
    # Belt runs, brute force over the eight runs: (length, X speed, Z speed) per unit of (x', z'), plus q' on X.
    L1, L2 = s + z, s + sz - z
    runs = [(L1, (1, 0), (0, 1)), (L1, (1, 0), (0, 1)), (L2, (1, 0), (2, 1)), (L2, (1, 0), (-2, 1)),
            (s + x, (1, 1), (0, 0)), (s + x, (-1, -1), (0, 0)), (s + sx - x, (-1, 1), (0, 0)), (s + sx - x, (1, -1), (0, 0))]
    # minus what is lumped on the pulleys (held here): mu L / 2 on each, turning at (vx +- vz) / r.
    for l, vxc, vzc in runs:
        vX = np.zeros(n); vZ = np.zeros(n)
        vX[0], vX[1] = vxc; vZ[0], vZ[1] = vzc
        if frame:
            vX[i['q']] += 1
        M += mu * l * (np.outer(vX, vX) + np.outer(vZ, vZ))
    Lb = sum(r[0] for r in runs)
    for sign in (1, -1):
        v = np.zeros(n); v[0], v[1] = 1, sign
        M -= 0.5 * mu * Lb * np.outer(v, v)
    if lean:
        h = zcom - z
        kl = 1 / (1 / lat + h * h / tilt)
        # The weight rises by h theta^2 / 2, theta = F h / tilt, F = kl u: pendulum c = kl^2 mz g h^3 / tilt^2.
        cw = kl ** 2 * mz * g * h ** 3 / tilt ** 2
        # Runs: lateral give and its product with the tilt, c = kl^2 (1 / (lat^2 |d|) + 2 sign(d) h / (lat tilt)).
        cr = lambda d: kl ** 2 * (1 / (lat ** 2 * abs(d)) + 2 * np.sign(d) * h / (lat * tilt))
        geo_k = (T[0] + T[2]) * cr(-(s + z)) + 2 * T[1] * cr(s + sz - z)
        K[i['u'], i['u']] += kl + cw + geo_k
    w2, vecs = np.linalg.eig(np.linalg.solve(M, K))
    o = np.argsort(w2.real); w2, vecs = w2.real[o], vecs.real[:, o]
    f = np.sqrt(w2) / (2 * np.pi)
    # Only modes that move the tool: x + q + tip u or z.
    out = []
    if lean:
        h, ht = zcom - z, ztool - z
        kl = 1 / (1 / lat + h * h / tilt)
        tip = kl * (1 / lat + h * ht / tilt)
    for fi, v in zip(f, vecs.T):
        tx = v[0] + (v[i['q']] if frame else 0) + (tip * v[i['u']] if lean else 0)
        if np.hypot(tx, v[1]) > 1e-9 * abs(v).max():
            out.append(fi)
    return out


if __name__ == '__main__':
    for mu_ in (0.1, 0.0):
        for pose in ([0.9, 0.05], [0.5, 0.25], [0.5, 0.05], [0.1, 0.45]):
            print('lossless belt', mu_, pose, ' '.join('%.6f' % f for f in modes(*pose, mu=mu_)))
    for pose in ([0.9, 0.05], [0.9, 0.15]):
        print('default frame off', pose, ' '.join('%.6f' % f for f in modes(*pose, frame=False)))
        print('default', pose, ' '.join('%.6f' % f for f in modes(*pose)))
    print('no lean no frame 0.5 0.25', ' '.join('%.6f' % f for f in modes(0.5, 0.25, frame=False, lean=False)))
