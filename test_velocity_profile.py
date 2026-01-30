#!/usr/bin/env python3
import math

def build_velocity_constrained_trajectory(path, max_wheel_accel=1.0, max_linear_vel=0.5, max_lateral_accel=1.0):
    n = len(path)
    if n < 2:
        return [], [], [], []

    dists = []
    tangents = []
    for i in range(n-1):
        x1,y1 = path[i]
        x2,y2 = path[i+1]
        dx = x2-x1; dy = y2-y1
        dist = math.hypot(dx,dy)
        dists.append(dist)
        if dist < 1e-6:
            tangents.append((1.0,0.0))
        else:
            tangents.append((dx/dist, dy/dist))

    kappas = [0.0]*n
    # cumulative distance
    s = [0.0]*n
    for i in range(1,n):
        s[i] = s[i-1] + max(1e-9, dists[i-1])
    total_s = s[-1]

    v_forward = [0.0]*n
    for i in range(n):
        v_f = math.sqrt(max(0.0, 2.0 * max_wheel_accel * s[i]))
        v_forward[i] = min(v_f, max_linear_vel)

    v_backward = [0.0]*n
    for i in range(n-1, -1, -1):
        dist_to_end = max(0.0, total_s - s[i])
        v_b = math.sqrt(max(0.0, 2.0 * max_wheel_accel * dist_to_end))
        v_backward[i] = min(v_b, max_linear_vel)

    velocities = [min(v_forward[i], v_backward[i]) for i in range(n)]

    # curvature limit (kappas all zero for n<3)
    for i in range(n):
        k = kappas[i]
        if k > 1e-9:
            v_curv = math.sqrt(max(1e-6, max_lateral_accel / k))
            velocities[i] = min(velocities[i], v_curv)

    def wheel_coeffs_for_tangent(tx, ty):
        s1 = (2.0 / 3.0) * ty
        s2 = (math.sqrt(3) / 2.0) * tx - 0.5 * ty
        s3 = -(math.sqrt(3) / 2.0) * tx - 0.5 * ty
        return [s1, s2, s3]

    # iterative per-wheel accel limiting
    for _pass in range(3):
        for i in range(n-1):
            v_i = velocities[i]
            v_j = velocities[i+1]
            dist = max(1e-6, dists[i])
            tx,ty = tangents[i]
            coeffs = wheel_coeffs_for_tangent(tx,ty)
            max_coeff = max(abs(c) for c in coeffs) if coeffs else 0.0
            if max_coeff < 1e-9:
                continue
            avg_speed = max(1e-3, 0.5*(v_i+v_j))
            dt = dist / avg_speed
            allowed_delta = max_wheel_accel * dt / max_coeff
            if v_j > v_i + allowed_delta:
                velocities[i+1] = v_i + allowed_delta
        for i in range(n-2, -1, -1):
            v_i = velocities[i]
            v_j = velocities[i+1]
            dist = max(1e-6, dists[i])
            tx,ty = tangents[i]
            coeffs = wheel_coeffs_for_tangent(tx,ty)
            max_coeff = max(abs(c) for c in coeffs) if coeffs else 0.0
            if max_coeff < 1e-9:
                continue
            avg_speed = max(1e-3, 0.5*(v_i+v_j))
            dt = dist / avg_speed
            allowed_delta = max_wheel_accel * dt / max_coeff
            if v_i > v_j + allowed_delta:
                velocities[i] = v_j + allowed_delta

    for i in range(n):
        velocities[i] = max(0.0, min(velocities[i], max_linear_vel))

    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    yaws = [0.0]*n
    return xs, ys, yaws, velocities


if __name__ == '__main__':
    # single waypoint
    start = (0.0, 0.0)
    goal = (-0.7, 1.5)
    path = [start, goal]
    # mimic node behavior: densify two-point paths before profiling
    if len(path) == 2:
        x0, y0 = path[0]
        x1, y1 = path[1]
        total_dist = math.hypot(x1 - x0, y1 - y0)
        spacing = 0.05
        n_samples = max(3, int(math.ceil(total_dist / spacing)) + 1)
        dense = []
        for i in range(n_samples):
            t = i / max(1, n_samples - 1)
            dense.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
        path = dense

    xs, ys, yaws, vels = build_velocity_constrained_trajectory(path)
    print('Path:', path)
    print('Distances:', [math.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1]) for i in range(len(path)-1)])
    print('Velocities:', vels)

    # dense resampled imitation
    # simulate 33 points along straight line
    dense = []
    for i in range(33):
        t = i/32.0
        x = start[0] + t*(goal[0]-start[0])
        y = start[1] + t*(goal[1]-start[1])
        dense.append((x,y))
    xs2, ys2, yaws2, vels2 = build_velocity_constrained_trajectory(dense)
    print('\nDense path length:', len(dense))
    print('Velocities (dense):', vels2)
    print('Vel max dense:', max(vels2) if vels2 else 0.0)
