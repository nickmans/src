% interpolate_trajectory.m
% Load trajectory from last_trajectory.json and save to traj.mat
% Trajectory is already at 0.01s dt (100Hz) - no interpolation needed

% Read JSON file
txt = fileread('last_trajectory.json');
D = jsondecode(txt);

% Extract trajectory data (already at 0.01s dt)
N = length(D.points);
dt = D.dt;

xs = [D.points.x].';
ys = [D.points.y].';
yaws = [D.points.yaw].';
vxs = [D.points.vx].';
vys = [D.points.vy].';

% Apply Savitzky-Golay filter for smooth accelerations
% This preserves the shape while ensuring C1 continuity (smooth derivatives)
window_size = 11;  % Must be odd, larger = smoother
poly_order = 3;    % Cubic polynomial

if N > window_size
    vxs = sgolayfilt(vxs, poly_order, window_size);
    vys = sgolayfilt(vys, poly_order, window_size);
end

% Additional pass: limit acceleration magnitude
max_accel = 1.5;  % m/s^2 - adjust based on your system
for i = 2:N
    dvx = vxs(i) - vxs(i-1);
    dvy = vys(i) - vys(i-1);
    
    accel_mag = sqrt(dvx^2 + dvy^2) / dt;
    
    if accel_mag > max_accel
        scale = max_accel * dt / sqrt(dvx^2 + dvy^2);
        vxs(i) = vxs(i-1) + dvx * scale;
        vys(i) = vys(i-1) + dvy * scale;
    end
end

% Build Nx5 trajectory matrix: [x y yaw vx vy]
traj = [xs ys yaws vxs vys];

% Save as .mat
save('traj.mat', 'traj', 'D');

fprintf('Loaded %d points at dt=%.3fs (100Hz)\n', N, dt);
fprintf('Applied Savitzky-Golay smoothing (window=%d, order=%d)\n', window_size, poly_order);
fprintf('Saved to traj.mat as Nx5 matrix [x y yaw vx vy]\n');


