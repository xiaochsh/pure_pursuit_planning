clear; close all;

load routing.mat

veh.LB = 2.8; % vehicle wheel base, unit m
veh.LF = 0.95; % vehicle front distance, unit m
veh.LR = 1.05; % vehicle rear distance, unit m
veh.W = 1.8; % vehicle width, unit m
veh.WR = 0.3; % vehicle wheel radius, unit m
veh.ratio = 1.0 / 7; % ratio between front wheel and steer

vr = 10.0 / 3.6; % reference speed, unit m/s
ld0 = 0.3; % default look ahead distance, unit m
ds = 0.05; % default discrete distance, unit m
max_steer = 1.25 * pi; % max steer angle, unit rad
max_steer_rate = max_steer / 1.0; % max steer change rate, unit rad/s

num_pose = ceil(s(end) / ds) + 1;
ds = s(end) / (num_pose - 1);
input = zeros(num_pose - 1, 1);
state.s = zeros(num_pose, 1);
state.x = zeros(num_pose, 1);
state.y = zeros(num_pose, 1);
state.th = zeros(num_pose, 1);

start_index = 1;
for i = 1 : num_pose - 1
    index = find_look_ahead_point(xr, yr, start_index, ld0, state.x(i), state.y(i));
    % update ld
    tarx = xr(index);
    tary = yr(index);
    ld = sqrt((state.x(i) - tarx) ^ 2 + (state.y(i) - tary) ^ 2);
    % calculate distance error
    ed = (tary - state.y(i)) * cos(state.th(i)) - (tarx - state.x(i)) * sin(state.th(i));
    % calculate input
    wheel_ang = atan(2.0 * veh.LB * ed / ld ^ 2);
    if i > 1
        wheel_ang = min(max(input(i - 1) - max_steer_rate / vr * ds, wheel_ang), ...
            input(i - 1) + max_steer_rate / vr * ds);
    end
    input(i) = sign(wheel_ang) * min(max_steer * veh.ratio, abs(wheel_ang));
    % simulate vehicle state
    delta_theta = ds * tan(input(i)) / veh.LB;
    state.th(i + 1) = state.th(i) + delta_theta;
    state.x(i + 1) = state.x(i) + ds * cos(state.th(i) + 0.5 * delta_theta);
    state.y(i + 1) = state.y(i) + ds * sin(state.th(i) + 0.5 * delta_theta);
    state.s(i + 1) = state.s(i) + ds;
    start_index = index;
end

subplot(4, 2, 1:2:7)
axis equal;
plot(xr, yr);hold on; plot(state.x, state.y, 'o') ;grid on
xlabel('x / m'); ylabel('y / m')
legend('reference path', 'smoothed path')
subplot(422)
plot(s, xr);hold on;plot(state.s, state.x, '.'); grid on
xlabel('s / m'); ylabel('x / m')
legend('reference path', 'smoothed path')
subplot(424)
plot(s, yr);hold on;plot(state.s, state.y, '.'); grid on
xlabel('s / m'); ylabel('y / m')
legend('reference path', 'smoothed path')
subplot(426)
plot(s, thetar);hold on;plot(state.s, state.th, '.'); grid on
xlabel('s / m'); ylabel('\theta / rad')
legend('reference path', 'smoothed path')
subplot(428)
plot(state.s(1:end-1), input, '.'); grid on
xlabel('s / m'); ylabel('\delta / rad')
legend('input steer angle')

figure;
for i = 1 : length(input)
    clf;
    axis equal;
    hold on;grid on;
    xlabel('x / m'); ylabel('y / m')
    x = state.x(i + 1);
    y = state.y(i + 1);
    psi = state.th(i + 1);
    steer_ang = input(i) / veh.ratio;
    
    text(x - 9.5, y + 9.5, ['state.s: ' num2str(state.s(i + 1))], 'FontSize', 12);
    text(x - 9.5, y + 8.5, ['state.x: ' num2str(x)], 'FontSize', 12);
    text(x - 9.5, y + 7.5, ['state.y: ' num2str(y)], 'FontSize', 12);
    text(x - 9.5, y + 6.5, ['state.th: ' num2str(psi)], 'FontSize', 12);
    text(x - 9.5, y + 5.5, ['steering: ' num2str(steer_ang)], 'FontSize', 12);
    
    xlim(x + [-10, 10])
    ylim(y + [-10, 10])
    plot(xr, yr, 'r', 'LineWidth', 2)
    
    R = [cos(psi), -sin(psi); sin(psi), cos(psi)]; 
    FL = [x; y] + R * [veh.LB + veh.LF; 0.5 * veh.W];
    FR = [x; y] + R * [veh.LB + veh.LF; -0.5 * veh.W];
    RL = [x; y] + R * [-veh.LR; 0.5 * veh.W];
    RR = [x; y] + R * [-veh.LR; -0.5 * veh.W];

    vehx = [FL(1),FR(1),RR(1),RL(1),FL(1)];
    vehy = [FL(2),FR(2),RR(2),RL(2),FL(2)];
    plot(vehx, vehy, 'b', 'LineWidth', 0.5);

    FAC = [x; y] + R * [veh.LB; 0.0];
    plot([x, FAC(1)], [y, FAC(2)], 'k', 'LineWidth', 3)

    RAL = [x; y] + R * [0.0; 0.5 * veh.W - 0.2];
    RAR = [x; y] + R * [0.0; -0.5 * veh.W + 0.2];
    plot([RAL(1), RAR(1)], [RAL(2), RAR(2)], 'k', 'LineWidth', 3)

    FAL = [x; y] + R * [veh.LB; 0.5 * veh.W - 0.2];
    FAR = [x; y] + R * [veh.LB; -0.5 * veh.W + 0.2];
    plot([FAL(1), FAR(1)], [FAL(2), FAR(2)], 'k', 'LineWidth', 3)

    RWL1 = [x; y] + R * [veh.WR; 0.5 * veh.W - 0.2];
    RWL2 = [x; y] + R * [-veh.WR; 0.5 * veh.W - 0.2];
    plot([RWL1(1), RWL2(1)], [RWL1(2), RWL2(2)], 'k', 'LineWidth', 3)

    RWR1 = [x; y] + R * [veh.WR; -0.5 * veh.W + 0.2];
    RWR2 = [x; y] + R * [-veh.WR; -0.5 * veh.W + 0.2];
    plot([RWR1(1), RWR2(1)], [RWR1(2), RWR2(2)], 'k', 'LineWidth', 3)

    FWL1 = [x; y] + R * ([veh.LB; 0.5 * veh.W - 0.2] + veh.WR * [cos(input(i)); sin(input(i))]);
    FWL2 = [x; y] + R * ([veh.LB; 0.5 * veh.W - 0.2] - veh.WR * [cos(input(i)); sin(input(i))]);
    plot([FWL1(1), FWL2(1)], [FWL1(2), FWL2(2)], 'k', 'LineWidth', 3)

    FWR1 = [x; y] + R * ([veh.LB; -0.5 * veh.W + 0.2] + veh.WR * [cos(input(i)); sin(input(i))]);
    FWR2 = [x; y] + R * ([veh.LB; -0.5 * veh.W + 0.2] - veh.WR * [cos(input(i)); sin(input(i))]);
    plot([FWR1(1), FWR2(1)], [FWR1(2), FWR2(2)], 'k', 'LineWidth', 3)

    plot(x, y, 'cp') 
    pause(ds / vr)
end

function index = find_look_ahead_point(xr, yr, start_index, ld, x, y)
    % look up look ahead point
    index = min(start_index, length(xr));
    for idx = start_index : length(xr)
        if distance(x, y, xr(idx), yr(idx)) > ld
            index = idx;
            break;
        end
    end
end