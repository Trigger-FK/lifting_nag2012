clear;
close all;

% System params
a = 8.5879;
b = 36.081;
x0 = 0.0; % initial state
xf = 1.5; % target state
w = 30; % from model matching
a_1 = 3; % from model matching
a_2 = 3; % from model matching

% I-PD controller params
kI = w^3 / b;
kP = (a_1*w^2) / b;
kD = (a_2*w-a) / b;

% Sampling param
h = 0.02;
t_sim = 0.001;

% Simulation
sim('ipd_controller.slx')

% Show the result
figure;
plot(ans.t, ans.yc, 'b', 'LineWidth', 1.5);hold on;
plot(ans.t, ans.yd, 'r', 'LineWidth', 1.5);
plot([0, 1.0], [xf, xf], 'k--', 'LineWidth', 0.5);
legend({'Continuous-time control', 'Descrete-time control', 'Target'}, 'Location', 'southwest');
title('System response of I-PD control');
ylim([-1 3])
xlim([0 1])
xlabel('time [s]');
ylabel('output');
grid on;
hold off;

figure;
plot(ans.t, ans.uc, 'b', 'LineWidth', 1.5);hold on;
plot(ans.t, ans.ud, 'r', 'LineWidth', 1.5);
legend({'Continuous-time control', 'Descrete-time control'}, 'Location', 'northeast');
title('Control inputs of I-PD control');
ylim([-5 15])
xlim([0 1])
xlabel('time [s]');
ylabel('input');
grid on;
hold off;