%% This code is based on Prof. Yamamoto's code

clear;
close all;

% Set model parameter
a = 8.5879;
b = 36.081;

x0 = [1.5;
      0.0 ];

% I-PD controller params
A = [0,  1; 
     0, -a ];
B = [0;
     b ];
C = eye(2);
D = [0;
     0 ];
%------------------------
Q = eye(2);
R = 1;

% Continuous LQR
[K,S,X] = lqr(A,B,Q,R)

% Sampled-data LQR
h = 0.07;
t_sim = 0.001;
M = [A,B;zeros(size(B,2),size(A,1)), zeros(size(B,2))];
fun = @(t) expm(M'*t)*blkdiag(Q,R)*expm(M*t);
Pd = integral(fun,0,h,'ArrayValued',true);
Pd11 = Pd(1:2,1:2);
Pd12 = Pd(1:2,3);

Pd22 = Pd(3,3);
[Ad, Bd] = c2d(A,B,h);
[Ks,Ss,Xs] = dlqr(Ad,Bd,Pd11,Pd22,Pd12)

% Simulation
sim('ipd_lifting_sim.slx')

% Create figure
% L1 norm of state
% figure;
% plot(ans.tout,ans.continuous, 'k', 'LineWidth', 2.0); hold on;
% plot(ans.tout,ans.lifting, 'r-.', 'LineWidth', 2.0);
% plot(ans.tout,ans.nonlifting, 'b:', 'LineWidth', 2.0);
% xlabel('time [s]')
% ylabel('$|x(t)|$','Interpreter','latex')
% title('L1 norms of state')
% ylim([-0.1, 3.0])
% legend('Ideal response','Sampled-data design','Discretization of Kc')
% grid on;
% hold off;

% State trajectory
figure;
plot(ans.tout,ans.x_continuous, 'k', 'LineWidth', 2.0); hold on;
plot(ans.tout,ans.x_lifting, 'r-.', 'LineWidth', 2.0);
plot(ans.tout,ans.x_nonlifting, 'b:', 'LineWidth', 2.0);
xlabel('time [s]')
ylabel('$x_{1}(t)$','Interpreter','latex')
title('the trajectories of x1')
ylim([-0.1, 1.6])
legend('Ideal response','Sampled-data design','Discretization of Kc')
grid on;
hold off;

% Control input
figure;
plot(ans.tout,ans.u_continuous, 'k', 'LineWidth', 2.0); hold on;
plot(ans.tout,ans.u_lifting, 'r-.', 'LineWidth', 2.0);
plot(ans.tout,ans.u_nonlifting, 'b:', 'LineWidth', 2.0);
xlabel('time [s]')
ylabel('$x_{2}(t)$','Interpreter','latex')
title('the trajectories of x2')
ylim([-2.6 0.1])
legend({'Ideal response','Sampled-data design','Discretization of Kc'},'Location', 'southeast')
grid on;
hold off;
