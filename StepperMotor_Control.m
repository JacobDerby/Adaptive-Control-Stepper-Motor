%% MAE544 Project Simulation [Derby]
close all
clear
clc

%% Define Parameters & Model
% These are based on a standard NEMA 23 Hybrid Stepper Motor,which is
% a commonly found motor used to drive an industrial gantry or
% manufacturing platform
J = 4.5e-5; % Rotor inertia [kg*m^2]
b = 1e-3;   % Viscous friction [N*m*s/rad]
Kt = 0.5;   % Torque constant [N*m/A]
T = 0.001;  % Sampling period [s] - matches Paper [3]

% Continuous Time State-Space Model
Ac = [0   1  ; 
      0 -b/J];
Bc = [0 Kt/J]';
Cc = [1,0];    % Track angular position
Dc = 0;

% Continuous-Time System
sys_c = ss(Ac,Bc,Cc,Dc);

% Convert to discrete-time w/ Zero-Order Hold [ZOH]
sys_d = c2d(sys_c,T,'zoh');

% Extract the Discrete State-Space Model
[Ad,Bd,Cd,Dd] = ssdata(sys_d);

%% Baseline PID Simulation
tf = 5; % Length of sim: 5 [s]

% Gains 
Kp = 1.5;  
Ki = 1;  
Kd = 0.1; 

% Create Discrete-Time PID 
C_d = pid(Kp,Ki,Kd,T,T);

% Define Systems
sys_cl = feedback(C_d*sys_d(1,1),1); % Unity feedback loop (1 instead of T)
sys_u = feedback(C_d,sys_d(1,1));

% Define Reference Trajectory
t = 0:T:tf;      % Time vector
r = sin(2*pi*t); % 1 [Hz] sine wave

% Simulate CL Response
[y_pid,t] = lsim(sys_cl,r,t);
u_PID = lsim(sys_u,r,t);

% Baseline Tracking Error (w/ Noise)
e0 = r' - y_pid + 0.001*randn(size(r'));

% Reference Trajectory Plot
figure
plot(t,r,'k','linewi',2)
ylabel('$\theta$ [rad]','Interpreter','latex')
xlabel('$t$ [s]','Interpreter','latex')
title(sprintf('Reference Trajectory $y_d$'),'Interpreter','latex')
axis equal
grid on

% Visualize
figure

% Tracking
subplot(211)
hold on
plot(t,r,'--m','DisplayName','Reference Signal ($\theta_k^*$)','linewi',2)
plot(t,y_pid,'k','DisplayName','Output Signal ($\theta_k$)','linewi',2)
ylabel('$\theta$ [rad]','Interpreter','latex')
title(sprintf('Baseline Tracking Performance'),'Interpreter','latex')
legend('Interpreter','latex')
grid on
hold off

% Error
subplot(212)
plot(t,e0,'r','linewi',2)
xlabel('$t$ [s]','Interpreter','latex')
ylabel('$e_k$ [rad]','Interpreter','latex')
title(sprintf('Baseline Tracking Error'),'Interpreter','latex')
grid on

%% ILC Implementation
% Parameters
trials = 30; % # of iterations (matches Paper [3])

disp('---- CHOOSE ILC PARAMETERS ----')
fc = input('Cutoff Frequency [Hz]: ');          % Cutoff frequency (50 Hz)]
alpha = input('        Learning Rate: ');       % Learning rate 

% Call ILC Function
[e_ilc,RMS_ilc,u_ff] = ilc(t,C_d,sys_d,trials,T,e0,alpha,fc);
u_ilc = u_PID + u_ff;

% Visualize
figure

% Error
subplot(211)
hold on
plot(t,e_ilc(:,1),'r','DisplayName','Trial 1 (Baseline)','linewi',2)
plot(t,e_ilc(:,5),'m','DisplayName','Trial 5','linewi',2)
plot(t,e_ilc(:,30),'k','DisplayName','Trial 30','linewi',2)
ylim([-0.01 0.01])
xlabel('$t$ [s]','Interpreter','latex')
ylabel('Tracking Error $e_k$ [rad]','Interpreter','latex')
title(sprintf(['Error Reduction Over ILC Trials ($f_c = %g$ [Hz],' ...
    '    $\\alpha = %g$)'],fc,alpha),'Interpreter','latex')
legend('Interpreter','latex')
grid on
hold off

% RMS Error
subplot(212)
semilogy(1:trials,RMS_ilc,'-ok','linewi',2,'MarkerFaceColor','k')
title(sprintf('Convergence of RMS Error'),'Interpreter','Latex')
xlabel('Trial \#','Interpreter','latex')
ylabel('RMS Error (Log Scale)','Interpreter','latex')
grid on

%% MRAC Implementation
% Desired performance characteristics 
ts = 1e-2;        % Settling time [s]
zeta = 0.8;       % Almost critically damped (little oscillation)
wn = 4/(zeta*ts); % Natural frequency [rad/s]

% Continuous-Time Reference Model in CCF
Amc = [  0         1     ; 
       -wn^2,-2*zeta*wn];
Bmc = [ 0   ; 
       wn^2]; % wn^2 ensures a unitary DC gain of 1
Cmc = [1,0]; 
Dmc = 0;

sys_mc = ss(Amc,Bmc,Cmc,Dmc);

% Discretize
sys_m = c2d(sys_mc,T,'zoh');

% Call MRAC Function
Q = eye(2)*T;      % Positive definite matrix for Lyapunov equation (scaled)
alpha_mrac = 0.2;  % Learning rate
[e_mrac,RMS_mrac,u_mrac,y_mrac] = mrac_demcs(t,sys_d,sys_m,r,alpha_mrac,Q);

% Visualize
figure

% Tracking
subplot(211)
hold on
plot(t,r,'--m','DisplayName','Reference Signal ($\theta_k^*$)','linewi',2)
plot(t,y_mrac,'k','DisplayName','Output Signal ($\theta_k$)','linewi',2)
ylabel('$\theta$ [rad]','Interpreter','latex')
title(sprintf('MRAC Tracking Performance'),'Interpreter','latex')
legend('Interpreter','latex')
ylim([-1 1])
grid on
hold off

% Error
subplot(212)
plot(t,e_mrac,'r','linewi',2)
xlabel('$t$ [s]','Interpreter','latex')
ylabel('$e_k$ [rad]','Interpreter','latex')
title(sprintf('MRAC Tracking Error'),'Interpreter','latex')
grid on

%% ILC vs MRAC Comparisons
figure

% Time-Domain Tracking Error
subplot(311)
hold on
plot(t,e_mrac,'m','DisplayName','MRAC','linewi',2)
plot(t,e_ilc(:,trials),'k','DisplayName','ILC (Trial 30)','linewi',2)
ylim([-0.04 0.04])
xlabel('$t$ [s]','Interpreter','latex')
ylabel('Tracking Error $e_k$ [rad]','Interpreter','latex')
title('Time-Domain Tracking Error','Interpreter','latex')
legend('Interpreter','latex')
grid on
hold off

% Time-Domain Control Effort
subplot(312)
hold on
plot(t,u_mrac,'b','DisplayName','MRAC','linewi',2)
plot(t,u_ilc(:,trials),'k','DisplayName','ILC (Trial 30)','linewi',2)
ylim([-1 1])
xlabel('$t$ [s]','Interpreter','latex')
ylabel('Control Effort [A]','Interpreter','latex')
title('Actuator Demand (Control Effort)','Interpreter','latex')
legend('Interpreter','latex','Location','best')
grid on
hold off

% RMS Error
subplot(313)
hold on
semilogy(1:trials,RMS_ilc,'-ok','linewi',2,'MarkerFaceColor','k', ...
    'DisplayName','ILC')
yline(RMS_mrac,'--b','linewi',2,'DisplayName','MRAC Performance Threshold')
ylim([0 0.025])
xlabel('Trial \#','Interpreter','latex')
ylabel('RMS Error (Log Scale)','Interpreter','latex')
title('Overall Performance Benchmark','Interpreter','latex')
legend('Interpreter','latex','Location','best')
grid on
hold off