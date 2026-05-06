%% MRAC Function Code [Derby]
function [e,RMS_error,u_mrac,y_mrac] = mrac_demcs(t,sys_d,sys_m,r,alpha,Q)
    % Discrete-Time Enhanced Minimal Control Synthesis MRAC
    % Based on Montanaro et al. (Paper 4)
    
    % Extract Plant Matrices
    [A,B,C,~] = ssdata(sys_d);
    
    % Extract Reference Model Matrices
    [Am,Bm,Cm,~] = ssdata(sys_m);
    
    % Ensure system is 2nd order (for Be dimensioning)
    n = size(A,1);
    Be = [zeros(n-1,1);1]; % Extract final column for canonical form 
    
    % Solve the discrete Lyapunov equation
    P = dlyap(Am',Q); 
    
    % Tuning Weights (based on Empirical Rules from Paper Section IV-C) 
    beta = 0.1*alpha;     
    alpha_I = alpha;
    beta_I = 0.1*alpha_I;
    rho = alpha;
    epsilon = 1e-3; % Smoothing factor for robust signum function 
    
    % Initialize Arrays
    N = length(t);
    u_mrac = zeros(1,N);
    y_mrac = zeros(1,N);
    e = zeros(1,N);
    
    % Initialize States and Integrators
    x = zeros(n,1);     % Plant state
    xm = zeros(n,1);    % Reference model state
    xI = zeros(n,1);    % Integral tracking error state 
    
    % Initialize Adaptive Gain Integrators
    Int_L  = zeros(1,n);
    Int_LR = 0;
    Int_LI = zeros(1,n);
    Int_LE = 0;
    
    for k = 1:N-1
        % Induce measurement noise
        noise = 0.001*randn();
        x_meas = x + [noise;0];

        % Tracking errors
        xe = xm - x_meas;   % State tracking error 
        ye = Be'*P*xe;      % Output tracking error 
        xI = xI + xe;     % Update integral state 
        
        % External output error
        y_mrac(k) = C*x_meas;
        e(k) = r(k) - y_mrac(k);
        
        % Adaptive Gains (Eq. 7a - 7d) 
        % (Using ye(k) as the standard digital approximation for ye(k+1))
        Int_L  = Int_L  + ye*x_meas';
        L = alpha*Int_L + beta*ye*x_meas';
        
        Int_LR = Int_LR + ye*r(k);
        LR = alpha*Int_LR + beta*ye*r(k);
        
        Int_LI = Int_LI + ye*xI';
        LI = alpha_I*Int_LI + beta_I*ye*xI';
        
        Int_LE = Int_LE + abs(ye);
        LE = rho*Int_LE;
        
        % Control Actions (Eq. 6a - 6c) 
        u_MCS = L*x_meas + LR*r(k);
        u_I = LI*xI;
        
        % Smoothed robust term (Eq. 31) 
        u_E = LE*(ye/(abs(ye) + epsilon)); 
        
        % Total control effort
        u_mrac(k) = u_MCS + u_I + u_E;
        
        % Simulate the next time step
        x = A*x + B*u_mrac(k);
        xm = Am*xm + Bm*r(k);
    end
    
    % Final step storage
    y_mrac(N) = C*x;
    e(N) = r(N) - y_mrac(N);
    
    % RMS error for analysis
    RMS_error = rms(e);
end