%% ILC Function Code [Derby]
function [e,RMS_error,u_ff] = ilc(t,C_d,sys_d,trials,T,e0,alpha,fc)
    % Find the transfer function from the Feedforward Input to the Output
    sys_ff = sys_d(1,1)/(1 + C_d*sys_d(1,1));
    
    % Get discrete impulse response of this closed-loop path
    h_ff = impulse(sys_ff, t);
    
    % Construct the "Lifted" Lower-Triangular Toeplitz Matrix, G
    G_lifted = tril(toeplitz(h_ff));
    
    % Define the Learning Filter (L) from Paper [1]
    L = pinv(G_lifted); % pinv for numerical stability against tiny values
    
    % Define Robustness Filter (Q) from Paper [2]
    fs = 1/T;                           % Sampling frequency (1000 Hz)
    [b_f,a_f] = butter(2,fc/(fs/2));    % Filter coefficients
     
    % Initialize arrays
    N = length(t);
    u_ff = zeros(N,trials);         % Feedforward control effort for each trial
    e  = zeros(N,trials);           % Tracking error for each trial
    RMS_error = zeros(trials,1);    % RMS Error
    
    e(:,1) = e0; % Trial 1 is just baseline PID error (u_ff = 0)
    RMS_error(1) = rms(e0);
    
    % Iterative Learning Loop
    for k = 1:(trials-1)
        % ILC Update Law
        u_raw = u_ff(:,k) + alpha*(L*e(:,k));
        u_ff(:,k+1) = filtfilt(b_f,a_f,u_raw); % For "zero-phase" filtering (no time delay introduced)
        
        % Simulate the next trial's output
        e(:,k+1) = e0 - (G_lifted*u_ff(:,k+1));
        
        % Calculate RMS error to track performance
        RMS_error(k+1) = rms(e(:,k+1));
    end
end 