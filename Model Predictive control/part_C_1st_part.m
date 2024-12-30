clear all;
clc;

% Tank and system parameters
A = [28; 32; 28; 32];                   % Cross-sectional areas of the tanks (cm^2)
a = [0.071; 0.057; 0.071; 0.057];       % Outlet areas (cm^2)
kc = 1;                                 % Sensor gain (V/cm)
g = 981;                                % Gravitational acceleration (cm/s^2)

% Valve parameters
gamma1 = 0.7; gamma2 = 0.6;  

% Pump parameters
k1 = 3.33; k2 = 3.35;  

% Initial conditions
h0 = [12.4; 12.7; 1.8; 1.4];            % Initial water levels in the tanks (cm)

% tank time constants and system matrices
T = (A ./ a) .* sqrt(2 * h0 / g);

Am = [-1/T(1), 0, A(3)/(A(1)*T(3)), 0;
       0, -1/T(2), 0, A(4)/(A(2)*T(4));
       0, 0, -1/T(3), 0;
       0, 0, 0, -1/T(4)];
    
Bm = [gamma1 * k1 / A(1), 0;
      0, gamma2 * k2 / A(2);
      0, (1 - gamma2) * k2 / A(3);
      (1 - gamma1) * k1 / A(4), 0];

Cm = [kc, 0, 0, 0;                      % Measuring h1
      0, 0, 0, kc];                     % Measuring h4

[m1,~]=size(Cm);
[n1,n_in]=size(Bm);


% Kalman filter initialization
X_post = h0;                            % Initial state estimate
P_post = 100 * eye(4);                  % Initial estimate covariance

% Process and measurement noise covariance matrices
Q = [0.1, 0, 0, 0;
     0, 5, 0, 0;
     0, 0, 5, 0;
     0, 0, 0, 0.1]; 
R = 100 * eye(2);

% MPC parameters
Np = 20  ;                              % Prediction horizon
Nc = 10;                                 % Control horizon
Rmpc = 2 * eye(Nc * size(Bm, 2));

% Simulation parameters
Ts = 0.1;                               % Sampling time (s)
time = 0:Ts:20;                         % Simulation time
num_iterations = length(time);

% Input and state initialization
U = [3; 3];                             % Initial pump voltages
X_estimated = zeros(4, num_iterations);
X_estimated(:, 1) = h0;
U_applied = zeros(size(Bm, 2), num_iterations);

% True state initialization
X_true = h0;

% Augmented matrices for delta U model
A_aug=eye(n1+m1,n1+m1);
A_aug(1:n1,1:n1)=Am;
A_aug(n1+1:n1+m1,1:n1)=Cm*Am;
B_aug=zeros(n1+m1,n_in);
B_aug(1:n1,:)=Bm;
B_aug(n1+1:n1+m1,:)=Cm*Bm;
C_aug= zeros(2,6);
C_aug(:,5:6)=eye(2,2);

Xk_aug_list = zeros(6, num_iterations);

% F and Phi matrices
F = [];
for i = 1:Np
    F = [F; C_aug * (A_aug^i)];
end

Phi = zeros(Np * size(C_aug, 1), Nc * size(B_aug, 2));
for i = 1:Np
    for j = 1:Nc
        if i >= j
            Phi((i-1)*size(C_aug,1)+1:i*size(C_aug,1), (j-1)*size(B_aug,2)+1:j*size(B_aug,2)) = C_aug * (A_aug^(i-j)) * B_aug;
        end
    end
end

% Defining constraints
U_min = [0; 0];                                 % Minimum input voltage
U_max = [20; 20];                               % Maximum input voltage
Delta_U_min = -5 * ones(size(Bm, 2), 1);        % Minimum change in input
Delta_U_max = 5 * ones(size(Bm, 2), 1);         % Maximum change in input
Rs = repmat([13.7; 2.8], Np, 1);                % Reference trajectory


% Simulation loop
for k = 1:num_iterations-1

     
    % True system dynamics (with noise)
    process_noise = sqrt(diag(Q)) .* randn(4, 1);
    measurement_noise = sqrt(diag(R)) .* randn(2, 1);
    
    X_true = Am * X_true + Bm * U + process_noise;     % State evolution
    Z_true = Cm * X_true + measurement_noise;          % Measurement
    
    % Kalman filter for state estimation
    % Prediction step
    X_prior = Am * X_post + Bm * U;
    P_prior = Am * P_post * Am' + Q;
    
    % Measurement update step
    K = P_prior * Cm' / (Cm * P_prior * Cm' + R);     % Kalman gain
    X_post = X_prior + K * (Z_true - Cm * X_prior);
    P_post = (eye(4) - K * Cm) * P_prior;
    X_estimated(:, k+1) = X_post;

    % Augmented state vector
    Xk = [X_estimated(:,k+1)-X_estimated(:,k); Cm * X_post];
    Xk_aug_list(:, k) = Xk;

    % Computing cost matrices
    H = Phi' * Phi + Rmpc;
    f = -Phi' * (Rs - F * Xk);
    

    M2=[-eye(2*Nc);eye(2*Nc)];
    N2=[repmat(-Delta_U_min, Nc, 1);repmat(Delta_U_max, Nc, 1)];
    C2 = tril(ones(Nc*2));
    
    M1=[-C2;C2];
    N1=[repmat((-U_min+U),Nc,1);repmat((U_max-U),Nc,1)];
    G=[M1;M2];
    h=[N1;N2];

    delta_U=hildreth_qp(H,f,G,h);
    % Applying only the first control input increment
    U = U + delta_U(1:size(Bm, 2));
    
    % Storing results
    U_applied(:, k) = U;
    
end
% Storing the final control input
U_applied(:, end) = U; 
Xk_aug_list(:, end) = Xk;

% Plot results
figure;
subplot(2, 1, 1);
plot(time, U_applied');
title('Control Inputs');
xlabel('Time (s)');
ylabel('Voltage (V)');  
legend('v1', 'v2');

subplot(2, 1, 2);
plot(time, C_aug * Xk_aug_list);  
title('Output Measurements');
xlabel('Time (s)');
ylabel('Measured Outputs (cm)');
legend('y2', 'y3');


