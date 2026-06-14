% inverted pendulum LQR/G

% LQR code finds the K matrix

M = 1; m = 1; L = 1.0; g = 9.81;

% SYSTEM DEFINITION (STATE SPACE)
A = [0 1 0 0;
     0 0 -m*g/M 0;
     0 0 0 1;
     0 0 (M+m)*g/(L*M) 0];  % A (DYNAMICS) IS THE STATE MATRIX
B = [0; 1/M; 0; -1/(L*M)];  % B (DYNAMICS) IS THE CONTROL MATRIX 
C = eye(4);                 % C (MEASUREMENT) IS THE OUTPUT MATRIX (WHAT IS MEASURED)
D = zeros(4, 1);            % D (MEASUREMENT) IS THE FEEDBACK MATRIX (ZEROS FOR REAL SYSTEMS)

sys = ss(A, B, C, D);


% LQR DESIGN KNOBS - WWHAT DOES GOOD CONTROL MEAN 
Q = diag([1 1 10 1]);      % Q IS THE STATE COST MATRIX (PENALTY FOR EACH STATE - LARGER IS MORE PENALTY)
R = 1;                     % R IS THE FORCE USED -- LARGER R MEANS BE GENTLE
K = lqr(A, B, Q, R);       % K IS THE RESULT MATRIX 


% KALMAN FILTER (MAKES IT LQG) USES THE NOISY STATES WE DO HAVE AND X_HAT
% TO FIND FULL STATE
Q_kalman = diag([0.001, 0.001, 0.001, 0.001]);
R_kalman = diag([0.001, 0.001, 0.001, 0.001]);

G = eye(4);                % G IS PROCESS NOISE INPUT MATRIX
H = zeros(4, 4);           % H IS NO DIRECT NOISE FEEDBACK TO MEASRUMENTS

sys_kalman = ss(A, [B G], C, [D H]);  % INPUTS CONTROLS FOR PROCESS NOISE

[kalmf, L_kalman, P] = kalman(sys_kalman, Q_kalman, R_kalman);
