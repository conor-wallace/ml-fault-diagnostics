% Observer-Based Control
% Dynamics
A = [-2 1; 0 4];
B = [0; 2];
C = [1 0];

% Desired Eigen Values
desired_eig = [-2, -3];

% Controller Gain
K = [0 7/2];
% Observer Gain
L = place(A', C', desired_eig)';
% Observer-Based Control Dynamics
A_cl = [A -B*K; L*C A-L*C-B*K]

x0 = [0, 0];
