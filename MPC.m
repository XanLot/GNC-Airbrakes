%% MPC Solver (Fixed h value)
clc;
clear;
close all

% System Setups

Ac = [0, 1; 0, 0];
Bc = [0; 1];
C = [1, 0];

% Other Stuff
dimAc = size(Ac,1);
p = size(C,1);
n = dimAc;

% Define the size of control and prediction horizon

f = 20; % Prediction Horizon
v = 15; % Control Horizon

% Define new system matrices, time step etc.
h = 0.02; % Time Step in Solver
I = eye(n);
A = (I - h*Ac) \ I;         % (I - h*A_c)^(-1)
B = (I - h*Ac) \ (h*Bc);    % (I - h*A_c)^(-1) * h*B_c


% Define big block matrices
F = zeros(f*p, n);          % size = (f*p) x n
A_pow = eye(n);             % A^0
for i = 1:f
    A_pow = A * A_pow;      % now A^i
    rows = (i-1)*p + (1:p); % p-row block for z_{k+i|k}
    F(rows, :) = C * A_pow; % C*A^i
end

m = size(B,2);
G = zeros(f*p, v*m);

for i = 1:f
    rows = (i-1)*p + (1:p);

    % Base lower-triangular: columns j = 0..min(i-1, v-1)
    for j = 0:min(i-1, v-1)
        cols = j*m + (1:m);
        G(rows, cols) = C * (A^(i-1-j)) * B;
    end

    % Fold contributions beyond control horizon onto the last column
    if i > v
        S = zeros(p, m);
        for ell = 0:(i-1-v)          % sums I, A, A^2, ...
            S = S + C * (A^ell) * B;
        end
        last = (v-1)*m + (1:m);
        G(rows, last) = G(rows, last) + S;
    end
end

Qy = 8;                % or a p×p if p>1
Rv = 0.15;             % move penalty
Qf    = kron(eye(f), Qy);   % (f*p) x (f*p)
Rvblk = kron(eye(v), Rv);   % (v*m) x (v*m)

H = G.'*Qf*G + Rvblk;       % (v*m) x (v*m)
Rchol = chol(H, 'upper');   % factor once
% ---- Runtime tuning  ----
Ts     = h;                % sample time
Tfinal = 6;                % seconds
N      = round(Tfinal/Ts); % steps
L = 1;                    % Scale Factor
umin   = -1.2*L;           % input bounds (fix with motor data)
umax   =  1.2*L;
du_max =  0.12*L;          % max change per step

xk     = [0; 0];           % initial state (n×1)
u_prev = 0;                % last applied input (scalar)

% logs
t_log    = zeros(N,1);
x_log    = zeros(N,n);
u_cmd_log= zeros(N,1);
u_ap_log = zeros(N,1);

% ---- Receding-horizon loop ----
ref_value = 1;                    % desired output (position units)
r = ref_value * ones(f*p,1);       % keep it constant for now

for k = 1:N
    % RHS for normal equations
    b = G.' * Qf * (r - F*xk);

    % solve H Uopt = b using Cholesky
    y    = Rchol.' \ b;
    Uopt = Rchol    \ y;          % size v×1 (m=1)
    u_cmd = Uopt(1);              % take the first action

    % rate + saturation guard (outer wrapper)
    du        = max(min(u_cmd - u_prev, du_max), -du_max);
    u_applied = min(max(u_prev + du, umin), umax);

    % plant update
    xk = A*xk + B*u_applied;

    % log
    t_log(k)     = (k-1)*Ts;
    x_log(k,:)   = xk.';
    u_cmd_log(k) = u_cmd;
    u_ap_log(k)  = u_applied;

    % shift
    u_prev = u_applied;
end

% ---- quick plots ----
figure(1); clf;
subplot(3,1,1);
plot(t_log, x_log(:,1),'LineWidth',1.5); hold on; yline(ref_value,'--');
ylabel('position'); grid on; title('MPC (unconstrained solve) with rate/sat guard');

subplot(3,1,2);
plot(t_log, x_log(:,2),'LineWidth',1.5);
ylabel('velocity'); grid on;

subplot(3,1,3);
plot(t_log, u_cmd_log,':','LineWidth',1.2); hold on;
plot(t_log, u_ap_log,'LineWidth',1.6);
yline(umin,'--'); yline(umax,'--');
ylabel('u'); xlabel('time'); grid on; legend('u_{cmd}','u_{applied}','Location','best');

