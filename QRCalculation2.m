clc; clear; close all;
%% Quadcopter Properties
Rotor.RHO   = 1.225;    % 空气密度                            
Rotor.G     = 9.81;     % 重力加速度                       
Rotor.MASS  = 190.4057;   % 飞机总重量                                
Rotor.IB    = [47.23, 0, 0; 0, 90.95, 0; 0, 0, 111.5];  % 飞机惯性矩阵      
Rotor.SB    = 2.83;     % 飞机等效阻力面积                            
Rotor.CDB0  = 0.2;      % 飞机阻力系数                            
Rotor.CT    = 0.012;  % 螺旋桨升力系数                              
Rotor.L     = 2.0124;   % 螺旋桨距质心距离                                
Rotor.CQ    = 0.0011; % 螺旋桨反扭系数                          
Rotor.R     = 0.9;      % 螺旋桨半径                           
Rotor.Ir    = 0.273;    % 螺旋桨转动惯量                             


Rotor.XE0   = [0 0 0];         
Rotor.VB0   = [0 0 0];         
Rotor.PTS0  = [0 0 0];         
Rotor.WB0   = [0 0 0];         


%% Control Section
load('C:\Users\zozo\Desktop\暂定\Grad1\SYDE652\全动四旋翼模型2\Level_U_TrimR.mat');

% Check size
disp('size(U_TrimR) = ')
disp(size(U_TrimR))

% First trim point = 0 m/s
idx = 1;
u_trim6 = U_TrimR(:,idx);

% Extract trim values
w_trim_vec = sqrt(u_trim6(1:4));
eta_trim   = u_trim6(5);
gamma_trim = u_trim6(6);
u0 = [w_trim_vec(1);w_trim_vec(2);w_trim_vec(3);w_trim_vec(4); ...
      eta_trim;eta_trim;eta_trim;eta_trim; ...
      gamma_trim;gamma_trim;gamma_trim;gamma_trim];

disp('Selected trim column = ')
disp(idx)

disp('u0 = ')
disp(u0)

%IC
x0 = zeros(12,1);

assignin('base','Rotor',Rotor);
assignin('base','u0',u0);
assignin('base','x0',x0);

mdl = 'QRTM_1';
open_system(mdl);

% linearization
x_ref = zeros(12,1);
assignin('base','x_ref',x_ref);

% first simulate to let model move to the trim condition
set_param(mdl,'SimulationMode','normal');
simTime = '5';
sim(mdl, 'StopTime', simTime);

% get operating point from current model state after simulation
op = operpoint(mdl);

% define linearization I/O explicitly
io(1) = linio([mdl '/Force&Moment'],1,'input');   % input U port
io(2) = linio([mdl '/Dynamics'],1,'output');      % output Y port

% linearize around this operating point
sys = linearize(mdl, io, op);

A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;

disp('size(A) = '); disp(size(A))
disp('size(B) = '); disp(size(B))
disp('size(C) = '); disp(size(C))
disp('size(D) = '); disp(size(D))

% stop if linearization failed
if isempty(A) || isempty(B)
    error(['Linearization failed: A or B is empty. ', ...
           'Check open-loop model, linear analysis points, and AeroForce sqrt path.']);
end

% controllability
Co = ctrb(A,B);
rankCo = rank(Co);

disp(['rank(ctrb) = ', num2str(rankCo)])
disp(['number of states = ', num2str(size(A,1))])

%LQR weight
Q = diag([200 200 50 20 20 20 30 30 30 50 50 100 ]);   % x y z
R = diag([1 1 1 1 10 10 ]);

% dimension check
if size(A,1) ~= size(Q,1)
    error('Q size does not match A.')
end

if size(B,2) ~= size(R,1)
    error('R size does not match B.')
end

% LQR Gain
K = lqr(A,B,Q,R);
disp('K = ')
disp(K)

T = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 1 0;
     0 0 0 0 1 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1;
     0 0 0 0 0 1;
     0 0 0 0 0 1;
     0 0 0 0 0 1];

assignin('base','T',T);


assignin('base','A',A);
assignin('base','B',B);
assignin('base','C',C);
assignin('base','D',D);
assignin('base','K',K);
assignin('base','T',T);
assignin('base','x_ref',x_ref);
assignin('base','u0',u0);

disp('Done.')