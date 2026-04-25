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

%% control section
A=[0 1 0 0 0 0; 
   0 0 0 0 0 0; 
   0 0 0 1 0 0; 
   0 0 0 0 0 0; 
   0 0 0 0 0 1; 
   0 0 0 0 0 0];
B=[0 0 0; 
   1 0 0;
   0 0 0; 
   0 1 0; 
   0 0 0; 
   0 0 1];

Q=diag([50 5 50 5 100 10]);
R=diag([1 1 1]);
K=lqr(A,B,Q,R);

