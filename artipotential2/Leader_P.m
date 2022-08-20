%长机指令控制系统

function [V1_next, phi1_next, theta1_next, X1_next]=Leader_P(u,X1,V1,Position_phi,Position_theta)
% 输入参数：
%   u=[a_tau, theta_dot, phi_dot] 控制量，a_tau为切向加速度，theta_dot为偏航角角速度，phi_dot为俯仰角角速度
%   X1=[x,y,z] 当前地面坐标系下的长机坐标
%   V1 当前长机飞行速度
%   Position_phi 当前偏航角
%   Position_theta 当前俯仰角
% 输出参数：
%   V1_next
%   phi1_next
%   theta1_next
%   X1_next
V1_next = V1+u(1);         % 速度更新
phi1_next = Position_phi+u(2); % 航向角更新
theta1_next = Position_theta+u(3); %俯仰角更新

if (abs(phi1_next)>pi)                       %将夹角转化为（-pi，pi）
    phi1_next=(2*pi-abs(phi1_next))*(-abs(phi1_next)/phi1_next);
end

%速度坐标变换
v1x = V1_next*cos(theta1_next)*cos(phi1_next);
v1y = V1_next*cos(theta1_next)*sin(phi1_next);
v1z = V1_next*sin(theta1_next);

%位置更新
X1_next(1) = X1(1) + v1x;
X1_next(2) = X1(2) + v1y;
X1_next(3) = X1(3) + v1z;



