%����ָ�����ϵͳ

function [V1_next, phi1_next, theta1_next, X1_next]=Leader_P(u,X1,V1,Position_phi,Position_theta)
% ���������
%   u=[a_tau, theta_dot, phi_dot] ��������a_tauΪ������ٶȣ�theta_dotΪƫ���ǽ��ٶȣ�phi_dotΪ�����ǽ��ٶ�
%   X1=[x,y,z] ��ǰ��������ϵ�µĳ�������
%   V1 ��ǰ���������ٶ�
%   Position_phi ��ǰƫ����
%   Position_theta ��ǰ������
% ���������
%   V1_next
%   phi1_next
%   theta1_next
%   X1_next
V1_next = V1+u(1);         % �ٶȸ���
phi1_next = Position_phi+u(2); % ����Ǹ���
theta1_next = Position_theta+u(3); %�����Ǹ���

if (abs(phi1_next)>pi)                       %���н�ת��Ϊ��-pi��pi��
    phi1_next=(2*pi-abs(phi1_next))*(-abs(phi1_next)/phi1_next);
end

%�ٶ�����任
v1x = V1_next*cos(theta1_next)*cos(phi1_next);
v1y = V1_next*cos(theta1_next)*sin(phi1_next);
v1z = V1_next*sin(theta1_next);

%λ�ø���
X1_next(1) = X1(1) + v1x;
X1_next(2) = X1(2) + v1y;
X1_next(3) = X1(3) + v1z;



