function [R,Position_angle,Rotat,Rotatc,Q,fuyang_angle,pitch,pitchc,distance,U]=initialization(r,q,t,V)
R=0.*t; %初始化航向角速度 单位：rad/s
R(1)=r; %初始朝向角
Position_angle=0.*t;
Rotat=0.*t;
Rotatc=0.*t;
Q=0.*t;%初始化俯仰角速度 单位：rad/s
Q(1)=q; %初始俯仰角
fuyang_angle=0.*t;
pitch=0.*t;
pitchc=0.*t;
distance=0.*t;
U=0.*t;
U(1)=V;