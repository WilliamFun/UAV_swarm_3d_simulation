function [Yatx,Yaty,Yatz,Yatr]=compute_weiyi3(X,Xsum,k,beta,alpha)
R=(X(1)-Xsum(1))^2+(X(2)-Xsum(2))^2+(X(3)-Xsum(3))^2;
r=sqrt(R);%Ŀ��λ�þ���
Yatr=k*sqrt((X(1)-Xsum(1))^2+(X(2)-Xsum(2))^2);
Yatz=k*r*sin(alpha);%zλ����*k
Yatx=k*r*cos(alpha)*cos(beta);%xλ����*k
Yaty=k*r*cos(alpha)*sin(beta);%yλ����*k