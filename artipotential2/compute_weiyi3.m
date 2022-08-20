function [Yatx,Yaty,Yatz,Yatr]=compute_weiyi3(X,Xsum,k,beta,alpha)
R=(X(1)-Xsum(1))^2+(X(2)-Xsum(2))^2+(X(3)-Xsum(3))^2;
r=sqrt(R);%目标位置距离
Yatr=k*sqrt((X(1)-Xsum(1))^2+(X(2)-Xsum(2))^2);
Yatz=k*r*sin(alpha);%z位移量*k
Yatx=k*r*cos(alpha)*cos(beta);%x位移量*k
Yaty=k*r*cos(alpha)*sin(beta);%y位移量*k