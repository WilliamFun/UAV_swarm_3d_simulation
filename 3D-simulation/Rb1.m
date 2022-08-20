function Xsum = Rb1(T,beta,alpha,X1,X2,X3)
Xsum(1)=X1*cos(beta)*cos(alpha)-X2*sin(beta)+X3*cos(beta)*sin(alpha)+T(1);
Xsum(2)=X1*sin(beta)*cos(alpha)+X2*cos(beta)+X3*sin(alpha)*sin(beta)+T(2);
Xsum(3)=-X1*sin(alpha)+X3*cos(alpha)+T(3);
