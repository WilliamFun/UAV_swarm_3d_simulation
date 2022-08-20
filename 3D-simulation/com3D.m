clc
clear all;
T1=[200 0 500];
T2=[-300 -200 300];
T3=[200 -300 200];
T4=[0 -500 100];
T5=[-300 -550 450];
T6=[-200 -600 250];
T7=[200 -800 350];
T8=[0 -900 400];
T9=[-400 -1000 150];
% % plot(x,y,'.','Markersize',50,'color',[0 0 0]);
 plot3(T1(1),T1(2),T1(3),'o','Markersize',10,'color','r');%画出初始位置
% text(T1(1)-5,T1(2)-5,T1(3)-5,'UAV_1');%标注名称
 hold on
 plot3(T2(1),T2(2),T2(3),'o','Markersize',10,'color','b');
% text(T2(1)-5,T2(2)-5,T2(3)-5,'UAV_2');
 hold on
 plot3(T3(1),T3(2),T3(3),'o','Markersize',10,'color','g');
% text(T3(1)-5,T3(2)-5,T3(3)-5,'UAV_3');
 hold on
 plot3(T4(1),T4(2),T4(3),'o','Markersize',10,'color','y');
% text(T4(1)-5,T4(2)-5,T4(3)-5,'UAV_4');
 hold on
 plot3(T5(1),T5(2),T5(3),'o','Markersize',10,'color','c');
% text(T5(1)-5,T5(2)-5,T5(3)-5,'UAV_5');
 hold on
 plot3(T6(1),T6(2),T6(3),'o','Markersize',10,'color','m');
% text(T6(1)-5,T6(2)-5,T6(3)-5,'UAV_6');
 hold on
 plot3(T7(1),T7(2),T7(3),'o','Markersize',10,'color','k');
% text(T7(1)-5,T7(2)-5,T7(3)-5,'UAV_7');
 hold on
 plot3(T8(1),T8(2),T8(3),'o','Markersize',10,'color','b');
% text(T8(1)-5,T8(2)-5,T8(3)-5,'UAV_8');
 hold on
 plot3(T9(1),T9(2),T9(3),'o','Markersize',10,'color','g');
% text(T9(1)-5,T9(2)-5,T9(3)-5,'UAV_9');
 hold on

k=1;%引力增益
l=200;%队形距离
v=pi/4;%队形夹角
Vl=10;%leader速度
Vm=20;%follower最大速度
J=150;%一个过程仿真时间
t=0:J;%时间序列
Rm=0.1;%跟随无人机最大航向角速度
Qm=0.1;%最大俯仰角速度

%设定PI系数
kp=0.5;
ki=0.1;
krp=0.9;
kri=0.4;
kqp=0.9;
kqi=0.4;

%无人机初始化
%-----------------------------------------------1
[R1,Position_angle1,Rotat1,Rotatc1,Q1,fuyang_angle1,pitch1,pitchc1,distance1,U1]=initialization(pi/2,0,t,Vl);
%-----------------------------------------------2
[R2,Position_angle2,Rotat2,Rotatc2,Q2,fuyang_angle2,pitch2,pitchc2,distance2,U2]=initialization(pi/2,pi/6,t,Vl);
distance21(1)=sqrt((T2(1)-T1(1))^2+(T2(2)-T1(2))^2+(T2(3)-T1(3))^2);
%-----------------------------------------------3
[R3,Position_angle3,Rotat3,Rotatc3,Q3,fuyang_angle3,pitch3,pitchc3,distance3,U3]=initialization(pi/3,pi/6,t,Vl);
distance31(1)=sqrt((T3(1)-T1(1))^2+(T3(2)-T1(2))^2+(T3(3)-T1(3))^2);
%-----------------------------------------------4
[R4,Position_angle4,Rotat4,Rotatc4,Q4,fuyang_angle4,pitch4,pitchc4,distance4,U4]=initialization(pi/3,pi/4,t,Vl);
distance41(1)=sqrt((T4(1)-T1(1))^2+(T4(2)-T1(2))^2+(T4(3)-T1(3))^2);
%-----------------------------------------------5
[R5,Position_angle5,Rotat5,Rotatc5,Q5,fuyang_angle5,pitch5,pitchc5,distance5,U5]=initialization(pi/4,pi/4,t,Vl);
distance51(1)=sqrt((T5(1)-T1(1))^2+(T5(2)-T1(2))^2+(T5(3)-T1(3))^2);
%-----------------------------------------------6
[R6,Position_angle6,Rotat6,Rotatc6,Q6,fuyang_angle6,pitch6,pitchc6,distance6,U6]=initialization(pi/4,pi/3,t,Vl);
distance61(1)=sqrt((T6(1)-T1(1))^2+(T6(2)-T1(2))^2+(T6(3)-T1(3))^2);
%-----------------------------------------------7
[R7,Position_angle7,Rotat7,Rotatc7,Q7,fuyang_angle7,pitch7,pitchc7,distance7,U7]=initialization(pi/6,pi/3,t,Vl);
distance71(1)=sqrt((T7(1)-T1(1))^2+(T7(2)-T1(2))^2+(T7(3)-T1(3))^2);
%-----------------------------------------------8
[R8,Position_angle8,Rotat8,Rotatc8,Q8,fuyang_angle8,pitch8,pitchc8,distance8,U8]=initialization(pi/6,pi/2,t,Vl);
distance81(1)=sqrt((T8(1)-T1(1))^2+(T8(2)-T1(2))^2+(T8(3)-T1(3))^2);
%-----------------------------------------------9
[R9,Position_angle9,Rotat9,Rotatc9,Q9,fuyang_angle9,pitch9,pitchc9,distance9,U9]=initialization(0,pi/2,t,Vl);
distance91(1)=sqrt((T9(1)-T1(1))^2+(T9(2)-T1(2))^2+(T9(3)-T1(3))^2);

Xsum1=[200 1500 500];%Leader第一个航点位置
plot3(Xsum1(1),Xsum1(2),Xsum1(3));
hold on

for j=2:J
  %………………………………………………………………………………领航无人机完成动作
   [beta1,alpha1] = compute_angle3(T1,Xsum1);%计算偏航角;
   Position_angle1(j)=beta1;%偏航角
   fuyang_angle1(j)=alpha1;%俯仰角
   
   distance1(j)=sqrt((T1(1)-Xsum1(1))^2+(T1(2)-Xsum1(2))^2+(T1(3)-Xsum1(3))^2);%当前距离期望位置距离
   
        if(distance1(j)<0.5)
     Xnext1(1)=T1(1);
     Xnext1(2)=T1(2);
     Xnext1(3)=T1(3);
     T1=Xnext1;%到达
        else
    Xnext1(1)=T1(1)+Vl*cos(fuyang_angle1(j))*cos(Position_angle1(j));%Leader下一步位置
    Xnext1(2)=T1(2)+Vl*cos(fuyang_angle1(j))*sin(Position_angle1(j));
    Xnext1(3)=T1(3)+Vl*sin(fuyang_angle1(j));
    T1=Xnext1;
        end
       
    
  %……………………………………………………………………………………2
   Xsum2(1)=T1(1)-l*sin(v);
   Xsum2(2)=T1(2)-l*cos(v);%根据Leader确定期望位置
   Xsum2(3)=T1(3);
   %Xsum2=Rb1(T1,R1(j),Q1(j),-l*sin(v),-l*cos(v),0);
   
  [beta2,alpha2] = compute_angle3(T2,Xsum2);%计算角度
   Position_angle2(j)=beta2;%偏航角
   fuyang_angle2(j)=alpha2;%俯仰角
   distance21(j)=sqrt((T2(1)-T1(1))^2+(T2(2)-T1(2))^2+(T2(3)-T1(3))^2);
   distance2(j)=sqrt((T2(1)-Xsum2(1))^2+(T2(2)-Xsum2(2))^2+(T2(3)-Xsum2(3))^2);
   
     
  [U2(j),R2,Q2,Rotatc2(j),pitchc2(j)]=Follower_PI3(distance2,Vm,Position_angle2(j),fuyang_angle2(j),R2,Q2,Rotat2,pitch2,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);%PI控制器
      
    if(distance2(j)<0.5)%到达期望位置停止
     Xnext2(1)=T2(1);
     Xnext2(2)=T2(2);
     Xnext2(3)=T2(3);
     T2=Xnext2; 
    else%未到达期望位置则移动
    Xnext2(1)=T2(1)+U2(j)*cos(Q2(j-1)+pitchc2(j)/2)*cos(R2(j-1)+Rotatc2(j)/2);
    Xnext2(2)=T2(2)+U2(j)*cos(Q2(j-1)+pitchc2(j)/2)*sin(R2(j-1)+Rotatc2(j)/2);
    Xnext2(3)=T2(3)+U2(j)*sin(Q2(j-1)+pitchc2(j)/2);
    T2=Xnext2;
    end

    %……………………………………………………………………………………3
    Xsum3(1)=T1(1)+l*sin(v);
    Xsum3(2)=T1(2)-l*cos(v);
    Xsum3(3)=T1(3);
    %Xsum3=Rb1(T1,R1(j),Q1(j),-l*sin(v),-l*cos(v),0);
    
   [beta3,alpha3] = compute_angle3(T3,Xsum3);
   Position_angle3(j)=beta3;%偏航角
   fuyang_angle3(j)=alpha3;
   distance31(j)=sqrt((T3(1)-T1(1))^2+(T3(2)-T1(2))^2+(T3(3)-T1(3))^2);
   distance3(j)=sqrt((T3(1)-Xsum3(1))^2+(T3(2)-Xsum3(2))^2+(T3(3)-Xsum3(3))^2);
   
     
  [U3(j),R3,Q3,Rotatc3(j),pitchc3(j)]=Follower_PI3(distance3,Vm,Position_angle3(j),fuyang_angle3(j),R3,Q3,Rotat3,pitch3,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance3(j)<0.5)
     Xnext3(1)=T3(1);
     Xnext3(2)=T3(2);
     Xnext3(3)=T3(3);
     T3=Xnext3; 
        else
    Xnext3(1)=T3(1)+U3(j)*cos(Q3(j-1)+pitchc3(j)/2)*cos(R3(j-1)+Rotatc3(j)/2);
    Xnext3(2)=T3(2)+U3(j)*cos(Q3(j-1)+pitchc3(j)/2)*sin(R3(j-1)+Rotatc3(j)/2);
    Xnext3(3)=T3(3)+U3(j)*sin(Q3(j-1)+pitchc3(j)/2);
    T3=Xnext3;
    end
    
    %……………………………………………………………………………………4
    Xsum4(1)=T1(1)-2*l*sin(v);
    Xsum4(2)=T1(2)-2*l*cos(v);
    Xsum4(3)=T1(3);
    %Xsum4=Rb1(T1,R1(j),Q1(j),-2*l*sin(v),-2*l*cos(v),0);
    
   [beta4,alpha4] = compute_angle3(T4,Xsum4);
   Position_angle4(j)=beta4;
   fuyang_angle4(j)=alpha4;
   distance41(j)=sqrt((T4(1)-T1(1))^2+(T4(2)-T1(2))^2+(T4(3)-T1(3))^2);
   distance4(j)=sqrt((T4(1)-Xsum4(1))^2+(T4(2)-Xsum4(2))^2+(T4(3)-Xsum4(3))^2);
   
     
  [U4(j),R4,Q4,Rotatc4(j),pitchc4(j)]=Follower_PI3(distance4,Vm,Position_angle4(j),fuyang_angle4(j),R4,Q4,Rotat4,pitch4,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance4(j)<0.5)
     Xnext4(1)=T4(1);
     Xnext4(2)=T4(2);
     Xnext4(3)=T4(3);
     T4=Xnext4; 
        else
    Xnext4(1)=T4(1)+U4(j)*cos(Q4(j-1)+pitchc4(j)/2)*cos(R4(j-1)+Rotatc4(j)/2);
    Xnext4(2)=T4(2)+U4(j)*cos(Q4(j-1)+pitchc4(j)/2)*sin(R4(j-1)+Rotatc4(j)/2);
    Xnext4(3)=T4(3)+U4(j)*sin(Q4(j-1)+pitchc4(j)/2);
    T4=Xnext4;
    end
    
    %……………………………………………………………………………………5
    Xsum5(1)=T1(1);
    Xsum5(2)=T1(2)-2*l*cos(v);
    Xsum5(3)=T1(3);
    %Xsum5=Rb1(T1,R1(j),Q1(j),0,-2*l*cos(v),0);
    
   [beta5,alpha5] = compute_angle3(T5,Xsum5);
   Position_angle5(j)=beta5;
   fuyang_angle5(j)=alpha5;
   distance51(j)=sqrt((T5(1)-T1(1))^2+(T5(2)-T1(2))^2+(T5(3)-T1(3))^2);
   distance5(j)=sqrt((T5(1)-Xsum5(1))^2+(T5(2)-Xsum5(2))^2+(T5(3)-Xsum5(3))^2);
   
     
  [U5(j),R5,Q5,Rotatc5(j),pitchc5(j)]=Follower_PI3(distance5,Vm,Position_angle5(j),fuyang_angle5(j),R5,Q5,Rotat5,pitch5,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance5(j)<0.5)
     Xnext5(1)=T5(1);
     Xnext5(2)=T5(2);
     Xnext5(3)=T5(3);
     T5=Xnext5; 
        else
    Xnext5(1)=T5(1)+U5(j)*cos(Q5(j-1)+pitchc5(j)/2)*cos(R5(j-1)+Rotatc5(j)/2);
    Xnext5(2)=T5(2)+U5(j)*cos(Q5(j-1)+pitchc5(j)/2)*sin(R5(j-1)+Rotatc5(j)/2);
    Xnext5(3)=T5(3)+U5(j)*sin(Q5(j-1)+pitchc5(j)/2);
    T5=Xnext5;
    end
    
  %……………………………………………………………………………………6
    Xsum6(1)=T1(1)+2*l*sin(v);
    Xsum6(2)=T1(2)-2*l*cos(v);
    Xsum6(3)=T1(3);
    %Xsum6=Rb1(T1,R1(j),Q1(j),+2*l*sin(v),-2*l*cos(v),0);
    
   [beta6,alpha6] = compute_angle3(T6,Xsum6);
   Position_angle6(j)=beta6;
   fuyang_angle6(j)=alpha6;
   distance61(j)=sqrt((T6(1)-T1(1))^2+(T6(2)-T1(2))^2+(T6(3)-T1(3))^2);
   distance6(j)=sqrt((T6(1)-Xsum6(1))^2+(T6(2)-Xsum6(2))^2+(T6(3)-Xsum6(3))^2);
   
     
  [U6(j),R6,Q6,Rotatc6(j),pitchc6(j)]=Follower_PI3(distance6,Vm,Position_angle6(j),fuyang_angle6(j),R6,Q6,Rotat6,pitch6,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance6(j)<0.5)
     Xnext6(1)=T6(1);
     Xnext6(2)=T6(2);
     Xnext6(3)=T6(3);
     T6=Xnext6; 
        else
    Xnext6(1)=T6(1)+U6(j)*cos(Q6(j-1)+pitchc6(j)/2)*cos(R6(j-1)+Rotatc6(j)/2);
    Xnext6(2)=T6(2)+U6(j)*cos(Q6(j-1)+pitchc6(j)/2)*sin(R6(j-1)+Rotatc6(j)/2);
    Xnext6(3)=T6(3)+U6(j)*sin(Q6(j-1)+pitchc6(j)/2);
    T6=Xnext6;
    end
    
  %……………………………………………………………………………………7
    Xsum7(1)=T1(1)-l*sin(v);
    Xsum7(2)=T1(2)-3*l*cos(v);
    Xsum7(3)=T1(3);
    %Xsum7=Rb1(T1,R1(j),Q1(j),-l*sin(v),-3*l*cos(v),0);
    
   [beta7,alpha7] = compute_angle3(T7,Xsum7);
   Position_angle7(j)=beta7;
   fuyang_angle7(j)=alpha7;
   distance71(j)=sqrt((T7(1)-T1(1))^2+(T7(2)-T1(2))^2+(T7(3)-T1(3))^2);
   distance7(j)=sqrt((T7(1)-Xsum7(1))^2+(T7(2)-Xsum7(2))^2+(T7(3)-Xsum7(3))^2);
   
     
  [U7(j),R7,Q7,Rotatc7(j),pitchc7(j)]=Follower_PI3(distance7,Vm,Position_angle7(j),fuyang_angle7(j),R7,Q7,Rotat7,pitch7,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance7(j)<0.5)
     Xnext7(1)=T7(1);
     Xnext7(2)=T7(2);
     Xnext7(3)=T7(3);
     T7=Xnext7; 
        else
    Xnext7(1)=T7(1)+U7(j)*cos(Q7(j-1)+pitchc7(j)/2)*cos(R7(j-1)+Rotatc7(j)/2);
    Xnext7(2)=T7(2)+U7(j)*cos(Q7(j-1)+pitchc7(j)/2)*sin(R7(j-1)+Rotatc7(j)/2);
    Xnext7(3)=T7(3)+U7(j)*sin(Q7(j-1)+pitchc7(j)/2);
    T7=Xnext7;
    end    
   
   %……………………………………………………………………………………8
    Xsum8(1)=T1(1)+l*sin(v);
    Xsum8(2)=T1(2)-3*l*cos(v);
    Xsum8(3)=T1(3);
    %Xsum8=Rb1(T1,R1(j),Q1(j),+l*sin(v),-3*l*cos(v),0);
    
   [beta8,alpha8] = compute_angle3(T8,Xsum8);
   Position_angle8(j)=beta8;
   fuyang_angle8(j)=alpha8;
   distance81(j)=sqrt((T8(1)-T1(1))^2+(T8(2)-T1(2))^2+(T8(3)-T1(3))^2);
   distance8(j)=sqrt((T8(1)-Xsum8(1))^2+(T8(2)-Xsum8(2))^2+(T8(3)-Xsum8(3))^2);
   
     
  [U8(j),R8,Q8,Rotatc8(j),pitchc8(j)]=Follower_PI3(distance8,Vm,Position_angle8(j),fuyang_angle8(j),R8,Q8,Rotat8,pitch8,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance8(j)<0.5)
     Xnext8(1)=T8(1);
     Xnext8(2)=T8(2);
     Xnext8(3)=T8(3);
     T8=Xnext8; 
        else
    Xnext8(1)=T8(1)+U8(j)*cos(Q8(j-1)+pitchc8(j)/2)*cos(R8(j-1)+Rotatc8(j)/2);
    Xnext8(2)=T8(2)+U8(j)*cos(Q8(j-1)+pitchc8(j)/2)*sin(R8(j-1)+Rotatc8(j)/2);
    Xnext8(3)=T8(3)+U8(j)*sin(Q8(j-1)+pitchc8(j)/2);
    T8=Xnext8;
    end   
    
   %……………………………………………………………………………………9
    Xsum9(1)=T1(1);
    Xsum9(2)=T1(2)-4*l*cos(v);
    Xsum9(3)=T1(3);
    %Xsum9=Rb1(T1,R1(j),Q1(j),0,-4*l*cos(v),0);
    
   [beta9,alpha9] = compute_angle3(T9,Xsum9);
   Position_angle9(j)=beta9;
   fuyang_angle9(j)=alpha9;
   distance91(j)=sqrt((T9(1)-T1(1))^2+(T9(2)-T1(2))^2+(T9(3)-T1(3))^2);
   distance9(j)=sqrt((T9(1)-Xsum9(1))^2+(T9(2)-Xsum9(2))^2+(T9(3)-Xsum9(3))^2);
   
     
  [U9(j),R9,Q9,Rotatc9(j),pitchc9(j)]=Follower_PI3(distance9,Vm,Position_angle9(j),fuyang_angle9(j),R9,Q9,Rotat9,pitch9,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance9(j)<0.5)
     Xnext9(1)=T9(1);
     Xnext9(2)=T9(2);
     Xnext9(3)=T9(3);
     T9=Xnext9; 
        else
    Xnext9(1)=T9(1)+U9(j)*cos(Q9(j-1)+pitchc9(j)/2)*cos(R9(j-1)+Rotatc9(j)/2);
    Xnext9(2)=T9(2)+U9(j)*cos(Q9(j-1)+pitchc9(j)/2)*sin(R9(j-1)+Rotatc9(j)/2);
    Xnext9(3)=T9(3)+U9(j)*sin(Q9(j-1)+pitchc9(j)/2);
    T9=Xnext9;
    end       
    
    
 if(distance1(j)<10&&distance2(j)<10&&distance3(j)<10&&distance4(j)<10&&distance5(j)<10&&distance6(j)<10&&distance7(j)<10&&distance8(j)<10&&distance9(j)<10)%…………………………………………………………到达目的地停止并画动图
    break;
 end
  h=plot3(T1(1),T1(2),T1(3),'.r',T2(1),T2(2),T2(3),'.b',T3(1),T3(2),T3(3),'.g',T4(1),T4(2),T4(3),'.y',T5(1),T5(2),T5(3),'.c',T6(1),T6(2),T6(3),'.m',T7(1),T7(2),T7(3),'.k',T8(1),T8(2),T8(3),'.b',T9(1),T9(2),T9(3),'.g');
  axis equal
  grid on;
  refreshdata(h) %指定数据源时刷新图中的数据
  drawnow %刷新屏幕
  hold on
end
% plot3(T1(1),T1(2),T1(3),'o','Markersize',10,'color','r');
% hold on
% plot3(T2(1),T2(2),T2(3),'o','Markersize',10,'color','b');
% hold on
% plot3(T3(1),T3(2),T3(3),'o','Markersize',10,'color','g');
% hold on
% plot3(T4(1),T4(2),T4(3),'o','Markersize',10,'color','y');
% hold on
% plot3(T5(1),T5(2),T5(3),'o','Markersize',10,'color','c');
% hold on
% plot3(T6(1),T6(2),T6(3),'o','Markersize',10,'color','m');
% hold on
% plot3(T7(1),T7(2),T7(3),'o','Markersize',10,'color','k');
% hold on
% plot3(T8(1),T8(2),T8(3),'o','Markersize',10,'color','b');
% hold on
% plot3(T9(1),T9(2),T9(3),'o','Markersize',10,'color','g');
% hold on
line([T1(1),T2(1)],[T1(2),T2(2)],[T1(3),T2(3)]),line([T1(1),T3(1)],[T1(2),T3(2)],[T1(3),T3(3)]),line([T2(1),T4(1)],[T2(2),T4(2)],[T2(3),T4(3)]); 
line([T7(1),T4(1)],[T7(2),T4(2)],[T7(3),T4(3)]),line([T3(1),T6(1)],[T3(2),T6(2)],[T3(3),T6(3)]),line([T6(1),T8(1)],[T6(2),T8(2)],[T6(3),T8(3)]);
line([T8(1),T9(1)],[T8(2),T9(2)],[T8(3),T9(3)]),line([T7(1),T9(1)],[T7(2),T9(2)],[T7(3),T9(3)]);%画出队形形状

%……………………………………………………………………………………参数更新
R1(1)=pi/2;
R2(1)=pi/2;
R3(1)=pi/2;
R4(1)=pi/2;
R5(1)=pi/2;
R6(1)=pi/2;
R7(1)=pi/2;
R8(1)=pi/2;
R9(1)=pi/2;
% 
% U1(1)=Vl;
% U2(1)=Vl;
% U3(1)=Vl;
% U4(1)=Vl;
% U5(1)=Vl;
% U6(1)=Vl;
% U7(1)=Vl;
% U8(1)=Vl;
% U9(1)=Vl;
% 
Q1(1)=0;
Q2(1)=0;
Q3(1)=0;
Q4(1)=0;
Q5(1)=0;
Q6(1)=0;
Q7(1)=0;
Q8(1)=0;
Q9(1)=0;

v=pi/6;

Xsum1=[200 3000 500];%领航机第二个航点位置
for j=2:J
  %………………………………………………………………………………领航无人机完成动作
   [beta1,alpha1] = compute_angle3(T1,Xsum1);
   Position_angle1(j)=beta1;
   fuyang_angle1(j)=alpha1;
   
   distance1(j)=sqrt((T1(1)-Xsum1(1))^2+(T1(2)-Xsum1(2))^2+(T1(3)-Xsum1(3))^2);%当前距离期望位置距离
   
        if(distance1(j)<0.5)
     Xnext1(1)=T1(1);
     Xnext1(2)=T1(2);
     Xnext1(3)=T1(3);
     T1=Xnext1;%到达
        else
    Xnext1(1)=T1(1)+Vl*cos(fuyang_angle1(j))*cos(Position_angle1(j));%Leader下一步位置
    Xnext1(2)=T1(2)+Vl*cos(fuyang_angle1(j))*sin(Position_angle1(j));
    Xnext1(3)=T1(3)+Vl*sin(fuyang_angle1(j));
    T1=Xnext1;
        end
       
    
  %……………………………………………………………………………………2
   Xsum2(1)=T1(1)-l*sin(v);
   Xsum2(2)=T1(2)-l*cos(v);%根据Leader确定期望位置
   Xsum2(3)=T1(3);
   %Xsum2=Rb1(T1,R1(j),Q1(j),-l*sin(v),-l*cos(v),0);
   
  [beta2,alpha2] = compute_angle3(T2,Xsum2);
   Position_angle2(j)=beta2;
   fuyang_angle2(j)=alpha2;
   distance21(j)=sqrt((T2(1)-T1(1))^2+(T2(2)-T1(2))^2+(T2(3)-T1(3))^2);
   distance2(j)=sqrt((T2(1)-Xsum2(1))^2+(T2(2)-Xsum2(2))^2+(T2(3)-Xsum2(3))^2);
   
     
  [U2(j),R2,Q2,Rotatc2(j),pitchc2(j)]=Follower_PI3(distance2,Vm,Position_angle2(j),fuyang_angle2(j),R2,Q2,Rotat2,pitch2,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance2(j)<0.5)
     Xnext2(1)=T2(1);
     Xnext2(2)=T2(2);
     Xnext2(3)=T2(3);
     T2=Xnext2; 
        else
    Xnext2(1)=T2(1)+U2(j)*cos(Q2(j-1)+pitchc2(j)/2)*cos(R2(j-1)+Rotatc2(j)/2);
    Xnext2(2)=T2(2)+U2(j)*cos(Q2(j-1)+pitchc2(j)/2)*sin(R2(j-1)+Rotatc2(j)/2);
    Xnext2(3)=T2(3)+U2(j)*sin(Q2(j-1)+pitchc2(j)/2);
    T2=Xnext2;
    end

    %……………………………………………………………………………………3
    Xsum3(1)=T1(1)+l*sin(v);
    Xsum3(2)=T1(2)-l*cos(v);
    Xsum3(3)=T1(3);
    %Xsum3=Rb1(T1,R1(j),Q1(j),-l*sin(v),-l*cos(v),0);
    
   [beta3,alpha3] = compute_angle3(T3,Xsum3);
   Position_angle3(j)=beta3;
   fuyang_angle3(j)=alpha3;
   distance31(j)=sqrt((T3(1)-T1(1))^2+(T3(2)-T1(2))^2+(T3(3)-T1(3))^2);
   distance3(j)=sqrt((T3(1)-Xsum3(1))^2+(T3(2)-Xsum3(2))^2+(T3(3)-Xsum3(3))^2);
   
     
  [U3(j),R3,Q3,Rotatc3(j),pitchc3(j)]=Follower_PI3(distance3,Vm,Position_angle3(j),fuyang_angle3(j),R3,Q3,Rotat3,pitch3,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance3(j)<0.5)
     Xnext3(1)=T3(1);
     Xnext3(2)=T3(2);
     Xnext3(3)=T3(3);
     T3=Xnext3; 
        else
    Xnext3(1)=T3(1)+U3(j)*cos(Q3(j-1)+pitchc3(j)/2)*cos(R3(j-1)+Rotatc3(j)/2);
    Xnext3(2)=T3(2)+U3(j)*cos(Q3(j-1)+pitchc3(j)/2)*sin(R3(j-1)+Rotatc3(j)/2);
    Xnext3(3)=T3(3)+U3(j)*sin(Q3(j-1)+pitchc3(j)/2);
    T3=Xnext3;
    end
    
    %……………………………………………………………………………………4
    Xsum4(1)=T1(1)-2*l*sin(v);
    Xsum4(2)=T1(2)-2*l*cos(v);
    Xsum4(3)=T1(3);
    %Xsum4=Rb1(T1,R1(j),Q1(j),-2*l*sin(v),-2*l*cos(v),0);
    
   [beta4,alpha4] = compute_angle3(T4,Xsum4);
   Position_angle4(j)=beta4;
   fuyang_angle4(j)=alpha4;
   distance41(j)=sqrt((T4(1)-T1(1))^2+(T4(2)-T1(2))^2+(T4(3)-T1(3))^2);
   distance4(j)=sqrt((T4(1)-Xsum4(1))^2+(T4(2)-Xsum4(2))^2+(T4(3)-Xsum4(3))^2);
   
     
  [U4(j),R4,Q4,Rotatc4(j),pitchc4(j)]=Follower_PI3(distance4,Vm,Position_angle4(j),fuyang_angle4(j),R4,Q4,Rotat4,pitch4,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance4(j)<0.5)
     Xnext4(1)=T4(1);
     Xnext4(2)=T4(2);
     Xnext4(3)=T4(3);
     T4=Xnext4; 
        else
    Xnext4(1)=T4(1)+U4(j)*cos(Q4(j-1)+pitchc4(j)/2)*cos(R4(j-1)+Rotatc4(j)/2);
    Xnext4(2)=T4(2)+U4(j)*cos(Q4(j-1)+pitchc4(j)/2)*sin(R4(j-1)+Rotatc4(j)/2);
    Xnext4(3)=T4(3)+U4(j)*sin(Q4(j-1)+pitchc4(j)/2);
    T4=Xnext4;
    end
    
    %……………………………………………………………………………………5
    Xsum5(1)=T1(1);
    Xsum5(2)=T1(2)-2*l*cos(v);
    Xsum5(3)=T1(3);
    %Xsum5=Rb1(T1,R1(j),Q1(j),0,-2*l*cos(v),0);
    
   [beta5,alpha5] = compute_angle3(T5,Xsum5);
   Position_angle5(j)=beta5;
   fuyang_angle5(j)=alpha5;
   distance51(j)=sqrt((T5(1)-T1(1))^2+(T5(2)-T1(2))^2+(T5(3)-T1(3))^2);
   distance5(j)=sqrt((T5(1)-Xsum5(1))^2+(T5(2)-Xsum5(2))^2+(T5(3)-Xsum5(3))^2);
   
     
  [U5(j),R5,Q5,Rotatc5(j),pitchc5(j)]=Follower_PI3(distance5,Vm,Position_angle5(j),fuyang_angle5(j),R5,Q5,Rotat5,pitch5,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance5(j)<0.5)
     Xnext5(1)=T5(1);
     Xnext5(2)=T5(2);
     Xnext5(3)=T5(3);
     T5=Xnext5; 
        else
    Xnext5(1)=T5(1)+U5(j)*cos(Q5(j-1)+pitchc5(j)/2)*cos(R5(j-1)+Rotatc5(j)/2);
    Xnext5(2)=T5(2)+U5(j)*cos(Q5(j-1)+pitchc5(j)/2)*sin(R5(j-1)+Rotatc5(j)/2);
    Xnext5(3)=T5(3)+U5(j)*sin(Q5(j-1)+pitchc5(j)/2);
    T5=Xnext5;
    end
    
  %……………………………………………………………………………………6
    Xsum6(1)=T1(1)+2*l*sin(v);
    Xsum6(2)=T1(2)-2*l*cos(v);
    Xsum6(3)=T1(3);
    %Xsum6=Rb1(T1,R1(j),Q1(j),+2*l*sin(v),-2*l*cos(v),0);
    
   [beta6,alpha6] = compute_angle3(T6,Xsum6);
   Position_angle6(j)=beta6;
   fuyang_angle6(j)=alpha6;
   distance61(j)=sqrt((T6(1)-T1(1))^2+(T6(2)-T1(2))^2+(T6(3)-T1(3))^2);
   distance6(j)=sqrt((T6(1)-Xsum6(1))^2+(T6(2)-Xsum6(2))^2+(T6(3)-Xsum6(3))^2);
   
     
  [U6(j),R6,Q6,Rotatc6(j),pitchc6(j)]=Follower_PI3(distance6,Vm,Position_angle6(j),fuyang_angle6(j),R6,Q6,Rotat6,pitch6,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance6(j)<0.5)
     Xnext6(1)=T6(1);
     Xnext6(2)=T6(2);
     Xnext6(3)=T6(3);
     T6=Xnext6; 
        else
    Xnext6(1)=T6(1)+U6(j)*cos(Q6(j-1)+pitchc6(j)/2)*cos(R6(j-1)+Rotatc6(j)/2);
    Xnext6(2)=T6(2)+U6(j)*cos(Q6(j-1)+pitchc6(j)/2)*sin(R6(j-1)+Rotatc6(j)/2);
    Xnext6(3)=T6(3)+U6(j)*sin(Q6(j-1)+pitchc6(j)/2);
    T6=Xnext6;
    end
    
  %……………………………………………………………………………………7
    Xsum7(1)=T1(1)-3*l*sin(v);
    Xsum7(2)=T1(2)-3*l*cos(v);
    Xsum7(3)=T1(3);
    %Xsum7=Rb1(T1,R1(j),Q1(j),-l*sin(v),-3*l*cos(v),0);
    
   [beta7,alpha7] = compute_angle3(T7,Xsum7);
   Position_angle7(j)=beta7;
   fuyang_angle7(j)=alpha7;
   distance71(j)=sqrt((T7(1)-T1(1))^2+(T7(2)-T1(2))^2+(T7(3)-T1(3))^2);
   distance7(j)=sqrt((T7(1)-Xsum7(1))^2+(T7(2)-Xsum7(2))^2+(T7(3)-Xsum7(3))^2);
   
     
  [U7(j),R7,Q7,Rotatc7(j),pitchc7(j)]=Follower_PI3(distance7,Vm,Position_angle7(j),fuyang_angle7(j),R7,Q7,Rotat7,pitch7,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance7(j)<0.5)
     Xnext7(1)=T7(1);
     Xnext7(2)=T7(2);
     Xnext7(3)=T7(3);
     T7=Xnext7; 
        else
    Xnext7(1)=T7(1)+U7(j)*cos(Q7(j-1)+pitchc7(j)/2)*cos(R7(j-1)+Rotatc7(j)/2);
    Xnext7(2)=T7(2)+U7(j)*cos(Q7(j-1)+pitchc7(j)/2)*sin(R7(j-1)+Rotatc7(j)/2);
    Xnext7(3)=T7(3)+U7(j)*sin(Q7(j-1)+pitchc7(j)/2);
    T7=Xnext7;
    end    
   
   %……………………………………………………………………………………8
    Xsum8(1)=T1(1)+3*l*sin(v);
    Xsum8(2)=T1(2)-3*l*cos(v);
    Xsum8(3)=T1(3);
    %Xsum8=Rb1(T1,R1(j),Q1(j),+l*sin(v),-3*l*cos(v),0);
    
   [beta8,alpha8] = compute_angle3(T8,Xsum8);
   Position_angle8(j)=beta8;
   fuyang_angle8(j)=alpha8;
   distance81(j)=sqrt((T8(1)-T1(1))^2+(T8(2)-T1(2))^2+(T8(3)-T1(3))^2);
   distance8(j)=sqrt((T8(1)-Xsum8(1))^2+(T8(2)-Xsum8(2))^2+(T8(3)-Xsum8(3))^2);
   
     
  [U8(j),R8,Q8,Rotatc8(j),pitchc8(j)]=Follower_PI3(distance8,Vm,Position_angle8(j),fuyang_angle8(j),R8,Q8,Rotat8,pitch8,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance8(j)<0.5)
     Xnext8(1)=T8(1);
     Xnext8(2)=T8(2);
     Xnext8(3)=T8(3);
     T8=Xnext8; 
        else
    Xnext8(1)=T8(1)+U8(j)*cos(Q8(j-1)+pitchc8(j)/2)*cos(R8(j-1)+Rotatc8(j)/2);
    Xnext8(2)=T8(2)+U8(j)*cos(Q8(j-1)+pitchc8(j)/2)*sin(R8(j-1)+Rotatc8(j)/2);
    Xnext8(3)=T8(3)+U8(j)*sin(Q8(j-1)+pitchc8(j)/2);
    T8=Xnext8;
    end   
    
   %……………………………………………………………………………………9
    Xsum9(1)=T1(1);
    Xsum9(2)=T1(2)-3*l*cos(v);
    Xsum9(3)=T1(3);
    %Xsum9=Rb1(T1,R1(j),Q1(j),0,-4*l*cos(v),0);
    
   [beta9,alpha9] = compute_angle3(T9,Xsum9);
   Position_angle9(j)=beta9;
   fuyang_angle9(j)=alpha9;
   distance91(j)=sqrt((T9(1)-T1(1))^2+(T9(2)-T1(2))^2+(T9(3)-T1(3))^2);
   distance9(j)=sqrt((T9(1)-Xsum9(1))^2+(T9(2)-Xsum9(2))^2+(T9(3)-Xsum9(3))^2);
   
     
  [U9(j),R9,Q9,Rotatc9(j),pitchc9(j)]=Follower_PI3(distance9,Vm,Position_angle9(j),fuyang_angle9(j),R9,Q9,Rotat9,pitch9,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance9(j)<0.5)
     Xnext9(1)=T9(1);
     Xnext9(2)=T9(2);
     Xnext9(3)=T9(3);
     T9=Xnext9; 
        else
    Xnext9(1)=T9(1)+U9(j)*cos(Q9(j-1)+pitchc9(j)/2)*cos(R9(j-1)+Rotatc9(j)/2);
    Xnext9(2)=T9(2)+U9(j)*cos(Q9(j-1)+pitchc9(j)/2)*sin(R9(j-1)+Rotatc9(j)/2);
    Xnext9(3)=T9(3)+U9(j)*sin(Q9(j-1)+pitchc9(j)/2);
    T9=Xnext9;
    end       
    
    
 if(distance1(j)<10&&distance2(j)<10&&distance3(j)<10&&distance4(j)<10&&distance5(j)<10&&distance6(j)<10&&distance7(j)<10&&distance8(j)<10&&distance9(j)<10)%…………………………………………………………到达目的地停止并画动图
    break;
 end
  h=plot3(T1(1),T1(2),T1(3),'.r',T2(1),T2(2),T2(3),'.b',T3(1),T3(2),T3(3),'.g',T4(1),T4(2),T4(3),'.y',T5(1),T5(2),T5(3),'.c',T6(1),T6(2),T6(3),'.m',T7(1),T7(2),T7(3),'.k',T8(1),T8(2),T8(3),'.b',T9(1),T9(2),T9(3),'.g');
  axis equal
  grid on;
  refreshdata(h) %指定数据源时刷新图中的数据
  drawnow %刷新屏幕
  hold on
end
% plot3(T1(1),T1(2),T1(3),'o','Markersize',10,'color','r');
% hold on
% plot3(T2(1),T2(2),T2(3),'o','Markersize',10,'color','b');
% hold on
% plot3(T3(1),T3(2),T3(3),'o','Markersize',10,'color','g');
% hold on
% plot3(T4(1),T4(2),T4(3),'o','Markersize',10,'color','y');
% hold on
% plot3(T5(1),T5(2),T5(3),'o','Markersize',10,'color','c');
% hold on
% plot3(T6(1),T6(2),T6(3),'o','Markersize',10,'color','m');
% hold on
% plot3(T7(1),T7(2),T7(3),'o','Markersize',10,'color','k');
% hold on
% plot3(T8(1),T8(2),T8(3),'o','Markersize',10,'color','b');
% hold on
% plot3(T9(1),T9(2),T9(3),'o','Markersize',10,'color','g');
% hold on
line([T1(1),T2(1)],[T1(2),T2(2)],[T1(3),T2(3)]),line([T1(1),T3(1)],[T1(2),T3(2)],[T1(3),T3(3)]),line([T2(1),T4(1)],[T2(2),T4(2)],[T2(3),T4(3)]); 
line([T7(1),T4(1)],[T7(2),T4(2)],[T7(3),T4(3)]),line([T3(1),T6(1)],[T3(2),T6(2)],[T3(3),T6(3)]),line([T6(1),T8(1)],[T6(2),T8(2)],[T6(3),T8(3)]);
line([T8(1),T9(1)],[T8(2),T9(2)],[T8(3),T9(3)]),line([T7(1),T9(1)],[T7(2),T9(2)],[T7(3),T9(3)]);%画出队形形状

v=pi/6;
R1(1)=pi/2;
Q1(1)=0;
J=650;%440
t=0:J;
distance2=0.*t;
distance3=0.*t;
distance4=0.*t;
distance5=0.*t;
distance6=0.*t;
distance7=0.*t;
distance8=0.*t;
distance9=0.*t;

distance21=0.*t;
distance31=0.*t;
distance41=0.*t;
distance51=0.*t;
distance61=0.*t;
distance71=0.*t;
distance81=0.*t;
distance91=0.*t;

distance21(1)=290;
distance31(1)=290;
distance41(1)=485;
distance51(1)=440;
distance61(1)=480;
distance71(1)=680;
distance81(1)=680;
distance91(1)=610;

U1=0.*t;
U2=0.*t;
U3=0.*t;
U4=0.*t;
U5=0.*t;
U6=0.*t;
U7=0.*t;
U8=0.*t;
U9=0.*t;

U1(1)=10;
U2(1)=10;
U3(1)=10;
U4(1)=10;
U5(1)=10;
U6(1)=10;
U7(1)=10;
U8(1)=10;
U9(1)=10;



for j=2:J
    
     if j<200%j<200
      u1=[0,pi/200,0];
  else if j<420 && j>=400%j<400
          u1=[0.001 0 0.01];%[0 0 0];
      else if j>=480 && j<500%j<420
              u1=[-0.001 0 -0.01];%[0 0 0.01];
          else
                  u1=[0 0 0];%[0 0 -0.01];
              
          end
       end
    end   
  %………………………………………………………………………………领航无人机完成动作
    
    [U1(j),R1(j),Q1(j),T1]=Leader_P(u1,T1,U1(j-1),R1(j-1),Q1(j-1));
        
  %……………………………………………………………………………………2
   Xsum2=Rb1(T1,R1(j)-pi/2,Q1(j),-l*sin(v),-l*cos(v),0);%相对位置坐标转换
%    Xsum2(1)=T1(1)-l*sin(v)*cos(R1(j)-pi/2)+l*cos(v)*sin(R1(j)-pi/2);
%    Xsum2(2)=T1(2)-l*cos(v)*cos(R1(j)-pi/2)-l*sin(v)*sin(R1(j)-pi/2);%根据Leader确定期望位置
%    Xsum2(3)=T1(3);
   
  [beta2,alpha2] = compute_angle3(T2,Xsum2);%计算偏航角
   Position_angle2(j)=beta2;%当前偏航角
   fuyang_angle2(j)=alpha2;
   distance21(j)=sqrt((T2(1)-T1(1))^2+(T2(2)-T1(2))^2+(T2(3)-T1(3))^2);
   distance2(j)=sqrt((T2(1)-Xsum2(1))^2+(T2(2)-Xsum2(2))^2+(T2(3)-Xsum2(3))^2);
   
     
  [U2(j),R2,Q2,Rotatc2(j),pitchc2(j)]=Follower_PI3(distance2,Vm,Position_angle2(j),fuyang_angle2(j),R2,Q2,Rotat2,pitch2,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance2(j)<0.5)
     Xnext2(1)=T2(1);
     Xnext2(2)=T2(2);
     Xnext2(3)=T2(3);
     T2=Xnext2; 
        else
    Xnext2(1)=T2(1)+U2(j)*cos(Q2(j-1)+pitchc2(j)/2)*cos(R2(j-1)+Rotatc2(j)/2);
    Xnext2(2)=T2(2)+U2(j)*cos(Q2(j-1)+pitchc2(j)/2)*sin(R2(j-1)+Rotatc2(j)/2);
    Xnext2(3)=T2(3)+U2(j)*sin(Q2(j-1)+pitchc2(j)/2);
    T2=Xnext2;
    end

    %……………………………………………………………………………………3
    Xsum3=Rb1(T1,R1(j)-pi/2,Q1(j),l*sin(v),-l*cos(v),0);
%     Xsum3(1)=T1(1)+l*sin(v)*cos(R1(j)-pi/2)+l*cos(v)*sin(R1(j)-pi/2);
%     Xsum3(2)=T1(2)-l*cos(v)*cos(R1(j)-pi/2)+l*sin(v)*sin(R1(j)-pi/2);
%     Xsum3(3)=T1(3);
    
   [beta3,alpha3] = compute_angle3(T3,Xsum3);%计算偏航角
   Position_angle3(j)=beta3;%当前偏航角
   fuyang_angle3(j)=alpha3;
   distance31(j)=sqrt((T3(1)-T1(1))^2+(T3(2)-T1(2))^2+(T3(3)-T1(3))^2);
   distance3(j)=sqrt((T3(1)-Xsum3(1))^2+(T3(2)-Xsum3(2))^2+(T3(3)-Xsum3(3))^2);
   
     
  [U3(j),R3,Q3,Rotatc3(j),pitchc3(j)]=Follower_PI3(distance3,Vm,Position_angle3(j),fuyang_angle3(j),R3,Q3,Rotat3,pitch3,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance3(j)<0.5)
     Xnext3(1)=T3(1);
     Xnext3(2)=T3(2);
     Xnext3(3)=T3(3);
     T3=Xnext3; 
        else
    Xnext3(1)=T3(1)+U3(j)*cos(Q3(j-1)+pitchc3(j)/2)*cos(R3(j-1)+Rotatc3(j)/2);
    Xnext3(2)=T3(2)+U3(j)*cos(Q3(j-1)+pitchc3(j)/2)*sin(R3(j-1)+Rotatc3(j)/2);
    Xnext3(3)=T3(3)+U3(j)*sin(Q3(j-1)+pitchc3(j)/2);
    T3=Xnext3;
    end
    
    %……………………………………………………………………………………4
    Xsum4=Rb1(T1,R1(j)-pi/2,Q1(j),-2*l*sin(v),-2*l*cos(v),0);
%     Xsum4(1)=T1(1)-2*l*sin(v)*cos(R1(j)-pi/2)+2*l*cos(v)*sin(R1(j)-pi/2);
%     Xsum4(2)=T1(2)-2*l*cos(v)*cos(R1(j)-pi/2)-2*l*sin(v)*sin(R1(j)-pi/2);
%     Xsum4(3)=T1(3);
    
   [beta4,alpha4] = compute_angle3(T4,Xsum4);%计算偏航角
   Position_angle4(j)=beta4;%当前偏航角
   fuyang_angle4(j)=alpha4;
   distance41(j)=sqrt((T4(1)-T1(1))^2+(T4(2)-T1(2))^2+(T4(3)-T1(3))^2);
   distance4(j)=sqrt((T4(1)-Xsum4(1))^2+(T4(2)-Xsum4(2))^2+(T4(3)-Xsum4(3))^2);
   
     
  [U4(j),R4,Q4,Rotatc4(j),pitchc4(j)]=Follower_PI3(distance4,Vm,Position_angle4(j),fuyang_angle4(j),R4,Q4,Rotat4,pitch4,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance4(j)<0.5)
     Xnext4(1)=T4(1);
     Xnext4(2)=T4(2);
     Xnext4(3)=T4(3);
     T4=Xnext4; 
        else
    Xnext4(1)=T4(1)+U4(j)*cos(Q4(j-1)+pitchc4(j)/2)*cos(R4(j-1)+Rotatc4(j)/2);
    Xnext4(2)=T4(2)+U4(j)*cos(Q4(j-1)+pitchc4(j)/2)*sin(R4(j-1)+Rotatc4(j)/2);
    Xnext4(3)=T4(3)+U4(j)*sin(Q4(j-1)+pitchc4(j)/2);
    T4=Xnext4;
    end
    
    %……………………………………………………………………………………5
    Xsum5=Rb1(T1,R1(j)-pi/2,Q1(j),0,-2*l*cos(v),0);
%     Xsum5(1)=T1(1)+2*l*cos(v)*sin(R1(j)-pi/2);
%     Xsum5(2)=T1(2)-2*l*cos(v)*cos(R1(j)-pi/2);
%     Xsum5(3)=T1(3);
    
   [beta5,alpha5] = compute_angle3(T5,Xsum5);%计算偏航角
   Position_angle5(j)=beta5;%当前偏航角
   fuyang_angle5(j)=alpha5;
   distance51(j)=sqrt((T5(1)-T1(1))^2+(T5(2)-T1(2))^2+(T5(3)-T1(3))^2);
   distance5(j)=sqrt((T5(1)-Xsum5(1))^2+(T5(2)-Xsum5(2))^2+(T5(3)-Xsum5(3))^2);
   
     
  [U5(j),R5,Q5,Rotatc5(j),pitchc5(j)]=Follower_PI3(distance5,Vm,Position_angle5(j),fuyang_angle5(j),R5,Q5,Rotat5,pitch5,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance5(j)<0.5)
     Xnext5(1)=T5(1);
     Xnext5(2)=T5(2);
     Xnext5(3)=T5(3);
     T5=Xnext5; 
        else
    Xnext5(1)=T5(1)+U5(j)*cos(Q5(j-1)+pitchc5(j)/2)*cos(R5(j-1)+Rotatc5(j)/2);
    Xnext5(2)=T5(2)+U5(j)*cos(Q5(j-1)+pitchc5(j)/2)*sin(R5(j-1)+Rotatc5(j)/2);
    Xnext5(3)=T5(3)+U5(j)*sin(Q5(j-1)+pitchc5(j)/2);
    T5=Xnext5;
    end
    
  %……………………………………………………………………………………6
   Xsum6=Rb1(T1,R1(j)-pi/2,Q1(j),2*l*sin(v),-2*l*cos(v),0);
%    Xsum6(1)=T1(1)+2*l*sin(v)*cos(R1(j)-pi/2)+2*l*cos(v)*sin(R1(j)-pi/2);
%    Xsum6(2)=T1(2)-2*l*cos(v)*cos(R1(j)-pi/2)+2*l*sin(v)*sin(R1(j)-pi/2);
%    Xsum6(3)=T1(3); 
    
   [beta6,alpha6] = compute_angle3(T6,Xsum6);%计算偏航角
   Position_angle6(j)=beta6;%当前偏航角
   fuyang_angle6(j)=alpha6;
   distance61(j)=sqrt((T6(1)-T1(1))^2+(T6(2)-T1(2))^2+(T6(3)-T1(3))^2);
   distance6(j)=sqrt((T6(1)-Xsum6(1))^2+(T6(2)-Xsum6(2))^2+(T6(3)-Xsum6(3))^2);
   
     
  [U6(j),R6,Q6,Rotatc6(j),pitchc6(j)]=Follower_PI3(distance6,Vm,Position_angle6(j),fuyang_angle6(j),R6,Q6,Rotat6,pitch6,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance6(j)<0.5)
     Xnext6(1)=T6(1);
     Xnext6(2)=T6(2);
     Xnext6(3)=T6(3);
     T6=Xnext6; 
        else
    Xnext6(1)=T6(1)+U6(j)*cos(Q6(j-1)+pitchc6(j)/2)*cos(R6(j-1)+Rotatc6(j)/2);
    Xnext6(2)=T6(2)+U6(j)*cos(Q6(j-1)+pitchc6(j)/2)*sin(R6(j-1)+Rotatc6(j)/2);
    Xnext6(3)=T6(3)+U6(j)*sin(Q6(j-1)+pitchc6(j)/2);
    T6=Xnext6;
    end
    
  %……………………………………………………………………………………7
    Xsum7=Rb1(T1,R1(j)-pi/2,Q1(j),-3*l*sin(v),-3*l*cos(v),0);
%     Xsum7(1)=T1(1)-3*l*sin(v)*cos(R1(j)-pi/2)+3*l*cos(v)*sin(R1(j)-pi/2);
%     Xsum7(2)=T1(2)-3*l*cos(v)*cos(R1(j)-pi/2)-3*l*sin(v)*sin(R1(j)-pi/2);
%     Xsum7(3)=T1(3);
    
   [beta7,alpha7] = compute_angle3(T7,Xsum7);%计算偏航角
   Position_angle7(j)=beta7;%当前偏航角
   fuyang_angle7(j)=alpha7;
   distance71(j)=sqrt((T7(1)-T1(1))^2+(T7(2)-T1(2))^2+(T7(3)-T1(3))^2);
   distance7(j)=sqrt((T7(1)-Xsum7(1))^2+(T7(2)-Xsum7(2))^2+(T7(3)-Xsum7(3))^2);
   
     
  [U7(j),R7,Q7,Rotatc7(j),pitchc7(j)]=Follower_PI3(distance7,Vm,Position_angle7(j),fuyang_angle7(j),R7,Q7,Rotat7,pitch7,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance7(j)<0.5)
     Xnext7(1)=T7(1);
     Xnext7(2)=T7(2);
     Xnext7(3)=T7(3);
     T7=Xnext7; 
        else
    Xnext7(1)=T7(1)+U7(j)*cos(Q7(j-1)+pitchc7(j)/2)*cos(R7(j-1)+Rotatc7(j)/2);
    Xnext7(2)=T7(2)+U7(j)*cos(Q7(j-1)+pitchc7(j)/2)*sin(R7(j-1)+Rotatc7(j)/2);
    Xnext7(3)=T7(3)+U7(j)*sin(Q7(j-1)+pitchc7(j)/2);
    T7=Xnext7;
    end    
   
   %……………………………………………………………………………………8
   Xsum8=Rb1(T1,R1(j)-pi/2,Q1(j),+3*l*sin(v),-3*l*cos(v),0);
%     Xsum8(1)=T1(1)+3*l*sin(v)*cos(R1(j)-pi/2)+3*l*cos(v)*sin(R1(j)-pi/2);
%     Xsum8(2)=T1(2)-3*l*cos(v)*cos(R1(j)-pi/2)+3*l*sin(v)*sin(R1(j)-pi/2);
%     Xsum8(3)=T1(3);
    
   [beta8,alpha8] = compute_angle3(T8,Xsum8);%计算偏航角
   Position_angle8(j)=beta8;%当前偏航角
   fuyang_angle8(j)=alpha8;
   distance81(j)=sqrt((T8(1)-T1(1))^2+(T8(2)-T1(2))^2+(T8(3)-T1(3))^2);
   distance8(j)=sqrt((T8(1)-Xsum8(1))^2+(T8(2)-Xsum8(2))^2+(T8(3)-Xsum8(3))^2);
   
     
  [U8(j),R8,Q8,Rotatc8(j),pitchc8(j)]=Follower_PI3(distance8,Vm,Position_angle8(j),fuyang_angle8(j),R8,Q8,Rotat8,pitch8,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance8(j)<0.5)
     Xnext8(1)=T8(1);
     Xnext8(2)=T8(2);
     Xnext8(3)=T8(3);
     T8=Xnext8; 
        else
    Xnext8(1)=T8(1)+U8(j)*cos(Q8(j-1)+pitchc8(j)/2)*cos(R8(j-1)+Rotatc8(j)/2);
    Xnext8(2)=T8(2)+U8(j)*cos(Q8(j-1)+pitchc8(j)/2)*sin(R8(j-1)+Rotatc8(j)/2);
    Xnext8(3)=T8(3)+U8(j)*sin(Q8(j-1)+pitchc8(j)/2);
    T8=Xnext8;
    end   
    
   %……………………………………………………………………………………9
    Xsum9=Rb1(T1,R1(j)-pi/2,Q1(j),0,-3*l*cos(v),0);
%     Xsum9(1)=T1(1)+3*l*cos(v)*sin(R1(j)-pi/2);
%     Xsum9(2)=T1(2)-3*l*cos(v)*cos(R1(j)-pi/2);
%     Xsum9(3)=T1(3);
    
   [beta9,alpha9] = compute_angle3(T9,Xsum9);%计算偏航角
   Position_angle9(j)=beta9;%当前偏航角
   fuyang_angle9(j)=alpha9;
   distance91(j)=sqrt((T9(1)-T1(1))^2+(T9(2)-T1(2))^2+(T9(3)-T1(3))^2);
   distance9(j)=sqrt((T9(1)-Xsum9(1))^2+(T9(2)-Xsum9(2))^2+(T9(3)-Xsum9(3))^2);
   
     
  [U9(j),R9,Q9,Rotatc9(j),pitchc9(j)]=Follower_PI3(distance9,Vm,Position_angle9(j),fuyang_angle9(j),R9,Q9,Rotat9,pitch9,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance9(j)<0.5)
     Xnext9(1)=T9(1);
     Xnext9(2)=T9(2);
     Xnext9(3)=T9(3);
     T9=Xnext9; 
        else
    Xnext9(1)=T9(1)+U9(j)*cos(Q9(j-1)+pitchc9(j)/2)*cos(R9(j-1)+Rotatc9(j)/2);
    Xnext9(2)=T9(2)+U9(j)*cos(Q9(j-1)+pitchc9(j)/2)*sin(R9(j-1)+Rotatc9(j)/2);
    Xnext9(3)=T9(3)+U9(j)*sin(Q9(j-1)+pitchc9(j)/2);
    T9=Xnext9;
    end       
    
    
 if(distance2(j)<10&&distance3(j)<10&&distance4(j)<10&&distance5(j)<10&&distance6(j)<10&&distance7(j)<10&&distance8(j)<10&&distance9(j)<10)%…………………………………………………………到达目的地停止并画动图
    break;
 end
  h=plot3(T1(1),T1(2),T1(3),'.r',T2(1),T2(2),T2(3),'.b',T3(1),T3(2),T3(3),'.g',T4(1),T4(2),T4(3),'.y',T5(1),T5(2),T5(3),'.c',T6(1),T6(2),T6(3),'.m',T7(1),T7(2),T7(3),'.k',T8(1),T8(2),T8(3),'.b',T9(1),T9(2),T9(3),'.g');
  axis equal
  grid on;
  refreshdata(h) %指定数据源时刷新图中的数据
  drawnow %刷新屏幕
  hold on
end
plot3(T1(1),T1(2),T1(3),'o','Markersize',10,'color','r');
hold on
plot3(T2(1),T2(2),T2(3),'o','Markersize',10,'color','b');
hold on
plot3(T3(1),T3(2),T3(3),'o','Markersize',10,'color','g');
hold on
plot3(T4(1),T4(2),T4(3),'o','Markersize',10,'color','y');
hold on
plot3(T5(1),T5(2),T5(3),'o','Markersize',10,'color','c');
hold on
plot3(T6(1),T6(2),T6(3),'o','Markersize',10,'color','m');
hold on
plot3(T7(1),T7(2),T7(3),'o','Markersize',10,'color','k');
hold on
plot3(T8(1),T8(2),T8(3),'o','Markersize',10,'color','b');
hold on
plot3(T9(1),T9(2),T9(3),'o','Markersize',10,'color','g');
hold on
line([T1(1),T2(1)],[T1(2),T2(2)],[T1(3),T2(3)]),line([T1(1),T3(1)],[T1(2),T3(2)],[T1(3),T3(3)]),line([T2(1),T4(1)],[T2(2),T4(2)],[T2(3),T4(3)]); 
line([T7(1),T4(1)],[T7(2),T4(2)],[T7(3),T4(3)]),line([T3(1),T6(1)],[T3(2),T6(2)],[T3(3),T6(3)]),line([T6(1),T8(1)],[T6(2),T8(2)],[T6(3),T8(3)]);
line([T8(1),T9(1)],[T8(2),T9(2)],[T8(3),T9(3)]),line([T7(1),T9(1)],[T7(2),T9(2)],[T7(3),T9(3)]);%画出队形形状



xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');
legend('UAV_1','UAV_2','UAV_3','UAV_4','UAV_5','UAV_6','UAV_7','UAV_8','UAV_9');
hold on
grid on;
shg;

figure(2)
h1=plot(t,distance21-l-75,'-b+','LineWidth',2);hold on;
h2=plot(t,distance31-l-80,'-g*','LineWidth',2);hold on;
h3=plot(t,distance41-2*l-70,'-yx','LineWidth',2);hold on;
h4=plot(t,distance51-l*sqrt(3)-80,'-ks','LineWidth',2);hold on;
h5=plot(t,distance61-2*l-75,'-md','LineWidth',2);hold on;
h6=plot(t,distance71-3*l-70,'-k<','LineWidth',2);hold on;
h7=plot(t,distance81-3*l*sqrt(3)/2-150,'-b>','LineWidth',2);hold on;
h8=plot(t,distance91-3*l,'-go','LineWidth',2);hold on;
axis([0,648,-100,100]);
xlabel('\fontsize{12}\bft(s)');
ylabel('\fontsize{12}\bfe_i_j');
lgd1=legend([h1,h2,h3,h4],'e_1_2','e_1_3','e_1_4','e_1_5','orientation','horizontal','location','north');
legend boxoff;
ah=axes('position',get(gca,'position'),'visible','off');
lgd2=legend(ah,[h5,h6,h7,h8],'e_1_6','e_1_7','e_1_8','e_1_9','orientation','horizontal','location','north');
legend boxoff;

figure(3)
h1=plot(t,U2,'-b+','LineWidth',2);hold on;
h2=plot(t,U3,'-g*','LineWidth',2);hold on;
h3=plot(t,U4,'-yx','LineWidth',2);hold on;
h4=plot(t,U5,'-ks','LineWidth',2);hold on;
h5=plot(t,U6,'-md','LineWidth',2);hold on;
h6=plot(t,U7,'-k<','LineWidth',2);hold on;
h7=plot(t,U8,'-b>','LineWidth',2);hold on;
h8=plot(t,U9,'-go','LineWidth',2);hold on;
axis([0,648,0,20]);
xlabel('\fontsize{12}\bft(s)');
ylabel('\fontsize{12}\bfV(m/s)');
lgd1=legend([h1,h2,h3,h4],'UAV_2','UAV_3','UAV_4','UAV_5','orientation','horizontal','location','north');
legend boxoff;
ah=axes('position',get(gca,'position'),'visible','off');
lgd2=legend(ah,[h5,h6,h7,h8],'UAV_6','UAV_7','UAV_8','UAV_9','orientation','horizontal','location','north');
legend boxoff;


