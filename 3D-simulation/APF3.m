clc
clear all;
T1=[200 0 300];%Leader
T2=[130 -70 300];%1
T3=[270 -70 300];%2
T4=[200 -140 300];%3
T=[T1;T2;T3;T4];  %初始参数矩阵
% plot(x,y,'.','Markersize',50,'color',[0 0 0]);
plot3(T1(1),T1(2),T1(3),'o','Markersize',10,'color','r');%画出初始位置
%text(T1(1)-5,T1(2)-5,T1(3)-5,'UAV_1');%标注名称
hold on
plot3(T2(1),T2(2),T2(3),'o','Markersize',10,'color','b');
%text(T2(1)-5,T2(2)-5,T2(3)-5,'UAV_2');
hold on
plot3(T3(1),T3(2),T3(3),'o','Markersize',10,'color','g');
%text(T3(1)-5,T3(2)-5,T3(3)-5,'UAV_3');
hold on
plot3(T4(1),T4(2),T4(3),'o','Markersize',10,'color','y');
%text(T4(1)-5,T4(2)-5,T4(3)-5,'UAV_4');
hold on

k=1;%引力增益
l=200;%队形距离
%v=pi/4;%队形夹角
Vl=10;%leader速度
V_M=20;%follower最大速度
V_m=0;%follower最小速度
J=200;%时间
t=0:J;%时间序列
Rm=1.5;%跟随无人机最大航向角向心加速度
Qm=0.1;%最大俯仰角速度

kp=0.5;%PI系数
ki=0.1;
krp=0.9;
kri=0.4;
kqp=0.9;
kqi=0.4;

F=[0  , 0  ,0 ;
    -l, -l, 0;
    -l, l, 0;
    -2*l, 0, 0];  %编队矩阵，与长机之间的相对位移
%-----------------------------------------------leader
R1=0.*t; %初始化航向角速度 单位：rad/s
R1(1)=0; %初始朝向角
Position_angle1=0.*t;
Rotat1=0.*t;
Rotatc1=0.*t;
Q1=0.*t;%初始化俯仰角速度 单位：rad/s
Q1(1)=0; %初始俯仰角
fuyang_angle1=0.*t;
pitch1=0.*t;
pitchc1=0.*t;

%-----------------------------------------------follower_1
R2=0.*t; %初始化角速度 单位：rad/s
R2(1)=pi/2; %初始朝向角
Position_angle2=0.*t;
Rotat2=0.*t;
Rotatc2=0.*t;
Q2=0.*t;%初始化俯仰角速度 单位：rad/s
Q2(1)=0; %初始俯仰角
fuyang_angle2=0.*t;
pitch2=0.*t;
pitchc2=0.*t;

distance21(1)=sqrt((T2(1)-T1(1))^2+(T2(2)-T1(2))^2+(T2(3)-T1(3))^2);

%-----------------------------------------------follower_2
R3=0.*t; %初始化角速度 单位：rad/s
R3(1)=pi/2; %初始朝向角
Position_angle3=0.*t;
Rotat3=0.*t;
Rotatc3=0.*t;
Q3=0.*t;%初始化俯仰角速度 单位：rad/s
Q3(1)=0; %初始俯仰角
fuyang_angle3=0.*t;
pitch3=0.*t;
pitchc3=0.*t;

distance31(1)=sqrt((T3(1)-T1(1))^2+(T3(2)-T1(2))^2+(T3(3)-T1(3))^2);

%-----------------------------------------------follower_3
R4=0.*t; %初始化角速度 单位：rad/s
R4(1)=pi/2; %初始朝向角
Position_angle4=0.*t;
Rotat4=0.*t;
Rotatc4=0.*t;
Q4=0.*t;%初始化俯仰角速度 单位：rad/s
Q4(1)=0; %初始俯仰角
fuyang_angle4=0.*t;
pitch4=0.*t;
pitchc4=0.*t;

distance41(1)=sqrt((T4(1)-T1(1))^2+(T4(2)-T1(2))^2+(T4(3)-T1(3))^2);

U1(1)=Vl;
U2(1)=Vl;
U3(1)=Vl;
U4(1)=Vl;

l=300;

F=[0,       0,          0;
    -l/2*sqrt(3),l/2,0;
    -l/2*sqrt(3),-2*l/3,0;
    -l,0,0];

T=[T1;T2;T3;T4];
k_pot = 130; % 人工势场斥力的系数
V_m=4; 
a=[norm(T2-T3, 2)];
b=[norm(T2-T4, 2)];
c=[norm(T3-T4, 2)];
d=[norm(T1-T2, 2)];
e=[norm(T1-T3, 2)];
f=[norm(T1-T4, 2)];
Position_angle1(1)=pi/2;
J=650;
t=1:J;

for j=2:J
  %………………………………………………………………………………领航无人机完成动作
  
  distance1(j)=0;
  if j<300&&j>=101  %-----------------------------------------设置领航无人机的操控输入
      u_ctrl1=[0,pi/200,0];
      else if j<420 && j>=400
              u_ctrl1=[0.001 0 0.01];
          else if j>=480 && j<500
                  u_ctrl1=[-0.001 0 -0.01];
              else 
                  u_ctrl1=[0 0 0];
              end
              
          end
      
  end
  
  [Vl_next, phi1_next, theta1_next, T1_next]=Leader_P(u_ctrl1,T(1,:),Vl,Position_angle1(j-1),fuyang_angle1(j-1));

%   [beta1,alpha1] = compute_angle3(T1,Xsum1);%计算偏航角
%    [Fatx1,Faty1,Fatz1,Fatr1]=compute_weiyi3(T1,Xsum1,k,beta1,alpha1);
%     Fsumy1=Faty1;%y位移量
%     Fsumx1=Fatx1;%x位移量
%     Fsumz1=Fatz1;%z位移量
%     Fsumr1=Fatr1;
%    Position_angle1(j)=atan(Fsumy1/Fsumx1);%当前偏航角
%    fuyang_angle1(j)=atan(Fsumz1/Fsumr1);
   Position_angle1(j) = phi1_next;
    fuyang_angle1(j) = theta1_next;
    Vl = Vl_next;
    T(1,:) = T1_next;
%    distance1(j)=sqrt((T1(1)-Xsum1(1))^2+(T1(2)-Xsum1(2))^2+(T1(3)-Xsum1(3))^2);%当前距离期望位置距离
%    
%         if(distance1(j)<0.5)
%      Xnext1(1)=T1(1);
%      Xnext1(2)=T1(2);
%      Xnext1(3)=T1(3);
%      T1=Xnext1;%到达
%         else
%     Xnext1(1)=T1(1)+Vl*cos(fuyang_angle1(j))*cos(Position_angle1(j));%Leader下一步位置
%     Xnext1(2)=T1(2)+Vl*cos(fuyang_angle1(j))*sin(Position_angle1(j));
%     Xnext1(3)=T1(3)+Vl*sin(fuyang_angle1(j));
%     T1=Xnext1;
%         end
       
       Transmatrix=[cos(Position_angle1(j-1)),sin(Position_angle1(j-1)),0;
        -sin(Position_angle1(j-1)), cos(Position_angle1(j-1)),0;
        0,                       0,                     1]...
        *[cos(fuyang_angle1(j-1)),0,sin(fuyang_angle1(j-1));
        0,                      1,              0;
        -sin(fuyang_angle1(j-1)),0,cos(fuyang_angle1(j-1))];
 
  %……………………………………………………………………………………跟随无人机1移动
  Xsum2 = T(1,:) + F(2,:)*Transmatrix; 
  pot2 = [0,0,0];
  for i_follower = 1:4
      pot2=pot2+k_pot*potential(2,i_follower,T);
  end
  Xsum2 = Xsum2 + pot2;
%   Xsum2(1)=T1(1)-l/2;
%    Xsum2(2)=T1(2)-l/2*sqrt(3);%根据Leader确定期望位置
%    Xsum2(3)=T1(3); 
  [beta2,alpha2] = compute_angle3(T(2,:),Xsum2);
   [Fatx2,Faty2,Fatz2,Fatr2]=compute_weiyi3(T(2,:),Xsum2,k,beta2,alpha2);
    Fsumy2=Faty2;%y位移量
    Fsumx2=Fatx2;%x位移量
    Fsumz2=Fatz2;%z位移量
    Fsumr2=Fatr2;
   Position_angle2(j)=beta2;
   fuyang_angle2(j)=atan(Fsumz2/Fsumr2);
   T2=T(2,:);
   distance21(j)=sqrt((T2(1)-T1(1))^2+(T2(2)-T1(2))^2+(T2(3)-T1(3))^2);
   distance2(j)=sqrt((T2(1)-Xsum2(1))^2+(T2(2)-Xsum2(2))^2+(T2(3)-Xsum2(3))^2);
   
     
  [U2(j),R2,Q2,Rotatc2(j),pitchc2(j)]=Follower_PI3(distance2,V_M,V_m,Position_angle2(j),fuyang_angle2(j),R2,Q2,Rotat2,pitch2,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance2(j)<0.5)
     Xnext2(1)=T2(1);
     Xnext2(2)=T2(2);
     Xnext2(3)=T2(3);
     T(2,:)=Xnext2; 
        else
    Xnext2(1)=T2(1)+U2(j)*cos(Q2(j-1)+pitchc2(j)/2)*cos(R2(j-1)+Rotatc2(j)/2);
    Xnext2(2)=T2(2)+U2(j)*cos(Q2(j-1)+pitchc2(j)/2)*sin(R2(j-1)+Rotatc2(j)/2);
    Xnext2(3)=T2(3)+U2(j)*sin(Q2(j-1)+pitchc2(j)/2);
    T(2,:)=Xnext2;
    end

    %……………………………………………………………………………………跟随无人机2移动
    Xsum3 = T(1,:) + F(3,:)*Transmatrix;
    pot3 = [0,0,0];
  for i_follower = 1:4
      pot3=pot3+k_pot*potential(3,i_follower,T);
  end
  Xsum3 = Xsum3 + pot3;
%     Xsum3(1)=T1(1)+l/2;
%     Xsum3(2)=T1(2)-l/2*sqrt(3);
%     Xsum3(3)=T1(3);
    
   [beta3,alpha3] = compute_angle3(T(3,:),Xsum3);
   [Fatx3,Faty3,Fatz3,Fatr3]=compute_weiyi3(T(3,:),Xsum3,k,beta3,alpha3);
    Fsumy3=Faty3;%y位移量
    Fsumx3=Fatx3;%x位移量
    Fsumz3=Fatz3;%z位移量
    Fsumr3=Fatr3;
    T3=T(3,:);
   Position_angle3(j)=beta3;%偏航角
   fuyang_angle3(j)=atan(Fsumz3/Fsumr3);
   distance31(j)=sqrt((T3(1)-T1(1))^2+(T3(2)-T1(2))^2+(T3(3)-T1(3))^2);
   distance3(j)=sqrt((T3(1)-Xsum3(1))^2+(T3(2)-Xsum3(2))^2+(T3(3)-Xsum3(3))^2);
   
     
  [U3(j),R3,Q3,Rotatc3(j),pitchc3(j)]=Follower_PI3(distance3,V_M,V_m,Position_angle3(j),fuyang_angle3(j),R3,Q3,Rotat3,pitch3,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance3(j)<0.5)
     Xnext3(1)=T3(1);
     Xnext3(2)=T3(2);
     Xnext3(3)=T3(3);
     T(3,:)=Xnext3; 
        else
    Xnext3(1)=T3(1)+U3(j)*cos(Q3(j-1)+pitchc3(j)/2)*cos(R3(j-1)+Rotatc3(j)/2);
    Xnext3(2)=T3(2)+U3(j)*cos(Q3(j-1)+pitchc3(j)/2)*sin(R3(j-1)+Rotatc3(j)/2);
    Xnext3(3)=T3(3)+U3(j)*sin(Q3(j-1)+pitchc3(j)/2);
    T(3,:)=Xnext3;
    end
    
     %……………………………………………………………………………………跟随无人机3移动
    Xsum4 = T(1,:) + F(4,:)*Transmatrix;
    pot4 = [0,0,0];
  for i_follower = 1:4
      pot4=pot4+k_pot*potential(4,i_follower,T);
  end
  Xsum4 = Xsum4 + pot4;
%      Xsum4(1)=T1(1);
%     Xsum4(2)=T1(2)-l/2*sqrt(3);
%     Xsum4(3)=T1(3)-l;
    
   [beta4,alpha4] = compute_angle3(T(4,:),Xsum4);
   [Fatx4,Faty4,Fatz4,Fatr4]=compute_weiyi3(T(4,:),Xsum4,k,beta4,alpha4);
    Fsumy4=Faty4;%y位移量
    Fsumx4=Fatx4;%x位移量
    Fsumz4=Fatz4;%z位移量
    Fsumr4=Fatr4;
   Position_angle4(j)=beta4;
   fuyang_angle4(j)=alpha4;
   T4=T(4,:);
   distance41(j)=sqrt((T4(1)-T1(1))^2+(T4(2)-T1(2))^2+(T4(3)-T1(3))^2);
   distance4(j)=sqrt((T4(1)-Xsum4(1))^2+(T4(2)-Xsum4(2))^2+(T4(3)-Xsum4(3))^2);
   
     
  [U4(j),R4,Q4,Rotatc4(j),pitchc4(j)]=Follower_PI3(distance4,V_M,V_m,Position_angle4(j),fuyang_angle4(j),R4,Q4,Rotat4,pitch4,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    if(distance4(j)<0.5)
     Xnext4(1)=T4(1);
     Xnext4(2)=T4(2);
     Xnext4(3)=T4(3);
     T(4,:)=Xnext4; 
        else
    Xnext4(1)=T4(1)+U4(j)*cos(Q4(j-1)+pitchc4(j)/2)*cos(R4(j-1)+Rotatc4(j)/2);
    Xnext4(2)=T4(2)+U4(j)*cos(Q4(j-1)+pitchc4(j)/2)*sin(R4(j-1)+Rotatc4(j)/2);
    Xnext4(3)=T4(3)+U4(j)*sin(Q4(j-1)+pitchc4(j)/2);
    T(4,:)=Xnext4;
    end
    
    a=[a,norm(T2-T3,2)];
    b=[b,norm(T2-T4, 2)];
    c=[c,norm(T3-T4, 2)];
    d=[d,norm(T2-T1, 2)];
    e=[e,norm(T3-T1, 2)];
    f=[f,norm(T4-T1, 2)];
        
    
 if(distance1(j)<1&&distance2(j)<1&&distance3(j)<1&&distance4(j)<1&&j>500)%…………………………………………………………到达目的地停止并画动图
    break;
 end
  h=plot3(T(1,1),T(1,2),T(1,3),'.r',T(2,1),T(2,2),T(2,3),'.b',T(3,1),T(3,2),T(3,3),'.g',T(4,1),T(4,2),T(4,3),'.y');
  axis equal
  grid on;
  refreshdata(h) %指定数据源时刷新图中的数据
  drawnow %刷新屏幕
  hold on
end
plot3(T(1,1),T(1,2),T(1,3),'o','Markersize',10,'color','r');
hold on
plot3(T(2,1),T(2,2),T(2,3),'o','Markersize',10,'color','b');
hold on
plot3(T(3,1),T(3,2),T(3,3),'o','Markersize',10,'color','g');
hold on
plot3(T(4,1),T(4,2),T(4,3),'o','Markersize',10,'color','y');
hold on
line([T(1,1),T(2,1)],[T(1,2),T(2,2)],[T(1,3),T(2,3)]),line([T(1,1),T(3,1)],[T(1,2),T(3,2)],[T(1,3),T(3,3)]),line([T2(1),T4(1)],[T2(2),T4(2)],[T2(3),T4(3)]),line([T3(1),T4(1)],[T3(2),T4(2)],[T3(3),T4(3)]); %画出队形形状





xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');
legend('UAV_1','UAV_2','UAV_3','UAV_4');
hold on
grid on;
shg;

figure(2)
plot(t,a,'-b+','LineWidth',2),hold on;
plot(t,b,'-g*','LineWidth',2),hold on;
plot(t,c,'-yx','LineWidth',2),hold on;
legend('|p_2-p_3|','|p_2-p_4|','|p_3-p_4|');
axis([0,200,0,1300]);
xlabel('\fontsize{12}\bft(s)');
ylabel('\fontsize{12}\bfL(m)');
