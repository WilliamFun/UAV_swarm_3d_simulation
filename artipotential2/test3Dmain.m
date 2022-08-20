clc
clear all;
T=[200 0 500;%Leader             %��ʼλ���������
    -300 -200 300;%1
    200 -500 200;%2
    0 -600 100;%3
    0 -800 100;%4
    0 -450 80;%5
    100 250 0;%6
    50 200 0;%7
    20 100 0;%8
    40 50 0;%9
    160 0 0;%10
    80 -130 0;%11
    100 -60 0;%12
    200 -500 0;%13
    100 -400 0;%14
    0 -300 0;%15
    -100 -200 0;%16
    -200 -100 0;%17
    -300 0 0;%18
    -300 300 0;%19
    -400 400 0];%20

plot3(T(1,1),T(1,2),T(1,3),'o','Markersize',10,'color','r');%������ʼλ��
% text(T(1,1)-5,T(1,2)-5,T(1,3)-5,'UAV_1');%��ע����
hold on
plot3(T(2,1),T(2,2),T(2,3),'o','Markersize',10,'color','b');
hold on
plot3(T(3,1),T(3,2),T(3,3),'o','Markersize',10,'color','g');
hold on
plot3(T(4,1),T(4,2),T(4,3),'o','Markersize',10,'color','y');
hold on
plot3(T(5,1),T(5,2),T(5,3),'o','Markersize',10,'color','r');%������ʼλ��
hold on
plot3(T(6,1),T(6,2),T(6,3),'o','Markersize',10,'color','b');
hold on
plot3(T(7,1),T(7,2),T(7,3),'o','Markersize',10,'color','g');
hold on
plot3(T(8,1),T(8,2),T(8,3),'o','Markersize',10,'color','y');
hold on
plot3(T(9,1),T(9,2),T(9,3),'o','Markersize',10,'color','r');%������ʼλ��
hold on
plot3(T(10,1),T(10,2),T(10,3),'o','Markersize',10,'color','b');
hold on
plot3(T(11,1),T(11,2),T(11,3),'o','Markersize',10,'color','g');
hold on
plot3(T(12,1),T(12,2),T(12,3),'o','Markersize',10,'color','y');
hold on
plot3(T(13,1),T(13,2),T(13,3),'o','Markersize',10,'color','r');%������ʼλ��
hold on
plot3(T(14,1),T(14,2),T(14,3),'o','Markersize',10,'color','b');
hold on
plot3(T(15,1),T(15,2),T(15,3),'o','Markersize',10,'color','g');
hold on
plot3(T(16,1),T(16,2),T(16,3),'o','Markersize',10,'color','y');
hold on
plot3(T(17,1),T(17,2),T(17,3),'o','Markersize',10,'color','r');%������ʼλ��
hold on
plot3(T(18,1),T(18,2),T(18,3),'o','Markersize',10,'color','b');
hold on
plot3(T(19,1),T(19,2),T(19,3),'o','Markersize',10,'color','g');
hold on
plot3(T(20,1),T(20,2),T(20,3),'o','Markersize',10,'color','y');
hold on
plot3(T(21,1),T(21,2),T(21,3),'o','Markersize',10,'color','r');%������ʼλ��
hold on




k=1;%��������
l=300;%���ξ���
%v=pi/4;%���μн�
k_pot = 130; % �˹��Ƴ�������ϵ��
V_L=100;%leader����
V_M=200;%follower��󲽳�
V_m=40;%follower��С����
J=200;%����
t=0:J;
Rm=20;%�������˻����������ļ��ٶ�
Qm=0.1;%��������ٶ�

kp=0.5;%PIϵ��
ki=0.1;
krp=0.9;
kri=0.4;
kqp=0.9;
kqi=0.4;

F=[0  , 0  ,0 ;     %���ξ���
    -l/2, -l, 0;
    -l/2, l, 0;
    -l, -2*l, 0
    -l, 0, 0;
    -l, 2*l, 0;
    -3*l/2, -3*l, 0;
    -3*l/2, -l, 0;
    -3*l/2, l, 0;
    -3*l/2, 3*l, 0;
    -2*l, -4*l, 0;
    -2*l, -2*l, 0;
    -2*l, 0, 0;
    -2*l, 2*l, 0;
    -2*l, 4*l, 0;
    -5*l/2, -5*l, 0;
    -5*l/2, -3*l, 0;
    -5*l/2, -l, 0;
    -5*l/2, l, 0;
    -5*l/2, 3*l, 0;
    -5*l/2, 5*l, 0;];  %��Ӿ����볤��֮������λ��; ���������

U=zeros(21,10*length(t)); %��ʼ���ٶȴ�С ��λm/s
R=zeros(21,10*length(t)); %��ʼ������� ��λ��rad
Position_angle=zeros(21,10*length(t));
Rotat=zeros(21,10*length(t));
Rotatc=zeros(21,10*length(t)); %��ʼ��������ٶ� ��λ��rad/s
Q=zeros(21,10*length(t));%��ʼ�������� ��λ��rad
fuyang_angle=zeros(21,10*length(t));
pitch=zeros(21,10*length(t));
pitchc=zeros(21,10*length(t)); %��ʼ��������ٶ� ��λ��rad/s
distancex1=zeros(21,10*length(t)); %��ʼ���Ż��볤�������
distance=zeros(21,10*length(t));


%-----------------------------------------------leader
R(1,1)=0; %��ʼ�����
U(1,1)=V_L;

%-----------------------------------------------followers
R(2,1)=pi/2; %��ʼ�����
R(3,1)=pi/3; %��ʼ�����
Q(3,1)=0.13; %��ʼ������
R(4,1)=pi/3; %��ʼ�����
R(5,1)=-pi/3;
R(6,1)=-pi/2;

min_temp=655360;
min_distance=[];

for i_init=2:21
    U(i_init,1)=V_L;
    distancex1(i_init,1)=sqrt((T(i_init,1)-T(1,1))^2+(T(i_init,2)-T(1,2))^2+(T(i_init,3)-T(1,3))^2); %�볤���ĳ�ʼ�������
end

%���������С����
for i_distance=1:20
    for j_distance=i_distance+1:21
        distanceij=norm(T(i_distance,:)-T(j_distance,:),2);
        if distanceij<min_temp
            min_temp=distanceij;
        end
    end
end
min_distance=[min_temp,min_distance];


Xsum1=[200 30000 500];      %����Ŀ��λ��
obs=[1000 15000 500;
    -1000 6000 800];        %Բ��״�ϰ����x���꣬y���꣬�뾶��
plot3(Xsum1(1),Xsum1(2),Xsum1(3));%Leader����λ��һ
hold on

for j=2:J
  %�������������������������������������������������������������캽���˻���ɶ���
   [beta1,alpha1] = compute_angle3(T(1,:),Xsum1);%����ƫ����
   [Fatx1,Faty1,Fatz1,Fatr1]=compute_weiyi3(T(1,:),Xsum1,k,beta1,alpha1);
    Fsumy1=Faty1;%yλ����
    Fsumx1=Fatx1;%xλ����
    Fsumz1=Fatz1;%zλ����
    Fsumr1=Fatr1;
   Position_angle(1,j)=atan(Fsumy1/Fsumx1);%��ǰƫ����
   fuyang_angle(1,j)=atan(Fsumz1/Fsumr1);
   
  T1=T(1,:);
   
   distance(1,j)=sqrt((T(1,1)-Xsum1(1))^2+(T(1,2)-Xsum1(2))^2+(T(1,3)-Xsum1(3))^2);%��ǰ��������λ�þ���
   
        if(distance(1,j)<0.5)
     Xnext1(1)=T1(1);
     Xnext1(2)=T1(2);
     Xnext1(3)=T1(3);
     T(1,:)=Xnext1;%����
        else
    Xnext1(1)=T1(1)+V_L*cos(fuyang_angle(1,j))*cos(Position_angle(1,j));%Leader��һ��λ��
    Xnext1(2)=T1(2)+V_L*cos(fuyang_angle(1,j))*sin(Position_angle(1,j));
    Xnext1(3)=T1(3)+V_L*sin(fuyang_angle(1,j));
    T(1,:)=Xnext1;
        end
        
        % ���θ��ݳ����˶����������ת
        Transmatrix=[cos(Position_angle(1,j)),sin(Position_angle(1,j)),0;
        -sin(Position_angle(1,j)), cos(Position_angle(1,j)),0;
        0,                       0,                     1]...
        *[cos(fuyang_angle(1,j)),0,sin(fuyang_angle(1,j));
        0,                      1,              0;
        -sin(fuyang_angle(1,j)),0,cos(fuyang_angle(1,j))];
    
  %�����������������������������������������������������������������������˻��ƶ�
  for i_follower=2:21
      
   Xsum(i_follower-1,:) = T(1,:) + F(i_follower,:)*Transmatrix;%����Leaderȷ������λ��
   
   pot_i=[0,0,0]; %�˹��Ƴ�����
    for j_follower = 1:4
      pot_i=pot_i+k_pot*potential(i_follower,j_follower,T); %�����˹��Ƴ�����
    end
    pot_i=pot_i+k_pot*potential_obs(i_follower,obs,T,U,R(i_follower,j-1),Q(i_follower,j-1)); %�ϰ������
    
    Xsum(i_follower-1,:) = Xsum(i_follower-1,:)+pot_i; %�˹��Ƴ�����ת��Ϊ����λ�øı䣨��������
  [beta,alpha] = compute_angle3(T(i_follower,:),Xsum(i_follower-1,:));%����ƫ���ǡ�������
   [Fatx,Faty,Fatz,Fatr]=compute_weiyi3(T(i_follower,:),Xsum(i_follower-1,:),k,beta,alpha);
    Fsumy=Faty;%yλ����
    Fsumx=Fatx;%xλ����
    Fsumz=Fatz;%zλ����
    Fsumr=Fatr;
   Position_angle(i_follower,j)=beta;   %��ǰ����ƫ����
   fuyang_angle(i_follower,j)=atan(Fsumz/Fsumr);    %��ǰ����������
   T_i=T(i_follower,:); %��ǰλ�ƻ���
   distancex1(i_follower,j)=norm(T(i_follower,:)-T(1,:),2); %�볤����ľ��룬norm()Ϊ������
   distance(i_follower,j)=norm(T_i-Xsum(i_follower-1,:),2); %������λ�ü�ľ���
   
  % �����PI������   
  [U(i_follower,j),R(i_follower,:),Q(i_follower,:),Rotatc(i_follower,j),pitchc(i_follower,j)]...
      =Follower_PI3(distance(i_follower,:),V_M,V_m,Position_angle(i_follower,j),fuyang_angle(i_follower,j),...
      R(i_follower,:),Q(i_follower,:),Rotat(i_follower,:),pitch(i_follower,:),Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    %λ�ø���
    Xnext(1)=T_i(1)+U(i_follower,j)*cos(Q(i_follower,j-1)+pitchc(i_follower,j)/2)*cos(R(i_follower,j-1)+Rotatc(i_follower,j)/2);
    Xnext(2)=T_i(2)+U(i_follower,j)*cos(Q(i_follower,j-1)+pitchc(i_follower,j)/2)*sin(R(i_follower,j-1)+Rotatc(i_follower,j)/2);
    Xnext(3)=T_i(3)+U(i_follower,j)*sin(Q(i_follower,j-1)+pitchc(i_follower,j)/2);
    T(i_follower,:)=Xnext;
    
  end
  
  % ����������Сֵ
  min_temp=655360;
  for i_distance=1:20
    for j_distance=i_distance+1:21
        distanceij=norm(T(i_distance,:)-T(j_distance,:),2);
        if distanceij<min_temp
            min_temp=distanceij;
        end
    end
  end
min_distance=[min_temp,min_distance];
        
    
%  if(distance1(j)<1&&distance2(j)<1&&distance3(j)<1&&distance4(j)<1)%������������������������������������������������Ŀ�ĵ�ֹͣ������ͼ
%     break;
%  end
  h=plot3(T(1,1),T(1,2),T(1,3),'.r',T(2,1),T(2,2),T(2,3),'.b',T(3,1),T(3,2),T(3,3),'.g',T(4,1),T(4,2),T(4,3),'.y',...
      T(5,1),T(5,2),T(5,3),'.r',T(6,1),T(6,2),T(6,3),'.b',T(7,1),T(7,2),T(7,3),'.g',T(8,1),T(8,2),T(8,3),'.y',...
      T(9,1),T(9,2),T(9,3),'.r',T(10,1),T(10,2),T(10,3),'.b',T(11,1),T(11,2),T(11,3),'.g',T(12,1),T(12,2),T(12,3),'.y',...
      T(13,1),T(13,2),T(13,3),'.r',T(14,1),T(14,2),T(14,3),'.b',T(15,1),T(15,2),T(15,3),'.g',T(16,1),T(16,2),T(16,3),'.y',...
      T(17,1),T(17,2),T(17,3),'.r',T(18,1),T(18,2),T(18,3),'.b',T(19,1),T(19,2),T(19,3),'.g',T(20,1),T(20,2),T(20,3),'.y',...
      T(21,1),T(21,2),T(21,3),'.r');
  axis equal
  grid on;
  refreshdata(h) %ָ������Դʱˢ��ͼ�е�����
  drawnow %ˢ����Ļ
  hold on
end
plot3(T(1,1),T(1,2),T(1,3),'o','Markersize',10,'color','r');
%text(T(1,1)-5,T(1,2)-5,T(1,3)-5,'UAV_1');%��ע����
hold on
plot3(T(2,1),T(2,2),T(2,3),'o','Markersize',10,'color','b');
hold on
plot3(T(3,1),T(3,2),T(3,3),'o','Markersize',10,'color','g');
hold on
plot3(T(4,1),T(4,2),T(4,3),'o','Markersize',10,'color','y');
hold on
plot3(T(5,1),T(5,2),T(5,3),'o','Markersize',10,'color','r');
hold on
plot3(T(6,1),T(6,2),T(6,3),'o','Markersize',10,'color','b');
hold on
plot3(T(8,1),T(8,2),T(8,3),'o','Markersize',10,'color','y');
hold on
plot3(T(9,1),T(9,2),T(9,3),'o','Markersize',10,'color','r');
hold on
plot3(T(10,1),T(10,2),T(10,3),'o','Markersize',10,'color','b');
hold on
plot3(T(11,1),T(11,2),T(11,3),'o','Markersize',10,'color','g');
hold on
plot3(T(12,1),T(12,2),T(12,3),'o','Markersize',10,'color','y');
hold on
plot3(T(13,1),T(13,2),T(13,3),'o','Markersize',10,'color','r');
hold on
plot3(T(14,1),T(14,2),T(14,3),'o','Markersize',10,'color','b');
hold on
plot3(T(15,1),T(15,2),T(15,3),'o','Markersize',10,'color','g');
hold on
plot3(T(16,1),T(16,2),T(16,3),'o','Markersize',10,'color','y');
hold on
plot3(T(17,1),T(17,2),T(17,3),'o','Markersize',10,'color','r');
hold on
plot3(T(18,1),T(18,2),T(18,3),'o','Markersize',10,'color','b');
hold on
plot3(T(19,1),T(19,2),T(19,3),'o','Markersize',10,'color','g');
hold on
plot3(T(20,1),T(20,2),T(20,3),'o','Markersize',10,'color','y');
hold on
plot3(T(21,1),T(21,2),T(21,3),'o','Markersize',10,'color','r');
hold on

%����ع�
F=[0  , 0  ,0 ;
    -l/2, -l, 0.04*l;
    -l/2, l, 0.04*l;
    -l/2, -l, -0.04*l;
    -l/2, l, -0.04*l;
    -l, -2*l, -2*0.04*l;
    -l, 0, -2*0.04*l;
    -l, 2*l, -2*0.04*l;
    -l, -2*l, 0;
    -l, 0, 2*0.04*l;
    -l, 2*l, 0;
    -l, -2*l, 2*0.04*l;
    -l, 0, 0;
    -l, 2*l, 2*0.04*l;
    -3*l/2, -l, 0;
    -3*l/2, l, 0;
    -3*l/2, 3*l, 0;
    -3*l/2, -3*l, 0;
    -2*l, -2*l, 0;
    -2*l, 0, 0;
    -2*l, 2*l, 0];
tmp=R(:,J);
R(:,1)=tmp;
tmp=Q(:,J);
Q(:,1)=tmp;
R(1,1)=Position_angle(1,J);



J=2;%650
for j=2:J
  %�������������������������������������������������������������캽���˻���ɶ���
  
  distance(j)=0;
  if j<300&&j>=101  %-----------------------------------------�����캽���˻��Ĳٿ�����
      u_ctrl1=[0,pi/200,0]; %ˮƽת��
      else if j<420 && j>=400
              u_ctrl1=[0.001 0 0.01]; %������������
          else if j>=480 && j<500
                  u_ctrl1=[-0.001 0 -0.01]; %������������
              else 
                  u_ctrl1=[0 0 0];
              end
              
          end
      
  end
  
  %����ָ�����ϵͳ
  [Vl_next, phi1_next, theta1_next, T1_next]=Leader_P(u_ctrl1,T(1,:),U(1),R(1,j-1),Q(1,j-1));

  % �������� 
  R(1,j) = phi1_next;
    Q(1,j) = theta1_next;
    U(1) = Vl_next;
    T(1,:) = T1_next;

    
       % ���θ��ݳ����˶����������ת
       Transmatrix=[cos(R(1,j)),sin(R(1,j)),0;
        -sin(R(1,j)), cos(R(1,j)),0;
        0,           0,              1]...
        *[cos(Q(1,j)),0,sin(Q(1,j));
        0,           1,              0;
        -sin(Q(1,j)),0,cos(Q(1,j))];
 
  %�����������������������������������������������������������������������˻�1�ƶ�
  for i_follower=2:21
      
   Xsum(i_follower-1,:) = T(1,:) + F(i_follower,:)*Transmatrix;%����Leaderȷ������λ��
   
   pot_i=[0,0,0];
    for j_follower = 1:4
      pot_i=pot_i+k_pot*potential(i_follower,j_follower,T); %�����˹��Ƴ�
    end
    pot_i=pot_i+k_pot*potential_obs(i_follower,obs,T,U,R(i_follower,j-1),Q(i_follower,j-1)); %�ϰ������
    
    Xsum(i_follower-1,:) = Xsum(i_follower-1,:)+pot_i; %�˹��Ƴ���ת��Ϊ����λ�õ�����
  [beta,alpha] = compute_angle3(T(i_follower,:),Xsum(i_follower-1,:));%����ƫ���Ǻ͸�����
   [Fatx,Faty,Fatz,Fatr]=compute_weiyi3(T(i_follower,:),Xsum(i_follower-1,:),k,beta,alpha);
    Fsumy=Faty;%yλ����
    Fsumx=Fatx;%xλ����
    Fsumz=Fatz;%zλ����
    Fsumr=Fatr;
   Position_angle(i_follower,j)=beta;%��ǰ����ƫ����
   fuyang_angle(i_follower,j)=atan(Fsumz/Fsumr); %��ǰ����������
   T_i=T(i_follower,:);
   distancex1(i_follower,j)=norm(T(i_follower,:)-T(1,:),2);
   distance(i_follower,j)=norm(T_i-Xsum(i_follower-1,:),2);
   
  %�����PI������   
  [U(i_follower,j),R(i_follower,:),Q(i_follower,:),Rotatc(i_follower,j),pitchc(i_follower,j)]...
      =Follower_PI3(distance(i_follower,:),V_M,V_m,Position_angle(i_follower,j),fuyang_angle(i_follower,j),...
      R(i_follower,:),Q(i_follower,:),Rotat(i_follower,:),pitch(i_follower,:),Rm,Qm,j,kp,ki,krp,kri,kqp,kqi);
      
    Xnext(1)=T_i(1)+U(i_follower,j)*cos(Q(i_follower,j-1)+pitchc(i_follower,j)/2)*cos(R(i_follower,j-1)+Rotatc(i_follower,j)/2);
    Xnext(2)=T_i(2)+U(i_follower,j)*cos(Q(i_follower,j-1)+pitchc(i_follower,j)/2)*sin(R(i_follower,j-1)+Rotatc(i_follower,j)/2);
    Xnext(3)=T_i(3)+U(i_follower,j)*sin(Q(i_follower,j-1)+pitchc(i_follower,j)/2);
    T(i_follower,:)=Xnext;
   
  end
  
  %�������С����
  min_temp=655360;
  for i_distance=1:20
    for j_distance=i_distance+1:21
        distanceij=norm(T(i_distance,:)-T(j_distance,:),2);
        if distanceij<min_temp
            min_temp=distanceij;
        end
    end
  end
min_distance=[min_temp,min_distance];


%     
%     a=[a,norm(T2-T3,2)];
%         
%     
%  if(distance1(j)<1&&distance2(j)<1&&distance3(j)<1&&distance4(j)<1&&j>500)%������������������������������������������������Ŀ�ĵ�ֹͣ������ͼ
%     break;
%  end
h=plot3(T(1,1),T(1,2),T(1,3),'.r',T(2,1),T(2,2),T(2,3),'.b',T(3,1),T(3,2),T(3,3),'.g',T(4,1),T(4,2),T(4,3),'.y',...
      T(5,1),T(5,2),T(5,3),'.r',T(6,1),T(6,2),T(6,3),'.b',T(7,1),T(7,2),T(7,3),'.g',T(8,1),T(8,2),T(8,3),'.y',...
      T(9,1),T(9,2),T(9,3),'.r',T(10,1),T(10,2),T(10,3),'.b',T(11,1),T(11,2),T(11,3),'.g',T(12,1),T(12,2),T(12,3),'.y',...
      T(13,1),T(13,2),T(13,3),'.r',T(14,1),T(14,2),T(14,3),'.b',T(15,1),T(15,2),T(15,3),'.g',T(16,1),T(16,2),T(16,3),'.y',...
      T(17,1),T(17,2),T(17,3),'.r',T(18,1),T(18,2),T(18,3),'.b',T(19,1),T(19,2),T(19,3),'.g',T(20,1),T(20,2),T(20,3),'.y',...
      T(21,1),T(21,2),T(21,3),'.r');  axis equal
  grid on;
  refreshdata(h) %ָ������Դʱˢ��ͼ�е�����
  drawnow %ˢ����Ļ
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
% line([T(1,1),T(2,1)],[T(1,2),T(2,2)],[T(1,3),T(2,3)]),line([T(1,1),T(3,1)],[T(1,2),T(3,2)],[T(1,3),T(3,3)]),line([T2(1),T4(1)],[T2(2),T4(2)],[T2(3),T4(3)]),line([T3(1),T4(1)],[T3(2),T4(2)],[T3(3),T4(3)]); %����������״
plot3(T(5,1),T(5,2),T(5,3),'o','Markersize',10,'color','r');
hold on
plot3(T(6,1),T(6,2),T(6,3),'o','Markersize',10,'color','b');
hold on
plot3(T(8,1),T(8,2),T(8,3),'o','Markersize',10,'color','y');
hold on
plot3(T(9,1),T(9,2),T(9,3),'o','Markersize',10,'color','r');
hold on
plot3(T(10,1),T(10,2),T(10,3),'o','Markersize',10,'color','b');
hold on
plot3(T(11,1),T(11,2),T(11,3),'o','Markersize',10,'color','g');
hold on
plot3(T(12,1),T(12,2),T(12,3),'o','Markersize',10,'color','y');
hold on
plot3(T(13,1),T(13,2),T(13,3),'o','Markersize',10,'color','r');
hold on
plot3(T(14,1),T(14,2),T(14,3),'o','Markersize',10,'color','b');
hold on
plot3(T(15,1),T(15,2),T(15,3),'o','Markersize',10,'color','g');
hold on
plot3(T(16,1),T(16,2),T(16,3),'o','Markersize',10,'color','y');
hold on
plot3(T(17,1),T(17,2),T(17,3),'o','Markersize',10,'color','r');
hold on
plot3(T(18,1),T(18,2),T(18,3),'o','Markersize',10,'color','b');
hold on
plot3(T(19,1),T(19,2),T(19,3),'o','Markersize',10,'color','g');
hold on
plot3(T(20,1),T(20,2),T(20,3),'o','Markersize',10,'color','y');
hold on
plot3(T(21,1),T(21,2),T(21,3),'o','Markersize',10,'color','r');
hold on


xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');
% legend('UAV_1','UAV_2','UAV_3','UAV_4');
hold on
%title('\fontsize{16} ���˻�·��ͼ')
grid on;
shg;
