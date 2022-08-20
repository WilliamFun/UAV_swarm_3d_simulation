function [U,R,Q,Rotatc,pitchc] = Follower_PI3(distance,VM,Vm,Position_angle,fuyang_angle,R,Q,Rotat,pitch,Rm,Qm,j,kp,ki,krp,kri,kqp,kqi)
      ed=(distance(j)-distance(j-1));%P
      U=kp*ed+ki*distance(j); %�������˻�����ٶ�
      U=abs(U);
      if (U>VM)                    %Ϊ�˲�ʹ���˻�����
          U=VM; 
      else if (U<Vm)
              U=Vm;
          end
      end
      
      Rotat(j)=Position_angle-R(j-1);
      if (abs(Rotat(j))>pi)                       %���н�ת��Ϊ��-pi��pi��
    Rotat(j)=(2*pi-abs(Rotat(j)))*(-abs(Rotat(j))/Rotat(j));
      end
      Rotatc=kri*(Rotat(j)-Rotat(j-1))+krp*Rotat(j-1);     %���ٶ�PI����
      if (abs(Rotatc)*U>=Rm)                     %�������ļ��ٶ�
    Rotatc=(Rotatc/abs(Rotatc))*Rm/U;       
      end
      R(j)=R(j-1)+Rotatc;                      %�������˻����ٶȷ���
      if (abs(R(j))>pi)                       %���н�ת��Ϊ��-pi��pi��
    R(j)=(2*pi-abs(R(j)))*(-abs(R(j))/R(j));
      end
      
      pitch(j)=fuyang_angle-Q(j-1);
      if (abs(pitch(j))>pi)                       %���н�ת��Ϊ��-pi��pi��
    pitch(j)=(2*pi-abs(pitch(j)))*(-abs(pitch(j))/pitch(j));
      end
      pitchc=kqi*(pitch(j)-pitch(j-1))+kqp*pitch(j-1);     %���ٶ�PI����
      if (abs(pitchc)>=Qm)                     %���ƽ��ٶ�
    pitchc=(pitchc/abs(pitchc))*Qm;       
      end
      Q(j)=Q(j-1)+pitchc;                      %�������˻����ٶȷ���
      
      
      
      
      
      
      
      
      