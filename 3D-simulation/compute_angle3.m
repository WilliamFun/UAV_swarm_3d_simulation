function [beta,alpha] = compute_angle3(X,Xsum1)
      r=sqrt((Xsum1(1)-X(1))^2+(Xsum1(2)-X(2))^2);%��Ŀ��λ��ӳ�����
      beta=sign(Xsum1(2)-X(2))*acos((Xsum1(1)-X(1))/r);%ƫ���������¸�
      l=sqrt((Xsum1(1)-X(1))^2+(Xsum1(2)-X(2))^2+(Xsum1(3)-X(3))^2);%ֱ�߾���
      alpha=sign(Xsum1(3)-X(3))*acos(r/l);
end