function pot=potential_obs(i,obs,T,U,R,Q)
d=70;
[num_obs,~]=size(obs);

pot=[0 0 0];
for i_obs=1:num_obs
obstacle=[obs(i_obs,1),obs(i_obs,2),T(i,3)];
pot=pot+2*d^2*(T(i,:)-obstacle)/abs((norm(T(i,:)-obstacle, 2)-obs(i_obs,3))^3); % norm取模长，斥力与距离成二次方反比
if norm(T(i,:)-obstacle,2)-obs(i_obs,3)<5*d
    emg=(T(i,:)-obstacle)/norm(T(i,:)-obstacle,2)*[0,1,0;-1,0,0;0,0,1];
    emg_T=emg';
    U_unit=[U(i)*cos(R)*cos(Q),U(i)*sin(R)*cos(Q),U(i)*sin(Q)];
    emg_1=100000*norm(pot)*sign(U_unit*emg_T)*emg;
    pot=pot+emg_1;
end
end