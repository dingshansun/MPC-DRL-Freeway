function [InitialObservation,LoggedSignals] = RL_only_resetfnc
%RL_ONLY_RESETFNC Initialize the freeway network
% scenario=randi([1 10]);
scenario=1;
noise_o1=random('Normal',0,225,1,151); % normal distributed noise on the demand
noise_o2=random('Normal',0,90,1,151);
x=[zeros(22,1);0];
u=[200,200,1]';
xx=zeros(23,60);
for i=1:60
    x=Freeway_model_initial(x,u,scenario);
    xx(:,i)=x;
end
k=x(23);
rou_11=xx(1,end-5:end);
rou_12=xx(4,end-5:end);
rou_13=xx(7,end-5:end);
rou_14=xx(10,end-5:end);
w_o1=xx(14,end-5:end);
w_o2=xx(16,end-5:end);
rou_21=xx(17,end-5:end);
rou_22=xx(20,end-5:end);
norm_x=[100 100 1000 100 100 1000 100 100 1000 100 100 1000 1000 100 1000 100 100 100 1000 100 100 1000]';
TTS=(sum((rou_11+rou_12+rou_13+rou_14+rou_21+rou_22))*1*2+sum(w_o1+w_o2))*10/3600;

% u_mpc=MPC_imp(x,1.0,scenario);
% u_mpc=1;
% LoggedSignals.umpc=u_mpc;
LoggedSignals.Scenario=scenario;
LoggedSignals.u_pre=[70 70 1]'; %% initial previous actions
LoggedSignals.x=x(1:22);
LoggedSignals.k=k;
LoggedSignals.TTS=TTS;
LoggedSignals.noiseo1=noise_o1;
LoggedSignals.noiseo2=noise_o2;
InitialObservation=[x(1:22)./norm_x; (demando1(k,scenario)+noise_o1(ceil((k-59)/6)))/1000; (demando2(k,scenario)+noise_o2(ceil((k-59)/6)))/1000];
end