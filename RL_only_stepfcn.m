function [Observation,Reward,IsDone,LoggedSignals] = RL_only_stepfcn(u,LoggedSignals)
%RL_ONLY_STEPFCN Environment of the freeway network: generate the next state and the rewards
% u_pre=LoggedSignals.u_pre; % the previous control inputs
x=LoggedSignals.x;
k=LoggedSignals.k;
% TTS_pre=LoggedSignals.TTS;
scenario=LoggedSignals.Scenario;
xx=zeros(22,6);
u_pre=LoggedSignals.u_pre;
noise_o1=LoggedSignals.noiseo1;
noise_o2=LoggedSignals.noiseo2;
% u_mpc=LoggedSignals.umpc;

% u_com=u_mpc+u;
% 
% if u_com>=1.0
%     u_com=1;
% elseif u_com<=0
%     u_com=0;
% end

u_imp=u.*[100 100 1]';
for i=1:6
    x=Freeway_model_Noise([x(1:22);k],u_imp,scenario,noise_o1,noise_o2);
    x=x(1:end-1);
    k=k+1;
    xx(:,i)=x;
end
rou_11=xx(1,:);
v_11=xx(2,:);
q_11=xx(3,:);
rou_12=xx(4,:);
v_12=xx(5,:);
q_12=xx(6,:);
rou_13=xx(7,:);
v_13=xx(8,:);
q_13=xx(9,:);
rou_14=xx(10,:);
v_14=xx(11,:);
q_14=xx(12,:);
q_o1=xx(13,:);
w_o1=xx(14,:);
q_o2=xx(15,:);
w_o2=xx(16,:);
rou_21=xx(17,:);
v_21=xx(18,:);
q_21=xx(19,:);
rou_22=xx(20,:);
v_22=xx(21,:);
q_22=xx(22,:);

TTS=(sum((rou_11+rou_12+rou_13+rou_14+rou_21+rou_22))*1*2+sum(w_o1+w_o2))*10/3600;
% if rem(k,30)==0
%     u_mpc=MPC_imp([x;k],u_pre,scenario);
% end
% Velocity=sum(v_11+v_12+v_13+v_14+v_21+v_22)/6;
% Queue=-sum(w_o1)-sum(w_o2);
penalty=-10.*(max(w_o1-200)>0)-10.*(max(w_o2-100)>0);
% wv=5;
% wq=1;
% wp=1;
% wd=5;
% if TTS_pre-TTS>0
%     Reward=10*(TTS_pre-TTS);
% else
%     Reward=TTS_pre-TTS;
% end
Diff=u_imp-u_pre;
Diff_speed=Diff(1:2);
Diff_ramp=Diff(3);
Diff_penalty=-sum(Diff_ramp.^2)-sum((Diff_speed/102).^2);
% Reward=wv*Velocity+wq*Queue+wp*penalty+wd*Diff_penalty;
Reward=-TTS+penalty+0.4*Diff_penalty; % Consider constraint, while minimizing TTS
norm_x=[100 100 1000 100 100 1000 100 100 1000 100 100 1000 1000 100 1000 100 100 100 1000 100 100 1000]';
Observation=[x./norm_x; (demando1(k,scenario)+noise_o1(ceil((k-59)/6)))/1000; (demando2(k,scenario)+noise_o2(ceil((k-59)/6)))/1000];
if k>959
    IsDone=true;
else
    IsDone=false;
end
LoggedSignals.u_pre=u_imp;
LoggedSignals.x=x;
LoggedSignals.k=k;
LoggedSignals.TTS=TTS;
% LoggedSignals.umpc=u_mpc;
LoggedSignals.Scenario=scenario;
LoggedSignals.noiseo1=noise_o1;
LoggedSignals.noiseo2=noise_o2;
end

