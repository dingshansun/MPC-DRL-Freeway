% clear; 
% clc
% close all
%% Initialize
x=[zeros(22,1);0];
scenario=1;
% rng('default')
noise_o1=random('Normal',0,225,1,151); % normal distributed noise on the demand
noise_o2=random('Normal',0,90,1,151);
u=[200,200,1]';Time=[];time_sum=0;
for i=1:60
    x=Freeway_model_initial(x,u,scenario);
end
%% Parameters
Np=2;
Nc=2;
M=30;
Mc=6;
N=900;
Mo=Mc; % operation sampling time
N_max=30; % multi-start point
dim_theta=3;
dim_speed=2;
dim_ramp=1;
xx=[];
Theta=zeros(dim_theta,N);
U=zeros(dim_speed+dim_ramp,N);
u_pre=u; %
theta_pre=repmat(ones(dim_theta,1), 1, Nc);
[tao, kai, yita, rou_max, sigma, am, v_free, rou_crit, alpha, T, lambda, Lm, ~, v_min, Co2, xi_r, xi_s] = parameters_real;
[~, ~, ~, ~, ~, ~, v_free_esti, rou_crit_esti, ~, ~, ~, ~, ~, ~, ~, ~, ~] = parameters_esti;
para=[Lm, lambda, v_free, M, dim_speed, T, xi_r, xi_s, Mc, rou_crit_esti, v_free_esti];
options = optimoptions(@fmincon,'Algorithm','sqp','Display','off','TolFun',1e-2, 'TolX',1e-2, 'TolCon', 1e-2);
%%
for i=1:N/Mo
    % options = optimoptions(@fmincon,'Algorithm','sqp','Display','off','TolFun',1e-2, 'TolX',1e-2, 'TolCon', 1e-2);
    theta_lb=repmat(-10,dim_theta,Nc);
    theta_ub=repmat(10,dim_theta,Nc);
    objfun=@(theta) ModelState(x, theta, Nc, Np, para, u_pre(:,1), scenario);
    confun=@(theta) ModelCons(x, theta, Nc, Np, para, u_pre(:,1), scenario);
    theta0=cell(1,N_max);theta_t=cell(1,N_max);Fval=nan(1,N_max);
    tic
    parfor l=1:N_max
        if l==1
            theta0{l}=theta_pre;
        elseif l==2
            theta0{l}=theta_lb;
        elseif l==3
            theta0{l}=theta_ub;
        elseif l==4
            theta0{l}=(theta_lb+theta_ub)/2;
        else
            theta0{l}=repmat(20*rand-10,dim_theta, Nc);
        end
        [theta_t{l}, Fval(l), exitflag] = fmincon(objfun, theta0{l}, [], [], [], [], theta_lb, theta_ub, confun, options);
        if exitflag == 0
            Fval(l)=Fval(l)+1e6;
        elseif exitflag < 0
            Fval(l)=Fval(l)+1e6;
        end
    end
    time=toc;
    time_sum=time_sum+time;
    if rem(i,5)==0
        Time=[Time time_sum];
        time_sum=0;
    end
    [~, index]=min(Fval);
    theta_opt=theta_t{index};
    for m=1:Mo/Mc
        u=u_para(theta_opt(:,1), x, u_pre, rou_crit_esti, v_free_esti);
        for j=1:Mc
            x=Freeway_model_Noise(x,u,scenario,noise_o1,noise_o2);
            xx=[xx x];
        end
        Theta(:,Mc*(i-1)+1:Mc*i)=repmat(theta_opt(:,1),1,Mc);
        u_pre=u;
        U(:,Mc*(i-1)+1:Mc*i)=repmat(u,1,Mc);
    end
    theta_pre=theta_opt;
    
end
%% figure
u_speed=U(1:dim_speed,:);
u_ramp=U(dim_speed+1:dim_speed+dim_ramp,:);
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
TTS=T/3600.*((rou_11+rou_12+rou_13+rou_14+rou_21+rou_22).*Lm./1000.*lambda+w_o1+w_o2);
%%
fprintf('TTS is %.3f veh*h \n', sum(TTS))
Velocity=[v_22;v_21;v_14;v_13;v_12;v_11];
fprintf('Minimal speed is %.3f km/h \n', min(Velocity, [], 'all'));
% figure();
% t=1/360:1/360:N/360;
% subplot(4,2,1)
% plot(t, v_11, '-', 'linewidth', 1.0);
% hold on;
% plot(t, v_12, '--', 'linewidth', 1.0);
% hold on;
% plot(t, v_13, ':', 'linewidth', 1.0);
% hold on;
% plot(t, v_14, '-.', 'linewidth', 1.0);
% hold on;
% plot(t, v_21, '-o', 'MarkerIndices',1:30:length(v_21), 'linewidth', 1.0);
% hold on;
% plot(t, v_22, '-x',  'MarkerIndices',1:30:length(v_22), 'linewidth', 1.0);
% legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Speed [km/h]')
% 
% subplot(4,2,2)
% plot(t, q_11, '-', 'linewidth', 1.0);
% hold on;
% plot(t, q_12, '--', 'linewidth', 1.0);
% hold on;
% plot(t, q_13, ':', 'linewidth', 1.0);
% hold on;
% plot(t, q_14, '-.', 'linewidth', 1.0);
% hold on;
% plot(t, q_21, '-o', 'MarkerIndices',1:30:length(q_21), 'linewidth', 1.0);
% hold on;
% plot(t, q_22, '-x',  'MarkerIndices',1:30:length(q_22), 'linewidth', 1.0);
% % legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Flow [veh/h]')
% 
% subplot(4,2,3)
% plot(t, rou_11, '-', 'linewidth', 1.0);
% hold on;
% plot(t, rou_12, '--', 'linewidth', 1.0);
% hold on;
% plot(t, rou_13, ':', 'linewidth', 1.0);
% hold on;
% plot(t, rou_14, '-.', 'linewidth', 1.0);
% hold on;
% plot(t, rou_21, '-o', 'MarkerIndices',1:30:length(rou_21), 'linewidth', 1.0);
% hold on;
% plot(t, rou_22, '-x',  'MarkerIndices',1:30:length(rou_22), 'linewidth', 1.0);
% % legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Density [veh/km]')
% 
% subplot(4,2,4)
% plot(t, u_speed(1,:), '-', 'linewidth', 1.0);
% hold on;
% plot(t, u_speed(2,:), '--', 'linewidth', 1.0);
% legend('u_1','u_2')
% xlabel('Time [h]');
% ylabel('Speed limit [km/h]')
% ylim([0 120])
% 
% subplot(4,2,5)
% plot(t, q_o1, '-', 'linewidth', 1.0);
% hold on;
% plot(t, q_o2, '--', 'linewidth', 1.0);
% legend('O_1','O_2')
% xlabel('Time [h]');
% ylabel('Original flow [veh/h]')
% % ylim([0 4000])
% 
% subplot(4,2,6)
% plot(t,w_o1,'-', 'linewidth', 1.0);
% hold on;
% plot(t,w_o2,'--', 'linewidth', 1.0);
% legend('O_1','O_2')
% xlabel('Time [h]');
% ylabel('Queue length [veh]')
% ylim([-10 250])
% 
% subplot(4,2,7)
% plot(t,u_ramp,'-','linewidth',1.0);
% ylim([0 1]);
% xlabel('Time [h]');
% ylabel('Ramp metering')
% 
% subplot(4,2,8)
% plot(t, TTS.*360, 'LineWidth', 1.0);
% xlabel('Time [h]');
% ylabel('Total vehicles')
% sgtitle(['Scenario ' num2str(scenario) ' simulation results'])
%%
function J=ModelState(x, theta, Nc, Np, para, u_pre, scenario)
    Lm=para(1);
    Lambda=para(2);
    v_free=para(3);
    M=para(4);
    dim_speed=para(5);
    T=para(6);
    xi_r=para(7);
    xi_s=para(8);
    Mc=para(9);
    rou_crit_esti=para(10);
    v_free_esti=para(11);

    theta_full=[theta, repmat(theta(:,end),Np-Nc)];
    yy=[];
    uc_full=[];
    for i=1:Np
        for j=1:M/Mc
            u=u_para(theta_full(:,i), x, u_pre, rou_crit_esti, v_free_esti);
            uc_full=[uc_full u];
            for k=1:Mc
                x=Freeway_model(x,u,scenario);
                yy=[yy x];
            end
            u_pre=u;
        end
    end
    u_diff=[u_pre uc_full];
    Diff=zeros(size(u,1),Nc*M/Mc); % the erros between the successive control inputs
    for i=1:Nc*M/Mc
        Diff(:,i)=u_diff(:,i+1)-u_diff(:,i);
    end

    rou_11=yy(1,:);rou_12=yy(4,:);rou_13=yy(7,:);rou_14=yy(10,:);w_o1=yy(14,:);w_o2=yy(16,:);rou_21=yy(17,:);rou_22=yy(20,:);
    TTS=(((rou_11+rou_12+rou_13+rou_14+rou_21+rou_22))*Lm/1000*Lambda+(w_o1+w_o2))*T/3600;
    Diff_speed=Diff(1:dim_speed,:);
    Diff_r=Diff(dim_speed+1:end,:);
    penalty=xi_r*sum(sum(Diff_r.^2))+xi_s*sum(sum((Diff_speed/v_free).^2));
    J=sum(TTS)+penalty;
end
function J=ModelState_Noise(x, u, Nc, Np, para, u_pre, scenario)
    Lm=para(1);
    Lambda=para(2);
    v_free=para(3);
    M=para(4);
    dim_speed=para(5);
    T=para(6);
    xi_r=para(7);
    xi_s=para(8);
    uc=[u repelem(u(:, end),1,Np-Nc)];
    uc_full=repelem(uc,1,M);
    u_diff=[u_pre u];
    yy=nan(size(x,1),Np*M);
    Diff=zeros(size(u,1),Nc); % the erros between the successive control inputs
    for i=1:Nc
        Diff(:,i)=u_diff(:,i+1)-u_diff(:,i);
    end
    for i=1:Np*M
        x=Freeway_model(x,uc_full(:,i), scenario);
        yy(:,i)=x;
    end
    rou_11=yy(1,:);rou_12=yy(4,:);rou_13=yy(7,:);rou_14=yy(10,:);w_o1=yy(14,:);w_o2=yy(16,:);rou_21=yy(17,:);rou_22=yy(20,:);
    TTS=(((rou_11+rou_12+rou_13+rou_14+rou_21+rou_22))*Lm/1000*Lambda+(w_o1+w_o2))*T/3600;
    Diff_speed=Diff(1:dim_speed,:);
    Diff_r=Diff(dim_speed+1:end,:);
    penalty=xi_r*sum(sum(Diff_r.^2))+xi_s*sum(sum((Diff_speed/v_free).^2));
    J=sum(TTS)+penalty;
end
function [c, ceq]=ModelCons(x, theta, Nc, Np, para, u_pre, scenario);
    ceq=[];
    M=para(4);
    Mc=para(9);
    rou_crit_esti=para(10);
    v_free_esti=para(11);

    theta_full=[theta, repmat(theta(:,end),Np-Nc)];
    yy=[];
    uc_full=[];
    for i=1:Np
        for j=1:M/Mc
            u=u_para(theta_full(:,i), x, u_pre, rou_crit_esti, v_free_esti);
            uc_full=[uc_full u];
            for k=1:Mc
                x=Freeway_model(x,u,scenario);
                yy=[yy x];
            end
            u_pre=u;
        end
    end
    w_o1=yy(14,:);w_o2=yy(16,:);
    c=[w_o1'-200;w_o2'-100];
end
function [c, ceq]=ModelCons_Noise(x, u, Nc, Np, M, scenario)
    ceq=[];
    uc=[u repelem(u(:, end),1,Np-Nc)];
    uc_full=repelem(uc,1,M);
    yy=zeros(size(x,1),Np*M);
    for i=1:Np*M
        x=Freeway_model(x,uc_full(:,i),scenario);
        yy(:,i)=x;
    end
    w_o1=yy(14,:);w_o2=yy(16,:);
    c=[w_o1'-200;w_o2'-100];
end








