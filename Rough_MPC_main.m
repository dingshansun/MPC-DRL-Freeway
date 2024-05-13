% clear; 
% clc
% close all
%% Initialize
x=[zeros(22,1);0];
scenario=3;
% rng('default')
noise_o1=random('Normal',0,225,1,151); % normal distributed noise on the demand
noise_o2=random('Normal',0,90,1,151);
u=[200,200,1]'; Time=[];
for i=1:60
    x=Freeway_model_initial(x,u,scenario);
end
%% Parameters
ramp_only=0;
Np=2; % min
Nc=2;  % min
M=30;
N=900;
N_max=30; % multi-start point
xx=zeros(size(x,1),N);
[tao, kai, yita, rou_max, sigma, am, v_free, rou_crit, alpha, T, lambda, Lm, ~, v_min, Co2, xi_r, xi_s] = parameters_esti;
dim_speed=2;
dim_ramp=1;
U=zeros(dim_speed+dim_ramp,N);

para=[Lm, lambda, v_free, M, dim_speed, T, xi_r, xi_s];
options = optimoptions(@fmincon,'Algorithm','sqp','Display','off','TolFun',1e-2, 'TolX',1e-2, 'TolCon', 1e-2);
if (ramp_only)
    dim_control=dim_ramp;
    v_control=repmat(100*ones(dim_speed,1),1,Nc);
    v_control_pre=v_control(:,1);
    u_pre=ones(dim_control,Nc); % 
    u_lb=repmat(zeros(dim_control,1),1,Nc);
    u_ub=repmat(ones(dim_control,1),1,Nc);
else
    v_control=[];
    v_control_pre=[];
    dim_control=dim_ramp+dim_speed;
    u_lb=repmat([20*ones(dim_speed,1);zeros(dim_ramp,1)],1,Nc);
    u_ub=repmat([v_free*ones(dim_speed,1);ones(dim_ramp,1)],1,Nc);
    u_pre=[70*ones(dim_speed,Nc);ones(dim_ramp,Nc)]; % 
end
%%
for i=1:N/M
%     objfun=@(co) ModelState(x, [v_control; co], Nc, Np, para, [v_control_pre;u_pre(:,1)],scenario);
%     confun=@(co) ModelCons(x, [v_control; co], Nc, Np, M, scenario);
%     u0=cell(1,N_max);u_t=cell(1,N_max);Fval=nan(1,N_max);
%     parfor l=1:N_max
%         if l==1
%             u0{l}=u_pre;
%         elseif l==2
%             u0{l}=u_lb;
%         elseif l==3
%             u0{l}=u_ub;
%         else
%             if (ramp_only)
%                 u0{l}=repmat(rand,dim_control, Nc);
%             else
%                 u0{l}=repmat([rand(dim_speed,1)*80+20;rand(dim_ramp,1)],1, Nc);
% %                 u0{l}=[rand(dim_speed,Nc)*80+20;rand(dim_ramp,Nc)];
%             end
%         end
%         [u_t{l}, Fval(l), exitflag] = fmincon(objfun, u0{l}, [], [], [], [], u_lb, u_ub, confun, options);
%         if exitflag == 0
% %             warning('the solver does not succeed');
%             Fval(l)=1e6;
%         elseif exitflag < 0
% %             warning('the solver does not succeed');
%             Fval(l)=1e6;
%         end
%     end
%     [fval, index]=min(Fval);
%     u_opt=[v_control;u_t{index}];
%     u_opt=[v_control;[1 1 1]];
    tic
    if rem(x(23),M)==0
        u_opt=MPC_imp_(x,u_pre,scenario);
    end
    time=toc;
    Time=[Time time];
    U(:,M*(i-1)+1:M*i)=repmat(u_opt(:,1),1,M);
    for j=1:M
        x=Freeway_model_Noise(x,u_opt(:,1),scenario,noise_o1,noise_o2);
        xx(:,M*(i-1)+j)=x;
    end
    u_pre=u_opt;
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
TTS=T/3600.*((rou_11+rou_12+rou_13+rou_14+rou_21+rou_22).*1000./1000.*lambda+w_o1+w_o2);
%%
fprintf('TTS is %.3f veh*h \n', sum(TTS))

% (max(w_o1)-200)/200*100
% (max(w_o2)-100)/100*100
% mean(Time)
% max(Time)
Rou=[rou_22;rou_21;rou_14;rou_13;rou_12;rou_11];
Velocity=[v_22;v_21;v_14;v_13;v_12;v_11];
Flow=[q_22;q_21;q_14;q_13;q_12;q_11];
% fprintf('Average flow is %.3f veh*h \n', mean(Flow, 'all'))
% fprintf('Max flow is %.3f veh*h \n', max(Flow, [], 'all'))

fprintf('Average speed is %.3f km/h \n', min(Velocity, [], 'all'));

% figure
% subplot(3,1,1)
% imagesc(Velocity)
% xticks([180 360 540 720 900])
% xticklabels({'0.5','1.0','1.5','2.0','2.5'})
% yticks([1 2 3 4 5 6])
% yticklabels({'6','5','4','3','2','1'})
% xlabel('Time [h]')
% ylabel('Segment')
% colormap('Turbo')
% colorbar
% title('Speed [km/h]')
% subplot(3,1,2)
% imagesc(Rou)
% xticks([180 360 540 720 900])
% xticklabels({'0.5','1.0','1.5','2.0','2.5'})
% yticks([1 2 3 4 5 6])
% yticklabels({'6','5','4','3','2','1'})
% xlabel('Time [h]')
% ylabel('Segment')
% colormap('Turbo')
% colorbar
% title('Density [veh/km/lane]')
% subplot(3,1,3)
% imagesc(Flow)
% xticks([180 360 540 720 900])
% xticklabels({'0.5','1.0','1.5','2.0','2.5'})
% yticks([1 2 3 4 5 6])
% yticklabels({'6','5','4','3','2','1'})
% xlabel('Time [h]')
% ylabel('Segment')
% colormap('Turbo')
% colorbar
% title('Flow [veh/h]')
% 
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
% ylim([0 4500]);
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
% ylim([0 300])
% yline(200, '--', 'Constraint', 'linewidth', 1.0)
% xlabel('Time [h]');
% ylabel('Queue length [veh]')
% % ylim([-10 250])
% 
% subplot(4,2,7)
% plot(t,u_ramp,'-','linewidth',1.0);
% ylim([0 1]);
% xlabel('Time [h]');
% ylabel('Ramp metering')
% 
% subplot(4,2,8)
% plot(t, u_speed(1,:), '-', 'linewidth', 1.0);
% hold on;
% plot(t, u_speed(2,:), '--', 'linewidth', 1.0);
% legend('u_1','u_2')
% xlabel('Time [h]');
% ylabel('Speed limit [km/h]')
% ylim([0 120])


% subplot(4,2,8)
% plot(t, TTS.*360, 'LineWidth', 1.0);
% xlabel('Time [h]');
% ylabel('Total vehicles')
% sgtitle(['Scenario ' num2str(scenario) ' simulation results'])
% 
% figure()
% plot(t, demando1(61:900+60,3)+repelem(noise_o1(2:end),1,6), '-', 'linewidth', 1.0);
% hold on;
% plot(t, demando2(61:900+60,3)+repelem(noise_o2(2:end),1,6), '--', 'linewidth', 1.0);
% xlabel('Time [h]');
% ylabel('Demand [veh/h]');
% legend('O_1','O_2')
% ylim([0 4000])
% title(['Scenario ' num2str(scenario) ' real demand'])
% 
% figure()
% plot(t, demando1(61:N+60,scenario), '-', 'linewidth', 1.0);
% hold on;
% plot(t, demando2(61:N+60,scenario), '--', 'linewidth', 1.0);
% xlabel('Time [h]');
% ylabel('Demand [veh/h]');
% legend('O_1','O_2')
% ylim([0 4000])
% title(['Scenario ' num2str(scenario) ' predicted demand'])
%%
function J=ModelState(x, u, Nc, Np, para, u_pre, scenario)
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
        x=Freeway_model_Noise(x,uc_full(:,i), scenario);
        yy(:,i)=x;
    end
    rou_11=yy(1,:);rou_12=yy(4,:);rou_13=yy(7,:);rou_14=yy(10,:);w_o1=yy(14,:);w_o2=yy(16,:);rou_21=yy(17,:);rou_22=yy(20,:);
    TTS=(((rou_11+rou_12+rou_13+rou_14+rou_21+rou_22))*Lm/1000*Lambda+(w_o1+w_o2))*T/3600;
    Diff_speed=Diff(1:dim_speed,:);
    Diff_r=Diff(dim_speed+1:end,:);
    penalty=xi_r*sum(sum(Diff_r.^2))+xi_s*sum(sum((Diff_speed/v_free).^2));
    J=sum(TTS)+penalty;
end
function [c, ceq]=ModelCons(x, u, Nc, Np, M, scenario)
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
function [c, ceq]=ModelCons_Noise(x, u, Nc, Np, M, scenario)
    ceq=[];
    uc=[u repelem(u(:, end),1,Np-Nc)];
    uc_full=repelem(uc,1,M);
    yy=zeros(size(x,1),Np*M);
    for i=1:Np*M
        x=Freeway_model_Noise(x,uc_full(:,i),scenario);
        yy(:,i)=x;
    end
    w_o1=yy(14,:);w_o2=yy(16,:);
    c=[w_o1'-200;w_o2'-100];
end








