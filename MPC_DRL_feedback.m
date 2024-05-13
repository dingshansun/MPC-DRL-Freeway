% clear; 
% clc
% close all
%% Initial state
x=[zeros(22,1);0];
u=[200,200,1];Time=[];time_sum=0;
scenario=3;
% rng('default')
noise_o1=random('Normal',0,75,1,151); % normal distributed noise on the demand
noise_o2=random('Normal',0,30,1,151);
N=900;M=6;
u_mpcpre=repmat([70 70 1]',1,2);
u_mpcpre_p=u_mpcpre;
u_imppre=[70 70 1]';
u_mpc=u_mpcpre;
xx=zeros(size(x,1),N);
xx_p=zeros(size(x,1),N);
U=zeros(3,N);
U_p=zeros(3,N);
for i=1:60
    x=Freeway_model_initial(x,u,scenario);
end
x_p=x;
% u=[100,100,1];
% u=[100*ones(2,900);u_ramp];
% u=repelem(U,1,6);
norm_x=[100 100 1000 100 100 1000 100 100 1000 100 100 1000 1000 100 1000 100 100 100 1000 100 100 1000]';
for i=1:N/M
    tic
    if rem(x(23),30)==0
        u_mpc=MPC_imp_(x,u_mpcpre,scenario);
    end
    k=x(23);
    Observation=[x(1:22)./norm_x; (demando1(k,scenario)+noise_o1(ceil((k-59)/6)))/1000; (demando2(k,scenario)+noise_o2(ceil((k-59)/6)))/1000; u_mpc(:,1)./[102 102 1]'; u_imppre./[102 102 1]'];
    action=getAction(agent,Observation);
    u_rl=action{1}.*[100 100 1]';
% %     u_rl=zeros(3,1);
    u=sat(u_mpc(:,1)+u_rl);
    time=toc;
    time_sum=time_sum+time;
    if rem(i,5)==0
        Time=[Time time_sum];
        time_sum=0;
    end
    u_imppre=u;
%     u=u_mpc;
    if rem(x_p(23),30)==0
        u_mpc_p=MPC_imp_(x_p,u_mpcpre_p,scenario);
    end
    u_mpcpre=u_mpc;
    u_mpcpre_p=u_mpc_p;
    U(:,M*(i-1)+1:M*i)=repmat(u(:,1),1,M);
    U_p(:,M*(i-1)+1:M*i)=repmat(u_mpc(:,1),1,M);
%     u=[100,100,1];
    for j=1:M
        x=Freeway_model_Noise(x,u(:,1),scenario,noise_o1,noise_o2);
        xx(:,M*(i-1)+j)=x;
        x_p=Freeway_model_Noise(x_p,u_mpc_p(:,1),scenario,noise_o1,noise_o2);
        xx_p(:,M*(i-1)+j)=x_p;
    end
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

TTS=10/3600.*((rou_11+rou_12+rou_13+rou_14+rou_21+rou_22).*1000./1000.*2+w_o1+w_o2);

rou_11_p=xx_p(1,:);
v_11_p=xx_p(2,:);
q_11_p=xx_p(3,:);
rou_12_p=xx_p(4,:);
v_12_p=xx_p(5,:);
q_12_p=xx_p(6,:);
rou_13_p=xx_p(7,:);
v_13_p=xx_p(8,:);
q_13_p=xx_p(9,:);
rou_14_p=xx_p(10,:);
v_14_p=xx_p(11,:);
q_14_p=xx_p(12,:);
q_o1_p=xx_p(13,:);

w_o1_p=xx_p(14,:);

q_o2_p=xx_p(15,:);
w_o2_p=xx_p(16,:);
rou_21_p=xx_p(17,:);
v_21_p=xx_p(18,:);
q_21_p=xx_p(19,:);
rou_22_p=xx_p(20,:);
v_22_p=xx_p(21,:);
q_22_p=xx_p(22,:);

TTS_p=10/3600.*((rou_11_p+rou_12_p+rou_13_p+rou_14_p+rou_21_p+rou_22_p).*1000./1000.*2+w_o1_p+w_o2_p);
%%
u_speed=U(1:2,:);
u_ramp=U(3,:);
u_speed_p=U_p(1:2,:);
u_ramp_p=U_p(3,:);
fprintf('TTS for MPC-DRL is %.3f veh*h \n', sum(TTS))
fprintf('TTS for MPC is %.3f veh*h \n', sum(TTS_p))

% Rou=[rou_22;rou_21;rou_14;rou_13;rou_12;rou_11];
% Velocity=[v_22;v_21;v_14;v_13;v_12;v_11];
% Flow=[q_22;q_21;q_14;q_13;q_12;q_11];
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
% 
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
% % sgtitle(['Scenario ' num2str(scenario) ' open-loop simulation'])
% 
% subplot(4,2,7)
% plot(t, u_speed(1,:), '-', 'linewidth', 1.0);
% hold on;
% plot(t, u_speed(2,:), '--', 'linewidth', 1.0);
% legend('u_1','u_2')
% xlabel('Time [h]');
% ylabel('Speed limit [km/h]')
% ylim([0 120])
% 
% subplot(4,2,8)
% plot(t,u_ramp,'-','linewidth',1.0);
% ylim([0 1]);
% xlabel('Time [h]');
% ylabel('Ramp metering')

%%
% subplot(4,2,8)
% plot(t, TTS.*360, 'LineWidth', 1.0);
% xlabel('Time [h]');
% ylabel('Total vehicles')
% sgtitle(['MPC-DRL Scenario ' num2str(scenario) ' simulation results'])

% figure()
% plot(t,u_ramp,'-','linewidth',1.0);
% ylim([0 1]);
% xlabel('Time [h]');
% ylabel('Ramp metering')

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
% figure()
% subplot(4,2,1)
% plot(t, v_11_p, '-', 'linewidth', 1.0);
% hold on;
% plot(t, v_12_p, '--', 'linewidth', 1.0);
% hold on;
% plot(t, v_13_p, ':', 'linewidth', 1.0);
% hold on;
% plot(t, v_14_p, '-.', 'linewidth', 1.0);
% hold on;
% plot(t, v_21_p, '-o', 'MarkerIndices',1:30:length(v_21_p), 'linewidth', 1.0);
% hold on;
% plot(t, v_22_p, '-x',  'MarkerIndices',1:30:length(v_22_p), 'linewidth', 1.0);
% legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Speed [km/h]')
% 
% subplot(4,2,2)
% plot(t, q_11_p, '-', 'linewidth', 1.0);
% hold on;
% plot(t, q_12_p, '--', 'linewidth', 1.0);
% hold on;
% plot(t, q_13_p, ':', 'linewidth', 1.0);
% hold on;
% plot(t, q_14_p, '-.', 'linewidth', 1.0);
% hold on;
% plot(t, q_21_p, '-o', 'MarkerIndices',1:30:length(q_21_p), 'linewidth', 1.0);
% hold on;
% plot(t, q_22_p, '-x',  'MarkerIndices',1:30:length(q_22_p), 'linewidth', 1.0);
% % legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Flow [veh/h]')
% 
% subplot(4,2,3)
% plot(t, rou_11_p, '-', 'linewidth', 1.0);
% hold on;
% plot(t, rou_12_p, '--', 'linewidth', 1.0);
% hold on;
% plot(t, rou_13_p, ':', 'linewidth', 1.0);
% hold on;
% plot(t, rou_14_p, '-.', 'linewidth', 1.0);
% hold on;
% plot(t, rou_21_p, '-o', 'MarkerIndices',1:30:length(rou_21_p), 'linewidth', 1.0);
% hold on;
% plot(t, rou_22_p, '-x',  'MarkerIndices',1:30:length(rou_22_p), 'linewidth', 1.0);
% % legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Density [veh/km]')
% 
% subplot(4,2,4)
% plot(t, u_speed_p(1,:), '-', 'linewidth', 1.0);
% hold on;
% plot(t, u_speed_p(2,:), '--', 'linewidth', 1.0);
% legend('u_1','u_2')
% xlabel('Time [h]');
% ylabel('Speed limit [km/h]')
% ylim([0 120])
% 
% subplot(4,2,5)
% plot(t, q_o1_p, '-', 'linewidth', 1.0);
% hold on;
% plot(t, q_o2_p, '--', 'linewidth', 1.0);
% legend('O_1','O_2')
% xlabel('Time [h]');
% ylabel('Original flow [veh/h]')
% % ylim([0 4000])
% 
% subplot(4,2,6)
% plot(t,w_o1_p,'-', 'linewidth', 1.0);
% hold on;
% plot(t,w_o2_p,'--', 'linewidth', 1.0);
% legend('O_1','O_2')
% xlabel('Time [h]');
% ylabel('Queue length [veh]')
% sgtitle(['Scenario ' num2str(scenario) ' open-loop simulation'])
% 
% subplot(4,2,7)
% plot(t,u_ramp_p,'-','linewidth',1.0);
% ylim([0 1]);
% xlabel('Time [h]');
% ylabel('Ramp metering')
% 
% subplot(4,2,8)
% plot(t, TTS_p.*360, 'LineWidth', 1.0);
% xlabel('Time [h]');
% ylabel('Total vehicles')
% sgtitle(['MPC Scenario ' num2str(scenario) ' simulation results'])

% figure()
% plot(t,u_ramp,'-','linewidth',1.0);
% ylim([0 1]);
% xlabel('Time [h]');
% ylabel('Ramp metering')
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