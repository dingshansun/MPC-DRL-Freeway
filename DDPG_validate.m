% clear; 
% clc
% close all
%% Initial state
x=[zeros(22,1);0];
u=[200,200,1];
scenario=3;
% rng('default')
noise_o1=random('Normal',0,225,1,151); % normal distributed noise on the demand
noise_o2=random('Normal',0,90,1,151);
N=900;M=6;
% u_mpcpre=repmat([70 70 1]',1,2);
% u_mpcpre_p=u_mpcpre;
% u_imppre=[70 70 1]';
% u_mpc=u_mpcpre;
xx=zeros(size(x,1),N);
xx_p=zeros(size(x,1),N);
U=zeros(3,N);
U_p=zeros(3,N);
for i=1:60
    x=Freeway_model_initial(x,u,scenario);
end
% x_p=x;
% u=[100,100,1];
% u=[100*ones(2,900);u_ramp];
% u=repelem(U,1,6);
norm_x=[100 100 1000 100 100 1000 100 100 1000 100 100 1000 1000 100 1000 100 100 100 1000 100 100 1000]';
for i=1:N/M
    k=x(23);
    Observation=[x(1:22)./norm_x; (demando1(k,scenario)+noise_o1(ceil((k-59)/6)))/1000; (demando2(k,scenario)+noise_o2(ceil((k-59)/6)))/1000];
    action=getAction(agent,Observation);  % specify the agent to be validated here
    u_rl=action{1}.*[100 100 1]';
    u=sat(u_rl);
    U(:,M*(i-1)+1:M*i)=repmat(u(:,1),1,M);
%     u=[100,100,1];
    for j=1:M
        x=Freeway_model_Noise(x,u(:,1),scenario,noise_o1,noise_o2);
        xx(:,M*(i-1)+j)=x;
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

%%
u_speed=U(1:2,:);
u_ramp=U(3,:);
u_speed_p=U_p(1:2,:);
u_ramp_p=U_p(3,:);
fprintf('TTS for DRL is %.3f veh*h \n', sum(TTS))
figure();
t=1/360:1/360:N/360;
subplot(4,2,1)
plot(t, v_11, '-', 'linewidth', 1.0);
hold on;
plot(t, v_12, '--', 'linewidth', 1.0);
hold on;
plot(t, v_13, ':', 'linewidth', 1.0);
hold on;
plot(t, v_14, '-.', 'linewidth', 1.0);
hold on;
plot(t, v_21, '-o', 'MarkerIndices',1:30:length(v_21), 'linewidth', 1.0);
hold on;
plot(t, v_22, '-x',  'MarkerIndices',1:30:length(v_22), 'linewidth', 1.0);
legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
xlabel('Time [h]');
ylabel('Speed [km/h]')

subplot(4,2,2)
plot(t, q_11, '-', 'linewidth', 1.0);
hold on;
plot(t, q_12, '--', 'linewidth', 1.0);
hold on;
plot(t, q_13, ':', 'linewidth', 1.0);
hold on;
plot(t, q_14, '-.', 'linewidth', 1.0);
hold on;
plot(t, q_21, '-o', 'MarkerIndices',1:30:length(q_21), 'linewidth', 1.0);
hold on;
plot(t, q_22, '-x',  'MarkerIndices',1:30:length(q_22), 'linewidth', 1.0);
% legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
xlabel('Time [h]');
ylabel('Flow [veh/h]')
ylim([0 4500]);

subplot(4,2,3)
plot(t, rou_11, '-', 'linewidth', 1.0);
hold on;
plot(t, rou_12, '--', 'linewidth', 1.0);
hold on;
plot(t, rou_13, ':', 'linewidth', 1.0);
hold on;
plot(t, rou_14, '-.', 'linewidth', 1.0);
hold on;
plot(t, rou_21, '-o', 'MarkerIndices',1:30:length(rou_21), 'linewidth', 1.0);
hold on;
plot(t, rou_22, '-x',  'MarkerIndices',1:30:length(rou_22), 'linewidth', 1.0);
% legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
xlabel('Time [h]');
ylabel('Density [veh/km]')

subplot(4,2,5)
plot(t, q_o1, '-', 'linewidth', 1.0);
hold on;
plot(t, q_o2, '--', 'linewidth', 1.0);
legend('O_1','O_2')
xlabel('Time [h]');
ylabel('Original flow [veh/h]')
% ylim([0 4000])

subplot(4,2,6)
plot(t,w_o1,'-', 'linewidth', 1.0);
hold on;
ylim([0 300])
yline(200, '--', 'Constraint', 'linewidth', 1.0)
plot(t,w_o2,'--', 'linewidth', 1.0);
legend('O_1','O_2')
xlabel('Time [h]');
ylabel('Queue length [veh]')

subplot(4,2,7)
plot(t, u_speed(1,:), '-', 'linewidth', 1.0);
hold on;
plot(t, u_speed(2,:), '--', 'linewidth', 1.0);
legend('u_1','u_2')
xlabel('Time [h]');
ylabel('Speed limit [km/h]')
ylim([0 120])


subplot(4,2,8)
plot(t,u_ramp,'-','linewidth',1.0);
ylim([0 1]);
xlabel('Time [h]');
ylabel('Ramp metering')


% sgtitle(['Scenario ' num2str(scenario) ' open-loop simulation'])

% subplot(4,2,8)
% plot(t, TTS.*360, 'LineWidth', 1.0);
% xlabel('Time [h]');
% ylabel('Total vehicles')
% sgtitle(['DRL Scenario ' num2str(scenario) ' simulation results'])
