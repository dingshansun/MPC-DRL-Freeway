function x_=Freeway_model_Noise(x, u, scenario, noise_o1, noise_o2)
%% Initial conditions
% rou_11=zeros(N+1,1);
% v_11=zeros(N+1,1);
% q_11=zeros(N+1,1);
% rou_12=zeros(N+1,1);
% v_12=zeros(N+1,1);
% q_12=zeros(N+1,1);
% rou_13=zeros(N+1,1);
% v_13=zeros(N+1,1);
% q_13=zeros(N+1,1);
% rou_14=zeros(N+1,1);
% v_14=zeros(N+1,1);
% q_14=zeros(N+1,1);
% q_o1=zeros(N+1,1);
% w_o1=zeros(N+1,1);
% q_o2=zeros(N+1,1);
% w_o2=zeros(N+1,1);
% rou_21=zeros(N+1,1);
% v_21=zeros(N+1,1);
% q_21=zeros(N+1,1);
% rou_22=zeros(N+1,1);
% v_22=zeros(N+1,1);
% q_22=zeros(N+1,1);
rou_11=x(1);
v_11=x(2);
q_11=x(3);
rou_12=x(4);
v_12=x(5);
q_12=x(6);
rou_13=x(7);
v_13=x(8);
q_13=x(9);
rou_14=x(10);
v_14=x(11);
q_14=x(12);
q_o1=x(13);
w_o1=x(14);
q_o2=x(15);
w_o2=x(16);
rou_21=x(17);
v_21=x(18);
q_21=x(19);
rou_22=x(20);
v_22=x(21);
q_22=x(22);
k=x(23);
u_control_13=u(1);
u_control_14=u(2);
ro=u(3);
%% Parameters
[tao, kai, yita, rou_max, sigma, am, v_free, rou_crit, alpha, T, lambda, Lm, v_control, v_min, Co2, ~, ~] = parameters_real;
para_speed=[T,tao,yita,kai,Lm,sigma,lambda];
para_density=[T,Lm,lambda];
para_desire=[v_free, am, rou_crit, alpha];
para_outmain=[T, lambda, rou_crit, am, v_free, alpha];
para_outramp=[T, rou_max, rou_crit];
% Initial state of the source
% q_o1(1)=Outflow_main(demando1(1), v_free, w_o1(1), v_control, para_outmain);
% q_o2(1)=Outflow_ramp(demando2(1), w_o2(1), Co2, ro, rou_21(1), para_outramp);
% v_11(1)=v_free;
% v_12(1)=v_free;
% v_13(1)=v_free;
% v_14(1)=v_free;
% v_21(1)=v_free;
% v_22(1)=v_free;
%% State evolution
    % Segment 1,1
%     q_11(k)=rou_11(k)*v_11(k)*lambda;
    rou_11_=Density(rou_11, q_11, q_o1, para_density);
    V_desire_11=Desired_speed(rou_11, para_desire, v_control);
    v_11_=Speed(v_11, V_desire_11, v_11, rou_11, rou_12, para_speed, 0, v_min);
    q_11_=rou_11_*v_11_*lambda;
    % Segment 1,2
%     q_12_=rou_12*v_12*lambda;
    rou_12_=Density(rou_12, q_12, q_11, para_density);
    V_desire_12=Desired_speed(rou_12, para_desire, v_control);
    v_12_=Speed(v_12, V_desire_12, v_11, rou_12, rou_13, para_speed, 0, v_min);
    q_12_=rou_12_*v_12_*lambda;
    % Segment 1,3
%     q_13(k)=rou_13(k)*v_13(k)*lambda;
    rou_13_=Density(rou_13, q_13, q_12, para_density);
    V_desire_13=Desired_speed(rou_13, para_desire, u_control_13);
    v_13_=Speed(v_13, V_desire_13, v_12, rou_13, rou_14, para_speed, 0, v_min);
    q_13_=rou_13_*v_13_*lambda;
    % Segment 1,4
%     q_14(k)=rou_14(k)*v_14(k)*lambda;
    rou_14_=Density(rou_14, q_14, q_13, para_density);
    V_desire_14=Desired_speed(rou_14, para_desire, u_control_14);
    v_14_=Speed(v_14, V_desire_14, v_13, rou_14, rou_21, para_speed, 0, v_min);
    q_14_=rou_14_*v_14_*lambda;
    % Segment 2,1
%     q_21(k)=rou_21(k)*v_21(k)*lambda;
    rou_21_=Density(rou_21, q_21, q_14+q_o2, para_density);
    V_desire_21=Desired_speed(rou_21, para_desire, v_control);
    v_21_=Speed(v_21, V_desire_21, v_14, rou_21, rou_22, para_speed, q_o2, v_min);
    q_21_=rou_21_*v_21_*lambda;
    % Segment 2,2
%     q_22(k)=rou_22(k)*v_22(k)*lambda;
    rou_22_=Density(rou_22, q_22, q_21, para_density);
    V_desire_22=Desired_speed(rou_22, para_desire, v_control);
    v_22_=Speed(v_22, V_desire_22, v_21, rou_22, rou_crit, para_speed, 0, v_min);
    q_22_=rou_22_*v_22_*lambda;
    % Source nodes
    % o1
    w_o1_=w_o1+T/3600*((demando1(k-1,scenario)+noise_o1(max(ceil((k-60)/6),1)))-q_o1);
    q_o1_=Outflow_main((demando1(k,scenario)+noise_o1(ceil((k-59)/6))), v_11_, w_o1_, v_control, para_outmain);
    % o2
    w_o2_=w_o2+T/3600*((demando2(k-1,scenario)+noise_o2(max(ceil((k-60)/6),1)))-q_o2);
    q_o2_=Outflow_ramp((demando2(k,scenario)+noise_o2(ceil((k-59)/6))), w_o2_, Co2, ro, rou_21_, para_outramp);
%     TTS(k)=T/3600*((rou_11(k)+rou_12(k)+rou_13(k)+rou_14(k)+rou_21(k)+rou_22(k))*Lm/1000*lambda+w_o1(k)+w_o2(k));

x_=[rou_11_;
v_11_;
q_11_;
rou_12_;
v_12_;
q_12_;
rou_13_;
v_13_;
q_13_;
rou_14_;
v_14_;
q_14_;
q_o1_;
w_o1_;
q_o2_;
w_o2_;
rou_21_;
v_21_;
q_21_;
rou_22_;
v_22_;
q_22_;
k+1];
end
% fprintf('TTS is %d veh*h \n', sum(TTS(60:960)))
% figure();
% t=0:1/360:2.5;
% subplot(3,2,1)
% plot(t, v_11(61:961), '-', 'linewidth', 1.0);
% hold on;
% plot(t, v_12(61:961), '--', 'linewidth', 1.0);
% hold on;
% plot(t, v_13(61:961), ':', 'linewidth', 1.0);
% hold on;
% plot(t, v_14(61:961), '-.', 'linewidth', 1.0);
% hold on;
% plot(t, v_21(61:961), '-o', 'MarkerIndices',1:30:length(v_21(61:961)), 'linewidth', 1.0);
% hold on;
% plot(t, v_22(61:961), '-x',  'MarkerIndices',1:30:length(v_22(61:961)), 'linewidth', 1.0);
% legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Speed [km/h]')
% 
% subplot(3,2,2)
% plot(t, q_11(60:960), '-', 'linewidth', 1.0);
% hold on;
% plot(t, q_12(60:960), '--', 'linewidth', 1.0);
% hold on;
% plot(t, q_13(60:960), ':', 'linewidth', 1.0);
% hold on;
% plot(t, q_14(60:960), '-.', 'linewidth', 1.0);
% hold on;
% plot(t, q_21(60:960), '-o', 'MarkerIndices',1:30:length(q_21(61:961)), 'linewidth', 1.0);
% hold on;
% plot(t, q_22(60:960), '-x',  'MarkerIndices',1:30:length(q_22(61:961)), 'linewidth', 1.0);
% % legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Flow [veh/h]')
% 
% subplot(3,2,3)
% t=0:1/360:2.5;
% plot(t, rou_11(61:961), '-', 'linewidth', 1.0);
% hold on;
% plot(t, rou_12(61:961), '--', 'linewidth', 1.0);
% hold on;
% plot(t, rou_13(61:961), ':', 'linewidth', 1.0);
% hold on;
% plot(t, rou_14(61:961), '-.', 'linewidth', 1.0);
% hold on;
% plot(t, rou_21(61:961), '-o', 'MarkerIndices',1:30:length(rou_21(61:961)), 'linewidth', 1.0);
% hold on;
% plot(t, rou_22(61:961), '-x',  'MarkerIndices',1:30:length(rou_22(61:961)), 'linewidth', 1.0);
% % legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Density [veh/km]')
% 
% subplot(3,2,5)
% plot(t, q_o1(61:961), '-', 'linewidth', 1.0);
% hold on;
% plot(t, q_o2(61:961), '--', 'linewidth', 1.0);
% legend('O_1','O_2')
% 
% subplot(3,2,6)
% plot(t,w_o1(61:961),'-', 'linewidth', 1.0);
% hold on;
% plot(t,w_o2(61:961),'--', 'linewidth', 1.0);
% legend('O_1','O_2')

