%% Initialization
no_state = 30; % the number of states includes the speed and density for each segment, and the queue length of the origins
% the states does not include the time step (should not), but include the
% current demand for both origins; add the control input from MPC as an
% extra state
no_action = 3; % actions include the two speed limit inputs, and the ramp metering rate
% both states and actions are continuous
%% Environment
ObservationInfo=rlNumericSpec([no_state,1]);
ObservationInfo.Name='states';

ActionLb=[-0.32;-0.32;-0.4];
ActionUb=[0.32;0.32;0.4];
% ActionLb=0;
% ActionUb=1;
ActionInfo=rlNumericSpec([no_action,1],'LowerLimit',ActionLb,'UpperLimit',ActionUb);
ActionInfo.Name='Speed limits and ramp metering rate';

env = rlFunctionEnv(ObservationInfo,ActionInfo,'RL_MPC_stepfcn','RL_MPC_resetfnc');
%% Agent
% TD3 agent
% agentopt = rlTD3AgentOptions('MiniBatchSize',256,...
%                                                     'TargetSmoothFactor',1e-3,...
%                                                     'TargetUpdateFrequency',400);
% initOpts = rlAgentInitializationOptions('NumHiddenUnit',128);
% agent = rlTD3Agent(ObservationInfo,ActionInfo,initOpts);
% agent.AgentOptions.MiniBatchSize=256;
% agent.AgentOptions.TargetSmoothFactor=5e-4;
% agent.AgentOptions.TargetUpdateFrequency=40;
% agent.AgentOptions.PolicyUpdateFrequency=20;
% agent.AgentOptions.DiscountFactor=0.95;
% agent.AgentOptions.ExperienceBufferLength=1e5;
% agent.AgentOptions.ExplorationModel.StandardDeviation=[0.3 0.3 0.4]';
% agent.AgentOptions.ExplorationModel.StandardDeviationDecayRate=1e-4;
% % agent.AgentOptions.ExplorationModel.StandardDeviationMin=0.01;
% agent.AgentOptions.TargetPolicySmoothModel.StandardDeviation=[0.3 0.3 0.4]';
% agent.AgentOptions.TargetPolicySmoothModel.StandardDeviationDecayRate=1e-4;
% % agent.AgentOptions.TargetPolicySmoothModel.StandardDeviationMin=0.05;
% agent.AgentOptions.ActorOptimizerOptions.LearnRate=0.001;
% agent.AgentOptions.ActorOptimizerOptions.GradientThreshold=1;
% agent.AgentOptions.CriticOptimizerOptions(1,1).LearnRate=0.005;
% agent.AgentOptions.CriticOptimizerOptions(1,2).LearnRate=0.005;
% agent.AgentOptions.CriticOptimizerOptions(1,1).GradientThreshold=1;
% agent.AgentOptions.CriticOptimizerOptions(1,2).GradientThreshold=1;
% initOpts = rlAgentInitializationOptions('NumHiddenUnit',128);
% agent = rlDDPGAgent(ObservationInfo,ActionInfo,initOpts);
statePath = [featureInputLayer(prod(ObservationInfo.Dimension), ...
    'Normalization','none','Name','state')
    fullyConnectedLayer(256,'Name', 'fc1_state')];
actionPath = [featureInputLayer(prod(ActionInfo.Dimension), ...
    'Normalization','none','Name','action')
    fullyConnectedLayer(128,"Name",'fc_2_action')];
commonPath = [concatenationLayer(1,2,'Name','concat')
              reluLayer('Name','reLu')
              fullyConnectedLayer(256, ...
                'Name','StateValue')
                reluLayer('Name', 'relu_body')
                fullyConnectedLayer(128, 'Name','fc_body')
                reluLayer('Name','relu_body2')
                fullyConnectedLayer(1,'Name','output')];
criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork, actionPath);
criticNetwork = addLayers(criticNetwork, commonPath);
criticNetwork = connectLayers(criticNetwork, 'fc1_state','concat/in1');
criticNetwork = connectLayers(criticNetwork, 'fc_2_action','concat/in2');
critic = rlQValueFunction(criticNetwork,ObservationInfo,ActionInfo);

actorNet = [
    featureInputLayer(prod(ObservationInfo.Dimension),'Normalization','none','Name','observation')
    fullyConnectedLayer(256,'Name','ActorFC1')
    reluLayer('Name','ActorRelu1')
    fullyConnectedLayer(256,'Name','ActorFC2')
    reluLayer('Name','ActorRelu2')
%     fullyConnectedLayer(64,'Name','ActorFC3')
%     reluLayer('Name','ActorRelu3')
    fullyConnectedLayer(prod(ActionInfo.Dimension),'Name','ActorFC4')
    tanhLayer('Name','ActorTanh1')
    scalingLayer('Scale',[0.32 0.32 0.4]','Bias',[0 0 0]')
    ];
actor  = rlContinuousDeterministicActor(actorNet,ObservationInfo,ActionInfo);
agent = rlDDPGAgent(actor,critic);
agent.AgentOptions.MiniBatchSize=512;
agent.AgentOptions.ExperienceBufferLength=2e5;
agent.AgentOptions.TargetSmoothFactor=1e-2;
agent.AgentOptions.TargetUpdateFrequency=10;
agent.AgentOptions.DiscountFactor=0.99;
agent.AgentOptions.NumStepsToLookAhead=10;
agent.AgentOptions.ActorOptimizerOptions.LearnRate=0.001;
agent.AgentOptions.ActorOptimizerOptions.GradientThreshold=1;
agent.AgentOptions.CriticOptimizerOptions.LearnRate=0.001;
agent.AgentOptions.CriticOptimizerOptions.GradientThreshold=1;
agent.AgentOptions.NoiseOptions.StandardDeviation=0.2;
agent.AgentOptions.NoiseOptions.StandardDeviationDecayRate=2e-5;
%% Training settings
opt = rlTrainingOptions('MaxEpisodes',3000,...
                                        'MaxStepsPerEpisode',150,'UseParallel',false,'SaveAgentCriteria','AverageReward',...
                                        'SaveAgentValue',-1250);%,...
%                                         'StopTrainingCriteria','AverageReward',...
%                                         'StopTrainingValue',-1200,...
%                                         'UseParallel',false,...
%                                         'SaveAgentCriteria','AverageReward',...
%                                         'SaveAgentValue',-1200);
%% Training
Training = train(agent,env,opt);

%% Simulate
simOptions = rlSimulationOptions('MaxSteps',150);
experience = sim(env,agent,simOptions);
% experience = sim(env,saved_agent,simOptions);
% states = experience.Observation.states.Data;
% states=squeeze(states);
% norm_x=[100 100 1000 100 100 1000 100 100 1000 100 100 1000 1000 100 1000 100 100 100 1000 100 100 1000]';
% states=states.*[norm_x;1000;1000;100;100;1];
% U=experience.Action.SpeedLimitsAndRampMeteringRate.Data;
% U=squeeze(U);
save('Scenarios_3_Cons_MPCRL_LowRandomNoise.mat');
%% Figures
% u_speed=100*ones(2,150);
% u_ramp=repelem(U',1,6);
% t_u=1/360:1/360:2.5;
% rou_11=states(1,:);
% v_11=states(2,:);
% q_11=states(3,:);
% rou_12=states(4,:);
% v_12=states(5,:);
% q_12=states(6,:);
% rou_13=states(7,:);
% v_13=states(8,:);
% q_13=states(9,:);
% rou_14=states(10,:);
% v_14=states(11,:);
% q_14=states(12,:);
% q_o1=states(13,:);
% w_o1=states(14,:);
% q_o2=states(15,:);
% w_o2=states(16,:);
% rou_21=states(17,:);
% v_21=states(18,:);
% q_21=states(19,:);
% rou_22=states(20,:);
% v_22=states(21,:);
% q_22=states(22,:);
% t_x=0:1/60:2.5;
% % TTS=T/3600.*((rou_11+rou_12+rou_13+rou_14+rou_21+rou_22).*Lm./1000.*lambda+w_o1+w_o2);
% figure();
% subplot(3,2,1)
% plot(t_x, v_11, '-', 'linewidth', 1.0);
% hold on;
% plot(t_x, v_12, '--', 'linewidth', 1.0);
% hold on;
% plot(t_x, v_13, ':', 'linewidth', 1.0);
% hold on;
% plot(t_x, v_14, '-.', 'linewidth', 1.0);
% hold on;
% plot(t_x, v_21, '-o', 'MarkerIndices',1:30:length(v_21), 'linewidth', 1.0);
% hold on;
% plot(t_x, v_22, '-x',  'MarkerIndices',1:30:length(v_22), 'linewidth', 1.0);
% legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Speed [km/h]')
% 
% subplot(3,2,2)
% plot(t_x, q_11, '-', 'linewidth', 1.0);
% hold on;
% plot(t_x, q_12, '--', 'linewidth', 1.0);
% hold on;
% plot(t_x, q_13, ':', 'linewidth', 1.0);
% hold on;
% plot(t_x, q_14, '-.', 'linewidth', 1.0);
% hold on;
% plot(t_x, q_21, '-o', 'MarkerIndices',1:30:length(q_21), 'linewidth', 1.0);
% hold on;
% plot(t_x, q_22, '-x',  'MarkerIndices',1:30:length(q_22), 'linewidth', 1.0);
% % legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Flow [veh/h]')
% 
% subplot(3,2,3)
% plot(t_x, rou_11, '-', 'linewidth', 1.0);
% hold on;
% plot(t_x, rou_12, '--', 'linewidth', 1.0);
% hold on;
% plot(t_x, rou_13, ':', 'linewidth', 1.0);
% hold on;
% plot(t_x, rou_14, '-.', 'linewidth', 1.0);
% hold on;
% plot(t_x, rou_21, '-o', 'MarkerIndices',1:30:length(rou_21), 'linewidth', 1.0);
% hold on;
% plot(t_x, rou_22, '-x',  'MarkerIndices',1:30:length(rou_22), 'linewidth', 1.0);
% % legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Density [veh/km]')
% 
% % subplot(3,2,4)
% % plot(t_u, u_speed(1,:), '-', 'linewidth', 1.0);
% % hold on;
% % plot(t_u, u_speed(2,:), '--', 'linewidth', 1.0);
% % legend('u_1','u_2')
% % xlabel('Time [h]');
% % ylabel('Speed limit [km/h]')
% % % ylim([0 4000])
% 
% subplot(3,2,5)
% plot(t_x, q_o1, '-', 'linewidth', 1.0);
% hold on;
% plot(t_x, q_o2, '--', 'linewidth', 1.0);
% legend('O_1','O_2')
% xlabel('Time [h]');
% ylabel('Original flow [veh/h]')
% % ylim([0 4000])
% 
% subplot(3,2,6)
% plot(t_x,w_o1,'-', 'linewidth', 1.0);
% hold on;
% plot(t_x,w_o2,'--', 'linewidth', 1.0);
% legend('O_1','O_2')
% xlabel('Time [h]');
% ylabel('Queue length [veh]')
% % ylim([0 250])
% 
% figure()
% plot(t_u,u_ramp,'-','linewidth',1.0);
% % ylim([0 1]);
% xlabel('Time [h]');
% ylabel('Ramp metering')