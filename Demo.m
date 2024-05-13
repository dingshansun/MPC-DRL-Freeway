clear;
env = rlPredefinedEnv("CartPole-Discrete");
env.PenaltyForFalling = -10;
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);
rng(0)
criticNetwork = [
    featureInputLayer(4,'Normalization','none','Name','state')
    fullyConnectedLayer(32,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(1, 'Name', 'CriticFC')];
criticNetwork = dlnetwork(criticNetwork);
criticOpts = rlOptimizerOptions('LearnRate',1e-2,'GradientThreshold',1);
critic = rlValueFunction(criticNetwork,obsInfo);
actorNetwork = [
    featureInputLayer(4,'Normalization','none','Name','state')
    fullyConnectedLayer(32, 'Name','ActorStateFC1')
    reluLayer('Name','ActorRelu1')
    fullyConnectedLayer(2,'Name','ActorStateFC2')
    softmaxLayer('Name','actionProb')];
actorNetwork = dlnetwork(actorNetwork);

actorOpts = rlOptimizerOptions('LearnRate',1e-2,'GradientThreshold',1);

actor = rlDiscreteCategoricalActor(actorNetwork,obsInfo,actInfo);
agentOpts = rlACAgentOptions(...
    'ActorOptimizerOptions',actorOpts, ...
    'CriticOptimizerOptions',criticOpts,...
    'EntropyLossWeight',0.01);
agent = rlACAgent(actor,critic,agentOpts);
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',1000,...
    'MaxStepsPerEpisode',500,...
    'Verbose',false,...
    'Plots','training-progress',...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue',480,...
    'ScoreAveragingWindowLength',10); 
plot(env)
doTraining = true;

if doTraining    
    % Train the agent.
    trainingStats = train(agent,env,trainOpts);
else
    % Load the pretrained agent for the example.
    load('MATLABCartpoleAC.mat','agent');
end
simOptions = rlSimulationOptions('MaxSteps',500);
experience = sim(env,agent,simOptions);
totalReward = sum(experience.Reward)