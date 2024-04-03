%%  System params
%initalize the system params

clear all
clc

ms  = 5333;% Sprung Mass (kg) 
mus = 906.5;% Unsprung Mass (kg)
ks  = 430000;% Suspension Stiffness (N/m) ks 
kus = 2440000;% Wheel stiffness (N/m) kt
bs  = 20000;% Suspension Inherent Damping coefficient (sec/m) cs
bus = 40000;% Wheel Inhenrent Damping coefficient (sec/m) ct
%%

% Train DDPG

%%
obsInfo = rlNumericSpec([3 1],...
    LowerLimit=[-1000 -1000 -1000  ]',...
    UpperLimit=[ 1000  1000 1000]');
obsInfo.Name = "observations";
obsInfo.Description = "body acc, wheel deflection, displacement from road";

actInfo = rlNumericSpec([1 1], ...
     LowerLimit=[-5000 ]',...
    UpperLimit=[ 5000]');
actInfo.Name = "control_force";

%Build the environment interface object.

env = rlSimulinkEnv("rlsuspension","rlsuspension/RL Agent",...
    obsInfo,actInfo);

%Set a custom reset function that randomizes the reference values for the model.

env.ResetFcn = @(in)localResetFcn(in);


%Specify the simulation time Tf and the agent sample time Ts in seconds.
Ts = 0.16;
Tf = 5;


% Fix the random generator seed for reproducibility.

rng(0)

%% NETWORK ARCH

% Observation path
obsPath = [
    featureInputLayer(obsInfo.Dimension(1),Name="obsInputLayer")
    fullyConnectedLayer(50)
    reluLayer
    fullyConnectedLayer(50)
    reluLayer
    fullyConnectedLayer(25,Name="obsPathOutLayer")];

% Action path
actPath = [
    featureInputLayer(actInfo.Dimension(1),Name="actInputLayer")
    fullyConnectedLayer(50)
    reluLayer
    fullyConnectedLayer(25,Name="actPathOutLayer")];

% Common path
commonPath = [
    additionLayer(2,Name="add")
    reluLayer
    fullyConnectedLayer(1,Name="CriticOutput")];

criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,obsPath);
criticNetwork = addLayers(criticNetwork,actPath);
criticNetwork = addLayers(criticNetwork,commonPath);

criticNetwork = connectLayers(criticNetwork, ...
    "obsPathOutLayer","add/in1");
criticNetwork = connectLayers(criticNetwork, ...
    "actPathOutLayer","add/in2");




% View the critic network configuration.

figure
plot(criticNetwork)
%%
%Figure contains an axes object. The axes object contains an object of type graphplot.
%Convert the network to a dlnetwork object and summarize its properties.

criticNetwork = dlnetwork(criticNetwork);
summary(criticNetwork)
%   Initialized: true

 %  Number of learnables: 1.5k

  % Inputs:
   %   1   'obsInputLayer'   3 features
    %  2   'actInputLayer'   1 features
% Create the critic approximator object using the specified deep neural network, the environment specification objects, and the names if the network inputs to be associated with the observation and action channels.

critic = rlQValueFunction(criticNetwork, ...
    obsInfo,actInfo, ...
    ObservationInputNames="obsInputLayer", ...
    ActionInputNames="actInputLayer");


% Check the critic with a random input observation and action.

getValue(critic, ...
    {rand(obsInfo.Dimension)}, ...
    {rand(actInfo.Dimension)})


%%
%Create the Actor
%DDPG agents use a parametrized deterministic policy over continuous action spaces, which is learned by a continuous deterministic actor.


actorNetwork = [
    featureInputLayer(obsInfo.Dimension(1))
    fullyConnectedLayer(3)
    tanhLayer
    fullyConnectedLayer(actInfo.Dimension(1))
    ];
% Convert the network to a dlnetwork object and summarize its properties.

actorNetwork = dlnetwork(actorNetwork);
summary(actorNetwork)


%Create the actor approximator object using the specified deep neural network, the environment specification objects, and the name if the network input to be associated with the observation channel.

actor = rlContinuousDeterministicActor(actorNetwork,obsInfo,actInfo);



% Check the actor with a random input observation.

getAction(actor,{rand(obsInfo.Dimension)})



% Create the DDPG Agent
% Create the DDPG agent using the specified actor and critic approximator objects.

agent = rlDDPGAgent(actor,critic);



% Specify options for the agent, the actor, and the critic using dot notation.

agent.SampleTime = Ts;

agent.AgentOptions.TargetSmoothFactor = 1e-3;
agent.AgentOptions.DiscountFactor = 0.99;
agent.AgentOptions.MiniBatchSize = 64;
agent.AgentOptions.ExperienceBufferLength = 1e6; 

agent.AgentOptions.NoiseOptions.Variance = 0.1;
agent.AgentOptions.NoiseOptions.VarianceDecayRate = 1e-5;

agent.AgentOptions.CriticOptimizerOptions.LearnRate = 1e-03;
agent.AgentOptions.CriticOptimizerOptions.GradientThreshold = 1;
agent.AgentOptions.ActorOptimizerOptions.LearnRate = 1e-03;
agent.AgentOptions.ActorOptimizerOptions.GradientThreshold = 1;

% Check the agent with a random input observation.

getAction(agent,{rand(obsInfo.Dimension)})

% Train Agent
% To train the agent, first specify the training options. For this example, use the following options:
% 
% Run each training for at most 5000 episodes. Specify that each episode lasts for at most ceil(Tf/Ts) (that is 200) time steps.
% 
% Display the training progress in the Episode Manager dialog box (set the Plots option) and disable the command line display (set the Verbose option to false).
% 
% Stop training when the agent receives an average cumulative reward greater than 800 over 20 consecutive episodes. At this point, the agent can control the level of water in the tank.
% 
% For more information, see rlTrainingOptions.

trainOpts = rlTrainingOptions(...
    MaxEpisodes=5000, ...
    MaxStepsPerEpisode=ceil(Tf/Ts), ...
    ScoreAveragingWindowLength=20, ...
    Verbose=false, ...
    Plots="training-progress",...
    StopTrainingCriteria="AverageReward",...
    StopTrainingValue=800);




% Train the agent using the train function. Training is a computationally intensive process that takes several minutes to complete. To save time while running this example, load a pretrained agent by setting doTraining to false. To train the agent yourself, set doTraining to true.

doTraining = true;

if doTraining
    % Train the agent.
    trainingStats = train(agent,env,trainOpts);
else
    % Load the pretrained agent for the example.
    load("WaterTankDDPG.mat","agent")
end