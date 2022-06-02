%% SCRIPT FOR CLOSED LOOP SIMULATION OF THE DRONE SYSTEM
clear all
clc
close all

VelX    = 1;
VelY    = 2;
VelZ    = 3;
VelYaw  = 4;
PosX    = 5;
PosY    = 6;
PosZ    = 7;
PosYaw  = 8;
global small_k 
%% SYSTEM INIT AND SIM PARAMETERS
sphinx = false; %flag for which gamma we use


InitSTATE=0*[0;0;0;0;0.5;0;0;0];%0*rand([8,1]); %

small_k = 0.1; %gain value of the error dynamic gain
tol = 9;

simStep = 0.1;
Tfinal  = 40;

TRAJECTORY      = 'circle';
% TRAJECTORY      = 'openloop';
% TRAJECTORY = 'LemniscataBernoulli';


ControlType = 'MozelliTeo6';
% ControlType = 'WeiTeo1';
% MozelliTeo6
% SereniTeo2
% ControlType = 'openloop';
% ICUAS

Modeltype = 2;

MaxRotSpd       = 100; %max rotation speed in deg/s (yaw rate)
MaxXSpd         = 0.5; %max X speed in m/s 
MaxYSpd         = 0.5; %max Y speed in m/s 
MaxZSpd         = 0.5; %max Z speed in m/s 

%%
% Saturation on the control signal
Saturation = false;

% Wind disturbance simulation
Wind_Disturbance = false;
W_static = [0.03;
            0.03;
            0.03;
            0.03];
a=10*[0.03 0.03 0.03];
Gammas=[0 0 0];
Omegas=[2.2 2.5 2.7];
DisturbanceTime = [3 Tfinal];

%% Load Parameters and set time vector
if(sphinx)
    load gamma_sphinx
else
    load gamma_bebop
%     gamma = [4.0527, -0.1135, 4.1320, 0.0836 , 5.1451, 3.2975, 4.4168, 2.7023];
%     gamma = [4.3321, 0.2333, 4.1220, 0.4145, 4.4206, 3.1341, 5.9289, -0.3868];
gamma = [3.8195    0.1614    3.8117    0.3855    1.6844    1.7983    4.4994    2.0476];
end
%%    
SimStruct.trajectory=TRAJECTORY;
SimStruct.sys.param = gamma;
SimStruct.WindDisturbance.Type = Wind_Disturbance;
SimStruct.WindDisturbance.Ws = W_static;
SimStruct.WindDisturbance.a = a;
SimStruct.WindDisturbance.Gammas=Gammas;
SimStruct.WindDisturbance.Omegas=Omegas;
SimStruct.WindDisturbance.time=DisturbanceTime;

%% CONTROL DESIGN
ControlDesign

%% TRAJECTORY PLANNING
t=0:simStep:Tfinal;
[q_d,dq_d,ddq_d]=CalcDesTrajectory(TRAJECTORY,t);
DES_STATE = [dq_d;q_d]';

%% SYSTEM SIMULATION
InitERROR = InitSTATE - DES_STATE(1,:)';


% STATE = ode4(@DRONE_SANTANA,t,InitSTATE,...                            %TRAJECTORY, ControllerStruct, gamma);
%                                  SimStruct);
% ERROR = ode4(@ERROR_Fuzzy,t,InitERROR,...
%                                 SimStruct); 


options = odeset('RelTol',1e-9,'AbsTol',1e-9);
[t,STATE] = ode45(@(t,y) DRONE_SANTANA(t,y,SimStruct),t,InitSTATE,options);
[t,ERROR] = ode45(@(t,y) ERROR_Fuzzy(t,y,SimStruct),t,InitERROR,options);
                                 


%% OUTPUT PLOTTING
gamma
InitSTATE
simStep
Tfinal
TRAJECTORY
SimStruct.controller
SimStruct.sys.sat

Output;

Check_FuzzyTracking;
figure(1)

if(sphinx)
    
else
    disp('os Ks usados: ') 
    disp((gamma'))
    Kfuzzy = [];
    for i=1:length(K)
        Kfuzzy = [Kfuzzy;K{i}];
    end
    dlmwrite('K.cpp',Kfuzzy,'delimiter',',','precision','%.5f')
    disp('o arquivo esta em k.cpp')
end