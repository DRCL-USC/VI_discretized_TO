function [nx, robotStruct, robotParams] = getG1Struct2D()
%An initialization function that returns the robotStruct data structure
%containing the structure of the G1 Biped Robot
%nx is the number of generalized coordinates present in this model
%
% robotStruct contains a series of nodes of 3 types
%   "Joint" type nodes are revolute or prismatic joints that connect nodes
%   "Mass" type nodes are rigidly connected to joints and contain inertial data
%   "Contact" type nodes are massless and rigidly connected to joints and are the defined contact points with the terrain
%
%robotParams is an output which contains any relevant parameters that may
%not be directly used in dynamics generation, but are nonetheless useful 
%(for example parameters that are used for the Simulink simulation)

%% Create robotStruct structure
nx = 6;
q = sym('q', [nx 1]);

%% Origin and Mobile Base Frames

Origin.type = "Joint";
Origin.parent = "";
Origin.location = [0;0;0];
Origin.angle = 0;
Origin.axis = [0;1;0];
Origin.output = 0;

Base.type = "Joint";
Base.parent = "Origin";
Base.location = [q(1); 0; q(2)];
Base.angle = q(3);
Base.axis = [0; 1; 0];
Base.output = 1;

%% Pelvis Bodies

Pelvis.type = "Mass";
Pelvis.parent = "Base";
Pelvis.location = [0; 0; -0.07605];
Pelvis.mass = 3.813;
Pelvis.MoI = [0.010549; 0.0093089; 0.0079184];
Pelvis.PoI = [0;0;0];
Pelvis.output = 0;

PelvisContour.type = "Mass";
PelvisContour.parent = "Base";
PelvisContour.location = [0;0;0];
PelvisContour.mass = 0.001;
PelvisContour.MoI = [1e-07; 1e-07; 1e-07];
PelvisContour.PoI = [0;0;0];
PelvisContour.output = 0;

%% Trunk Bodies

WaistYaw.type = "Joint";
WaistYaw.parent = "Base";
WaistYaw.location = [-0.0039635; 0; 0.054];
WaistYaw.angle = 0; %Change if making 3D
WaistYaw.axis = [0;0;1];
WaistYaw.output = 0;

Torso.type = "Mass";
Torso.parent = "WaistYaw";
%Torso.location = [0.002601, 0.000257, 0.153719]';
Torso.location = [0.002601, 0, 0.153719]';
Torso.mass = 8.562;
Torso.MoI = [0.065674966, 0.053535188, 0.030808125];
Torso.PoI = [8.6899e-05, -0.001737252, -8.597e-05];
Torso.output = 0;

WaistYawFixed.type = "Mass";
WaistYawFixed.parent = "WaistYaw";
WaistYawFixed.location = [0.0039635, 0, -0.054]' + [0.003964, 0, 0.018769]';
WaistYawFixed.mass = 0.244;
WaistYawFixed.MoI = [9.9587e-05, 0.00012411, 0.00015586]';
WaistYawFixed.PoI = [-1.18e-07, -1.2617e-05, -1.833e-06]';
WaistYawFixed.output = 0;

WaistSupport.type = "Mass";
WaistSupport.parent = "WaistYaw";
WaistSupport.location = [0.0039635, 0, -0.054]' ;
WaistSupport.mass = 0.001;
WaistSupport.MoI = [1e-07, 1e-07, 1e-07]';
WaistSupport.PoI = [0;0;0];
WaistSupport.output = 0;

Head.type = "Mass";
Head.parent = "WaistYaw";
%Head.location = [0.0039635, 0, -0.054]' + [0.005267, 0.000299, 0.449869]';
Head.location = [0.0039635, 0, -0.054]' + [0.005267, 0, 0.449869]';
Head.mass = 1.036;
Head.MoI = [0.004085051, 0.004185212, 0.001807911]';
Head.PoI = [-3.726e-06, -6.9455e-05, -2.543e-06]';
Head.output = 1;

%% Hip and Thigh Joints/Bodies

HipPitch.type = "Joint";
HipPitch.parent = "Base";
%HipPitch.location = [0, -0.064452, -0.1027]';
HipPitch.location = [0, 0, -0.1027]';
HipPitch.angle = q(4);
HipPitch.axis = [0;1;0];
HipPitch.output = 1;

HipPitchMotor.type = "Mass";
HipPitchMotor.parent = "HipPitch";
%HipPitchMotor.location = [0.002741, -0.047791, -0.02606]';
HipPitchMotor.location = [0.002741, 0, -0.02606]';
HipPitchMotor.mass = 2 * 1.35;
HipPitchMotor.MoI = 2 * [0.001811, 0.0014193, 0.0012812]';
HipPitchMotor.PoI = 2 * [-0.000171, -3.44e-05, -3.68e-05]';
HipPitchMotor.output = 0;

HipRollOrigin.type = "Joint";
HipRollOrigin.parent = "HipPitch";
%HipRollOrigin.location = [0, -0.052, -0.030465]';
HipRollOrigin.location = [0, 0, -0.030465]';
HipRollOrigin.angle = -0.1749;
HipRollOrigin.axis = [0;1;0];
HipRollOrigin.output = 0;

HipRoll.type = "Joint";
HipRoll.parent = "HipRollOrigin";
HipRoll.location = [0;0;0];
HipRoll.angle = 0; %Change this to make 3D
HipRoll.axis = [1;0;0];
HipRoll.output = 0;

UpperThigh.type = "Mass";
UpperThigh.parent = "HipRoll";
%UpperThigh.location = [0.029812, 0.001045, -0.087934]';
UpperThigh.location = [0.029812, 0, -0.087934]';
UpperThigh.mass = 2 * 1.52; %TODO: Multiply by 2
UpperThigh.MoI = 2 * [0.0023773, 0.0024123, 0.0016595]';
UpperThigh.PoI = 2 * [-1.84e-05, -0.0003908, 3.8e-06]';
UpperThigh.output = 0;

HipYaw.type = "Joint";
HipYaw.parent = "HipRoll";
HipYaw.location = [0.025001, 0, -0.12412]';
HipYaw.angle = 0; %Change this for 3D
HipYaw.axis = [0;0;1];
HipYaw.output = 0;

LowerThigh.type = "Mass";
LowerThigh.parent = "HipYaw";
%LowerThigh.location = [-0.057709, 0.010981, -0.15078]';
LowerThigh.location = [-0.057709, 0, -0.15078]';
LowerThigh.mass = 2 * 1.702;
LowerThigh.MoI = 2 * [0.0057774, 0.0076124, 0.003149]';
LowerThigh.PoI = 2 * [0.0007072, -0.0023948, 0.0005411]';
LowerThigh.output = 0;

%% Calf Joints and Bodies

Knee.type = "Joint";
Knee.parent = "HipYaw";
%Knee.location = [-0.078273, -0.0021489, -0.17734]';
Knee.location = [-0.078273, 0, -0.17734]';
Knee.angle = q(5) + 0.1749;
Knee.axis = [0;1;0];
Knee.output = 1;

Calf.type = "Mass";
Calf.parent = "Knee";
%Calf.location = [0.005457, -0.003964, -0.12074]';
Calf.location = [0.005457, 0, -0.12074]';
Calf.mass = 2 * 1.932;
Calf.MoI = 2 * [0.011329, 0.011277, 0.0015168]';
Calf.PoI = 2 * [0.0007146, 4.49e-05, -4.82e-05]';
Calf.output = 0;

%% Ankle and Foot Joints/Bodies

AnklePitch.type = "Joint";
AnklePitch.parent = "Knee";
AnklePitch.location = [0, 0, -0.30001]';
AnklePitch.angle = q(6);
AnklePitch.axis = [0;1;0];
AnklePitch.output = 1;

AnklePitchLink.type = "Mass";
AnklePitchLink.parent = "AnklePitch";
AnklePitchLink.location = [-0.007269, 0, 0.011137]';
AnklePitchLink.mass = 2 * 0.074;
AnklePitchLink.MoI = 2 * [8.4e-06, 1.89e-05, 1.26e-05]';
AnklePitchLink.PoI = 2 * [0, -2.9e-06, 0]';
AnklePitchLink.output = 0;

AnkleRoll.type = "Joint";
AnkleRoll.parent = "AnklePitch";
AnkleRoll.location = [0, 0, -0.017558]';
AnkleRoll.angle = 0; %change for 3D
AnkleRoll.axis = [1;0;0];
AnkleRoll.output = 0;

Foot.type = "Mass";
Foot.parent = "AnkleRoll";
Foot.location = [0.026505, 0, -0.016425]';
Foot.mass = 2 * 0.608;
Foot.MoI = 2 * [0.0002231, 0.0016161, 0.0016667]';
Foot.PoI = 2 * [1e-07, 8.91e-05, -2e-07]';
Foot.output = 0;

%% Contact Frames

FootF.type = "Contact";
FootF.parent = "AnkleRoll";
FootF.location = [0 0 -0.035]' + [0.140; 0; 0];
FootF.output = 1;

FootR.type = "Contact";
FootR.parent = "AnkleRoll";
FootR.location = [0 0 -0.035]' + [-0.06; 0; 0];
FootR.output = 1;

%% Combine into robotStruct

robotStruct = struct('Origin', Origin, 'Base', Base, ...
                     'Pelvis', Pelvis, 'PelvisContour', PelvisContour, ...
                     'WaistYaw', WaistYaw, 'Torso', Torso, 'WaistYawFixed', WaistYawFixed, ...
                     'WaistSupport', WaistSupport, 'Head', Head, ... 
                     'HipPitch', HipPitch, 'HipPitchMotor', HipPitchMotor, ...
                     'HipRollOrigin', HipRollOrigin, 'HipRoll', HipRoll, ...,
                     "UpperThigh", UpperThigh, 'HipYaw', HipYaw,'LowerThigh', LowerThigh,...
                     'Knee', Knee, 'Calf', Calf, ...
                     'AnklePitch', AnklePitch, 'AnklePitchLink', AnklePitchLink, ...
                     'AnkleRoll', AnkleRoll, 'Foot', Foot, ...
                     'FootF', FootF, 'FootR', FootR);

%% Useful Robot Parameters
%Used for Simulink Setup
robotParams = struct([]);
end