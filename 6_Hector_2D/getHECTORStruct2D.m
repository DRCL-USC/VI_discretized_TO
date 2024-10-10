function [nx, robotStruct, robotParams] = getHECTORStruct2D()
%An initialization function that returns the robotStruct data structure
%containing the structure of the A1 Quadruped Robot
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

%% Generate Composite Trunk CG and Moment of Inertia
%Includes Hip Yaw and Roll Motors and Mounting Brackets

%Hector Hip Joint Transforms
TrunkRToHip1R = [0; 0; -0.1265];
Hip1RToHip2R = [0.053-0.014; 0;  -0.071];
Motor1RToHip2R = [-0.014; 0; 0];
Hip2RToMotor2R = [-0.06; 0; 0];

%Hip 1 Params
CG_Hip1 = [0.0268; 0; -0.0272];
mHip1 = 2*0.173+.00000011;
IHip1 = 2*0.00024;
HipJointToHip1CG = CG_Hip1 + -1*(Hip1RToHip2R + Hip2RToMotor2R);

%Hip 2 Params
CG_Hip2 = [-0.0308; 0; 0];
mHip2 = 2*0.0722;
IHip2 = 2*0.000120;
HipJointToHip2CG = CG_Hip2 + -1*Hip2RToMotor2R;

%Motor 1 Params (Hip Yaw)
CG_Motor1 = [0.02; 0; 0];
mMotor1 = 2*0.605;
IMotor1 = 2*0.00048;
HipJointToMotor1CG = CG_Motor1 + -1*(Motor1RToHip2R + Hip2RToMotor2R);

%Motor 2 Params (Hip Roll)
CG_Motor2 = [0; 0; 0];
mMotor2 = 2*0.605;
IMotor2 = 2*0.00071;
HipJointToMotor2CG = CG_Motor2;

%Body Params
CG_Body = [0; 0; 0];
mBody = 5.75;
IBody = 53149*1e-6;
HipJointToBodyCG = CG_Body + -1*(TrunkRToHip1R + Hip1RToHip2R + Hip2RToMotor2R);

% Calculate Composite Trunk CG
trunkMasses = [mHip1; mHip2; mMotor1; mMotor2; mBody];
trunkComponentCGs = [HipJointToHip1CG HipJointToHip2CG HipJointToMotor1CG HipJointToMotor2CG HipJointToBodyCG].';

trunkCG = sum((trunkMasses*ones(1,3)).*trunkComponentCGs)/sum(trunkMasses);

%Calculate Composite Trunk Moment of Inertia
trunkMOIs = [IHip1; IHip2; IMotor1; IMotor2; IBody];

%Calculate distances between each component and the composite CG
trunkComponentsCGsToTrunkCG = trunkCG - trunkComponentCGs;
trunkComponentDistToCG = vecnorm(trunkComponentsCGsToTrunkCG.').';

mTrunk = sum(trunkMasses);
ITrunk = sum(trunkMOIs + trunkMasses.*trunkComponentDistToCG.^2);

MoITrunk = [44350*1e-6, ITrunk, 21390*1e-6];
PoITrunk = [0, 0, 0];

%% Non-Composite Body Definitons
%Joint Transforms
BodyCGToHip = -1*trunkCG;
HipToKnee = [0; 0; -0.22];
KneeToAnkle = [0; 0; -0.22];

%CG Transforms
thighCG = [-0.00143; 0; -0.06115];
calfCG = [0.00165; 0; -0.06438];
footCG = [0.00239; 0; -0.02261];

%Masses
mThigh = 2*0.397;
mCalf = 2*0.136;
mFoot = 2*0.08425;

%MoIs
MoIThigh = 2*[0.00197, 0.00218, 0.00033];
PoIThigh = 2*[0, 0, 0];

MoICalf = 2*[0.00071, 0.00072, 0.00002];
PoICalf = 2*[0, 0, 0];

MoIFoot = 2*[0.00001, 0.00011, 0.00011];
PoIFoot = 2*[0, 0, 0];

%Contact Transforms
AnkleToToe = [0.09; 0; -0.039];
AnkleToHeel = [-0.06; 0; -0.039];

%% Create robotStruct structure
nx = 6;
q = sym('q', [nx 1]);

robotStruct.Origin.type = "Joint";
robotStruct.Origin.parent = "";
robotStruct.Origin.location = [0;0;0];
robotStruct.Origin.angle = 0;
robotStruct.Origin.axis = [0;1;0];
robotStruct.Origin.output = 0;

robotStruct.Base.type = "Joint";
robotStruct.Base.parent = "Origin";
robotStruct.Base.location = [q(1); 0; q(2)];
robotStruct.Base.angle = q(3);
robotStruct.Base.axis = [0; 1; 0];
robotStruct.Base.output = 0;

robotStruct.TrunkCG.type = "Mass";
robotStruct.TrunkCG.parent = "Base";
robotStruct.TrunkCG.location = [0;0;0];
robotStruct.TrunkCG.mass = mTrunk;
robotStruct.TrunkCG.MoI = MoITrunk;
robotStruct.TrunkCG.PoI = PoITrunk;
robotStruct.TrunkCG.output = 0;

robotStruct.Hip.type = "Joint";
robotStruct.Hip.parent = "Base";
robotStruct.Hip.location = BodyCGToHip;
robotStruct.Hip.angle = q(4);
robotStruct.Hip.axis = [0;1;0];
robotStruct.Hip.output = 1;

robotStruct.Tip.type = "Joint";
robotStruct.Tip.parent = "Base";
robotStruct.Tip.location = -1*BodyCGToHip;
robotStruct.Tip.angle = q(4);
robotStruct.Tip.axis = [0;1;0];
robotStruct.Tip.output = 1;

robotStruct.Thigh.type = "Mass";
robotStruct.Thigh.parent = "Hip";
robotStruct.Thigh.location = thighCG;
robotStruct.Thigh.mass = mThigh;
robotStruct.Thigh.MoI = MoIThigh;
robotStruct.Thigh.PoI = PoIThigh;
robotStruct.Thigh.output = 0;

robotStruct.Knee.type = "Joint";
robotStruct.Knee.parent = "Hip";
robotStruct.Knee.location = HipToKnee;
robotStruct.Knee.angle = q(5);
robotStruct.Knee.axis = [0;1;0];
robotStruct.Knee.output = 1;

robotStruct.Calf.type = "Mass";
robotStruct.Calf.parent = "Knee";
robotStruct.Calf.location = calfCG;
robotStruct.Calf.mass = mCalf;
robotStruct.Calf.MoI = MoICalf;
robotStruct.Calf.PoI = PoICalf;
robotStruct.Calf.output = 0;

robotStruct.Ankle.type = "Joint";
robotStruct.Ankle.parent = "Knee";
robotStruct.Ankle.location = KneeToAnkle;
robotStruct.Ankle.angle = q(6);
robotStruct.Ankle.axis = [0;1;0];
robotStruct.Ankle.output = 1;

robotStruct.Foot.type = "Mass";
robotStruct.Foot.parent = "Ankle";
robotStruct.Foot.location = footCG;
robotStruct.Foot.mass = mFoot;
robotStruct.Foot.MoI = MoIFoot;
robotStruct.Foot.PoI = PoIFoot;
robotStruct.Foot.output = 0;

robotStruct.FootF.type = "Contact";
robotStruct.FootF.parent = "Ankle";
robotStruct.FootF.location = AnkleToToe;
robotStruct.FootF.output = 1;

robotStruct.FootR.type = "Contact";
robotStruct.FootR.parent = "Ankle";
robotStruct.FootR.location = AnkleToHeel;
robotStruct.FootR.output = 1;

%% Useful Robot Parameters
%Used for Simulink Setup
robotParams.TrunkCGToRef = [trunkCG(1) - trunkComponentCGs(5,1) 0 trunkCG(3) - trunkComponentCGs(5,3)];
robotParams.trunkCG = trunkCG;
robotParams.trunkComponents = trunkComponentCGs;
end