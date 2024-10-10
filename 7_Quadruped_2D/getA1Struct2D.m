function [nx, robotStruct, robotParams] = getA1Struct2D()
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

%% Generate Composite Trunk Mass Properties
%Hip Joint Transforms
BodyCGToHipF = [0.1805; 0; 0];
BodyCGToHipR = [-0.1805; 0; 0];

%Front Motor Params
CG_MotorF = [-0.003311; 0; 0];
mMotor1 = 2*0.696;
IMotor1 = 2*0.00080749;
BodyCGToMotorFCG = BodyCGToHipF + CG_MotorF;

%Rear Motor Params (Hip Roll)
CG_MotorR = [0.003311; 0; 0];
mMotor2 = 2*0.696;
IMotor2 = 2*0.00080749;
BodyCGToMotorRCG = BodyCGToHipR + CG_MotorR;

%Body Params
CG_Body = [0; 0; 0];
mBody = 4.713;
IBody = 0.0377999;

% Calculate Composite Trunk CG
trunkMasses = [mMotor1; mMotor2; mBody];
trunkComponentCGs = [BodyCGToMotorFCG BodyCGToMotorRCG CG_Body].';

mTrunk = sum(trunkMasses);
trunkCG = sum((trunkMasses*ones(1,3)).*trunkComponentCGs)/mTrunk;

%Calculate Composite Trunk Moment of Inertia
trunkMOIs = [IMotor1; IMotor2; IBody];

%Calculate distances between each component and the composite CG
trunkComponentsCGsToTrunkCG = trunkCG - trunkComponentCGs;
trunkComponentDistToCG = vecnorm(trunkComponentsCGsToTrunkCG.').';

ITrunk = sum(trunkMOIs + trunkMasses.*trunkComponentDistToCG.^2);
MoITrunk = [0.002997972, ITrunk, 3.2426e-05];
PoITrunk = [0, -0.000141163, 0];

%% Composite Calf/Foot Mass Properties
%Foot Transform
KneeToFoot = [0; 0; -0.2];
rFoot = 0.02;

%Calf Params
mCalf = 0.166;
ICalf = 0.003014022;
KneeToCalfCG = [0.006435; 0; -0.107388];

%Rear Motor Params (Hip Roll)
mFoot = 0.06;
IFoot = 0;
CG_Foot = [0; 0; 0];
KneeToFootCG = KneeToFoot + CG_Foot;

% Calculate Composite Calf CG
calfMasses = [mCalf; mFoot];
calfComponentCGs = [KneeToCalfCG KneeToFootCG].';

calfCG = sum((calfMasses*ones(1,3)).*calfComponentCGs)/sum(calfMasses);

%Calculate Composite Calf Moment of Inertia
calfMOIs = [ICalf; IFoot];

%Calculate distances between each component and the composite CG
calfComponentsCGsToCalfCG = calfCG - calfComponentCGs;
calfComponentDistToCG = vecnorm(calfComponentsCGsToCalfCG.').';

ICalf = sum(calfMOIs + calfMasses.*calfComponentDistToCG.^2);
MoICalf = 2 * [0.002997972, ICalf, 3.2426e-05];
PoICalf = 2 * [0, -0.000141163, 0];
mCalf = 2*sum(calfMasses);


%% Non-Composite Body Definitons
%Joint Transforms
HipToKnee = [0; 0; -0.2];

%CG Transforms
thighCG = [-0.003237; 0; -0.027326];

%Masses
mThigh = 2*1.013;

%MoIs
MoIThigh = 2*[0.005529065, 0.005139339, 0.001367788];
PoIThigh = 2*[2.2448e-05, 0.000343869, 4.825e-06];

%% Create robotStruct structure
nx = 7;
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
robotStruct.TrunkCG.location = trunkCG;
robotStruct.TrunkCG.mass = mTrunk;
robotStruct.TrunkCG.MoI = MoITrunk;
robotStruct.TrunkCG.PoI = PoITrunk;
robotStruct.TrunkCG.output = 0;

robotStruct.HipF.type = "Joint";
robotStruct.HipF.parent = "Base";
robotStruct.HipF.location = BodyCGToHipF;
robotStruct.HipF.angle = q(4);
robotStruct.HipF.axis = [0;1;0];
robotStruct.HipF.output = 1;

robotStruct.HipR.type = "Joint";
robotStruct.HipR.parent = "Base";
robotStruct.HipR.location = BodyCGToHipR;
robotStruct.HipR.angle = q(6);
robotStruct.HipR.axis = [0;1;0];
robotStruct.HipR.output = 1;

robotStruct.ThighF.type = "Mass";
robotStruct.ThighF.parent = "HipF";
robotStruct.ThighF.location = thighCG;
robotStruct.ThighF.mass = mThigh;
robotStruct.ThighF.MoI = MoIThigh;
robotStruct.ThighF.PoI = PoIThigh;
robotStruct.ThighF.output = 0;

robotStruct.ThighR.type = "Mass";
robotStruct.ThighR.parent = "HipR";
robotStruct.ThighR.location = thighCG;
robotStruct.ThighR.mass = mThigh;
robotStruct.ThighR.MoI = MoIThigh;
robotStruct.ThighR.PoI = PoIThigh;
robotStruct.ThighR.output = 0;

robotStruct.KneeF.type = "Joint";
robotStruct.KneeF.parent = "HipF";
robotStruct.KneeF.location = HipToKnee;
robotStruct.KneeF.angle = q(5);
robotStruct.KneeF.axis = [0;1;0];
robotStruct.KneeF.output = 1;

robotStruct.KneeR.type = "Joint";
robotStruct.KneeR.parent = "HipR";
robotStruct.KneeR.location = HipToKnee;
robotStruct.KneeR.angle = q(7);
robotStruct.KneeR.axis = [0;1;0];
robotStruct.KneeR.output = 1;

robotStruct.CalfF.type = "Mass";
robotStruct.CalfF.parent = "KneeF";
robotStruct.CalfF.location = calfCG;
robotStruct.CalfF.mass = mCalf;
robotStruct.CalfF.MoI = MoICalf;
robotStruct.CalfF.PoI = PoICalf;
robotStruct.CalfF.output = 0;

robotStruct.CalfR.type = "Mass";
robotStruct.CalfR.parent = "KneeR";
robotStruct.CalfR.location = calfCG;
robotStruct.CalfR.mass = mCalf;
robotStruct.CalfR.MoI = MoICalf;
robotStruct.CalfR.PoI = PoICalf;
robotStruct.CalfR.output = 0;

robotStruct.FootCGF.type = "Joint";
robotStruct.FootCGF.parent = "KneeF";
robotStruct.FootCGF.location = KneeToFootCG;
robotStruct.FootCGF.angle = -(q(3) + q(4) + q(5));
robotStruct.FootCGF.axis = [0;1;0];
robotStruct.FootCGF.output = 0;

robotStruct.FootCGR.type = "Joint";
robotStruct.FootCGR.parent = "KneeR";
robotStruct.FootCGR.location = KneeToFootCG;
robotStruct.FootCGR.angle = -(q(3) + q(6) + q(7));
robotStruct.FootCGR.axis = [0;1;0];
robotStruct.FootCGR.output = 0;

robotStruct.FootF.type = "Contact";
robotStruct.FootF.parent = "FootCGF";
robotStruct.FootF.location = rFoot * [0;0;-1];
robotStruct.FootF.output = 1;

robotStruct.FootR.type = "Contact";
robotStruct.FootR.parent = "FootCGR";
robotStruct.FootR.location = rFoot * [0;0;-1];
robotStruct.FootR.output = 1;

%% Useful Robot Parameters
robotParams.rFoot = rFoot;
robotParams.mFoot = mFoot;
end