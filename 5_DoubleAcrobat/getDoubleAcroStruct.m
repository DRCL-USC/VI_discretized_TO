function [nx, robotStruct, robotParams] = getDoubleAcroStruct()
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


%% Pendulum Parameters

m1 = 1;
L1 = 1;
m2 = 1;
L2 = 1;
R = 0.00000001; % Radius of Cylindrical Pendula (for Izz)

%% Create robotStruct structure
nx = 4;
q = sym('q', [nx 1]);

robotStruct.Origin.type = "Joint";
robotStruct.Origin.parent = "";
robotStruct.Origin.location = [0;0;0];
robotStruct.Origin.angle = 0;
robotStruct.Origin.axis = [0;1;0];
robotStruct.Origin.output = 0;

robotStruct.Pivot.type = "Joint";
robotStruct.Pivot.parent = "Origin";
robotStruct.Pivot.location = [q(1); 0; q(2)];
robotStruct.Pivot.angle = q(3);
robotStruct.Pivot.axis = [0; 1; 0];
robotStruct.Pivot.output = 0;

robotStruct.Pin.type = "Contact";
robotStruct.Pin.parent = "Pivot";
robotStruct.Pin.location = [0;0;0];
robotStruct.Pin.output = 1;

robotStruct.Link1CG.type = "Mass";
robotStruct.Link1CG.parent = "Pivot";
robotStruct.Link1CG.location = [0; 0; -L1/2];
robotStruct.Link1CG.mass = m1;
robotStruct.Link1CG.MoI = [(1/12)*m1*(3*R^2+L1^2), (1/12)*m1*(3*R^2+L1^2), (1/2)*m1*R^2];
robotStruct.Link1CG.PoI = [0, 0, 0];
robotStruct.Link1CG.output = 0;

robotStruct.Elbow.type = "Joint";
robotStruct.Elbow.parent = "Pivot";
robotStruct.Elbow.location = [0; 0; -L1];
robotStruct.Elbow.angle = q(4);
robotStruct.Elbow.axis = [0;1;0];
robotStruct.Elbow.output = 1;

robotStruct.Link2CG.type = "Mass";
robotStruct.Link2CG.parent = "Elbow";
robotStruct.Link2CG.location = [0; 0; -L2/2];
robotStruct.Link2CG.mass = m2;
robotStruct.Link2CG.MoI = [(1/12)*m2*(3*R^2+L2^2), (1/12)*m2*(3*R^2+L2^2), (1/2)*m2*R^2];
robotStruct.Link2CG.PoI = [0, 0, 0];
robotStruct.Link2CG.output = 0;

robotStruct.Tip.type = "Joint";
robotStruct.Tip.parent = "Elbow";
robotStruct.Tip.location = [0; 0; -L2];
robotStruct.Tip.angle = 0;
robotStruct.Tip.axis = [0;1;0];
robotStruct.Tip.output = 1;

%% Useful Robot Parameters
robotParams.L1 = L1;
robotParams.L2 = L2;
robotParams.R = R;
end