function [nx, robotStruct, robotParams] = getSinglePendStruct()
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

m = 1;
L = 1;
R = 0.05; % Radius of Cylindrical Pendulum

%% Create robotStruct structure
nx = 1;
q = sym('q', [nx 1]);

robotStruct.Origin.type = "Joint";
robotStruct.Origin.parent = "";
robotStruct.Origin.location = [0;0;0];
robotStruct.Origin.angle = 0;
robotStruct.Origin.axis = [0;1;0];
robotStruct.Origin.output = 0;

robotStruct.Pivot.type = "Joint";
robotStruct.Pivot.parent = "Origin";
robotStruct.Pivot.location = [0; 0; 0];
robotStruct.Pivot.angle = q(1);
robotStruct.Pivot.axis = [0; 1; 0];
robotStruct.Pivot.output = 0;

robotStruct.LinkCG.type = "Mass";
robotStruct.LinkCG.parent = "Pivot";
robotStruct.LinkCG.location = [0; 0; -L/2];
robotStruct.LinkCG.mass = m;
robotStruct.LinkCG.MoI = [(1/12)*m*(3*R^2+L^2), (1/12)*m*(3*R^2+L^2), (1/2)*m*R^2];
robotStruct.LinkCG.PoI = [0, 0, 0];
robotStruct.LinkCG.output = 0;

robotStruct.Tip.type = "Joint";
robotStruct.Tip.parent = "Pivot";
robotStruct.Tip.location = [0; 0; -L];
robotStruct.Tip.angle = 0;
robotStruct.Tip.axis = [0;1;0];
robotStruct.Tip.output = 1;

%% Useful Robot Parameters
robotParams.L = L;
robotParams.R = R;
end