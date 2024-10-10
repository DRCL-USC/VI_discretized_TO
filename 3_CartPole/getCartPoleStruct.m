function [nx, robotStruct, robotParams] = getCartPoleStruct()
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


%% System Parameters

mCart = 1;
mPole = 1;
hCart = 0.3;
lCart = 0.4;
dCart = 0.1;
lPole = 1;
R = 0.03; % Radius of Cylindrical Pendula (for Izz)

%% Create robotStruct structure
nx = 2;
q = sym('q', [nx 1]);

robotStruct.Origin.type = "Joint";
robotStruct.Origin.parent = "";
robotStruct.Origin.location = [0;0;0];
robotStruct.Origin.angle = 0;
robotStruct.Origin.axis = [0;1;0];
robotStruct.Origin.output = 0;

robotStruct.Cart.type = "Joint";
robotStruct.Cart.parent = "Origin";
robotStruct.Cart.location = [q(1); 0; 0];
robotStruct.Cart.angle = 0;
robotStruct.Cart.axis = [0; 1; 0];
robotStruct.Cart.output = 1;

robotStruct.CartCG.type = "Mass";
robotStruct.CartCG.parent = "Cart";
robotStruct.CartCG.location = [0; 0; 0];
robotStruct.CartCG.mass = mCart;
robotStruct.CartCG.MoI = [(1/12)*mCart*(hCart^2+dCart^2), (1/12)*mCart*(hCart^2+lCart^2), (1/12)*mCart*(lCart^2+dCart^2)];
robotStruct.CartCG.PoI = [0, 0, 0];
robotStruct.CartCG.output = 0;

robotStruct.Pivot.type = "Joint";
robotStruct.Pivot.parent = "Cart";
robotStruct.Pivot.location = [0; 0; 0];
robotStruct.Pivot.angle = q(2);
robotStruct.Pivot.axis = [0;1;0];
robotStruct.Pivot.output = 0;

robotStruct.Pole.type = "Mass";
robotStruct.Pole.parent = "Pivot";
robotStruct.Pole.location = [0; 0; -lPole/2];
robotStruct.Pole.mass = mPole;
robotStruct.Pole.MoI = [(1/12)*mPole*(3*R^2+lPole^2), (1/12)*mPole*(3*R^2+lPole^2), (1/2)*mPole*R^2];
robotStruct.Pole.PoI = [0, 0, 0];
robotStruct.Pole.output = 0;

robotStruct.Tip.type = "Joint";
robotStruct.Tip.parent = "Pivot";
robotStruct.Tip.location = [0; 0; -lPole];
robotStruct.Tip.angle = 0;
robotStruct.Tip.axis = [0;1;0];
robotStruct.Tip.output = 1;

%% Useful Robot Parameters
robotParams.lPole = lPole;
robotParams.R = R;
robotParams.lCart = lCart;
robotParams.hCart = hCart;
robotParams.dCart = dCart;


end