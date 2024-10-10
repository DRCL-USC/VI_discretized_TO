function outStruct = genDynamics(nx, robotStruct)
% Quadruped 2D Robot (Modelled on A1 Robot)
    oldpath = path;
    path(pwd, oldpath);
    if isfolder("dynamicsFunctions")
        addpath("dynamicsFunctions")
        if isfile(fullfile("dynamicsFunctions", "currentRobotStruct.mat"))
            load(fullfile("dynamicsFunctions", "currentRobotStruct.mat"), "currentRobotStruct");
            if isequal(robotStruct, currentRobotStruct)
                fprintf("Previously generated dynamics functions are still valid - Using previous dynamics\n")
                addpath("dynamicsFunctions")
                outStruct = 1;
                return
            else
                fprintf("Previously generated dynamics functions are obsolete - Regenerating\n");
                delete(fullfile("dynamicsFunctions","*"))
            end
        end
    else
        mkdir("dynamicsFunctions")
        addpath("dynamicsFunctions")
    end

    currentRobotStruct = robotStruct;
    save(fullfile("dynamicsFunctions", "currentRobotStruct.mat"), "currentRobotStruct", '-mat');
    
%% Set Up Variables
    tic

    %Generate n symbolic variables for robot configuration and velocity
    q = sym('q', [nx 1]);
    dq = sym('dq', [nx 1]);
    assume(q, "real");
    assume(dq, "real");

%% Physical Constants
    %g = [0; 9.81; 0];
    g = [0; 0; 9.81];

%% Generate Robot Structure from Node Description
    fprintf("Generating Robot Structure...")
    names = fieldnames(robotStruct);
    
    for ii = 1:length(names)
        activeNode = robotStruct.(names{ii});
        if ~isfield(activeNode, "axis")
            activeNode.axis = [0;1;0];
            activeNode.angle = 0;
        end
        robotStruct.(names{ii}).H = homogTr(activeNode.angle, activeNode.axis, activeNode.location);
        nodeTree = [];
        while ~isequal(activeNode.parent, "")
            nodeTree = [string(activeNode.parent); nodeTree];
            activeNode = robotStruct.(string(activeNode.parent));
        end
        robotStruct.(names{ii}).tree = nodeTree;
    end
    fprintf("Done!\n")
       

%% Kinematics
    fprintf("Calculating Robot Kinematics...")
    
    for ii = 1:length(names)
        activeNode = robotStruct.(names{ii});
        activeTree = activeNode.tree;
        Htot = eye(4);
        for jj = 1:length(activeTree)
            Htot = Htot*robotStruct.(activeTree(jj)).H;
        end
        Htot = Htot*robotStruct.(names{ii}).H;
        node_X = Htot(1,4);
        node_vX = jacobian(node_X, q)*dq;
        node_Y = Htot(2,4);
        node_vY = jacobian(node_Y, q)*dq;
        node_Z = Htot(3,4);
        node_vZ = jacobian(node_Z, q)*dq;

        robotStruct.(names{ii}).pos = [node_X;node_Y;node_Z];
        robotStruct.(names{ii}).vel = [node_vX;node_vY;node_vZ];
        robotStruct.(names{ii}).R = Htot(1:3,1:3);
    end
    
    fprintf("Writing to function...")
    
    for ii = 1:length(names)
        if ~isfield(robotStruct.(names{ii}), "output")
            if ~isequal(robotStruct.(names{ii}).type, "Mass") && ~isequal(names{ii},"Origin")
                %By default, do not output functions for CG coordinates or the origin
                robotStruct.(names{ii}).output = 0;
            end
        end
        if robotStruct.(names{ii}).output
            functionName = "p" + names{ii};
            matlabFunction(robotStruct.(names{ii}).pos, "File", "dynamicsFunctions\" + functionName, "Vars", {q});
        end
    end
    
    fprintf("Done!\n")

%% Contact Jacobian
    fprintf("Calculating Feet Jacobian...")
    
    qFeet = [];
    for ii = 1:length(names)
        if isequal(robotStruct.(names{ii}).type, "Contact")
            qFeet = [qFeet; robotStruct.(names{ii}).pos];
        end
    end
    Jf = simplify(jacobian(qFeet,q));
    
    fprintf("Writing to function...")
    matlabFunction(Jf, "File", "dynamicsFunctions\Jf", "Vars", {q});
    fprintf("Done!\n")

%% Energy and Lagrangian
    fprintf("Calculating Energies and Lagrangian...")
    
    KE = 0;
    PE = 0;
    
    for ii = 1:length(names)
        if isequal(robotStruct.(names{ii}).type,"Mass")
            m = robotStruct.(names{ii}).mass;
            I = inertiaMat(robotStruct.(names{ii}).MoI, robotStruct.(names{ii}).PoI);
    
            PE = PE + m* g.'*robotStruct.(names{ii}).pos;
    
            %Add Translational Kinetic Energy 
            KE = KE + (1/2)*m*sum((robotStruct.(names{ii}).vel).^2);
    
            %Add Rotational Kinetic Energy 
            %(TODO: Implement rotational KE case for non-planar joints)

            R = robotStruct.(names{ii}).R;
            Rdot = zeros(3);
            for jj = 1:nx
                Rdot = Rdot + diff(R, q(jj))*dq(jj);
            end
            w_skew = simplify(R.'*Rdot);
            w = skewSymToVec(w_skew);
    
            KE = KE + (1/2)*(w.'*I*w);
        end
    end
    
    L = simplify(KE - PE);
    
    fprintf("Done!\n")

%% Euler Method Functions
    fprintf("Calculating Mass Matrix...")
    f_qdq = jacobian(L, dq)';
    M = simplify(jacobian(f_qdq,dq));

    fprintf("Writing to function...")
    matlabFunction(M, "File", "dynamicsFunctions\M", "Vars", {q});
    fprintf("Done!\n")

    fprintf("Calculating C Vector...")
    CMat = simplify(jacobian(f_qdq, q)*dq - jacobian(KE, q)' + jacobian(PE,q)'); 

    fprintf("Writing to function...")
    matlabFunction(CMat, "File", "dynamicsFunctions\C", "Vars", {[q;dq]});
    fprintf("Done!\n")

%% VI Momenta
    fprintf("Calculating Variational Integrator Momenta...")
    syms dt

    p0 = simplify((-dt/2)*jacobian(L,q)' + jacobian(L,dq)');
    p1 = simplify((dt/2)*jacobian(L,q)' + jacobian(L,dq)');

    fprintf("Writing to function...")
    matlabFunction(p0, "File", "dynamicsFunctions\p0", "Vars", {[q;dq], dt});
    matlabFunction(p1, "File", "dynamicsFunctions\p1", "Vars", {[q;dq], dt});
    fprintf("Done!\n")

%% Auxilliary Functions
    fprintf("Calculating Auxilliary Functions...")

    %Terminal VI Conditions
    if nx > 3
        pBody = sym('pBody', [3 1]);
        assume(pBody, "real");
        momenta = jacobian(L, dq).';
        momentaTerm = simplify(subs(momenta, dq(4:end), zeros(size(dq(4:end))))); % Momenta with Zero Joint Rates
        bodyRatesTermStruct = solve(momentaTerm(1:3) == pBody, [dq(1) dq(2) dq(3)]);
        bodyRatesTerm = [bodyRatesTermStruct.dq1; bodyRatesTermStruct.dq2; bodyRatesTermStruct.dq3];
        VI_JointTermConditions = subs(momentaTerm(4:end), [dq(1); dq(2); dq(3)], bodyRatesTerm);
    
        fprintf("Writing to function...")
        matlabFunction(VI_JointTermConditions, "File", "dynamicsFunctions\VI_JointTermCon", "Vars", {[q;pBody;dq(4:end)]});
    end

            fprintf("Done!\n")

    oldpath = path;
    path(genpath('dynamicsFunctions'),  oldpath);

    outStruct = robotStruct;
    toc

end

function H = homogTr(ang, axis, trans)
    if size(trans,2) > size(trans,1)
        trans = trans';
    end
    H = [rotFromAxis(ang,axis), trans;
        0 0 0 1];
end

function rotm = rotFromAxis(ang, axis)
    if size(axis,2) > size(axis,1)
        axis = axis';
    end
    normAxis = axis/norm(axis);

    rotm = cos(ang)*eye(3) + sin(ang)*skewSym(normAxis) + (1-cos(ang))*(normAxis)*normAxis';
end

function skewMat = skewSym(vec)
    skewMat = [0 -vec(3) vec(2);
               vec(3) 0 -vec(1);
               -vec(2) vec(1) 0;];
end

function vec = skewSymToVec(s)
    %Check if the Matrix is skew symmetric
    if ~isequal(s(1,2), -s(2,1)) || ~isequal(s(1,3), -s(3,1)) || ~isequal(s(2,3), -s(3,2)) || ~all(isAlways(diag(s)==0))
        error("Matrix is not skew symmetric! Check inputs")
    else
        vec = [s(2,1);s(1,3);s(3,2)];
    end
end