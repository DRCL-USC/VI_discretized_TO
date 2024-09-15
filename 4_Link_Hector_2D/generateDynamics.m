function generateDynamics(robotParams)
    oldpath = path;
    path(pwd,  oldpath);
    if isfolder("dynamicsFunctions")
        if isfile(fullfile("dynamicsFunctions", "oldRobotParams.mat"))
            load(fullfile("dynamicsFunctions", "oldRobotParams.mat"), "oldParams");
            if isequal(robotParams, oldParams)
                fprintf("Previously generated dynamics functions are still valid - Using previous dynamics\n")
                addpath("dynamicsFunctions")
                return
            else
                fprintf("Previously generated dynamics functions are obsolete - Regenerating\n");
                delete(fullfile("dynamicsFunctions","*"))
            end
        end
    else
        mkdir("dynamicsFunctions")
    end

    addpath("dynamicsFunctions")
    
    L1 = robotParams.L1;
    L2 = robotParams.L2;
    L3 = robotParams.L3;
    m1 = robotParams.m1;
    m2 = robotParams.m2;
    m3 = robotParams.m3;
    m4 = robotParams.m4;
    I1 = robotParams.I1;
    I2 = robotParams.I2;
    I3 = robotParams.I3;
    I4 = robotParams.I4;
    CG1 = robotParams.CG_TrunkToHipJoint;
    CG2 = robotParams.CG2;
    CG3 = robotParams.CG3;
    xfCG = robotParams.pfCG(1);
    yfCG = robotParams.pfCG(2);
    xf1 = robotParams.pfc1(1);
    yf1 = robotParams.pfc1(2);
    xf2 = robotParams.pfc2(1);
    yf2 = robotParams.pfc2(2);

    g = 9.81;

%% Set Up Variables
    tic
    q = sym('q', [6 1]);
    dq = sym('dq', [6 1]);
    assume(q, "real");
    assume(dq, "real");

%% Kinematics
    fprintf("Calculating Robot Kinematics...")
    
    H1 = homogTr(q(3), [0;0;-1], [q(1);q(2);0]);
    Hj1 = homogTr(q(4), [0;0;-1], [CG1;0]);
    H2 = homogTr(0, [0;0;1], [CG2;0]);
    Hj2 = homogTr(q(5), [0;0;-1], [0;-L2;0]);
    H3 = homogTr(0, [0;0;1], [CG3;0]);
    Hj3 = homogTr(q(6), [0;0;-1], [0;-L3;0]);
    H4 = homogTr(0, [0;0;1], [xfCG; yfCG;0]);
    Hf1 = homogTr(0, [0;0;1], [xf1; yf1;0]);
    Hf2 = homogTr(0, [0;0;1], [xf2; yf2;0]);

    %Trunk Coordinates
    tr1 = H1; x1 = tr1(1, 4); y1 = tr1(2, 4);
    vx1 = jacobian(x1, q)*dq; vy1 = jacobian(y1, q)*dq;
    %Hip Joint Coordinates
    trj1 = tr1*Hj1; xj1 = trj1(1, 4); yj1 = trj1(2, 4);
    %Thigh Coordinates
    tr2 = trj1*H2; x2 = tr2(1, 4); y2 = tr2(2, 4);
    vx2 = jacobian(x2, q)*dq; vy2 = jacobian(y2, q)*dq;
    %Knee Joint Coordinates
    trj2 = trj1*Hj2; xj2 = trj2(1, 4); yj2 = trj2(2, 4);
    %Calf Coordinates
    tr3 = trj2*H3; x3 = tr3(1, 4); y3 = tr3(2, 4);
    vx3 = jacobian(x3, q)*dq; vy3 = jacobian(y3, q)*dq;
    %Ankle Joint Coordinates
    trj3 = trj2*Hj3; xj3 = trj3(1, 4); yj3 = trj3(2, 4);
    %Foot CG Coordinates
    tr4 = trj3*H4; x4 = tr4(1, 4); y4 = tr4(2, 4);
    vx4 = jacobian(x4, q)*dq; vy4 = jacobian(y4, q)*dq;
    %Foot Contact 1 Coordinates
    trFoot1 = trj3*Hf1; xFoot1 = trFoot1(1, 4); yFoot1 = trFoot1(2, 4);
    %Foot Contact 2 Coordinates
    trFoot2 = trj3*Hf2; xFoot2 = trFoot2(1, 4); yFoot2 = trFoot2(2, 4);

    fprintf("Writing to function...")
    pJ1 = [xj1;yj1];
    pJ2 = [xj2;yj2];
    pJ3 = [xj3;yj3];
    pf1 = [xFoot1;yFoot1];
    pf2 = [xFoot2;yFoot2];

    matlabFunction(pJ1, "File", "dynamicsFunctions\pJ1", "Vars", {q});
    matlabFunction(pJ2, "File", "dynamicsFunctions\pJ2", "Vars", {q});
    matlabFunction(pJ3, "File", "dynamicsFunctions\pJ3", "Vars", {q});
    matlabFunction(pf1, "File", "dynamicsFunctions\pf1", "Vars", {q});
    matlabFunction(pf2, "File", "dynamicsFunctions\pf2", "Vars", {q});
    fprintf("Done!\n")

%% Jacobian
    fprintf("Calculating Feet Jacobian...")
    qFeet = [xFoot1;yFoot1;xFoot2;yFoot2];
    Jf = simplify(jacobian(qFeet,q));

    fprintf("Writing to function...")
    matlabFunction(Jf, "File", "dynamicsFunctions\Jf", "Vars", {q});
    fprintf("Done!\n")

%% Energy and Lagrangian
    fprintf("Calculating Energies and Lagrangian...")
    KE1 = (1/2)*m1*(vx1^2+vy1^2) + (1/2)*I1*(dq(3))^2;
    KE2 = (1/2)*m2*(vx2^2+vy2^2) + (1/2)*I2*(dq(3)+dq(4))^2;
    KE3 = (1/2)*m3*(vx3^2+vy3^2) + (1/2)*I3*(dq(3)+dq(4)+dq(5))^2;
    KE4 = (1/2)*m4*(vx4^2+vy4^2) + (1/2)*I4*(dq(3)+dq(4)+dq(5)+dq(6))^2;

    KE = KE1 + KE2 + KE3 + KE4;
    PE = m1*g*y1 + m2*g*y2 + m3*g*y3 + m4*g*y4;

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
    assume(dt, "real");

    p0 = simplify((-dt/2)*jacobian(L,q)' + jacobian(L,dq)');
    p1 = simplify((dt/2)*jacobian(L,q)' + jacobian(L,dq)');

    fprintf("Writing to function...")
    matlabFunction(p0, "File", "dynamicsFunctions\p0", "Vars", {[q;dq], dt});
    matlabFunction(p1, "File", "dynamicsFunctions\p1", "Vars", {[q;dq], dt});
    fprintf("Done!\n")

%% Auxilliary Functions
    fprintf("Calculating Auxilliary Functions...")
    
    % Initialization from Joint/Body Angles and Foot Location
    PF1 = sym('PF1', [2 1]);
    assume(PF1, "real");
    xyInitFromAnglesStruct = solve(pf1==PF1, [q(1) q(2)]);
    xyInitFromAngles = simplify([xyInitFromAnglesStruct.q1; xyInitFromAnglesStruct.q2]);

    %Terminal VI Conditions
    pBody = sym('pBody', [3 1]);
    assume(pBody, "real");
    momenta = jacobian(L, dq).';
    momentaTerm = simplify(subs(momenta, dq(4:end), zeros(size(dq(4:end))))); % Momenta with Zero Joint Rates
    bodyRatesTermStruct = solve(momentaTerm(1:3) == pBody, [dq(1) dq(2) dq(3)]);
    bodyRatesTerm = [bodyRatesTermStruct.dq1; bodyRatesTermStruct.dq2; bodyRatesTermStruct.dq3];
    VI_JointTermConditions = simplify(subs(momentaTerm(4:end), [dq(1); dq(2); dq(3)], bodyRatesTerm));
    % VI_JointTermConditions = subs(momentaTerm(4:end), [dq(1); dq(2); dq(3)], bodyRatesTerm);
    % TODO: Check if Simplify function here actually results in appreciable
    % difference in TO performance - Currently takes 18 seconds to
    % calculate, saves on order of 5e-7 seconds per execution, on avg.

    fprintf("Writing to function...")
    matlabFunction(xyInitFromAngles, "File", "dynamicsFunctions\xyInitFromAngles", "Vars", {PF1; [q(3:end)]});
    matlabFunction(VI_JointTermConditions, "File", "dynamicsFunctions\VI_JointTermCon", "Vars", {[q;pBody;dq(4:end)]});
    fprintf("Done!\n")
    toc

    oldParams = robotParams;
    save(fullfile("dynamicsFunctions", "oldRobotParams.mat"), "oldParams", '-mat');
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