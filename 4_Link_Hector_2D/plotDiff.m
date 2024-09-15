clc; close all
load("EulerForwardResults.mat");
load("VIForwardResults.mat");
% Terminal Error

fprintf("Euler X Terminal Error: %.4f\n", pos(1,end)-XSim(1, end))
fprintf("VI X Terminal Error: %.4f\n\n", posVI(1,end)-XSimVI(1, end))
fprintf("Euler Y Terminal Error: %.4f\n", pos(2,end)-XSim(2, end))
fprintf("VI Y Terminal Error: %.4f\n\n", posVI(2,end)-XSimVI(2, end))
fprintf("Euler Theta Terminal Error: %.3f\n", pos(3,end)-XSim(3, end))
fprintf("VI Theta Terminal Error: %.3f\n\n", posVI(3,end)-XSimVI(3, end))

eF = pf1(XSim(:,end)) - pf1(pos(:,end));
eFVI = pf1(XSimVI(:,end)) - pf1(posVI(:,end));

fprintf("Foot Terminal Error: %.3f\n", norm(eF))
fprintf("VI Foot Error: %.3f\n\n", norm(eFVI))


% fprintf("Euler Hip Joint Terminal Error: %.3f\n", pos(4,end)-XSim(4, end))
% fprintf("VI Hip Joint Terminal Error: %.3f\n\n", posVI(4,end)-XSimVI(4, end))
% fprintf("Euler Knee Joint Terminal Error: %.3f\n", pos(5,end)-XSim(5, end))
% fprintf("VI Knee Joint Terminal Error: %.3f\n", posVI(5,end)-XSimVI(5, end))

%%
T = tTO(end);
dt = 0.01;

scaleSize = 2.5;
lineSize = 4;

f = figure;
f.Position = [100 100 1400 600];
clf()
t = tiledlayout(1,2,'TileSpacing','Compact','Padding','Compact');
title(t,"Body X Tracking - VI vs. Euler")

nexttile
hold on
plot(tSim, XSim(1,:), 'LineWidth',lineSize);
plot(tTO, pos(1,:),'--', 'LineWidth',lineSize);
title("Euler")
xlabel("Time (s)")
%ylabel("Body X (m)")
legend([ "Simulated Traj.", "Reference Traj."], 'Location','northwest')

nexttile
hold on
plot(tSimVI, XSimVI(1,:), 'LineWidth',lineSize);
plot(tTOVI, posVI(1,:),'--', 'LineWidth',lineSize);
title("VI")
xlabel("Time (s)")
%ylabel("Body X (m)")
legend([ "Simulated Traj.", "Reference Traj."], 'Location','northwest')
fontsize(gcf, scale=scaleSize)

f = figure;
f.Position = [100 100 1400 600];
clf()
t = tiledlayout(1,2,'TileSpacing','Compact','Padding','Compact');
title(t,"Body Y Tracking - Euler vs. VI")

nexttile
hold on
plot(tSim, XSim(2,:), 'LineWidth',lineSize);
plot(tTO, pos(2,:),'--', 'LineWidth',lineSize);
title("Euler")
xlabel("Time (s)")
%ylabel("Body Y (m)")
legend([ "Simulated Traj.", "Reference Traj."], 'Location','southwest')


nexttile
hold on
plot(tSimVI, XSimVI(2,:), 'LineWidth',lineSize);
plot(tTOVI, posVI(2,:),'--', 'LineWidth',lineSize);
title("VI")
xlabel("Time (s)")
%ylabel("Body Y (m)")
legend([ "Simulated Traj.", "Reference Traj."], 'Location','southwest')
fontsize(gcf, scale=scaleSize)

f = figure;
f.Position = [100 100 1400 600];
clf()
t = tiledlayout(1,2,'TileSpacing','Compact','Padding','Compact');
title(t,"Body Theta Tracking - Euler vs. VI")

nexttile
hold on
plot(tSim, XSim(3,:), 'LineWidth',lineSize);
plot(tTO, pos(3,:),'--', 'LineWidth',lineSize);
title("Euler")
xlabel("Time (s)")
ylabel("Body Angle (rad)")
legend([ "Simulated Traj.", "Reference Traj."], 'Location','southwest')


nexttile
hold on
plot(tSimVI, XSimVI(3,:), 'LineWidth',lineSize);
plot(tTOVI, posVI(3,:),'--', 'LineWidth',lineSize);
title("VI")
xlabel("Time (s)")
%ylabel("Body Angle (rad)")
legend([ "Simulated Traj.", "Reference Traj."], 'Location','southwest')
fontsize(gcf, scale=scaleSize)


f = figure;
f.Position = [100 100 1400 600];
clf()
t = tiledlayout(1,2,'TileSpacing','Compact','Padding','Compact');
title(t,"Body Hip Joint Tracking - Euler vs. VI")

nexttile
hold on
plot(tSim, XSim(4,:), 'LineWidth',lineSize);
plot(tTO, pos(4,:),'--', 'LineWidth',lineSize);
title("Euler")
xlabel("Time (s)")
ylabel("Hip Angle (rad)")
legend([ "Simulated Traj.", "Reference Traj."], 'Location','southwest')


nexttile
hold on
plot(tSimVI, XSimVI(4,:), 'LineWidth',lineSize);
plot(tTOVI, posVI(4,:),'--', 'LineWidth',lineSize);
title("VI")
xlabel("Time (s)")
%ylabel("Hip Angle (rad)")
legend([ "Simulated Traj.", "Reference Traj."], 'Location','southwest')
fontsize(gcf, scale=scaleSize)

f = figure;
f.Position = [100 100 1400 600];
clf()
t = tiledlayout(1,2,'TileSpacing','Compact','Padding','Compact');
title(t,"Body Knee Joint Tracking - Euler vs. VI")

nexttile
hold on
plot(tSim, XSim(5,:), 'LineWidth',lineSize);
plot(tTO, pos(5,:),'--', 'LineWidth',lineSize);
title("Euler")
xlabel("Time (s)")
ylabel("Knee Angle (rad)")
%legend([ "Simulated Traj.", "Reference Traj."], 'Location','southwest')


nexttile
hold on
plot(tSimVI, XSimVI(5,:), 'LineWidth',lineSize);
plot(tTOVI, posVI(5,:),'--', 'LineWidth',lineSize);
title("VI")
xlabel("Time (s)")
%ylabel("Hip Angle (rad)")
%legend([ "Simulated Traj.", "Reference Traj."], 'Location','southwest')
fontsize(gcf, scale=scaleSize)

f = figure;
f.Position = [100 100 1250 700];
clf()
t = tiledlayout(1,2,'TileSpacing','Compact','Padding','Compact');
title(t,"Body XY Trajectory Tracking - Euler vs. VI")

nexttile
hold on
plot(XSim(1,:), XSim(2,:), 'LineWidth',lineSize);
plot(pos(1,:), pos(2,:),'--', 'LineWidth',lineSize);
title("Euler")
xlabel("X (m)")
ylabel("Y (m)")
legend([ "Simulated Traj.", "Reference Traj."], 'Location','southwest')
axis equal

nexttile
hold on
plot(XSimVI(1,:), XSimVI(2,:), 'LineWidth',lineSize);
plot(posVI(1,:), posVI(2,:),'--', 'LineWidth',lineSize);
title("VI")
xlabel("X (m)")
%ylabel("Y (m)")
legend([ "Simulated Traj.", "Reference Traj."], 'Location','southwest')
axis equal
fontsize(gcf, scale=scaleSize)



%% Foot Error
 
pF_TO = zeros([2,max(size(tTO))]);
pF_TOVI = zeros([2,max(size(tTOVI))]);
pF_Sim = zeros([2,max(size(tSim))]);
pF_SimVI = zeros([2,max(size(tSimVI))]);

for ii = 1:max(size(tTO))
    pF_TO(:,ii) = pf1(pos(:,ii));
end

for ii = 1:max(size(tTOVI))
    pF_TOVI(:,ii) = pf1(posVI(:,ii));
end

for ii = 1:max(size(tSim))
    pF_Sim(:,ii) = pf1(XSim(1:6,ii));
end

for ii = 1:max(size(tSimVI))
    pF_SimVI(:,ii) = pf1(XSimVI(1:6,ii));
end

scaleSize = 1;
lineSize = 2.5;

f = figure;
f.Position = [100 100 500 250];
clf()
t = tiledlayout(1,2,'Padding','Compact');
title(t,"Foot XY Trajectory Tracking - Euler vs. VI",'Interpreter','latex','FontSize', 12);

nexttile
hold on
plot(pF_TO(1,:), pF_TO(2,:), 'LineWidth',lineSize);
plot(pF_Sim(1,:), pF_Sim(2,:),'--', 'LineWidth',lineSize);
title("Euler",'Interpreter','latex','FontSize', 12);
xlabel("Foot X (m)",'Interpreter','latex','FontSize', 12);
ylabel("Foot Y (m)",'Interpreter','latex','FontSize', 12);
legend([ "Sim. Traj.", "Ref. Traj."], 'Location','southeast')
grid on
axis equal


nexttile
hold on
plot(pF_TOVI(1,:), pF_TOVI(2,:), 'LineWidth',lineSize);
plot(pF_SimVI(1,:), pF_SimVI(2,:),'--', 'LineWidth',lineSize);
grid on
title("VI",'Interpreter','latex','FontSize', 12);
xlabel("Foot X (m)",'Interpreter','latex','FontSize', 12);
%ylabel("Y (m)")
legend([ "Sim. Traj.", "Ref. Traj."], 'Location','southeast')
axis equal
fontsize(gcf, scale=scaleSize)







