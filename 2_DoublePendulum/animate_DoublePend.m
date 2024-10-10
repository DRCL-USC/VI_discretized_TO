function success = animate_DoublePend(q)

N = max(size(q));
pElbowVec = zeros(3,N);
pTipVec = zeros(3,N);

for k = 1:N
    pElbowVec(:,k) = pElbow(q(:,k));
    pTipVec(:,k) = pTip(q(:,k));
end

figure('Color',[1 1 1])
clf()
hold on
Link1 = plot([0 pElbowVec(1,1)], [0 pElbowVec(3,1)],"LineWidth", 5, "Color", [0.3010 0.7450 0.9330]);
Link2 = plot([pElbowVec(1,1) pTipVec(1,1)], [pElbowVec(3,1) pTipVec(3,1)],'-o',"Color", [0 0.4470 0.7410], "LineWidth", 5,"MarkerEdgeColor",'k',"MarkerFaceColor",'k', "MarkerSize",2.5);
Pivot = plot(0,0, '-o','LineWidth', 5, "MarkerEdgeColor",'k',"MarkerFaceColor",'k', "MarkerSize",5);

hold off
axis manual
xlim([-2.5 2.5])
ylim([-2.5 2.5])
set(gca,'XColor', 'none','YColor','none')

for k =2:N
    pause(0.01)
    Link1.XData = [0 pElbowVec(1,k)];
    Link1.YData = [0 pElbowVec(3,k)];
    Link2.XData = [pElbowVec(1,k) pTipVec(1,k)];
    Link2.YData = [pElbowVec(3,k) pTipVec(3,k)];
    Pivot.XData = [0];
    Pivot.YData = [0];
    drawnow
end


success = {pElbowVec,pTipVec};