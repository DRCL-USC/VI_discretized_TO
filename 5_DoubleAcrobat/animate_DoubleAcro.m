function success = animate_DoubleAcro(q)

N = max(size(q));
pPinVec = zeros(3,N);
pElbowVec = zeros(3,N);
pTipVec = zeros(3,N);

for k = 1:N
    pPinVec(:,k) = pPin(q(:,k));
    pElbowVec(:,k) = pElbow(q(:,k));
    pTipVec(:,k) = pTip(q(:,k));
end

figure('Color',[1 1 1])
clf()
hold on
Link1 = plot([pPinVec(1,1) pElbowVec(1,1)], [pPinVec(3,1) pElbowVec(3,1)],"LineWidth", 5, "Color", [0.3010 0.7450 0.9330]);
Link2 = plot([pElbowVec(1,1) pTipVec(1,1)], [pElbowVec(3,1) pTipVec(3,1)],'-o',"Color", [0 0.4470 0.7410], "LineWidth", 5,"MarkerEdgeColor",'k',"MarkerFaceColor",'k', "MarkerSize",2.5);
Pivot = plot(pPinVec(1,1),pPinVec(3,1), '-o','LineWidth', 5, "MarkerEdgeColor",'k',"MarkerFaceColor",'k', "MarkerSize",5);

hold off
axis manual
xlim([-3.0 3.0])
ylim([-3.0 3.0])
set(gca,'XColor', 'none','YColor','none')

for k =2:N
    pause(0.01)
    Link1.XData = [pPinVec(1,k) pElbowVec(1,k)];
    Link1.YData = [pPinVec(3,k) pElbowVec(3,k)];
    Link2.XData = [pElbowVec(1,k) pTipVec(1,k)];
    Link2.YData = [pElbowVec(3,k) pTipVec(3,k)];
    Pivot.XData = [pPinVec(1,k)];
    Pivot.YData = [pPinVec(3,k)];
    drawnow
end


success = {pPinVec,pElbowVec,pTipVec};