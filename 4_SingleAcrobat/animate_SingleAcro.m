function success = animate_SingleAcro(q)

N = max(size(q));
pTipVec = zeros(3,N);
pPinVec = zeros(3,N);


for k = 1:N
    pPinVec(:,k) = pPin(q(:,k));
    pTipVec(:,k) = pTip(q(:,k));
end

figure(99)
clf()
hold on
linkColor = [0 0.4470 0.7410];
Link1 = plot([pPinVec(1,1), pTipVec(1,1)], [pPinVec(3,1), pTipVec(3,1)],'-o',"Color", linkColor, "LineWidth", 5,"MarkerEdgeColor",'k',"MarkerFaceColor",'k', "MarkerSize",1);
Pivot = plot(pPinVec(1,1),pPinVec(3,1), '-o','LineWidth', 5, "MarkerEdgeColor",'k',"MarkerFaceColor",'k', "MarkerSize",4);

hold off
axis manual
xlim([-3 3])
ylim([-3 3])

for k =2:N
    pause(0.02)
    Link1.XData = [pPinVec(1,k), pTipVec(1,k)];
    Link1.YData = [pPinVec(3,k), pTipVec(3,k)];

    Pivot.XData = [pPinVec(1,k)];
    Pivot.YData = [pPinVec(3,k)];
    drawnow
end

success = {pTipVec};