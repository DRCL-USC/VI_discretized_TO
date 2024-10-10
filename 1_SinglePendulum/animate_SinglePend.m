function success = animate_SinglePend(q)

N = max(size(q));
pTipVec = zeros(3,N);


for k = 1:N
    pTipVec(:,k) = pTip(q(:,k));
end

figure(99)
clf()
hold on
linkColor = [0 0.4470 0.7410];
Link1 = plot([0, pTipVec(1,1)], [0, pTipVec(3,1)],'-o',"Color", linkColor, "LineWidth", 5,"MarkerEdgeColor",'k',"MarkerFaceColor",'k', "MarkerSize",1);
Pivot = plot(0,0, '-o','LineWidth', 5, "MarkerEdgeColor",'k',"MarkerFaceColor",'k', "MarkerSize",4);

hold off
axis manual
xlim([-1.5 1.5])
ylim([-1.5 1.5])

for k =2:N
    pause(0.02)
    Link1.XData = [0, pTipVec(1,k)];
    Link1.YData = [0, pTipVec(3,k)];

    Pivot.XData = [0];
    Pivot.YData = [0];
    drawnow
end

success = {pTipVec};