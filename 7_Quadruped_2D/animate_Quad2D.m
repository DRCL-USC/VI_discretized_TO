function success = animate_Quad2D(q)

N = max(size(q));
pHipFVec = zeros(3,N);
pKneeFVec = zeros(3,N);
pHipRVec = zeros(3,N);
pKneeRVec = zeros(3,N);
pFootFVec = zeros(3,N);
pFootRVec = zeros(3,N);

for k = 1:N
    pHipFVec(:,k) = pHipF(q(:,k));
    pKneeFVec(:,k) = pKneeF(q(:,k));
    pHipRVec(:,k) = pHipR(q(:,k));
    pKneeRVec(:,k) = pKneeR(q(:,k));
    pFootFVec(:,k) = pFootF(q(:,k));
    pFootRVec(:,k) = pFootR(q(:,k));

end

figure(99)
clf()
hold on
Link1 = plot([pHipFVec(1,1) pHipRVec(1,1)], [pHipFVec(3,1) pHipRVec(3,1)],"LineWidth", 4);
Link2 = plot([pHipFVec(1,1) pKneeFVec(1,1)], [pHipFVec(3,1) pKneeFVec(3,1)],'-o',"LineWidth", 4,"MarkerEdgeColor",'k',"MarkerFaceColor",'k');
Link3 = plot([pKneeFVec(1,1) pFootFVec(1,1)], [pKneeFVec(3,1) pFootFVec(3,1)],'-o',"LineWidth", 4,"MarkerEdgeColor",'k',"MarkerFaceColor",'k');
Link4 = plot([pHipRVec(1,1) pKneeRVec(1,1)], [pHipRVec(3,1) pKneeRVec(3,1)],'-o',"LineWidth", 4,"MarkerEdgeColor",'k',"MarkerFaceColor",'k');
Link5 = plot([pKneeRVec(1,1) pFootRVec(1,1)], [pKneeRVec(3,1) pFootRVec(3,1)],'-o',"LineWidth", 4,"MarkerEdgeColor",'k',"MarkerFaceColor",'k');

hold off
axis manual
xlim([-3.0 1.0])
ylim([-3.0 1.0])

for k =2:N
    pause(0.04)
    Link1.XData = [pHipFVec(1,k) pHipRVec(1,k)];
    Link1.YData = [pHipFVec(3,k) pHipRVec(3,k)];
    Link2.XData = [pHipFVec(1,k) pKneeFVec(1,k)];
    Link2.YData = [pHipFVec(3,k) pKneeFVec(3,k)];
    Link3.XData = [pKneeFVec(1,k) pFootFVec(1,k)];
    Link3.YData = [pKneeFVec(3,k) pFootFVec(3,k)];
    Link4.XData = [pHipRVec(1,k) pKneeRVec(1,k)];
    Link4.YData = [pHipRVec(3,k) pKneeRVec(3,k)];
    Link5.XData = [pKneeRVec(1,k) pFootRVec(1,k)];
    Link5.YData = [pKneeRVec(3,k) pFootRVec(3,k)];
    drawnow
end

success = {pHipFVec,pKneeFVec, pFootFVec, pHipRVec, pKneeRVec, pFootRVec};