function success = animate_G1_2D(q)

N = max(size(q));
pJ1Vec = zeros(3,N);
pJ2Vec = zeros(3,N);
pJ3Vec = zeros(3,N);
pT = zeros(3,N);
pF1 = zeros(3,N);
pF2 = zeros(3,N);

for k = 1:N
    pJ1Vec(:,k) = pHipPitch(q(:,k));
    pJ2Vec(:,k) = pKnee(q(:,k));
    pJ3Vec(:,k) = pAnklePitch(q(:,k));
    pF1(:,k) = pFootF(q(:,k));
    pF2(:,k) = pFootR(q(:,k));
    pT(:,k) = pHead(q(:,k));
end

figure(99)
clf()
hold on
Link2 = plot([pJ1Vec(1,1) pJ2Vec(1,1)], [pJ1Vec(3,1) pJ2Vec(3,1)],"LineWidth", 5);
Link1 = plot([pT(1,1) pJ1Vec(1,1)], [pT(3,1) pJ1Vec(3,1)],'-o',"LineWidth", 5,"MarkerEdgeColor",'k',"MarkerFaceColor",'k');
Link3 = plot([pJ2Vec(1,1) pJ3Vec(1,1)], [pJ2Vec(3,1) pJ3Vec(3,1)],'-o',"LineWidth", 5,"MarkerEdgeColor",'k',"MarkerFaceColor",'k');
Link4 = plot([pF2(1,1) pJ3Vec(1,1) pF1(1,1)], [pF2(3,1) pJ3Vec(3,1) pF1(3,1)],'-o',"LineWidth", 5,"MarkerEdgeColor",'k',"MarkerFaceColor",'k');
hold off
axis manual
xlim([-1 2.5])
ylim([-1.0 2.5])

for k =2:N
    pause(0.04)
    Link1.XData = [pT(1,k) pJ1Vec(1,k)];
    Link1.YData = [pT(3,k) pJ1Vec(3,k)];
    Link2.XData = [pJ1Vec(1,k) pJ2Vec(1,k)];
    Link2.YData = [pJ1Vec(3,k) pJ2Vec(3,k)];
    Link3.XData = [pJ2Vec(1,k) pJ3Vec(1,k)];
    Link3.YData = [pJ2Vec(3,k) pJ3Vec(3,k)];
    Link4.XData = [pF2(1,k) pJ3Vec(1,k) pF1(1,k)];
    Link4.YData = [pF2(3,k) pJ3Vec(3,k) pF1(3,k)];
    drawnow
end

success = {pJ1Vec,pJ2Vec, pF1, pF2, pT};