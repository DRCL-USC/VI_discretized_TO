function success = animate_3Link_With_Foot(q)

N = max(size(q));
pJ1Vec = zeros(2,N);
pJ2Vec = zeros(2,N);
pJ3Vec = zeros(2,N);
pT = zeros(2,N);
pF1 = zeros(2,N);
pF2 = zeros(2,N);

for k = 1:N
    pJ1Vec(:,k) = pJ1(q(:,k));
    pJ2Vec(:,k) = pJ2(q(:,k));
    pJ3Vec(:,k) = pJ3(q(:,k));
    pF1(:,k) = pf1(q(:,k));
    pF2(:,k) = pf2(q(:,k));
    pT(:,k) = 2*q(1:2,k) - pJ1Vec(:,k);
end

figure(99)
clf()
hold on
Link2 = plot([pJ1Vec(1,1) pJ2Vec(1,1)], [pJ1Vec(2,1) pJ2Vec(2,1)],"LineWidth", 5);
Link1 = plot([pT(1,1) pJ1Vec(1,1)], [pT(2,1) pJ1Vec(2,1)],'-o',"LineWidth", 5,"MarkerEdgeColor",'k',"MarkerFaceColor",'k');
Link3 = plot([pJ2Vec(1,1) pJ3Vec(1,1)], [pJ2Vec(2,1) pJ3Vec(2,1)],'-o',"LineWidth", 5,"MarkerEdgeColor",'k',"MarkerFaceColor",'k');
Link4 = plot([pF2(1,1) pJ3Vec(1,1) pF1(1,1)], [pF2(2,1) pJ3Vec(2,1) pF1(2,1)],'-o',"LineWidth", 5,"MarkerEdgeColor",'k',"MarkerFaceColor",'k');
hold off
axis manual
xlim([-1 1])
ylim([-0.1 1.5])

for k =2:N
    pause(0.04)
    Link1.XData = [pT(1,k) pJ1Vec(1,k)];
    Link1.YData = [pT(2,k) pJ1Vec(2,k)];
    Link2.XData = [pJ1Vec(1,k) pJ2Vec(1,k)];
    Link2.YData = [pJ1Vec(2,k) pJ2Vec(2,k)];
    Link3.XData = [pJ2Vec(1,k) pJ3Vec(1,k)];
    Link3.YData = [pJ2Vec(2,k) pJ3Vec(2,k)];
    Link4.XData = [pF2(1,k) pJ3Vec(1,k) pF1(1,k)];
    Link4.YData = [pF2(2,k) pJ3Vec(2,k) pF1(2,k)];
    drawnow
end

success = {pJ1Vec,pJ2Vec, pF1, pF2, pT};