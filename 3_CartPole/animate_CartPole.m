function success = animate_CartPole(q)

N = max(size(q));
pCartVec = zeros(3,N);
pTipVec = zeros(3,N);


for k = 1:N
    pCartVec(:,k) = pCart(q(:,k));
    pTipVec(:,k) = pTip(q(:,k));
end

lCart = 0.6;
hCart = 0.4;

cartXNodes = [lCart/2 lCart/2 -lCart/2 -lCart/2] + q(1,1);
cartZNodes = [hCart/2 -hCart/2 -hCart/2 hCart/2];

figure('Color',[1 1 1])
clf()
hold on
Cart = fill(cartXNodes, cartZNodes, [0.3010 0.7450 0.9330]);
Pole = plot([pCartVec(1,1) pTipVec(1,1)], [pCartVec(3,1) pTipVec(3,1)],'-o',"Color", [0 0.4470 0.7410], "LineWidth", 5,"MarkerEdgeColor",'k',"MarkerFaceColor",'k', "MarkerSize",1.5);
Pivot = plot(pCartVec(1,1),pCartVec(3,1), '-o','LineWidth', 5, "MarkerEdgeColor",'k',"MarkerFaceColor",'k', "MarkerSize",3);

hold off
axis manual
xlim([-2.5 2.5])
ylim([-2.5 2.5])
set(gca,'XColor', 'none','YColor','none')

for k =2:N
    pause(0.01)
    Pole.XData = [pCartVec(1,k) pTipVec(1,k)];
    Pole.YData = [pCartVec(3,k) pTipVec(3,k)];
    Pivot.XData = [pCartVec(1,k)];
    Pivot.YData = [0];
    Cart.Vertices(:,1) = cartXNodes.' + pCartVec(1,k);
    drawnow
end


success = {pCartVec,pTipVec};