function I = inertiaMat(MoI, PoI)
%Constructs an Inertia Matrix given Moments of Inertia, Products of Inertia

I = diag(MoI);
I(1,2) = PoI(1); I(1,3) = PoI(2); I(2,3) = PoI(3);
I(2,1) = PoI(1); I(3,1) = PoI(2); I(3,2) = PoI(3);
end