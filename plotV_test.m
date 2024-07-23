clc
clear all
close all

syms psi dpsi
assume(psi,'real');
assumeAlso(dpsi,'real');

G=[1,2];
max_psi=2*pi;

h = {-(cos(psi)^2*(sin(2*psi)/2 - 1))/2;
    (cos(psi)^2*(sin(2*psi)/2 + 1))/2;
    -sin(psi)^2*(sin(2*psi)/4 - 1/2);
    sin(psi)^2*(sin(2*psi)/4 + 1/2)};

P{1} = [-1   0
        0   2];

P{2} = [-2   0
        0   3];

P{3} = [3   0
        0   -3];

P{4} = [4   -1
        -1   -4];


Ph=0*h{1}*P{1};
for k=G
    Ph=Ph+h{k}*P{k};
    disp('eigP')
    disp(eig(P{k}))
end

V = [dpsi psi]*Ph*[dpsi;psi];
V = simplify(V)

handle=fcontour(V,[-max_psi max_psi -max_psi max_psi],'MeshDensity',100, 'LevelList', [-1 0 1 2 3 4 5]);
grid on;

H=hessian(V,[dpsi psi]);
H=simplify(H);

%Because the h functions depend only on psi, the contour sets are not
%bounded