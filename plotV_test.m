clc
clear all

syms psi dpsi

h = {-(cos(psi)^2*(sin(2*psi)/2 - 1))/2
    (cos(psi)^2*(sin(2*psi)/2 + 1))/2
    -sin(psi)^2*(sin(2*psi)/4 - 1/2)
    sin(psi)^2*(sin(2*psi)/4 + 1/2)}


P{1} = [2.0344   -0.6261
    -0.6261   0.5365]

P{3} = [2.0437   -0.6230
    -0.6230   0.5400]

V = [dpsi psi]*(h{1}*P{1}+h{3}*P{3})*[dpsi;psi]
V = simplify(V)

fcontour(V,[-2*pi 2*pi -2*pi 2*pi])

%Because the h functions depend only on psi, the contour sets are not
%bounded