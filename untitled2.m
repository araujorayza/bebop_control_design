clc;
clear all;
close all;

syms x1 x2
h1=sin(x1)^2;
h2=cos(x1)^2;

P1=[-1 0;
    0 1];
P2=[1 0;
    0 1];

V=[x1 x2]*(h1*P1+h2*P2)*[x1; x2];

figure;
fsurf(V,[-pi pi -pi pi])
figure;
fcontour(V,[-pi pi -pi pi])