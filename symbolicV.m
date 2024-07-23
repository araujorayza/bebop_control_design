clear all;
clc;
close all;

N = 8;
x = sym('x',[N 1]);
assume(x,'real');
syms psi
assumeAlso(psi,'real');
assumeAlso(x(8)==psi);

load drone_sys_data.mat

V =0;

for k=G
    V = V + x'*h(k)*P{k}*x;
end

V = simplify(V);
pretty(V);

gradV = sym('gradV',[N 1]);

for i=1:N
    gradV(i) = simplify(diff(V,x(i)));
end

H=hessian(V,x);
H=simplify(H)