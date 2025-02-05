clc;
clear;
close all;
load drone_sys_data.mat

h=cell(4,1);

h{1} = @(psi) -(cos(psi)^2*(sin(2*psi)/2 - 1))/2;
h{2} = @(psi) (cos(psi)^2*(sin(2*psi)/2 + 1))/2;
h{3} = @(psi) -sin(psi)^2*(sin(2*psi)/4 - 1/2);
h{4} = @(psi)  sin(psi)^2*(sin(2*psi)/4 + 1/2);

V = @(x) sum(arrayfun(@(k) x'*h{k}(x(8))*P{k}*x,[1,2,3,4]));

ub=[5;
    5;
    5;
    5;
    5;
    5;
    5;
    pi/3];
lb=-ub;

options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
x0=[1.0000	-1.0000	-0.0000	-0.0002	0.3344	-0.4095	-0.0000	0.0006]';
x0=ub;
[x,fval,exitflag,output,lambda,grad,hessian] = ...
    fmincon(V,x0,[],[],[],[],lb,ub,@(x) borderofZ(x,ub,lb),options)

function [c,ceq] = borderofZ(x,ub,lb)
    c=[];
%     ceq= prod(arrayfun(@(num) x(num)-lb(num),1:length(lb)))*...
%                 prod(arrayfun(@(num) x(num)-ub(num),1:length(ub)));
    ceq = (x(1)-lb(1))*(x(2)-lb(2))*(x(3)-lb(3))*(x(4)-lb(4))*(x(5)-lb(5))*(x(6)-lb(6))*(x(7)-lb(7))*(x(8)-lb(8))* ...
        (x(1)-ub(1))*(x(2)-ub(2))*(x(3)-ub(3))*(x(4)-ub(4))*(x(5)-ub(5))*(x(6)-ub(6))*(x(7)-ub(7))*(x(8)-ub(8));
end