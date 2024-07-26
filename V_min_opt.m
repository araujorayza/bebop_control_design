clc;
clear;
close all;
load drone_sys_data.mat

h=cell(4,1);

h{1} = @(psi) -(cos(psi)^2*(sin(2*psi)/2 - 1))/2;
h{2} = @(psi) (cos(psi)^2*(sin(2*psi)/2 + 1))/2;
h{3} = @(psi) -sin(psi)^2*(sin(2*psi)/4 - 1/2);
h{4} = @(psi)  sin(psi)^2*(sin(2*psi)/4 + 1/2);

V = @(x) sum(arrayfun(@(k) x'*h{k}(x(8))*P{k}*x,G));

ub=[1;
    1;
    1;
    1;
    1;
    1;
    1;
    pi/4];
lb=-ub;

function [c,ceq] = borderofZ(x,ub,lb)
    c=[];
    ceq= prod(arrayfun(@(num) x(num)-lb(num),1:length(lb)))*...
                prod(arrayfun(@(num) x(num)-ub(num),1:length(ub)));
end

options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
x0=lb;
[x,fval,exitflag,output,lambda,grad,hessian] = ...
    fmincon(V,x0,[],[],[],[],lb,ub,@(x) borderofZ(x,ub,lb),options)
