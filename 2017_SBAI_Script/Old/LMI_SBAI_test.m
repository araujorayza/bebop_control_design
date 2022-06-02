%   SCRIPT TO TEST IMPROVEMENT IN DESIGN PROCEDURE 
%   PROPOSED IN SBAI 2017
%       - second step of design procedure used here 
%         is a linear optimization
% 
%   G(s)= 3*(s+1)/(s^2+s+1)
%   K(s)= Kp + Ki/s
% 
%   Example 1 of the SBAI 2017 paper

%% Config iniciais
clear all, clc; close all;

%% Tolerancia de convergencia
tol = -1e-9;

%% Main
s = tf('s');

%% Sistema
Gn = 3*(s+1)/(s^2+s+1);

%realizacao da malha aberta com o controlador
A = [0 1  0;0 0 1; 0 -1 -1 ];
B = [0 0 1]';
C = [1 0]; 
D = 0;
J = [3 0;3 3;0 3];


%% TAMANHOS

[n,m] = size(B);
nc=size(J,2);

%% Vari�veis LMI
S = sdpvar(n, n ,'sym');
Y = sdpvar(m, n ,'full');
sdpvar Ro

T11 = S*A' + A*S - Y'*B' - B*Y ;
T12 = B;
T13 = Y';

T21 = B';
T22 = -Ro*eye(m);
T23 = zeros(m);

T31 = Y;
T32 = zeros(m);
T33 = -eye(m);

T = [T11 T12 T13;T21 T22 T23;T31 T32 T33];

LMIs = [S > 0, T < 0];

sol = optimize(LMIs,Ro)
[p,d] = checkset(LMIs);


if(min(p)>tol)
    polS = double(S)
    polY = double(Y)
    K = ((inv(polS)*polY')'/J')';
    double(Ro)
else
     disp('infactivel')     
end

%% segunda parte do procedimento de projeto

sdpvar err_tol
clear T

k = sdpvar(nc,1,'full');

P=polS\eye(n);
y=polY;


Acl=A-B*(J*k)';
cons=[Acl'*P + P*Acl < 0]; 

T=[-eye(n) P*y'-J*k;
    (P*y'-J*k)' -err_tol];

cons=[cons, T<0];
cons=[cons, err_tol>=0];

sol = optimize(cons,err_tol)
[p,d] = checkset(cons);

if(min(p)>tol)
    value(err_tol)
    value(k)
else         
     disp('infactivel')     
end

%%
Kvelho = K
K = value(k)
%%

Ab = A-B*(J*K)';
Bb = B;
Cb = (J*K)'; 
Db = 0; 

MF_sol = ss(Ab,Bb,Cb,Db);

close all
% [out,time]=step(MF_sol);
% % hold on
% out_plant=step(Gn,time);
% errr=ones(size(out))-out
% plot(time,errr)
% mean(errr)

step(MF_sol);