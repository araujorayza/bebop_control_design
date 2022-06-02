% GENERAL SCRIPT OF DESIGN PROCEDURE PROPOSED 
%               IN SBAI 2017
%       - First step of design procedure is an 
%         optimization of the Hinf norm of the 
%         closed-loop system composed by a PID
%         controller in series the plant
%       - Second step is a linear optimization
%         of the error tolerance when retrieving
%         the controller's gains
% Last edited by: Rayza Araujo
% 02/08/2018


%% Clean workspace and screen, set up environment
clear all;
clc; 
close all;
% hold on
% ROMIN = 10;

s = tf('s');
%% Solver convergence tolerance
tol = -1e-9;

%% SYSTEM
G = 3*(s+1)/(s^2+s+1);

%realizacao da malha aberta com o controlador
A = [0  1  0;
     0  0  1; 
     0 -1 -1];
B = [0 0 1]';

J = [3 0;3 3;0 3];


%% SIZES
[n,m] = size(B); %number of states and inputs
[nz,nc]=size(J); %size of J and controller order

%% LMI DEFINITION - FIRST STEP
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

% LMIs = [S > 0, T < 0, Ro >= ROMIN];
LMIs = [S > 0, T < 0];
sol = optimize(LMIs,Ro)

if sol.problem == 0
    polS = double(S)
    polY = double(Y)
else
 disp('*****************ERROR*********************');
 sol.info
 yalmiperror(sol.problem)
end

%% LMI DEFINITION - SECOND DESIGN STEP
if(nz==nc)  %if J is square, there exists an inverse
        K = ((inv(polS)*polY')'/J')';
else        %if J isn't square, carry second optmization
    disp('J is not square')
    sdpvar err_tol
    clear T

    k = sdpvar(nc,1,'full');

    P=polS\eye(n);
    y=polY;


    Acl=A-B*(J*k)';
    cons=[Acl'*P+P*Acl<0];

    T=[-eye(n) P*y'-J*k;
        (P*y'-J*k)' -err_tol];

    cons=[cons, T<0];
    cons=[cons, err_tol>=0];

    sol = optimize(cons,err_tol)
    if sol.problem == 0
        disp('Second optimization DONE')
        K = value(k)
    else
        disp('*****************ERROR** *******************');
        sol.info
        yalmiperror(sol.problem)
    end
   
end

%% CLOSED LOOP SYSTEM AND SIMULATION

Ab = A-B*(J*K)';
Bb = B;
Cb = (J*K)'; 
Db = 0; 

MF = ss(Ab,Bb,Cb,Db);
value(Ro)

impulse(MF,G)
legend('Controlled','Open loop');