% arquivo para comparar o resultado da 
% otimização não linear com a LMI que 
% eu escrevi para substituir.
% RESULTADO: os dois processos são equivalentes
% porém a LMI é mais simples
%% Config iniciais
clear all, clc;% close all;

%% Tolerancia de convergï¿½ncia
tol = -1e-9;

%% Main
s = tf('s');

%% Sistema 1

% A = [0 1 ; 0 -1 ];
% B = [0 1]';
% D = 0;
% J = [1 0 ;0 1 ];

% Gn = 1/(s+1);


%% Sistema 2

A = [0 1  0;0 0 1; 0 -1 -1 ];
B = [0 0 1]';
C = [1 0]; 
D = 0;
J = 3*[1 0;1 1; 0 1];

Gn = 3*(s+1)/(s^2+s+1);


%% Sistema 3


[n,m] = size(B);

%% Variï¿½veis LMI
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

T = [T11 T12 T13 ; T21 T22 T23 ; T31 T32 T33];

LMIs = [S > 0, T < 0 ];

sol = solvesdp(LMIs,Ro)
[p,d] = checkset(LMIs)
clc
if(min(p)>tol)
    polS = double(S);
    polY = double(Y);
    %K = ((inv(Ss)*Ys')'/J')';
    K = ((inv(polS)*polY')'/J')';
     double(Ro)
else
     disp('infactivel')     
end


%%

c = @(x) [real(eig(0.05*eye(n)+A-B*(J*[x(1);x(2)])'))]; % restriï¿½ï¿½es inequaï¿½ï¿½es 
ceq = @(x)[];

nonlinfcn = @(x)deal(c(x),ceq(x));
obj = @(x) mse(J*[x(1);x(2)],inv(polS)*polY'); % obj: erro mï¿½dio quadrï¿½tico ser mï¿½nimo
opts = optimoptions(@fmincon,'Algorithm','sqp');
z1 = fmincon(obj,[0;0],[],[],[],[],[],[],nonlinfcn,opts);
[cout,ceqout] = nonlinfcn(z1) % verifica se satisfez as inequaï¿½ï¿½es 



%%

sdpvar err_tol
clear T
nc=size(J,2);

k = sdpvar(nc,1,'full');

P=polS\eye(n);
y=polY;


obj=(P*y'-J*k)'*(P*y'-J*k);


Acl=A-B*(J*k)';
cons=[Acl'*P+P*Acl<0];

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
K = z1
Klin=value(k)



%%

Ab = A-B*(J*K)';
Bb = B;
Cb = (J*K)'; 
Db = 0; 


MF_sol = Cb*inv(s*eye(n)-Ab)*Bb+Db 



%% SimulaÃ§Ã£o com o K calculado por LMIs

Ab = A-B*(J*Klin)';
Bb = B;
Cb = (J*Klin)'; 
Db = 0; 


MF_lmi = tf(ss(Ab,Bb,Cb,Db));

%%
Cpid = K(2)+K(1)/s;
MF_real = feedback(Cpid*Gn,1)


% impulse(MF_sol)
% hold on 
% impulse(MF_real)
% 
% figure 

step(MF_sol)
hold on 
step(MF_real)
step(MF_lmi)

%% 
%clc; close all;

%% Display
value(Ro)
value(err_tol)

