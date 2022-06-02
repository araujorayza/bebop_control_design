clear all
clc
close all
load journal

InitSTATE=0*rand([8,1]); %[0;0;0;0;0;0;0;0];
simStep = 0.1;
Tfinal  = 40;

Modeltype = 4;
%Modelos:   1 = metodo nao linearidades por setor padrao
%           2 = modelo de dois indices
%           3 = 10 modelos locais [hi*hj == hj*hi]
%           4 = 8 modelos locais [A_{11} = A_{44}, etc]

t=0:simStep:Tfinal;

[A,B,h,C] = ErrorModeling(Modeltype,gamma)
K = ICUAS_Teo1(Modeltype,A,B,0.95,1)

%% Setting some vars
SimStruct.controller.type = ControlType;
SimStruct.controller.model.type = Modeltype;
SimStruct.controller.model.A = A;
SimStruct.controller.model.B = B;
SimStruct.controller.model.C = C;
SimStruct.controller.model.h = h;
SimStruct.controller.gain    = K;

[q_d,dq_d,ddq_d]=CalcDesTrajectory(TRAJECTORY,t);
DES_STATE = [dq_d;q_d]';

InitERROR = InitSTATE - DES_STATE(1,:)';
options = odeset('RelTol',1e-9,'AbsTol',1e-9);
%% Simulation
[t,STATE] = ode45(@(t,y) DRONE_SANTANA(t,y,SimStruct),t,InitSTATE,options);
[t,ERROR] = ode45(@(t,y) ERROR_Fuzzy(t,y,SimStruct),t,InitERROR,options);
 
%% Plots
%%%%%%%%%%%%%%%%%TRAJECTORY PLOT%%%%%%%%%%%%%%%%%%%%%%%
c=1;
figure(c);

hold on;
plot(STATE(:,PosX),STATE(:,PosY),'*');
plot(ERROR(:,PosX)+ DES_STATE(:,PosX),ERROR(:,PosY) + DES_STATE(:,PosY),'rd');
plot(DES_STATE(:,PosX),DES_STATE(:,PosY),'k*');
hold off
grid on
axis equal
title('Trajectory')
fprintf('%d Trajectory\n',c)
legend('NonLinSys','FuzzyErrorSys','Ref')
c=c+1;

%%%%%%%%%%%%%%%%%CONTROL SIGNAL PLOT%%%%%%%%%%%%%%%%%%%%%%%
figure(c);
U = zeros(4,length(t));
V = U;

for i = 1:length(t)
    [~,U(:,i)] = DRONE_SANTANA(t(i),STATE(i,:)',SimStruct);

end 
plot(t',U,'d');
grid on
title('Control Signal')
fprintf('%d Control Signal\n',c)
legend('U_x','U_y','U_z','U_y_a_w');
c=c+1;


%%%%%%%%%%%%%%%%%COMPONENT PLOT%%%%%%%%%%%%%%%%%%%%%%%
figure(c);
hold on
plot(t,STATE(:,PosX),'*');
plot(t,ERROR(:,PosX)+ DES_STATE(:,PosX),'rd');
plot(t,DES_STATE(:,PosX),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('x(t)')
fprintf('%d x(t)\n',c)
c=c+1;

figure(c);
hold on
plot(t,STATE(:,PosY),'*');
plot(t,ERROR(:,PosY)+ DES_STATE(:,PosY),'rd');
plot(t,DES_STATE(:,PosY),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('y(t)')
fprintf('%d y(t)\n',c)
c=c+1;

figure(c);
hold on
plot(t,STATE(:,PosZ),'*');
plot(t,ERROR(:,PosZ)+ DES_STATE(:,PosZ),'rd');
plot(t,DES_STATE(:,PosZ),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('z(t)')
fprintf('%d z(t)\n',c)
c=c+1;

figure(c);
hold on
plot(t,STATE(:,PosYaw),'*');
plot(t,ERROR(:,PosYaw)+ DES_STATE(:,PosYaw),'rd');
plot(t,DES_STATE(:,PosYaw),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('yaw(t)')
fprintf('%d yaw(t)\n',c)
c=c+1;

%%%%%%%%%%%%%%%%%ERROR PLOT%%%%%%%%%%%%%%%%%%%%%%%
figure(c);
plot(t,ERROR,'d');
grid on
title('Error')
fprintf('%d Error\n',c)
c=c+1;


%%%%%%%%%%%%%%%%%COMPONENT  VELOCITY PLOT%%%%%%%%%%%%%%%%%%%%%%%
figure(c);
hold on
plot(t,STATE(:,VelX),'*');
plot(t,ERROR(:,VelX)+ DES_STATE(:,VelX),'rd');
plot(t,DES_STATE(:,VelX),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('Velx(t)')
fprintf('%d Velx(t)\n',c)
c=c+1;

figure(c);
hold on
plot(t,STATE(:,VelY),'*');
plot(t,ERROR(:,VelY)+ DES_STATE(:,VelY),'rd');
plot(t,DES_STATE(:,VelY),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('Vely(t)')
fprintf('%d Vely(t)\n',c)
c=c+1;

figure(c);
hold on
plot(t,STATE(:,VelZ),'*');
plot(t,ERROR(:,VelZ)+ DES_STATE(:,VelZ),'rd');
plot(t,DES_STATE(:,VelZ),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('Velz(t)')
fprintf('%d Velz(t)\n',c)
c=c+1;

figure(c);
hold on
plot(t,STATE(:,VelYaw),'*');
plot(t,ERROR(:,VelYaw)+ DES_STATE(:,VelYaw),'rd');
plot(t,DES_STATE(:,VelYaw),'k*');
hold off
grid on
legend('NonLinSys','FuzzyErrorSys','Ref')
title('Velyaw(t)')
fprintf('%d Velyaw(t)\n',c)
c=c+1;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function K = ICUAS_Teo1(Modeltype,A,B,phi,Mu)
%LMIs implemeted based on Theorem 1 from paper "------" ICUAS 2020
theta1=0.06; theta2=1;
[nx,nu]=size(B{1});
r = length(A); 
%sdpvar Mu
W = sdpvar(nx,nx,'full');
Q = cell(1,r);
Y = cell(r);
LMIs = [];
%LMIs = [LMIs, Mu>=0];
for i=1:r
    Q{i}= sdpvar(nx,nx,'symmetric');
    Y{i}= sdpvar(nu,nx,'full');
    LMIs = [LMIs, Q{i}>=0];
end
if(Modeltype == 3 || Modeltype == 4)
    warning('Local Models not based on nonlinearity, check how the matrix G is made')
end
G=PdotConvex(log2(r))
eta = length(G);
Qbar = cell(1,eta);
for l=1:eta
    Qbar{l} = 0;
    for i=1:r
        Qbar{l} = Qbar{l} + phi*G(i,l)*Q{i};
    end
end

if(Modeltype == 2) %model with two indices
    B=B{1,1};
    for i = 1:r
        for j = 1:r
            for l = 1:eta
                a11 = Qbar{l} - A{i,j}*W - ...
                      W'*A{i,j}' + B*Y{i} + Y{i}'*B';
                a21 = Q{i} + W' - Mu*(A{i,j}*W - B*Y{i});
                a22 = Mu*(W+W');
                Upsilon{i,j,l} = [a11, a21';
                    a21, a22];
            end
        end
    end
else
    B=B{1,1};
    for i = 1:r
        for l = 1:eta
            a11 = Qbar{l} - A{i}*W - ...
                W'*A{i}' + B*Y{i} + Y{i}'*B';
            a21 = Q{i} + W' - Mu*(A{i}*W - B*Y{i});
            a22 = Mu*(W+W');
            Upsilon{i,l} = [a11, a21';
                a21, a22];
        end
    end
end


% Criando as LMIs
for i=1:r
    LMIs = [LMIs, Q{i} >= theta2*eye(nx)];
    SizeConstr = [theta1*eye(nu), Y{i};
                    Y{i}',    eye(nx)];
    LMIs = [LMIs, SizeConstr>=0];
    for j=1:r
        if(i==j)
            continue;
        end
        for l=1:eta
            if(Modeltype == 2) %model with two indices
                LMIs = [LMIs, Upsilon{i,i,l}<=0];
                LMIs = [LMIs, 2/(r-1)*Upsilon{i,i,l} + ...
                    Upsilon{i,j,l} + Upsilon{j,i,l}<=0];
            else
                LMIs = [LMIs, Upsilon{i,l}<=0];
            end
            
        end
    end
end

% Configurando o Solver.
opts=sdpsettings;
% opts.solver='lmilab';
opts.verbose=0;
% Resolvendo as LMIs
sol = solvesdp(LMIs,[],opts);
che=min(checkset(LMIs));
if che > 0
    % Encontra o valor num�rico das matrizes
    W=double(W);
    K = cell(1,r);
    for i=1:r
        K{i} = double(Y{i})/W;
    end
else
    error('LMIs infact�veis')
    Q=[]; W=[]; K=[];
end
end

function G=PdotConvex(p)

% Local models
ri=2^p;

% Total of columns possibilities
m2=2^ri;

% Number of columns of Total matrix
co=factorial(2^p)/(factorial(2^p/2)^2);


th=0:1:m2-1;

Vaux1=cell(1,m2);

for i=1:length(th)
    Vaux1{1,i}=de2bi([th(i)],ri);
end

Vaux2=zeros(ri,m2);
for j=1:m2
    for i=1:ri
        Vaux2(i,j)=(-1)^Vaux1{1,j}(i);
    end
end

% Total matrix
G=[];

for j=1:m2
        if sum(Vaux2(:,j))==0
            G=[G, Vaux2(:,j)];
       end
end
end