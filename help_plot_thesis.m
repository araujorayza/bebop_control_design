figure(c);
hold on;
% plot(STATE(:,PosX),STATE(:,PosY),'*');
plot(ERROR(:,PosX)+ DES_STATE(:,PosX),ERROR(:,PosY) + DES_STATE(:,PosY),'rd');
plot(DES_STATE(:,PosX),DES_STATE(:,PosY),'k-');
hold off
grid on
axis equal
% title('Trajectory Tracking')
fprintf('%d Trajectory\n',c)
legend('Quadrotor','Ref')
c=c+1;
%%%%%%%%%%%%%%%%%%%%%%%
figure(c);
plot(t,ERROR,'d');
grid on
% title('Error')
fprintf('%d Error\n',c)
legend('$e_1$',...
'$e_2$',...
'$e_3$',...
'$e_4$',...
'$e_5$',...
'$e_6$',...
'$e_7$',...
'$e_8$',...
    'Interpreter','latex');
c=c+1;
xlim([0 15])
newcolors = {'#F00','#fb7607','#ffd000','#0B0','#00F','#00b3ff','#A0F','#fa69c7'};
colororder(newcolors)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % psi = 0:0.1:2*pi
% psi = ERROR(:,PosYaw)+ DES_STATE(:,PosYaw);
% 
% h_1=-(cos(psi).^2.*(sin(2.*psi)./2 - 1))./2;
% h_2=(cos(psi).^2.*(sin(2.*psi)./2 + 1))./2;
% h_3= -sin(psi).^2.*(sin(2.*psi)./4 - 1/2);
% h_4= sin(psi).^2.*(sin(2.*psi)./4 + 1/2);
% 
% figure(c);
% hold on;
% plot(psi,h_1);
% plot(psi,h_2);
% plot(psi,h_3);
% plot(psi,h_4);
% hold off
% grid on
% axis equal
% % title('Trajectory Tracking')
% fprintf('%d Hs\n',c)
% c=c+1;
%%%%%%%%%%%%%%%%%
psi = ERROR(:,PosYaw)+ DES_STATE(:,PosYaw);
% dh/dpsi
dh1=             -(cos(psi).*(cos(3*psi) + 2*sin(psi)))/2;
dh2=              (cos(psi).*(cos(3*psi) - 2*sin(psi)))/2;
dh3= sin(2*psi)/2 - (5*cos(psi).^2)/2 + 2*cos(psi).^4 + 1/2;
dh4= sin(2*psi)/2 + (5*cos(psi).^2)/2 - 2*cos(psi).^4 - 1/2;

% dpsi/dt
dpsi=ERROR(:,VelYaw)+ DES_STATE(:,VelYaw);

% dh/dt = dh/dpsi* dpsi/dt
dh1_dt=dh1.*dpsi;
dh2_dt=dh2.*dpsi;
dh3_dt=dh3.*dpsi;
dh4_dt=dh4.*dpsi;

figure(c);
hold on;
plot(psi,dh1_dt,'d-');
plot(psi,dh2_dt,'d-');
plot(psi,dh3_dt,'d-');
plot(psi,dh4_dt,'d-');
hold off
grid on
axis equal
legend('$\dot{h}_1$',...
'$\dot{h}_2$',...
'$\dot{h}_3$',...
'$\dot{h}_4$',...
'Interpreter','latex');

% title('Trajectory Tracking')
fprintf('%d dh/dt\n',c)
c=c+1;
