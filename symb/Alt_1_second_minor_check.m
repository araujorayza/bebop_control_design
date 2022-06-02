clc;
clear all;
[gamma2,gamma4] = meshgrid(0:10);
z = gamma2.*gamma4 - (gamma2 - gamma4).^2;

mesh(gamma2,gamma4,z)
xlabel('\gamma_2')
ylabel('\gamma_4')
legend('\gamma_2\gamma_4 - (\gamma_2 - \gamma_4)^2')

cleanfigure;
matlab2tikz('filename','teste.tex');