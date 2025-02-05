% Calculate phi_i such that |h_i| <= phi_i

clear;
clc;
close all;

main;


h=simplify(h);
del_h = diff(h,'psi');
del_h = simplify(del_h);

% dot_h = sum of partial h derivative with respect to all components of x.
% They are all zero, except for the x_8 one.
% So dot_h = partial_h with respect to x_8 times dot_x_8
% dot_x_8 is given by a physical limitation of the quadrotor

dot_h=del_h*MaxRotSpd*(pi/180); 

dot_h_curve=cell(length(dot_h),1);
figure(1);
hold on;
for i=1:length(dot_h)
    dot_h_curve{i} = fplot(dot_h(i),[-pi/3,pi/3], 'DisplayName',strcat('h_',num2str(i)));
end
grid on
legend;

phi=zeros(length(dot_h),1);
for i=1:length(dot_h)
    phi(i) = max(dot_h_curve{i}.YData);
end

