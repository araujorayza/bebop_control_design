Definitions;

ModelType = 1
small_k = 1;

[A,B,h] = ErrorModeling(ModelType,gamma);
[Ah,Bh] = Defuzzy(A,B,h,ModelType);

Rset = 1:length(h);

dh = simplify(diff(h,psi));

x = sym('x',[8 1]);
P = sym('P',[8 8 3]); 
assumeAlso(x, 'real');
for i = 1:size(P,3)
    assumeAlso(P(:,:,i) == P(:,:,i)');
end
assumeAlso(x(8) == psi);
% Set estimation
G = 1;
V =  sum(arrayfun(@(k) x'*h(k)*P(:,:,k)*x,G));
V = simplify(V);
hdot = @(k) sum(arrayfun(@(j) [0, 0, 0, 0, 0, 0, 0, dh(k)]*h(j)*A{j}*x,Rset));
Dset = sum(arrayfun(@(k) x'*hdot(k)*P(:,:,k)*x,G));
Dset = simplify(Dset);

V/(sin(x(8))^2*(2*abs(gamma(2) - gamma(4)) + gamma(2)*sin(2*x(8)) - gamma(4)*sin(2*x(8)))) - Dset/(x(4)*(gamma(2)*cos(2*x(8)) - gamma(4)*cos(2*x(8)) - gamma(2)*(2*cos(2*x(8))^2 - 1) + gamma(4)*(2*cos(2*x(8))^2 - 1) + 2*sin(2*x(8))*abs(gamma(2) - gamma(4))))
V/(sin(x(8))^2*(2*abs(gamma(2) - gamma(4)) + gamma(2)*sin(2*x(8)) - gamma(4)*sin(2*x(8))))
Dset/(x(4)*(gamma(2)*cos(2*x(8)) - gamma(4)*cos(2*x(8)) - gamma(2)*(2*cos(2*x(8))^2 - 1) + gamma(4)*(2*cos(2*x(8))^2 - 1) + 2*sin(2*x(8))*abs(gamma(2) - gamma(4))))

