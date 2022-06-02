NonLinQuad;

syms x y z dx dy dz dpsi d2x d2y d2z d2psi real
q = [x;y;z;psi];
dq=[dx;dy;dz;dpsi];


syms ux uy uz upsi real
u = [ux; uy; uz; upsi];

% f = [d2x == gamma(1)*ux-gamma(2)*dx;
%      d2y == gamma(3)*uy-gamma(4)*dy;
%      d2z == gamma(5)*uz-gamma(6)*dz;
%      d2psi == gamma(7)*upsi-gamma(8)*dpsi;]
f = A*[dq;q] + B*u == [d2x; d2y; d2z; d2psi; dq];

rayza=solve(f,gamma);

gamma = [simplify(rayza.gamma1);
         simplify(rayza.gamma2);
         simplify(rayza.gamma3);
         simplify(rayza.gamma4);
         simplify(rayza.gamma5);
         simplify(rayza.gamma6);
         simplify(rayza.gamma7);
         simplify(rayza.gamma8)]