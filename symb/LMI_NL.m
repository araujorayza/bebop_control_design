Definitions;
clear small_k;

fprintf('Nonlinear error model dx = Ax + Bu\n with x = [de e]^T\n');


syms small_k real

A=[-N*R' -small_k*eye(n);
  eye(n) zeros(n)];

B=[eye(n);zeros(n)];

C=[zeros(n) eye(n)];

K = sym('K',[n n]);
assume(K,'real')


LMI=A+B*K*C

LMI = (LMI+LMI')/2;

i=8;

LMI(1:i,1:i)

de = det(LMI(1:i,1:i))
simplify(de)