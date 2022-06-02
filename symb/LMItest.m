Definitions;
fprintf('Nonlinear error model dx = Ax + Bu\n with x = [de e]^T\n');


ModelType = 2
small_k = sym('small_k');
assume(small_k, 'real');



%assumeAlso(gamma(2) > gamma(4))
[A,B,h] = ErrorModeling(ModelType,gamma);
[Ah,Bh] = Defuzzy(A,B,h,ModelType);



C=[zeros(n) eye(n)];
K = sym('K',[n n]);
assume(K,'real');

if(ModelType==2)
    for i=1:length(A)
        for j=1:length(A)
        LMI{i,j}=A{i,j}+B{i}*K*C;
        LMI{i,j} = (LMI{i,j}+LMI{i,j}')/2;
        end
    end
else
    for i=1:length(A)
        LMI{i}=A{i}+B{i}*K*C;
        LMI{i} = (LMI{i}+LMI{i}')/2;
    end
end



j=1;
i=1;

LMI{j}(1:i,1:i)

de = det(LMI{j}(1:i,1:i))
simplify(de)
