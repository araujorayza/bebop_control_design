function [out,outeq] = borderofZ(x)
%BORDEROFZ Summary of this function goes here
%   Z = { x in R^n : |x1| <= x1_max, ... |xn| <= xn_max}
    x_max = 1.5*ones(8,1);
    out = 1;
    outeq = [];
    for i =1:length(x)
        if abs(x(i)) == x_max(i)
            out = 0;
            break;
        end
    end
end

