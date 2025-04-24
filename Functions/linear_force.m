function Fx = linear_force(u)
    C_lambda = u(1);
    lambda = u(2);
    Fx = C_lambda * lambda;
end
