function Fx = nonlinear_force(u)
    lambda = u(1);   
    FN = u(2);  
    mu1 = u(3);
    mu2 = u(4);     
    mu3 = u(5);      

    mu_lambda = sign(lambda) * (mu1 * (1 - exp(-abs(lambda) * mu2)) - abs(lambda) * mu3);
    Fx = mu_lambda * FN;
end
