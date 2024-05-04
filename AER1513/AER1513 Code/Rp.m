function Rp = Rp(l,r_var,b_var)
    L = size(l,1);
    dia = zeros(2*L,1);
    for i = 1:(2*L)
        if mod(i,2) == 1
            dia(i) = r_var;
        else
            dia(i) = b_var;
        end
    end
    Rp = diag(dia);
end