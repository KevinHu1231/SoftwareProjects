function y = y(ri,bi)
    n = length(ri) + length(bi);
    y = zeros(n,1);
    for i = 1:n
        if mod(i,2) == 1
            y(i) = ri((i+1)/2);
        else
            y(i) = bi(i/2);
        end
    end
end