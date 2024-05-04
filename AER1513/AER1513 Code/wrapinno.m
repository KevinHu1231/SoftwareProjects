function inno = wrapinno(innovation)
    inno = innovation;
    n = length(inno);
    for i = 1:n
        if mod(i,2) == 0
            inno(i) = wrapToPi(inno(i));
        end
    end
end