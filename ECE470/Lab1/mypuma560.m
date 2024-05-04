function myrobot = mypuma560(DH)
    % Define the links
    for i = 1:6
        L(i) = Link('revolute', 'd', DH(i,2), 'a', DH(i,3), 'alpha', DH(i,4))
    end
    % Join together the links
    myrobot = SerialLink(L, 'name', 'puma560')
end

