function H = forward(joint, myrobot)
    % Get DH parameters from myrobot object
    a = myrobot.a;
    d = myrobot.d;
    theta = joint;
    alpha = myrobot.alpha;
    
    H = eye(4);
    % Calculate each of the A matrices
    for i = 1:6
        % Calculate H_i and multiply to overall homogenous transformation
        A = homo(a(i), d(i), theta(i), alpha(i));
        H = H * A;
    end 
end 


% Sub function used to determine the intermeiate homo transformation
% Based on the DH tables
function A = homo(a_i, d_i, theta_i, alpha_i)
    A = [cos(theta_i) -sin(theta_i)*cos(alpha_i) sin(theta_i)*sin(alpha_i) a_i*cos(theta_i);
            sin(theta_i) cos(theta_i)*cos(alpha_i) -cos(theta_i)*sin(alpha_i) a_i*sin(theta_i);
            0 sin(alpha_i) cos(alpha_i) d_i;
            0 0 0 1];
end
