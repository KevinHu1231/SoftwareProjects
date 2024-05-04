function q = inverse(H,myrobot)
    % Get the DH parameters
    a = myrobot.a;
    d = myrobot.d;
    alpha = myrobot.alpha;
    
    
    % Instantiate the desired joint, position
    q = zeros(1,6);
    Rd = H(1:3,1:3);
    od = H(1:3,4);
    
    % Calculate location for wrist center
    oc = od - Rd*[0;0;d(6)];
    
    % Compute the first three joints
    q(1) = atan2(oc(2),oc(1)) - atan2(-d(2), real(sqrt(oc(1)^2+oc(2)^2-d(2)^2)));
    D = ((oc(3)-d(1))^2 + oc(1)^2 + oc(2)^2 - d(2)^2 - a(2)^2 - d(4)^2)/(2*a(2)*d(4));
    q(3) = atan2(D, sqrt(1-D^2));
    q(2) = atan2(oc(3)-d(1), sqrt(oc(1)^2+oc(2)^2-d(2)^2)) - atan2(-d(4)*cos(q(3)), a(2)+d(4)*sin(q(3)));

    theta = [q(1), q(2), q(3), 0, 0, 0];
    
    % Use forward kinematics for the first three joints to find H03
    H03 = eye(4);
    for i = 1:3
        A = homo(a(i), d(i), theta(i), alpha(i));
        H03 = H03 * A;
    end
    
    % Compute the last three joints
    H36 = H03(1:3,1:3)'*Rd;
    q(4) = atan2(H36(2,3), H36(1,3));
    q(5) = atan2(sqrt(1-H36(3,3)^2), H36(3,3));
    q(6) = atan2(H36(3,2),-H36(3,1));
end


% Sub function used to determine the intermeiate homo transformation
% Based on the DH tables
function A = homo(a_i, d_i, theta_i, alpha_i)
    A = [cos(theta_i) -sin(theta_i)*cos(alpha_i) sin(theta_i)*sin(alpha_i) a_i*cos(theta_i);
            sin(theta_i) cos(theta_i)*cos(alpha_i) -cos(theta_i)*sin(alpha_i) a_i*sin(theta_i);
            0 sin(alpha_i) cos(alpha_i) d_i;
            0 0 0 1];
end