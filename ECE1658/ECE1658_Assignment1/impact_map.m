function Delta=impact_map(x,data)

%Extract q,qdot from x
q = x(1:5,1);
qdot = x(6:10,1);

%Create delta_qdot matrix
matrix_1 = [eye(5),zeros(5,4)];
matrix_2 = [data.Dbar(q),-data.E(q)'; data.E(q), zeros(2,2)];
inter = data.Dbar(q)*[eye(5);zeros(2,5)];
matrix_3 = [inter;zeros(2,5)];
delta_qdot = matrix_1*inv(matrix_2)*matrix_3;

%delta_1 = [q; delta_qdot*qdot];

% q~1 = q1 + q2 + q3 + q4 - 3*pi
% q~2 = - q4 + 2*pi 
% q~3 = - q3 + 2*pi 
% q~4 = - q2 + 2*pi
% q~5 = q5 - q3 + pi

%Get R and d from robot geometry
R = [1,1,1,1,0; 0,0,0,-1,0; 0,0,-1,0,0; 0,-1,0,0,0; 0,0,-1,0,1];
d = pi*[-3;2;2;2;1];

delta_2 = [R*q + d; R*(delta_qdot*qdot)];

%Get Delta
Delta = delta_2;
end