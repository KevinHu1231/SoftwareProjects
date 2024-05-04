A = eye(10)
B = zeros(10,1)
C = [B,A]
D = [-A,B]
E = C + D
F = [E;C]
G = eye(20)
H = F'*G*F
spy(H)