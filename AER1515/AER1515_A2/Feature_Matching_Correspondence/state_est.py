P_check_0 = 1 
x_check_0 = 1

Q = 1
R = 1
y_0 = 2

x_op = x_check_0
for i in range(0,10):
    y_op = (x_op**2 + 1**2)**(1/2)
    G_0 = x_op/((x_op**2+1)**(1/2))
    K_0 = P_check_0*G_0/(G_0*P_check_0*G_0+R)
    P_hat_0 = (1-K_0*G_0)*P_check_0
    x_hat_0 = x_check_0+K_0*(y_0-y_op-G_0*(x_check_0-x_op))
    x_op = x_hat_0
    print("P_hat_0",P_hat_0)
    print("x_hat_0",x_hat_0)


