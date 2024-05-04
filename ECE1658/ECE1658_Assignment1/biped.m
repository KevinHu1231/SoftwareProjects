function xdot=biped(t,x,data)
    q = x(1:5,1);
    qdot = x(6:10,1);

    tau = inv(data.H*inv(data.D(q))*data.B)*(data.H*inv(data.D(q))*(data.C([q; qdot])*qdot + data.G(q))-data.Kp*sin(data.H*q-data.qref)-data.Kd*data.H*qdot);
    qddot = inv(data.D(q))*(-data.C([q; qdot])*qdot-data.G(q) + data.B*tau);
    xdot = [qdot; qddot];
end