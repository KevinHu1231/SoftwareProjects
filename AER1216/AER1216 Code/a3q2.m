% To determine the best n and CT combination
CP = [0.0415,0.0415,0.0418,0.0420,0.0424,0.0425,0.0432,0.0437,0.041,0.0447,0.0452,0.0460,0.0464,0.0471,0.0477,0.0481];
CT = [0.0951,0.0963,0.0972,0.0976,0.0994,0.0998,0.1017,0.1025,0.1035,0.1048,0.1061,0.1077,0.1087,0.1106,0.1125,0.1143];
rpm = [3030,3226,3555,3808,4093,4337,4619,4861,5132,5411,5676,5946,6212,6484,6731,7015];
rps = rpm./60;
nsqrtct = rps.*sqrt(CT)
val = sqrt(2/(1.225*0.2032^4))
err = abs(nsqrtct-val)
[minimum, argmin] = min(err)
CT(argmin)
rpm(argmin)
CP(argmin)