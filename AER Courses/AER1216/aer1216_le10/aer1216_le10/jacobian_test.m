% ------------------------------------------------------------------------------
% MIT License
% 
% Copyright (c) 2023 Dr. Longhao Qian
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% ------------------------------------------------------------------------------
%% method 1: define systems
% define states and control inputs
x = sym('x',[6, 1]);
u = sym('u',[2, 1]);
% states:
% x(1) = y, x(2) = z, x(3) = phi, x(4) = y_dot, x(5) = z_dot, x(6) =
% phi_dot

% x_dot = f(x, u)

m = 2;
g = 9.8;
Ixx = 0.1;

f = matlabFunction([ x(4);
            x(5);
            x(6);
            -u(1) * sin(x(3)) / m;
            -g + u(1) * cos(x(3))/ m;
            u(2)/Ixx;
          ], 'Vars', {x, u});

As = jacobian(f(x, u), x)
Bs = jacobian(f(x, u), u)
      
Af = matlabFunction(As, 'Vars', {x, u});
Bf = matlabFunction(Bs, 'Vars', {x, u});
%% evaluate the jacobian to get the linearized equations of motion

x0 = zeros(6, 1);
u0 = [m * g; 0];

A = Af(x0, u0)
B = Bf(x0, u0)

%% method2: use simulink to obtain the linearized model

[As, Bs, Cs, Ds] = linmod('jacobian_test_sim', x0, u0);



