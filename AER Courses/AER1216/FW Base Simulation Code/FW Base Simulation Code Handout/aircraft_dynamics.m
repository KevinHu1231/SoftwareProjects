function aircraft_dynamics(block)
%MSFUNTMPL_BASIC A Template for a Level-2 MATLAB S-Function
%   The MATLAB S-function is written as a MATLAB function with the
%   same name as the S-function. Replace 'msfuntmpl_basic' with the 
%   name of your S-function.
%
%   It should be noted that the MATLAB S-function is very similar
%   to Level-2 C-Mex S-functions. You should be able to get more
%   information for each of the block methods by referring to the
%   documentation for C-Mex S-functions.
%
%   Copyright 2003-2010 The MathWorks, Inc.

% AER1216 Fall 2023 
% Fixed Wing Project Code
%
% aircraft_dynamics.m
%
% Fixed wing simulation model file, based on the Aerosonde UAV, with code
% structure adapted from Small Unmanned Aircraft: Theory and Practice by 
% R.W. Beard and T. W. McLain. 
% 
% Inputs: 
% delta_e           elevator deflection [deg]
% delta_a           aileron deflection [deg]
% delta_r           rudder deflection [deg]
% delta_t           normalized thrust []
%
% Outputs:
% pn                inertial frame x (north) position [m]
% pe                inertial frame y (east) position [m]
% pd                inertial frame z (down) position [m]
% u                 body frame x velocity [m/s]
% v                 body frame y velocity [m/s]
% w                 body frame z velocity [m/s]
% phi               roll angle [rad]
% theta             pitch angle [rad]
% psi               yaw angle [rad]
% p                 roll rate [rad/s]
% q                 pitch rate [rad/s]
% r                 yaw rate [rad/s]
%
% Last updated: Pravin Wedage 2023-10-26

%% TA NOTE
% The main code segements you must modify are located in the derivatives
% function in this .m file. In addition, for Q7, you may need to modify the
% setup function in order to input wind into the dynamics. 
% 
% Modify other sections at your own risk. 


%
% The setup method is used to set up the basic attributes of the
% S-function such as ports, parameters, etc. Do not add any other
% calls to the main body of the function.
%
setup(block);

end 


%% Function: setup ===================================================
% Abstract:
%   Set up the basic characteristics of the S-function block such as:
%   - Input ports
%   - Output ports
%   - Dialog parameters
%   - Options
%
%   Required         : Yes
%   C-Mex counterpart: mdlInitializeSizes
%
function setup(block)

% Register number of ports
block.NumInputPorts  = 1;
block.NumOutputPorts = 1;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
for i = 1:block.NumInputPorts
    block.InputPort(i).Dimensions        = 4;
    block.InputPort(i).DatatypeID  = 0;  % double
    block.InputPort(i).Complexity  = 'Real';
    block.InputPort(i).DirectFeedthrough = false; % important to be false 
end

% Override output port properties
for i = 1:block.NumOutputPorts
    block.OutputPort(i).Dimensions       = 12;
    block.OutputPort(i).DatatypeID  = 0; % double
    block.OutputPort(i).Complexity  = 'Real';
%     block.OutputPort(i).SamplingMode = 'Sample';
end

% Register parameters
block.NumDialogPrms     = 1;
P = block.DialogPrm(1).Data; % must duplicate this line in each function

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

% Register multiple instances allowable
% block.SupportMultipleExecInstances = true;

% Register number of continuous states
block.NumContStates = 12;

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

% -----------------------------------------------------------------
% The MATLAB S-function uses an internal registry for all
% block methods. You should register all relevant methods
% (optional and required) as illustrated below. You may choose
% any suitable name for the methods and implement these methods
% as local functions within the same file. See comments
% provided for each function for more information.
% -----------------------------------------------------------------

% block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup); % discrete states only
block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
block.RegBlockMethod('InitializeConditions',    @InitializeConditions);
% block.RegBlockMethod('Start',                   @Start); % Initialize Conditions is used
block.RegBlockMethod('Outputs',                 @Outputs); % Required
% block.RegBlockMethod('Update',                  @Update); % only required for discrete states
block.RegBlockMethod('Derivatives',             @Derivatives); % Required for continuous states
block.RegBlockMethod('Terminate',               @Terminate); % Required

end 


%% PostPropagationSetup:
%   Functionality    : Setup work areas and state variables. Can
%                      also register run-time methods here
%   Required         : No
%   C-Mex counterpart: mdlSetWorkWidths
%
function DoPostPropSetup(block)
block.NumDworks = 1;
  
  block.Dwork(1).Name            = 'x1';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;

end


%% InitializeConditions:
%   Functionality    : Called at the start of simulation and if it is 
%                      present in an enabled subsystem configured to reset 
%                      states, it will be called when the enabled subsystem
%                      restarts execution to reset the states.
%   Required         : No
%   C-MEX counterpart: mdlInitializeConditions
%
function InitializeConditions(block)

% Rename parameters
P = block.DialogPrm(1).Data; % must duplicate this line in each function

% Initialize continuous states
block.ContStates.Data(1) = P.pn0; 
block.ContStates.Data(2) = P.pe0;
block.ContStates.Data(3) = P.pd0;
block.ContStates.Data(4) = P.u0;
block.ContStates.Data(5) = P.v0;
block.ContStates.Data(6) = P.w0;
block.ContStates.Data(7) = P.phi0;
block.ContStates.Data(8) = P.theta0;
block.ContStates.Data(9) = P.psi0;
block.ContStates.Data(10) = P.p0;
block.ContStates.Data(11) = P.q0;
block.ContStates.Data(12) = P.r0;

end 

%% Start:
%   Functionality    : Called once at start of model execution. If you
%                      have states that should be initialized once, this 
%                      is the place to do it.
%   Required         : No
%   C-MEX counterpart: mdlStart
%
function Start(block)

block.Dwork(1).Data = 0;

end 

%% Input Port Sampling Method:
function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = 'Sample';
  for i = 1:block.NumOutputPorts
    block.OutputPort(i).SamplingMode  = 'Sample';   
  end
end

%% Outputs:
%   Functionality    : Called to generate block outputs in
%                      simulation step
%   Required         : Yes
%   C-MEX counterpart: mdlOutputs
%
function Outputs(block)

temp_mat = zeros(block.NumContStates,1); % thirteen states
for i = 1:block.NumContStates
     temp_mat(i) = block.ContStates.Data(i);
end

block.OutputPort(1).Data = temp_mat; % states

% for i = 1:block.NumOutputPorts
%     block.OutputPort(1).Data(i) = block.ContStates.Data(i);
% end

end 


%% Update:
%   Functionality    : Called to update discrete states
%                      during simulation step
%   Required         : No
%   C-MEX counterpart: mdlUpdate
%
function Update(block)

block.Dwork(1).Data = block.InputPort(1).Data;

end 


%% Derivatives:
%   Functionality    : Called to update derivatives of
%                      continuous states during simulation step
%   Required         : No
%   C-MEX counterpart: mdlDerivatives
%
function Derivatives(block)

% Rename parameters
P = block.DialogPrm(1).Data; % must duplicate this line in each function

% compute inertial constants
% K = ;
% k1 = ;
% k2 = ;
% k3 = ;
% k4 = ;
% k5 = ;
% k6 = ;
% k7 = ;
% k8 = ;

% map states and inputs
pn    = block.ContStates.Data(1);
pe    = block.ContStates.Data(2);
pd    = block.ContStates.Data(3);
u     = block.ContStates.Data(4);
v     = block.ContStates.Data(5);
w     = block.ContStates.Data(6);
phi   = block.ContStates.Data(7);
theta = block.ContStates.Data(8);
psi   = block.ContStates.Data(9);
p     = block.ContStates.Data(10);
q     = block.ContStates.Data(11);
r     = block.ContStates.Data(12);
delta_e = block.InputPort(1).Data(1)*pi/180 ; % converted inputs to radians
delta_a = block.InputPort(1).Data(2)*pi/180 ; % converted inputs to radians
delta_r = block.InputPort(1).Data(3)*pi/180 ; % converted inputs to radians
delta_t = block.InputPort(1).Data(4);

load("aer1216_2023F_proj_prop_data.mat");

% Geometric data
mass = 9.1;
Ixx = 0.8244;
Iyy = 1.135;
Izz = 1.759;
Ixz = 0.1204;
S = 0.55;
b = 2.8956;
c = 0.18994;
Sprop = 0.2027;
Dia = 2 * sqrt(Sprop/pi);
%n = omega / 60;
e = 0.9;
omega_max = 12500;
n = omega_max / 60;
FuelCap = 4;
SFC = 0.6651;
AR = (b^2)/S;

% Air Data 
Va = P.Va;      % = Va_trim = 15
ue = 19.4;      % steady speed
alpha = 0;
beta = 0;
theta_trim = 0;
delta_t_trim = 1;
h = pd;

fCT = fit(J, CT, 'poly3', 'Normalize', 'on', 'Robust', 'Bisquare');
fCQ = fit(J, CQ, 'poly3', 'Normalize', 'on', 'Robust', 'Bisquare');
Ji = Va/(n*Dia);
CTi = fCT(Ji);
CQi = fCQ(Ji);
CPi = 2*pi*CQi;
efficieny = CTi*Ji/CPi;
k = efficieny;  % motor efficiency for X_delta_t

% compute air density according to altitude
height = -pd;
T = 15.04 - 0.00649*height;
Pressure = 101.29 * ((T+273.1)/288.08)^5.256;
rho = Pressure/(0.2869*(T+273.1));

% rotation matrix
R = [cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta);
     sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*cos(theta);
     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*cos(theta)];

% Aerodynamic Coefficients 
% compute the nondimensional aerodynamic coefficients here
% Longitudial coefficients
CL_0 = 0.28;
CD_0 = 0.03;
Cm_0 = -0.02338;
CL_alpha = 3.45;
CD_alpha = 0.3;
Cm_alpha = -0.38;
CL_q = 0;
CD_q = 0;
Cm_q = -3.6;
CL_delta_e = -0.36;
CD_delta_e = 0;
Cm_delta_e = -0.5;

% Lateral coefficients
CY0 = 0;
Cl0 = 0;
Cn0 = 0;
CYbeta = -0.98;
Clbeta = -0.12;
Cnbeta = 0.25;
CYp = 0;
Clp = -0.26;
Cnp = 0.022;
CYr = 0;
Clr = 0.14;
Cnr = -0.35;
CYdeltaa = 0;
Cldeltaa = 0.08;
Cndeltaa = 0.06;
CYdeltar = -0.17;
Cldeltar = 0.105;
Cndeltar = -0.032;

% aerodynamic forces and moments
% compute the aerodynamic forces and moments here

% propulsion forces and moments
% compute the propulsion forces and moments here

% gravity
% compute the gravitational forces here

% total forces and moments (body frame)
% ----- Longitudinal
CX0 = -CD_0*cos(alpha) + CL_0*sin(alpha);
CXq = -CD_q*cos(alpha) + CL_q*sin(alpha);
CXalpha = -CD_alpha*cos(alpha) + CL_alpha*sin(alpha);
CXdeltae = -CD_delta_e*cos(alpha) + CL_delta_e*sin(alpha);

Xu = (ue*rho*S/mass) * CX0 - rho*Sprop*CTi*ue/mass;
Xw = rho*S*ue*CXalpha/(2*mass);
Xq = rho*Va*S*CXq*c/(4*mass);
Xdeltae = rho*(Va^2)*S*CXdeltae/(2*mass);
Xdeltat = rho*Sprop*CTi*(k^2)/mass; % initial throttle assumed to be 1

CZ0 = -CD_0*sin(alpha) - CL_0*cos(alpha);
CZq = -CD_q*sin(alpha) - CL_q*cos(alpha);
CZalpha = -CD_alpha*sin(alpha) - CL_alpha*cos(alpha);
CZdeltae = -CD_delta_e*sin(alpha) - CL_delta_e*cos(alpha);

Zu = (ue*rho*S/mass) *CZ0;
Zw = rho*S*ue*CZalpha/(2*mass);
Zq = ue + rho*Va*S*CZq*c/(4*mass);
Zdeltae = rho*(Va^2)*S*CZdeltae/(2*mass);

Mu = (ue*rho*S*c/Iyy) * Cm_0;
Mw = (ue*rho*S*c/(2*Iyy)) * Cm_alpha;
Mq = (rho*Va*S*(c^2)/(4*Iyy)) * Cm_q;
Mdeltae = (rho*(Va^2)*S*c/(2*Iyy)) * Cm_delta_e;

ALong = [Xu Xw Xq -P.g*cos(theta_trim) 0;
         Zu Zw Zq -P.g*sin(theta_trim) 0;
         Mu Mw Mq 0 0;
         0 0 1 0 0;
         sin(theta_trim) -cos(theta_trim) 0 ue*cos(theta_trim) 0];

BLong = [Xdeltae Xdeltat;
         Zdeltae 0;
         Mdeltae 0;
         0 0;
         0 0];
XLong = [u-Va; w; q; theta; h];
ULong = [delta_e; delta_t];

Longdot = ALong*XLong + BLong*ULong;

% ----- Lateral

Yv = (ue*rho*S/(2*mass)) * CYbeta;
Yp = rho*Va*S*b*CYp/(4*mass);
Yr = -ue + rho*Va*S*b*CYr/(4*mass);
Ydeltaa = rho*(Va^2)*S*CYdeltaa/(2*mass);
Ydeltar = rho*(Va^2)*S*CYdeltar/(2*mass);

Lv = (ue*rho*S/2) *Clbeta;
Lp = rho*Va*S*b^2*Clp/(4);
Lr = rho*Va*S*b^2*Clr/(4);
Ldeltaa = rho*(Va^2)*S*b*Cldeltaa/(2);
Ldeltar = rho*(Va^2)*S*b*Cldeltar/(2);

Nv = (ue*rho*S*b/2) *Cnbeta;
Np = rho*Va*S*b^2*Cnp/(4);
Nr = rho*Va*S*b^2*Cnr/(4);
Ndeltaa = rho*(Va^2)*S*b*Cndeltaa/(2);
Ndeltar = rho*(Va^2)*S*b*Cndeltar/(2);

ALat = [Yv Yp Yr P.g*cos(theta_trim) 0;
        Lv Lp Lr 0 0;
        Nv Np Nr 0 0;
        0 1 tan(theta_trim) 0 0;
        0 0 acos(theta_trim) 0 0];

BLat = [Ydeltaa Ydeltar;
        Ldeltaa Ldeltar;
        Ndeltaa Ndeltar;
        0 0;
        0 0];
XLat = [v; p; r; phi; psi];
ULat = [delta_a; delta_r];

Latdot = ALat*XLat + BLat*ULat;

posdot = R' * [u;v;w];

% state derivatives
% the full aircraft dynamics model is computed here
%pdot = ;
pndot = posdot(1);
pedot = posdot(2);
pddot = Longdot(5);

udot = Longdot(1);
vdot = Latdot(1);
wdot = Longdot(2);

phidot = Latdot(4);
thetadot = Longdot(4);
psidot = Latdot(5);

pdot = Latdot(2);
qdot = Longdot(3);
rdot = Latdot(3);

% map derivatives
block.Derivatives.Data(1) = pndot;
block.Derivatives.Data(2) = pedot;
block.Derivatives.Data(3) = pddot;
block.Derivatives.Data(4) = udot;
block.Derivatives.Data(5) = vdot;
block.Derivatives.Data(6) = wdot;
block.Derivatives.Data(7) = phidot;
block.Derivatives.Data(8) = thetadot;
block.Derivatives.Data(9) = psidot;
block.Derivatives.Data(10)= pdot;
block.Derivatives.Data(11)= qdot;
block.Derivatives.Data(12)= rdot;

end 


%% Terminate:
%   Functionality    : Called at the end of simulation for cleanup
%   Required         : Yes
%   C-MEX counterpart: mdlTerminate
%
function Terminate(block)

end 

