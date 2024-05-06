% AER1216 Fall 2023 
% Fixed Wing Project Code
%
% parameters.m
%
% Initialization file which generates and stores all required data into the 
% structure P, which is then stored in the workspace. Simulink model runs 
% this file at the start of every simulation. Code structure adapted from
% Small Unmanned Aircraft: Theory and Practice by R.W. Beard and T. W. 
% McLain. 
% 
% Inputs: 
% N/A
%
% Outputs:
% P                 structure that contains all aerodynamic, geometric, and
%                   initial condition data for the aircraft and simulation.
%
% Last updated: Pravin Wedage 2023-10-26

%% TA NOTE
% An easy way to store parameters for use in simulink is through the use of
% a structure. For example, P.g = 9.81 stores the value of gravitational
% acceleration in the field g that is contained within the structure P.
% Anytime P is called anywhere in the simulation code, the value of P.g is
% accessible. 

%% Parameter Computation
% Initial Conditions
clear all
% compute trim conditions            
P.Va0 = 15;         % initial airspeed (also used as trim airspeed)
P.Va_trim = 15; 
P.Va = P.Va_trim;


P.gravity = 9.81;
P.g = 9.81; 

% Aerosonde UAV Data
% physical parameters of airframe

% aerodynamic coefficients

% Control Input limits 
P.delta_e_max = deg2rad(45); % assumed symmetric
P.delta_a_max = deg2rad(45); 
P.delta_r_max = deg2rad(25);

% Initial Conditions % connects with aircraft_dynamics.m, do not modify
% structure field names
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -1000;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axisu_trim
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate
P.delta_e0 =0;
P.delta_a0 =0;
P.delta_r0 =0;
P.delta_t0 =0;

%% Lateral coefficients
% P.CY0 = 0;
% P.Cl0 = 0;
% P.Cn0 = 0;
% P.CYbeta = -0.98;
% P.Clbeta = -0.12;
% P.Cnbeta = 0.25;
% P.CYp = 0;
% P.Clp = -0.26;
% P.Cnp = 0.022;
% P.CYr = 0;
% P.Clr = 0.14;
% P.Cnr = -0.35;
% P.CYdeltaa = 0;
% P.Cldeltaa = 0.08;
% P.Cndeltaa = 0.06;
% P.CYdeltar = -0.17;
% P.Cldeltar = 0.105;
% P.Cndeltar = -0.032;