%% ECE 414 - Final Project
% Written by Josh Andrews, Riley McKay

%% Step One, Get the TF's in here and other variables needed
% Need N,D,C equations, and most parameters (make data structure?)

% Make structure containing constant overall parameters (ie no motor stuff)
d.Jp = 6.2e-6;          %pulley inertia
d.Js = 1.4e-6;          %position sensor inertia 
d.Gv = 5;               %Voltage amplifier gain (V/V)
d.Ks = 25;              %position sensor gain (V/m)
d.Rp = 0.0075;          %pulley radius (m)
d.Jb = 1.0125e-6;       %belt inertia
d.Mc = 0.150;           %nominal cartridge mass (kg)

%% Motor 1

m(1).Jm = 5e-5;         %inertia 
m(1).Bm = 3.0e-6;       %viscous friction
m(1).R = 15;            %resistance
m(1).L = 35e-3;         %inductance
m(1).Kt = 0.275;        %nominal torque constant

%% Motor 2

m(2).Jm = 5.5e-5;
m(2).Bm = 2.5e-6;
m(2).R = 7;
m(2).L = 25e-3;
m(2).Kt = 0.175;

%% Motor 3

m(3).Jm = 4.5e-5;
m(3).Bm = 3.0e-6;
m(3).R = 8;
m(3).L = 25e-3;
m(3).Kt = 0.235;

%% Motor 4

m(4).Jm = 5.0e-5;
m(4).Bm = 3.5e-6;
m(4).R = 4;
m(4).L = 7.5e-3;
m(4).Kt = 0.125;


%% Step Two, Do Root Locus of Plants
% Plot change in poles as Kt and Mi change.  Mi=cartridge mass, Kt = motor
% torque constant.


%% Step Three, Start Looking at Controllers