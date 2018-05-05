%% ECE 414 - Final Project
% Written by Josh Andrews, Riley McKay

%% Step One, Get the TF's in here and other variables needed
% Need N,D,C equations, and most parameters (make data structure?)
s = tf('s');

% Make structure containing constant overall parameters (ie no motor stuff)
d.Jp = 6.2e-6;          %pulley inertia
d.Js = 1.4e-6;          %position sensor inertia 
d.Gv = 5;               %Voltage amplifier gain (V/V)
d.Ks = 25;              %position sensor gain (V/m)
d.Rp = 0.0075;          %pulley radius (m)
d.Jb = 1.0125e-6;       %belt inertia
d.Mc = 0.150;           %nominal cartridge mass (kg)
d.Jc = d.Mc * (d.Rp)^2; %nominal cartridge inertia

%% Find the Nominal plants
% Need to do this for all for motors

% Find Total inertia - motor inertia
d.Jx = (d.Jp)*2 + d.Js + d.Jb + d.Jc;   %The constant part of inertia (only motor inertia change) when fixing Cartridge mass.

for m = 1:4
    motor = MotorNum(m); % Choose one of the four available motors
    % Inertia calculations
    motor.Jsys = motor.Jm + d.Jx;   % Calculate total system inertia
    
    motor.Numerator = (d.Gv * motor.Ktnom * d.Ks * d.Rp)/(motor.L * motor.Jsys);
    motor.sOneTerm = (motor.R/motor.L)+(motor.Bm/motor.Jsys);
    motor.sZeroTerm = (((motor.Ktnom)^2)+ motor.R * motor.Bm)/(motor.L * motor.Jsys);

    % Calculate plant transfer function
    motor.G = motor.Numerator/(s*(s^2 + motor.sOneTerm * s + motor.sZeroTerm)); % Calculate plant transfer function
    
    % Save the data so we have it for later
    Plant(m) = motor;
    % Plot Root Locus of plant transfer function
    figure(m); clf;
    rlocus(Plant(m).G);
end


%% Step Two, Do Root Locus of Plants as Parameters Change
% Plot change in poles as Kt and Mi change.  Mi=cartridge mass, Kt = motor
% torque constant.

d.Mrange = [50,100,150,200,250]./1000;

%% Step Three, Start Looking at Controllers

%% Our Reference Input
% refer to diagram in notebook, 5/4/18 pg. 2
u(1)=0;
for i = 0:1:1000
    if i < 200
        u(i+1)= u(i+1)+1;    
    elseif i > 700
        u(i+1)= u(i+1)+1;
    else
        u(i+1) = 44;
    end
            
end
