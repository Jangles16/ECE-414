function Motor=MotorNum(num)
%%
% Function to return motor parameters
%
% Idea taken from Dalton Binette
%
% Written 5/1/2018 
%
%%

Motor = struct();

if num==1
    Motor.Jm = 5e-5;
    Motor.Bm = 3e-6;
    Motor.Ktnom = .275;
    Motor.Ktmin = Motor.Ktnom * 0.9;
    Motor.KTmax = Motor.Ktnom * 1.1;
    Motor.R = 15;
    Motor.L = .035;

elseif num==2
    Motor.Jm = 5.5e-5;
    Motor.Bm = 2.5e-6;
    Motor.Ktnom = .175;
    Motor.Ktmin = Motor.Ktnom * 0.9;
    Motor.KTmax = Motor.Ktnom * 1.1;
    Motor.R = 7;
    Motor.L = .016;

elseif num==3
    Motor.Jm = 4.5e-5;
    Motor.Bm = 3e-6;
    Motor.Ktnom = .235;
    Motor.Ktmin = Motor.Ktnom * 0.9;
    Motor.KTmax = Motor.Ktnom * 1.1;
    Motor.R = 8;
    Motor.L = .025;

elseif num==4
    Motor.Jm = 5e-5;
    Motor.Bm = 3.5e-6;
    Motor.Ktnom = .125;
    Motor.Ktmin = Motor.Ktnom * 0.9;
    Motor.KTmax = Motor.Ktnom * 1.1;
    Motor.R = 4;
    Motor.L = .0075;
end
end