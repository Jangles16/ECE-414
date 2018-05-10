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
    Motor.Kt(1) = .275 * 1.1;   % max
    Motor.Kt(2) = .275 * 0.9;   % min
    Motor.Kt(3) = .275 * 1.1;   % max
    Motor.Kt(4) = .275 * 0.9;   % min
    Motor.Kt(5) = .275;         % nominal
    Motor.R = 15;
    Motor.L = .035;

elseif num==2
    Motor.Jm = 5.5e-5;
    Motor.Bm = 2.5e-6;
    Motor.Kt(1) = .175 * 1.1;   % max
    Motor.Kt(2) = .175 * 0.9;   % min
    Motor.Kt(3) = .175 * 1.1;   % max
    Motor.Kt(4) = .175 * 0.9;   % min
    Motor.Kt(5) = .175;         % nominal 
    Motor.R = 7;
    Motor.L = .016;

elseif num==3
    Motor.Jm = 4.5e-5;
    Motor.Bm = 3e-6;
    Motor.Kt(1) = .235 * 1.1;   % max
    Motor.Kt(2) = .235 * 0.9;   % min
    Motor.Kt(3) = .235 * 1.1;   % max
    Motor.Kt(4) = .235 * 0.9;   % min
    Motor.Kt(5) = .235;         % nominal  
    Motor.R = 8;
    Motor.L = .025;

elseif num==4
    Motor.Jm = 5e-5;
    Motor.Bm = 3.5e-6;
    Motor.Kt(1) = .125 * 1.1;   % max
    Motor.Kt(2) = .125 * 0.9;   % min
    Motor.Kt(3) = .125 * 1.1;   % max
    Motor.Kt(4) = .125 * 0.9;   % min
    Motor.Kt(5) = .125;         % nominal  
    Motor.R = 4;
    Motor.L = .0075;
end
end