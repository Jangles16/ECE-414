%% ECE 414 - Final Project
% Written by Josh Andrews, Riley McKay

%% Step One, Get the TF's in here and other variables needed
% Need N,D,C equations, and most parameters (make data structure?)
clear all; clc; 
clf;
s = tf('s');

% Make structure containing constant overall parameters (ie no motor stuff)
d.Jp = 6.2e-6;          %pulley inertia
d.Js = 1.4e-6;          %position sensor inertia 
d.Gv = 5;               %Voltage amplifier gain (V/V)
d.Ks = 25;              %position sensor gain (V/m)
d.Rp = 0.0075;          %pulley radius (m)
d.Jb = 1.0125e-6;       %belt inertia
d.Mc = [0.050 0.250 0.150];           %cartridge mass (set up to get 4 meaningful TF combos and nominal plant)
d.Jc = d.Mc * (d.Rp)^2; %nominal cartridge inertia
d.Jx = (d.Jp)*2 + d.Js + d.Jb + d.Jc;   %The constant part of inertia (only motor inertia change) when fixing Cartridge mass.


%% Find the Nominal plants
% Need to do this for all for motors

for m = 1:4
    
    motor = MotorNum(m); % Choose one of the four available motors
    % Inertia calculations
    motor.Jsys = motor.Jm + d.Jx;   % Calculate total system inertia
    
    % whole plant
    Numerator = (d.Gv .* motor.Kt * d.Ks * d.Rp)./(motor.L .* motor.Jsys);
    sOneTerm = (motor.R/motor.L)+(motor.Bm./motor.Jsys);
    sZeroTerm = (((motor.Kt).^2)+ motor.R * motor.Bm)./(motor.L .* motor.Jsys);
    
    % Velocity
    Znum =(motor.Kt)./(motor.L.*motor.Jsys);
    Zden =(s^2 + (motor.R/motor.L)*s+(motor.Bm./motor.Jsys).*s+((motor.Kt.^2 + motor.R*motor.Bm)./(motor.L.*motor.Jsys)));
    
    % Current
    Curr_plant_num = (d.Gv/motor.L)*(s+(motor.Bm./motor.Jsys));
    Curr_plant_den = s^2+((motor.R/motor.L)+(motor.Bm./motor.Jsys))*s + ((((motor.Kt).^2)+motor.R*motor.Bm)./(motor.L.*motor.Jsys));
    
    % Calculate open loop transfer functions
    for j = 1:3
        motor.G(j) = Numerator(j)/(s*(s^2 + sOneTerm(j) * s + sZeroTerm(j))); % Calculate plant transfer function
        Z(j) = Znum(j)/Zden(j);
        motor.Vel(j) = d.Ks*Z(j)*d.Gv*d.Rp;
        motor.Cur(j) = Curr_plant_num(j)/Curr_plant_den(j);
        motor.Volt(j) = d.Gv;
    end
    
    % Save the data so we have it for later
    Plant(m) = motor;
    
    % Plot Root Locus of plant transfer function
    %figure(m+10); clf;
	%rlocus(Plant(m).G);
end

% figure(40);clf;
% for i=1:4
%     subplot(2,2,i)
%     rlocus(Plant(i).G(3))
%     str = sprintf('Root Locus Motor %d',i);
%     title(str)
% end

%% Step Two, Do Root Locus of Plants as Parameters Change
% Plot change in poles as Kt and Mi change.  Mi=cartridge mass, Kt = motor
% torque constant. Will have 3 variations of each parameter (Km and Mi)

%%% Still Need to do (maybe)  %%%



%% Our Reference Input
% refer to diagram in notebook, 5/4/18 pg. 2
T = [0 .08 .585 .675];
F = 0.44;
Type = 1;   % zero gives me less overshoot, 1 has less dicontinuities
[fun,dfun,ifun] = spulse(T,F,Type);     %our reference inputs    

tfun = 0:0.0001:0.9;
pos = ifun(tfun);
vel = fun(tfun);
acc = dfun(tfun);
% figure(500);clf;
% plot(pos*100,vel*100)   %convert to cm and plot it
% xlabel('Position (cm)')
% ylabel('Velocity (cm/s)')


%% Test Stuff
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(10);clf;
% plot(tfun,vel);
% figure(11);clf;
% plot(tfun,pos);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Controller
% need to find out which plant setup to optimize for, might actually be several we
% have to look at. I'm thinking right now that Motor 1 with full ink and
% high inertia will be the slowest to respond.
% 
Kp = 319.8;
Kd = 5.443;
Tf = 1.392e-5;
C = Kp + Kd * (s/(Tf*s + 1));
%C=zpk(-105,-3000,3000);
%C=20;
% 
%opt=pidtuneOptions('DesignFocus','reference-tracking');
%C=pidtune(Plant(1).G(1),'PDF',opt);
% tune=pidTuner(Plant(3).G(1),Co);
% waitfor(tune);





%%% will need a method to find controller
%%% Step is actually faster than ours, so need to experiment.  Also want to
%%% keep phase margin high.




%% Calculate Closed Loop TFs
for i=1:4
    for j = 1:3
        L=C*Plant(i).G(j);
        Plant(i).Vel_tf(j) = C*Plant(i).Vel(j)/(1+L);      %Velocity TF
        Plant(i).Sys_tf(j) = L/(1+L);                   %System TF
        Plant(i).Volt_tf(j) = (C*Plant(i).Volt(j))/(1+L);  %Voltage TF
        Plant(i).Cur_tf(j) = (C*Plant(i).Cur(j))/(1+L);   %Current TF
    end
end

%% Position Response v Time

% for i=1:4
%     figure(i)
%     for j=1:3
% 
%         [Pos_act,tref]=lsim(Plant(i).Sys_tf(j),pos,tfun);
%         plot(tref*1000,Pos_act*100)
%         hold on
% 
%     end
%     plot(tfun*1000,pos*100)      %variables from spluse function
%     hold off
%     legend('LL','HH','NOM','REF')
%     title('Whole System Response')
%     xlabel('Time (msec)')
%     ylabel('Position (cm)')
% end


%% Voltage Response v Time
% for i=1:4
%     figure(i+4)
%     for j=1:3
% 
%         [Volt_act,tref]=lsim(Plant(i).Volt_tf(j),pos,tfun);
%         [Cur_act,~]=lsim(Plant(i).Cur_tf(j),pos,tfun);
%        
%         Power_act = abs(Volt_act.*Cur_act);
%         plot(tref, Power_act)
%        
%         
%         hold on
% 
%     end
%    % Plant(i).Volt_Act = Volt_act;
%     %plot(tfun,pos)      %variables from spluse function
%     hold off
%     legend('LL','HH','NOM')
%     title('Power v Time')
%     xlabel('Time (s)')
%     ylabel('Power (W)')
%     title('Voltage vs Time')
%     xlabel('Time (msec)')
%     ylabel('Voltage (V)')
% end

 
%% Current Response v Time
% for i=1:4
%     figure(i+8)
%     for j=1:3
% 
%         [Cur_act,tref]=lsim(Plant(i).Cur_tf(j),pos,tfun);
%         plot(tref*1000,Cur_act)
%         Plant(i).Cur_Act(j) = Cur_act;
%         hold on
% 
%     end
%     %plot(tfun,pos)      %variables from spluse function
%     hold off
%     legend('LL','HH','NOM')
%     title('Current vs Time')
%     xlabel('Time (msec)')
%     ylabel('Current (A)')
% end
 
%% Velocity Response v Time

% for i=1:4
%     figure(i+12)
%     for j=1:3
% 
%         [Vel_act,tref]=lsim(Plant(i).Vel_tf(j),pos,tfun);
%         plot(tref*1000,Vel_act*100)
%         hold on
%     end
%     plot(tfun*1000,vel*100)      %variables from spluse function
%     hold off
%     legend('LL','HH','NOM','REF')
%     title('Velocity vs Time')
%     xlabel('Time (msec)')
%     ylabel('Velocity (cm/s)')
% end 
%% Velocity Versus Position

figure(2);clf;
for i=1:4
    subplot(2,2,i)
    for j=1:3

        [Vel_act,tref]=lsim(Plant(i).Vel_tf(j),pos,tfun);
        plot(pos*100,Vel_act*100)
        hold on
    end
    %legend('LL','HH','NOM')
    plot([2 2],ylim,'--');      % 2cm edge
    hold on
    plot([24 24],ylim,'--');    % 24cm edge
    hold on
    plot([26 26],ylim,'--')     % opp. pos limit
    hold on
    %plot(xlim,[0.1 0.1],'--')   % max speed when hitting 26cm
    %hold on
%     plot(pos*100,vel*100,'r')   %% referece vel v pos
%     hold on
    plot(xlim,[44 44])
%     hold on
%     plot(xlim,[.9999*44 .9999*44])
    hold off
    str = sprintf('Velocity vs Position Motor %d',i);
    title(str)
    xlabel('Position (cm)')
    ylabel('Velocity (cm/s)')
    %xlim([1 3])
   % ylim([43 45])

end


%% Power Calculation and Plot
% probably don't have to plot this one, just need the max

% for i=1:4
%     figure(i+20);clf;
% 
%     Plant(i).Power_act = abs(Plant(i).Volt_act.*Plant(i).Curr_act);
%     plot(tref, Power_act)
%     title('Power')
%     xlabel('Time (s)')
%     ylabel('Power (W)')
% end

%% There are a few more plots requested



%% 































% figure(1);clf;
% for i = 1:4
%     [y,t] = lsim(Plant(i).G,pos,tfun);
%     plot(t,y)
%     hold on
%     
% end
% legend('1','2','3','4')


