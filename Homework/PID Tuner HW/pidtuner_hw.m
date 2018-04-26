%% Joshua Andrews - ECE414 - PID Tuner HW

%% Part 1, P control
% Testing the PID tuner and PID tune function for a P controller

clear all; clc;

G = tf(40, [1 30 200])

%% Find the baseline matlab generated P controller
Cp = pidtune(G, 'P');

%% Run PID Tuner with plant tf. and the baseline Cp
% Using the GUI the response time can be changed to alter the Kp parameter.
%  Because we are looking at minimizing %OS and controller effort, the
%  contoller effort can be plotted alongside the step response of the
%  system using the tools at the top of the GUI.  The controller parameters
%  is displayed by clicking the button at the top and the controller can be
%  exported to the workspace.

%% Finding the minimum controller effort
% Using the slider, the lowest %OS and controller effort occured when the
% response time was as slow as possible.  The lowest %OS was found by
% watching the parameters while the lowest controller effort was found by
% displaying the peak by right clicking on the controller effort graph.
% After tuning export it to the workspace as C.

Tune = pidTuner(G, Cp);
waitfor(Tune);   %Wait here until controller is exported and pidtune GUI closed

%% Export the tuned controller
% The best value of Kp was found to be 5 as it minimzed both parameters.
% The controller effort had a peak of 5 and the %OS was 2.84%

% Baseline controller effort and system transfer functions
Up = Cp/(1+(Cp*G));
Tp = (Cp*G)/(1+(G*Cp));

% Tuned controller effort and system transfer functions
Utp = C/(1+(C*G));
Ttp = (C*G)/(1+(G*C));

%% Ploting the step responses
% Using the transfer functions above the step responses we plotted for each

figure(1); clf;
step(Tp);
hold on;
step(Ttp);
grid on;
legend('Baseline', 'Tuned');
title('System Step Response');

figure(2); clf;
step(Up);
hold on;
step(Utp);
grid on;
legend('Baseline', 'Tuned');
title('Contoller Effort Step Response');

%%
% In the graphs above you can see that by only focusing on those two
% parameters we got a much better response with a longer rise time, which
% is still fairly low, with the tuned controller.  The peak controller effort also drops from
% 21 to 5


%% Part 2 - PD Controller
% By using the steps above we can do the same process for the P
% controller

%% 
% The controller effort transfer function is improper for any value of Kd
% other than 0.  This results in an ideal controller that is not
% implementable in the real world.  The controller would have to supply
% impluse outputs and the plant would have to handle impulse inputs.  But in order to get values for Kp and Kd, the controller was tuned based on %OS alone.  This occured when both sliders were taken all the way to the right. (Fast response, robust transient).    

Cpd = pidtune(G, 'PD');
Tune = pidTuner(G, Cpd);
waitfor(Tune);
disp('Parameters from PID Tune');
disp('Kp = ');
disp(C.Kp);
disp('Kd = ');
disp(C.Kd);

% Baseline controller effort and system transfer functions
disp('Baseline Controller Effort Transfer Function');
Upd = Cpd/(1+(Cpd*G))
disp('Baseline System Transfer Function');
Tpd = (Cpd*G)/(1+(G*Cpd))

% Tuned controller effort and system transfer functions
disp('Tuned Controller Effort Transfer Function');
Utpd = C/(1+(C*G))
disp('Tuned System Transfer Function');
Ttpd = (C*G)/(1+(G*C))


figure(3); clf;
step(Tpd);
hold on;
step(Ttpd);
grid on;
legend('Baseline', 'Tuned');
title('System Step Response');

%%
% Using the parameters above the %OS was 0.07%.  While the rise time
% could have been decreased much further by making the response time even faster, this resulted in a slightly
% larger overshoot and much more impractical values for Kd and Kp.


%% Part 3 PID Controller
% By using the steps above we can do the same process for the PD controller
%% 
% The controller effort transfer function is improper for any value of Kd
% or Ki other than 0.  This results in an ideal controller that is not
% implementable in the real world.  But in order to get values for Kp, Ki and Kd, the controller was tuned based
% on %OS alone and to have values for each paramter.

Cpid = pidtune(G, 'PID');
Tune = pidTuner(G, Cpid);
waitfor(Tune);
disp('Parameters from PID Tune');
disp('Kp = ');
disp(C.Kp);
disp('Ki = ');
disp(C.Ki);
disp('Kd = ');
disp(C.Kd);

% Baseline controller effort and system transfer functions
disp('Baseline Controller Effort Transfer Function');
Upid = Cpid/(1+(Cpid*G))
disp('Baseline System Transfer Function');
Tpid = (Cpid*G)/(1+(G*Cpid))

% Tuned controller effort and system transfer functions
disp('Tuned Controller Effort Transfer Function');
Utpid = C/(1+(C*G))
disp('Tuned System Transfer Function');
Ttpid = (C*G)/(1+(G*C))


figure(4); clf;
step(Tpid);
hold on;
step(Ttpid);
grid on;
legend('Baseline', 'Tuned');
title('System Step Response');

%%
% Using the parameters above the %OS was 0%.  The turned controller also
% had a similar rise time to the baseline but a much better %OS.    


%% Part 4 PDF Controller
% By using the steps above we can do the same process for the PD
% controller

%% 
% The controller effort transfer function is proper due to the introduced
% Tf term.  Now the controller can be tuned with both constraints in mind.
% This controller is significantly harder to tune over the other though and
% so a design guideline I followed was to keep %OS under 5% and Peak
% controller effort under 50.  This was found by slowing down the response
% time while increasing the robustness of the transient.  The response time
% was set at 0.0763 and transient was set to 0.84.  This resulted in a %0s = 4.85% and a peak controller effort of 48.9.  The PID parameters are
% shown below.

Cpdf = pidtune(G, 'PDF');
Tune = pidTuner(G, Cpdf);
waitfor(Tune);
disp('Parameters from PID Tune');
disp('Kp = ');
disp(C.Kp);
disp('Kd = ');
disp(C.Kd);
disp('Tf = ');
disp(C.Tf);

% Baseline controller effort and system transfer functions
disp('Baseline Controller Effort Transfer Function');
Updf = Cpdf/(1+(Cpdf*G))
disp('Baseline System Transfer Function');
Tpdf = (Cpdf*G)/(1+(G*Cpdf))

% Tuned controller effort and system transfer functions
disp('Tuned Controller Effort Transfer Function');
Utpdf = C/(1+(C*G))
disp('Tuned System Transfer Function');
Ttpdf = (C*G)/(1+(G*C))


figure(5); clf;
step(Tpdf);
hold on;
step(Ttpdf);
grid on;
legend('Baseline', 'Tuned');
title('System Step Response');

figure(6); clf;
step(Updf);
hold on;
step(Utpdf);
grid on;
legend('Baseline', 'Tuned');
ylim([0 100]);
title('Contoller Effort Step Response');

%%
% There may be a better solution to this problem as to minizing both
% constraints however the tuned controller had a much lower controller
% effort over the baseline, pratical values of Kp, Kd, and Tf, and a low
% peak control effort.

%% Part 5 PIDF Controller

%% 
% The controller effort transfer function is proper due to the introduced
% Tf term.  Now the controller can be tuned with both constraints in mind.
% This controller is significantly harder to tune over all the others and
% to keep all for Kpid and Tf present in the controller.  The value for
% response time found was 0.563, and the value for the transient response
% (though is didnt matter is it was any value below this) was 0.52.  This
% resulted in a %OS of 2.83% and a peak controller effort of 5.27.

Cpidf = pidtune(G, 'PIDF');
Tune = pidTuner(G, Cpidf);
waitfor(Tune);
disp('Parameters from PID Tune');
disp('Kp = ');
disp(C.Kp);
disp('Ki = ');
disp(C.Kp);
disp('Kd = ');
disp(C.Kd);
disp('Tf = ');
disp(C.Tf);

% Baseline controller effort and system transfer functions
disp('Baseline Controller Effort Transfer Function');
Upidf = Cpidf/(1+(Cpidf*G))
disp('Baseline System Transfer Function');
Tpidf = (Cpidf*G)/(1+(G*Cpidf))

% Tuned controller effort and system transfer functions
disp('Tuned Controller Effort Transfer Function');
Utpidf = C/(1+(C*G))
disp('Tuned System Transfer Function');
Ttpidf = (C*G)/(1+(G*C))


figure(7); clf;
step(Tpidf);
hold on;
step(Ttpidf);
grid on;
legend('Baseline', 'Tuned');
title('System Step Response');

figure(8); clf;
step(Upidf);
hold on;
step(Utpidf);
grid on;
legend('Baseline', 'Tuned');
title('Contoller Effort Step Response');

%% Which one is Best
% The best controller for this plant based on the contraints would be the
% PIDF controller.  It has the lowest %OS of the implementable controllers
% and the peak control effort is also very low.  A strong case can be made
% for the P controller as well as it has similar %OS and peak effort but
% much easier to implement.

