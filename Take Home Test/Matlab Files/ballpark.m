function Out = ballpark(RiseTime,SettlingTime,Overshoot,SettlingTimeThreshold)
% BALLPARK sketch ballpark region in the s-plane where poles must reside.
%
% BALLPARK(RiseTime,SettlingTime,Overshoot,SettlingTimeThreshold) sketches
% the s-plane showing where closed loop poles must reside to to meet or
% exceed the step response specifications
%
% RiseTime - 10% to 90% rise time
% SettlingTime - settling time to within 2% of y(infinity)
% Overshoot - percent overshoot relative to y(infinity)
% SettlingTimeThreshold - optional SettlingTime threshold in percent %
%
% Out = BALLPARK(...) returns the structure Out containing the following:
% Out.Wn = minimum undamped natural frequency
% Out.ZetaWn = minimum product Zeta*Wn
% Out.Zeta = Zeta that meets Overshoot spec
% Out.Phi = angle w.r.t. negative real axis of constant Zeta lines

% D. Hanselman
% University of Maine
% 2017-02-07

% inspired by Colin Leary's submission on 2017-02-07

if nargin == 3
   SettlingTimeThreshold = 2;
elseif SettlingTimeThreshold < 0.1
   error('SettlingTimeThreshold in percent must be larger than 0.1.')
end

% Rise Time to Wn circle radius
Wn = 1.8/RiseTime;

% SettlingTime to distance to the left in the s-plane
ZetaWn = -log(SettlingTimeThreshold/100)/SettlingTime;

% Overshoot to Zeta and Phi
s = Overshoot/100;
Zeta = sqrt(((log(s)/pi)^2)/(1+((log(s)/pi)^2)));
Phi = acos(Zeta);

% get scale for axis
xyscale = ceil(1.2*max(Wn,ZetaWn));

% create Wn half circle data
theta = linspace(pi/2,3*pi/2,180);
xcirc = Wn*cos(theta);
ycirc = Wn*sin(theta);

% create ZetaWn line data
xZWn = -ZetaWn+zeros(1,2);
yZWn = xyscale*[-1 1];

% create Overshoot line data
xPhi = [-xyscale 0 -xyscale];
y = xyscale*tan(Phi);
yPhi = [-y 0 y];

if nargout == 1 % create output variable if it was requested
   Out.Wn = Wn;
   Out.ZetaWn = ZetaWn;
   Out.Zeta = Zeta;
   Out.Phi = Phi*180/pi;
end

% plot all the data at once
plot(xcirc,ycirc,xZWn,yZWn,xPhi,yPhi)

% put the data on the plot
sTr = sprintf('T_r = %g',RiseTime);
sTs = sprintf('T_s = %g',SettlingTime);
sOS = sprintf('%%OS = %g',Overshoot);

sWn = sprintf('   \\omega_n = %g',round(Wn,3,'significant'));
sZWn = sprintf('\\zeta\\cdot\\omega_n = %g',round(ZetaWn,3,'significant'));
sZeta = sprintf('     \\zeta = %g',round(Zeta,3,'significant'));
text(0.2*xyscale,0.98*xyscale,...
   {sTr sTs sOS ' ' sWn sZWn sZeta},...
   'HorizontalAlignment','left',...
   'VerticalAlignment','top')

% mess with the axes to make it look right
set(gca,...
   'XAxisLocation','origin',...
   'YAxisLocation','origin',...
   'Box','off',...
   'DataAspectRatio',[1 1 1],...
   'XLim',[-xyscale xyscale/5],...
   'YLim',yZWn)

