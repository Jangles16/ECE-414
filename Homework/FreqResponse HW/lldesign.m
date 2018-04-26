function [D,L,T]=lldesign(G,PM,Wg,Ess)
%LLDESIGN Lead or Lag Controller Design.
% D = LLDESIGN(G,PM,Wg,Ess) returns a lead or lag controller D for the
% plant G that produces a phase margin PM in degrees at the gain crossover
% frequency Wg and a steady state error of Ess.
% 
% D is returned in ZPK Control Toolbox format:
%          s + z
% D(s) = K -----
%          s + p
%
% G describes the plant and is either in ZPK or TF Control Toolbox format.
% If G is type 0, Ess is the steady state error due to a unit step input.
% If G is type 1, Ess is the steady state error due to a unit ramp input.
%
% PM is the positive phase margin that is to occur
% at the gain crossover frequency Wg in radians per second.
%
% If Ess is missing or zero LLDESIGN(G,PM,Wg) or LLDESIGN(G,PM,Wg,0)
% the controller D does not change the steady state error.
%
% [D,L,T] = LLDESIGN(...) in addition returns
% L = L(s) = D(s)*G(s) = loop transfer function in ZPK format, and
% T = T(s) = L(s)/(1 + L(s)) = closed loop transfer function

% Reference:
% "Analytical Solutions for Lag/Lead and General Second Order Compensator
% Design Problems," Feu-Yue Wang, Int. Journal of Intelligent Control and
% Systems, vol. 13, no. 4, Dec. 2008, pp. 233-236.

% D. Hanselman, University of Maine
% 2013/03/21

% Error checking
if nargin ==3
   Ess = 0;
elseif nargin == 4
   if ~ismember(class(G),{'zpk','tf','ss'}) % see if first input is G
      error('G must be a transfer function object.')
   end
   if ~issiso(G)
      error('G must be an SISO system.')
   end
   if ~isproper(G)   % if G is proper
      error('G must be proper.')
   end
   if ~isreal(G) % to see if the plant is real
      error('The numerator or denominator of G must have real coefficients.')
   end
   if ~isscalar(Ess) || Ess<0 %checking for a numeric value
      error('Ess must be a positive numeric value.')
   end
   if ~isscalar(PM) || PM<=0 
      error('PM must be a positive value in degrees.')
   end
   if ~isscalar(Wg) || Wg<=0
      error('Wg must be a positive value in radians/s.')
   end
else
   error('3 or 4 input arguments required.')
end

if Ess>0 % find gain term K that will give the desired Ess
   switch sum(pole(G)==0); % Number of plant poles at origin
      case 0               % Type 0 Plant
         Kg = dcgain(G);   % DC gain
         K = (1/Ess - 1)/Kg;
      case 1               % Type 1 Plant
         sG = minreal(G*zpk(0,[],1));
         Kg = dcgain(sG);
         K = 1/(Ess*Kg);
      otherwise
         error('G must be Type 0 or Type 1.')
   end
else
   K = 1;
end

GatWg  = evalfr(G,Wg*1i);        % plant response at Wg
GatWgmag = abs(GatWg);           % plant magnitude at Wg
GatWgph = (180/pi)*angle(GatWg); % plant angle at Wg in degrees

% phase of D(s) at Wg
p = PM - 180 - GatWgph;
% parameters based on p
d = tand(p);
dd = sqrt(1 + d^2);

% gain of D(s) at Wg
c = 1/(K*GatWgmag);

% compute controller parameters
T = (c-dd)/(c*d*Wg);
% a = c*(c*dd - 1)/(c-dd);
aT = c*(c*dd - 1)/(c*d*Wg); % compute a*T directly to avoid (c-dd) term

% see if this design is feasible
% required |phase| must be less than ~85 degrees (90 is absolute max)
% coefficients must be positive so zero and pole are in left half plane
if abs(p)>85 || T<0 || aT<0
   error('No solution possible for given inputs.')
end

% create controller transfer function in zpk format
D = tf(K*[aT 1],[T 1]);
D = zpk(D);

% loop transfer function
L = zpk(D*G);

% compute closed loop transfer function
T = feedback(L,1);