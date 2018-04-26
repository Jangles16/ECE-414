function D = pidsearch(G,Di,Type,Q)
%PIDSEARCH Numerical Search for Good PID controller.
% D = PIDSEARCH(G,Di,Type) searches for a PID controller D for the plant G
% starting from the initial controller Di using the variable Type to
% determine what to Minimize. Type is one of the following character
% strings: 'ITAE', 'ISE', 'OS', 'Ts', 'OSTs', 'UTs', 'LQG'
%
% 'ITAE' is the integral of time and the absolute error:
%                              /inf
%                       ITAE = | t*|yfinal-y| dt
%                              /0
%
% 'ISTE' is the integral of the square of t*|yfinal-y|:
%                              /inf
%                       ISTE = | (t*|yfinal-y|)^2 dt
%                              /0
%
% 'ISE' is the integral of the squared error:
%                              /inf
%                        ISE = | |yfinal-y|^2 dt
%                              /0
%
% 'RISE' is the integral of the squared error starting at the Rise Time Tr
%                              /inf
%                       RISE = | |yfinal-y|^2 dt
%                              /Tr
%
% 'OS' is the step response overshoot
%
% 'Ts' is the step response setting time.
%
% 'OSTs' is the product of the overshoot and the settling time.
%
% 'UTs' is the product of the initial control effort U and the settling time.
%
% 'LQG' is the linear quadratic gaussian measure
%                              /inf
%                        LQG = | (Q*|yfinal-y|^2 + u^2) dt
%                              /0
%
% where u = unit step input, and Q is a positive scalar provided as a last
% input argument, PIDSEARCH(G,Di,Type,Q). When Q < 1 the input u weighs more
% heavily; when Q > 1 the error |yfinal-y| weighs more heavily.
%
% The plant G must have Control Toolbox tf or zpk form.
% The initial controller Di must have Control toolbox tf, zpk, or pid form. 
%
% Di can have one of the three following forms: PI, PIDf, and PDf, that can
% be expressed in factored form as
%
%      K*(s + z)            K*(s+z1)(s+z2)           K*(s+z)
% PI = ---------     PIDf = --------------     PDf = -------
%          s                   s*(s+p)                 s+p
%
% The extra pole term (s+p) in the PIDf and PDf forms appears so that the
% controllers are proper. In Control Toolbox terminology, these are
% "filtered" forms. For the PIDf form, z1 and z2 must be real values.
%
% If the plant G is Type 0, i.e., has no poles at the origin s = 0, and the
% PDf controller is chosen, the search process will keep the DC gain K*z/p
% constant so that the steady state error due to a step input is unchanged.
%
% The output D is returned in zpk form and has the same number of zeros and
% poles as Di.
%
% This function is not magic. It may or may not return good results, so
% it takes engineering to figure out what makes sense and what doesn't.
% This function uses the MATLAB optimization function FMINSEARCH to poke
% around intelligently to find a good solution starting with the intial
% controller Di. Different initial controllers Di will likely lead to
% different results.

% D. Hanselman, University of Maine, 2017/07/02

if nargin<3
   error('Three or Four Input Arguments Required.')
end
if ~(ismember(class(G),{'zpk','tf'})&&isproper(G))
   error('Input G Must be a Proper Transfer Function Object.')
end
if ~(ismember(class(Di),{'zpk','tf','pid'})&&isproper(Di))
   error('Input Di Must be a Proper Transfer Function Object.')
end
if ~ischar(Type)
   error('Input Type Must be a Character String.')
end
if nargin==3 % default Q values
   Q = [];
elseif nargin==3 && Type=='LQG'
   Q = 1;
end
if isa(Di,'pid') % If Di is a PID object convert to zpk
   Di = zpk(Di);
end
% figure out what Di is
[z,p,k] = zpkdata(Di);
if ~isreal(z{1})
   error('Complex Zeros in Di Not Allowed.')
end
% assume it is SISO, make values positive
z = abs(z{1});
p = abs(p{1});
k = abs(k(1));
if isempty(p)
   error('Controller Di Must Have at Least One Pole.')
end
DCg = []; % default value for DC gain of controller
if numel(z)==2 && numel(p)==2 % PIDf
   pidtype = 'pidf';
   p = p(p~=0); % get poles not at origin
   x0 = [z;p;k]; % vary zeros, pole not at origin, and gain
elseif numel(z)==1 && numel(p)==1
   if p==0
      pidtype = 'pi';
      x0 = [z;k]; % vary zero and gain
   else
      Gpoles = pole(G);
      Stype = any(abs(Gpoles)==0);
      if Stype>0 % Type 1 or greater plant
         pidtype = 'pdf';
         x0 = [z;p;k]; % vary zero, pole, and gain
      else % Type 0 plant
         pidtype = 'pdf0';
         DCg = abs(k*z/p); % DC gain of controller
         x0 = [z;p]; % k is chosen so DC gain is unchanged
      end
   end
else
   error('Di is not PI, PIDf, or PDf')
end
% set options for searches
options = optimset('MaxFunEvals',20000,'Display','notify');
fun = @(x) local_search(x,G,Type,pidtype,DCg,Q); % search function
x = fminsearch(fun,x0,options); % do the search

% create the final solution in zpk format
switch pidtype % put results back into controller transfer function
   case 'pi'
      D = zpk(-abs(x(1)),0,abs(x(2)));
   case 'pidf'
      D = zpk(-abs(x(1:2)),[0 -abs(x(3))],abs(x(4)));
   case 'pdf'
      D = zpk(-abs(x(1)),-abs(x(2)),abs(x(3)));
   case 'pdf0'
      k = abs(DCg*x(2)/x(1));
      D = zpk(-abs(x(1)),-abs(x(2)),k);
end
end
%------------------------------------------------------------------------
function out = local_search(x,G,Type,pidtype,DCg,Q)
switch pidtype % put guess into controller transfer function
   case 'pi'
      D = zpk(-abs(x(1)),0,abs(x(2)));
   case 'pidf'
      D = zpk(-abs(x(1:2)),[0 -abs(x(3))],abs(x(4)));
   case 'pdf'
      D = zpk(-abs(x(1)),-abs(x(2)),abs(x(3)));
   case 'pdf0'
      k = abs(DCg*x(2)/x(1));
      D = zpk(-abs(x(1)),-abs(x(2)),k);
end

T = feedback(D*G,1); % closed loop system
if isstable(T) % gather data if T is stable
   [y,t] = step(T);       % get step response
   yss = dcgain(T);       % dc gain is step response amplitude
   specs = stepinfo(y,t); % get step response specs
   e = yss-y;             % step response error
   switch Type
      case 'ITAE'
         out = trapz(t,t.*abs(e));
      case 'ISTE'
         te = t.*e;
         out = trapz(t,te.*te);
      case 'ISE'
         out = trapz(t,e.*e);
      case 'RISE'
         vals = t>specs.RiseTime;
         tre = e(vals);
         out = trapz(t(vals),tre.*tre);
      case 'OS'
         out = specs.Overshoot;
      case 'Ts'
         out = specs.SettlingTime;
      case 'OSTs'
         out = specs.Overshoot*specs.SettlingTime;
      case 'UTs'
         U = dcgain(minreal(T/G));
         out = U*specs.SettlingTime;
      case 'LQG'
         out = trapz(t,(Q*e.*e)+1);
      otherwise
         error('Unknown Input Variable ''Type''')
   end
else % not stable so panic!
   out = 100;
end
end
