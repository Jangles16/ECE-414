function T=steplqr(G,ts,sp)
%STEPLQR System Prototype for LQR Optimum Step Response.
% STEPLQR(G,Ts,SP) returns a continuous time system that minimizes
%               /inf 
%           J = |( q*[r(t) - y(t)]^2 + u(t)^2 ) dt
%               /0
% where r(t) is the unit step function system input, y(t) is the controlled
% system output, q is a weighting factor, and u(t) is the plant input.
% In addition,
% G is the transfer function of the plant in either tf or zpk format.
%
% Ts is the desired settling time in seconds.
%
% SP is the settling time percentage. For example, if SP = 2, Ts specifies
% the 2% settling time. SP must be between 0.1 and 10. If SP is not given,
% SP = 2 is assumed.
%
% Example: STEPLQR(G,5,1) returns the LQR optimum prototype system T(s) for
% the plant transfer function having N and D numerator and den0ominator
% polynomials that exhibits a 5 second settling time to within 1% of the
% final value.

% Reference: C.T. Chen, Analog and Digital Control System Deisgn: Transfer
% Function, State Space, and Algebraic Methods, Saunders College
% Publishing, ISBN: 0-03-094070-2, 1993.

% Algorithm: Given G, a symmetric root locus approach is used to
% find the weighting factor q that produces the desired settling time.

% D.C. Hanselman, University of Maine, Orono, ME 04469
% Mastering MATLAB 7
% 2007-02-26

if nargin==2
   sp=2;
elseif nargin<2
   error('Two or Three Input Arguments Required.')
end
if ~isnumeric(ts) || numel(ts)>1 || ts<1e-4 || ts>1e3
   error('Ts Must be a Scalar Between 1e-4 and 1e3.')
end
if ~isnumeric(sp) || numel(sp)>1 || sp<0.1 || sp>10
   error('SP Must be a Scalar Between 0.1 and 10.')
end
if ~(isa(G,'tf')||isa(G,'zpk'))
  error('G Must be a Transfer Function in TF or ZPK Format.')
end
n = poly(zero(G));
d = poly(pole(G));
nlen = length(n);
dlen = length(d);
if nlen>dlen
   error('The Plant Must be Proper.')
end
n = [zeros(1,dlen-nlen) n];                    % pad n with zeros to match d
s = (2*rem(1:dlen,2)-1)*(-1)^(dlen-1);         % signs for -s
nn = conv(n,n.*s);                             % N(s) * N(-s);
dd = conv(d,d.*s);                             % D(s) * D(-s);

% check to see if desired ts is achievable
q=10.^(-6:8);  % trial values for q
tsa=zeros(size(q));
for k=1:length(q)
   tsa(k)=findq(q(k),n,nn,dd,0,sp);
end
if all(tsa>ts)
   error('Chosen Settling Time Ts is too Small for this Plant.')
elseif all(tsa<ts)
   error('Chosen Settling Time Ts is too Large for this Plant.')
else
   qlim=[q(find(ts>tsa,1)) q(find(ts<tsa,1,'last'))]; % bound on q
end

fun=@(q) findq(q,n,nn,dd,ts,sp);    % objective function

options=optimset('TolX',1e-4);
q=fzero(fun,qlim,options);          % iterate q to find desired ts

[err,No,Do]=fun(q);                 %#ok<ASGLU> % No and Do at solution

% Now have solution in transfer function form

T = zpk(tf(No,Do));


%--------------------------------------------------------------------------
function [err,No,Do]=findq(q,n,nn,dd,ts,sp)
% iterative function to find q that matches desired settling time

pp=roots(dd + q*nn);
p=pp(real(pp)<0);
Do=poly(p);
No=n*Do(end)/n(end);

np=length(p);
tend=2*log(sp/100)/max(real(p));      % estimate final time for computation
t=linspace(0,tend,500)';
modes=zeros(length(t),np);               % storage for modes evaluated at t
r=zeros(np,1);
nk=1:np;
for k=1:np
   r(k)=polyval(No,p(k))./(p(k)*prod(p(k)-p(nk~=k)));        % k-th residue
   modes(:,k)=exp(p(k)*t);             % k-th mode evaluated at time points
end
y=1+real(modes*r);                 % system output is matrix multiplication

idx1=find(abs(y-1)>abs(sp/100),1,'last');
if y(idx1)>1                     % settling time using linear interpolation
   alpha=(y(idx1)-(1+sp/100))/(y(idx1)-y(idx1+1));
   tsa=t(idx1)+alpha*(t(idx1+1)-t(idx1));
else
   alpha=((1-sp/100)-y(idx1))/(y(idx1+1)-y(idx1));
   tsa=t(idx1)+alpha*(t(idx1+1)-t(idx1));
end
err=tsa-ts;