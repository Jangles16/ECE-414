function s=rlocusdata(varargin)
%RLOCUSDATA computes K > 0 root locus data of interest in control classes.
%
% S = RLOCUSDATA(B,A) where A and B are ROW vectors assumes that A and B
% are polynomial coefficent vectors for the polynomials A(s) and B(s)
% respectively. That is, the characteristice equation is:
% Delta(s) = A(s) + K*B(s) = 0 or 1 + K*B(s)/A(s) = 0.
%
% S = RLOCUSDATA(Z,P,G) where Z and P are COLUMN vectors assumes that Z and
% P are the polynomial roots of B(s) And A(s) respectively. G is the gain
% term multiplying the numerator polynomial B(s). That is, the
% characteristic equation is: Delta(s) = A(s) + K*B(s) = 0
% where A(s) = poly(P) and B(s) = G*poly(Z).
% If the numerator has no zeros, then set Z = [].
%
% RLOCUSDATA(TF) uses the Control Toolbox transfer function object TF.
% RLOCUSDATA(ZPK) uses the Control Toolbox zero-pole-gain object ZPK.
%
% The output S is a structure containing fieldnames that describe the
% computed results.
%
% S.Zeros = Z roots of B
% S.Poles = P roots of A
% S.Num = B polynomial
% S.Den = A polynomial
%
% S.AsymAngle = vector containing asymptote angles if they exist.
% S.AsymSigma = vector containing asymptote real axis intersections.
%
% S.Breaks = vector containing real axis breakaway and break in points.
% S.BreakK = vector containing K values for corresponding S.Breaks values.
%
% S.jwCross = frequencies where root locus crosses the jw axis.
% S.jwCrossK = values of K corresponding to S.jwCross.
%
% S.DepAngle = vector containing departure angles from complex roots in P.
% S.DepRoot = vector of roots associated with S.DepAngle values.
% S.ArrAngle = vector containing arrival angles to complex roots in Z.
% S.ArrRoot =  vector of roots associated with S.ArrAngle values.
%
% S.Delta = function handle that computes roots of the characteristic
% equation for a single value of K,e.g., S.Delta(k) finds the roots for K.
%
% This function is for academic use. This function assumes that K > 0.

% D.C. Hanselman, University of Maine, Orono, ME 04469
% Mastering MATLAB
% 2006-03-08, 2006-05-05, 2007-02-16, 2008-02-26, 2012-02-10, 2013-02-18
% 2013-04-24

Asyms = {180,[-90 90],[-60 60 180],[-135 -45 45 135],...
        [-108 -36 36 108 180], [-120 -90 -30 30 90 120]};
s = struct('Zeros',[],'Poles',[],'Num',[],'Den',[],...
           'AsymAngle',[],'AsymSigma',[],'Breaks',[],'BreakK',[],...
           'jwCross',[],'jwCrossK',[],'DepAngle',[],'DepRoot',[],...
           'ArrAngle',[],'ArrRoot',[],'Delta',[]);
tol = 1e-7;

% See if input is an LTI control system object.
% If it is, extract data from it.
% RLOCUSDATA(TF) or TLOCUSDATA(ZPK)
if nargin==1 && (isa(varargin{1},'tf') || isa(varargin{1},'zpk'))
  L = varargin{1};
  [num,den] = tfdata(L);
  s.Num = num{1};
  s.Den = den{1};
  
elseif nargin==2 % RLOCUSTDATA(B,A)
  s.Num = varargin{1};
  s.Den = varargin{2};
  An = s.den(1);
  if An~=1
    s.Den = s.Den/An;
    s.Num = s.Num/An;
  end
  L = tf(s.Num,s.Den);
  
elseif nargin==3 % RLOCUSDATA(Z,P,G)
  s.Num = varargin{3}*poly(varargin{1});
  s.Den = poly(varargin{2});
  L = zpk(varargin{:});
  
else
  error('Too Few or Too Many Input Arguments.')
end
% do pole-zero cancelation if needed
[s.Num,s.Den] = local_minreal(s.Num,s.Den);
s.Zeros = roots(s.Num);
s.Poles = roots(s.Den);

% make sure s.Num and s.Den have the same length
s.Num = [zeros(1,length(s.Den)-length(s.Num)) s.Num];

% make sure there are NOT more zeros than poles
PZexcess = length(s.Poles) - length(s.Zeros); % pole zero excess
if PZexcess<0
   error('Denominator must contain at least as many roots as numerator.')
end

% make sure this is a K>0 root locus
if sign(s.Num(find(s.Num,1))) ~= sign(s.Den(find(s.Den,1)))
   error('K < 0 Root Locus Not Supported.')
end

% find asymptotes if they exist
if PZexcess>6
   warning('rlocusdata:NA',...
      'Not equiped to compute asymptotes for this problem.')
elseif PZexcess>0
   s.AsymAngle = Asyms{PZexcess};
   s.AsymSigma = real(sum(s.Poles)-sum(s.Zeros))/PZexcess;
end

% find breakaway points and breakin points if they exist
[q,tmp] = polyder(-s.Den,s.Num); %#ok
tmp = roots(q);
s.Breaks = tmp(abs(imag(tmp))<tol); % only real roots can be breakaway pts.
s.Breaks = real(s.Breaks);
s.BreakK = polyval(-s.Den,s.Breaks)./polyval(s.Num,s.Breaks);
s.Breaks = s.Breaks(s.BreakK>0);
s.BreakK = s.BreakK(s.BreakK>0);
% eliminate any spurious ones not on root locus
isok = abs(1 + s.BreakK.*polyval(s.Num,s.Breaks)./...
                          polyval(s.Den,s.Breaks))<tol;
s.Breaks = s.Breaks(isok);
s.BreakK = s.BreakK(isok);

% find out if root locus goes into RHP, if so, find crossing
if any(real(s.Zeros)>=0) ||...                    % zero in RHP
   any(real(s.Poles)>=0) ||...                    % pole in RHP
   (~isempty(s.AsymSigma) && s.AsymSigma>0) ||... % asym intesect in RHP
   PZexcess>2                                     % asym angle points to RHP
 
   [~,~,wout] = bode(L); % get appropriate frequency axis limits
   wout = log10([wout(1),wout(end)]); % power of ten frequency axis limits
   wout = logspace(wout(1),wout(2),1000); % get detail on a finer scale
   [~,phase] = bode(L,wout); % recompute frequency response
   phase = squeeze(phase);
   s.jwCross = invinterp1(wout,abs(phase),180); % find jw axis crossings
   s.jwCrossK = squeeze(1./abs(freqresp(L,s.jwCross)));

end
% find angles of departure and arrival if they exist
if any(imag(s.Poles)>tol) % angle of departure exists
  
   s.DepRoot = s.Poles(imag(s.Poles)>tol);
   for k = 1:length(s.DepRoot)
      cr = s.DepRoot(k);
      dd = deconv(s.Den,[1 -cr]);
      a = -180 + 180/pi*(angle(polyval(s.Num,cr)) - angle(polyval(dd,cr)));
      s.DepAngle(k) = mod(a+180,360)-180;
   end
end
if any(imag(s.Zeros)>tol) % angle of arrival exists
  
   s.ArrRoot = s.Zeros(imag(s.Zeros)>tol);
   for k = 1:length(s.ArrRoot)
      cr = s.ArrRoot(k);
      dd = deconv(s.Num,[1 -cr]);
      a = 180 + 180/pi*(angle(polyval(s.Den,cr)) - angle(polyval(dd,cr)));
      s.ArrAngle(k) = mod(a+180,360)-180;
   end

end
% create function handle to characteristic equation
A = s.Den;
B = s.Num;
s.Delta = @(K) roots(A+K(1)*B); % simple function for finding roots given k

function [nm,dm]=local_minreal(num,den)
tol = 1e-5;
tol0 = sqrt(eps);

num = num(find(num,1):end);
den = den(find(den,1):end);
z = roots(num);
p = roots(den);
nz = length(z);
zm = true(nz,1);
for i=1:nz
	if abs(z(i)>tol0)
		TOL = tol*abs(z(i));
	else
		TOL = tol;
	end
	match = find(abs(p-z(i))<=TOL);
	if ~isempty(match)
		p(match(1)) = []; % throw out matching pole
		zm(i) = false;    % flag zero for elimination
	end
end
z = z(zm);
nm = real(num(1)*poly(z)/den(1));
dm = real(poly(p));

function [xo,yo]=invinterp1(x,y,yo)
%INVINTERP1 1-D Inverse Interpolation.
% [Xo, Yo]=INVINTERP1(X,Y,Yo) linearly interpolates the vector Y to find
% the scalar value Yo and returns all corresponding values Xo interpolated
% from the X vector. Xo is empty if no crossings are found. For
% convenience, the output Yo is simply the scalar input Yo replicated so
% that size(Xo)=size(Yo).
% If Y maps uniquely into X, use INTERP1(Y,X,Yo) instead.

x=x(:); % stretch input vectors into column vectors
y=y(:);
n = numel(y);
if yo<min(y) || yo>max(y) % quick exit if no values exist
   xo = [];
   yo = [];
else                      % find the desired points
   below = y<yo;          % True where below yo 
   above = y>yo;          % True where above yo
   on = y==yo;            % True where on yo   
   kth = (below(1:n-1)&above(2:n))|(above(1:n-1)&below(2:n)); % point k
   kp1 = [false; kth];                                        % point k+1
   
   if any(kth)
      alpha = (yo - y(kth))./(y(kp1)-y(kth));% distance between x(k+1) and x(k)
      xo = alpha.*(x(kp1)-x(kth)) + x(kth);  % linearly interpolate using alpha
      xo = sort([xo;x(on)]);
   else
      xo = x(on);
   end
   yo = repmat(yo,size(xo)); % duplicate yo to match xo points found
end
