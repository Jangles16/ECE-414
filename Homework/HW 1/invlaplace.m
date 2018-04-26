function fun=invlaplace(b,a)
%INVLAPLACE Time Function from Laplace Transform.
% FUN = INVLAPLACE(B,A) returns a function handle for evaluating the time
% function FUN(t) associated with the Laplace Transform B(s)/A(s), where B
% and A are the respective row vectors containing the polynomial
% coefficients.
% FUN = INVLAPLACE(TF) uses the Control Toolbox transfer function object TF.
% FUN = INVLAPLACE(ZPK) uses the Control Toolbox zero pole gain object ZPK.
% FUN = INVLAPLACE(SS) uses the Control Toolbox state space object SS.
%
% Inputs must be real valued and a single proper transfer function.
%
% FUN(t) evaluates the inverse Laplace transform at the time points in the
% array t. FUN(t) = 0 for t<0
% S = FUN('rpk') return a structure S with fields containing the residues,
% poles, and direct terms as found by the function RESIDUE. That is,
% S.r contains the residues,
% S.p contains the poles, and
% S.k contains the direct terms (that generally do not exist).
%
% This function uses the partial fraction expansion returned by the
% function RESIDUE. As a result, the accuracy of this function is limited
% by the accuracy of the function RESIDUE in MATLAB. See RESIDUE for its
% limitations.
 
% D.C. Hanselman, University of Maine, Orono, ME  04469
% 2009-02-24
% 2009-03-16
% 2012-12-17
 
if nargin==2 % INVLAPLACE(b,a)
   if ~isnumeric(b) || ~isvector(b) || ~isreal(b) || ...
      ~isnumeric(a) || ~isvector(a) || ~isreal(a)
      error('INVLAPLACE:argChk','Inputs Must be Real Valued Numeric Vectors.')
   end
   [r,p,k] = residue(b,a);
elseif nargin ~=1
   error('INVLAPLACE:argChk','One, or Two Input Arguments Required.')
elseif isa(b,'lti') % INVLAPLACE(tf) or INVLAPLACE(zpk) or zpkdata(ss)
   [b,a] = tfdata(tf(b));
   if ~isreal(b{1}) || ~isreal(a{1})
      error('INVLAPLACE:argChk','Inputs Must be Real Valued.')
   end
   if numel(a)~=1
      error('INVLAPLACE:argChk','System Must be Single Input, Single Output.')
   end
   [r,p,k] = residue(b{1},a{1});
else
   error('INVLAPLACE:argChk','Input Arguments Not Understood.')
end
if numel(k)>1
   error('INVLAPLACE:argChk','Transfer Function Must be Proper.')
end
 
fun=@(t) invlap(t); % create function handle to nested function below
 
% nested function has access to all variables created above
function y=invlap(t)
if ischar(t) && strcmpi(t,'rpk')
   y.r = r;
   y.p = p;
   y.k = k;
   return
end
if ~isnumeric(t)
   error('Numeric Input Array Expected.')
end
y = zeros(size(t)); % preallocate memory for result
t(t<0) = nan;       % throw out negative time points to avoid numerical issues
tol = 0.001;        % tolerance for repeated roots, same as used by residue
k = 1;
 
while k<=length(p)
   m = sum(abs(p(k)-p)<tol*max(abs(p(k)),1)); % multiplicity of this pole
   y = y+r(k)*exp(p(k).*t);  % basic term that always exists
   if m>1                    % multiplicity, add extra terms as needed
      for n = k+1:k+m-1
         y = y+(r(n)/prod(1:n-k))*t.^(n-k).*exp(p(k).*t);
      end
   end
   k = k+m; % next pole to consider
end
y(isnan(t)) = 0;   % output is zero for negative time
y = real(y);  % make sure result is real when poles come in complex conjugates
end % end of nested function invlap(t)
 
end % end of nesting function INVLAPLACE