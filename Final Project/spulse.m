function [fun,dfun,ifun] = spulse(T,F,Type)
%SPULSE Pulse function with sinusoidal transitions.
% Fun = SPULSE(T,F,Type) returns a function handle that evaluates a pulse
% having sinusoidal transitions of a given Type.
%
% T = [t1 t2 t3 t4] transition time points
%      t1 < t2 < t3 < t4 is required.
% F = Pulse amplitude
%
% For all t < t1 the function is 0.
%
% Between t = t1 and t2 the function transitions from zero to F
%
% Between t = t2 and t3 the output is equal to F
%
% Between t = t3 and t4 the output transitions from F back to zero.
%
% For all t > t4 the output is zero.
%
% If Type is Zero or False:
% The transition from zero to F follows a rising sine wave where
% the slope at t2 is zero.
% The transition from F to zero follows a falling sine wave where
% the slope at t4 is zero.
%
% If Type is Not Zero or True:
% The transition from zero to F follows one half of a period of a sine wave.
% The slope at t1 and t2 are zero.
% The transition from F to zero follows one half of a period of a sine wave.
% The slope at t3 and t4 are zero.
%
%
% [Fun,dFun,iFun] = spulse(T,F) in addition returns
% a function handle in dFun that evaluates the derivative of Fun(t), and
% a function handle in iFun that evaluates the integral of Fun(t).

% D.Hanselman, University of Maine, 2016/4/29

if nargin ~= 3
  error('Three input arguments required.')
end
if ~isnumeric(T)||numel(T) ~=4
  error('T must have 4 numeric elements.')
elseif ~isnumeric(T)||numel(F) ~=1
  error('F must be a scalar numeric value.')
end
dt = diff(T(:));
if any(dt<0)
  error('Elements in T must be increasing.')
end
if numel(Type)~=1
  error('Type must be a scalar.')
end
if isnumeric(Type) % convert to logical
  Type = Type~=0;
end
if Type                 % 1/2 period sinusoidal transitions
  
  To1 = 2*dt(1);
  Wo1 = 2*pi/To1;
  To2 = 2*dt(3);
  Wo2 = 2*pi/To2;
  
  fun = @(t) p2fun(t);
  dfun = @(t) dp2fun(t);
  ifun = @(t) ip2fun(t);
else                    % 1/4 period sinusoidal transitions
  
  To1 = 4*dt(1);
  Wo1 = 2*pi/To1;
  To2 = 4*dt(3);
  Wo2 = 2*pi/To2;
  
  fun = @(t) p4fun(t);
  dfun = @(t) dp4fun(t);
  ifun = @(t) ip4fun(t);
end

% 1/2 period functions

  function out = p2fun(t)
    out = zeros(size(t));
    
    idx = T(1)<t & t<T(2);
    out(idx) = F*(1 + sin(Wo1*(t(idx) - T(1))-pi/2))/2;
    
    idx = T(2)<=t & t<=T(3);
    out(idx) = F;
    
    idx = T(3)<t & t<T(4);
    out(idx) = F*(1 + cos(Wo2*(t(idx)-T(3))))/2;
  end
  function out = dp2fun(t)
    out = zeros(size(t));
    
    idx = T(1)<=t & t<T(2);
    out(idx) = Wo1*F*cos(Wo1*(t(idx) - T(1))-pi/2)/2;
    
    idx = T(3)<=t & t<T(4);
    out(idx) = -Wo2*F*sin(Wo2*(t(idx)-T(3)))/2;
  end

  function out = ip2fun(t)
    out = zeros(size(t));
    
    idx = T(1)<t & t<=T(2);
    out(idx) = F*(t(idx)-T(1) - (sin(Wo1*(t(idx)-T(1)))/Wo1))/2;
    
    idx = T(2)<t & t<=T(3);
    F1 = F*dt(1)/2;
    out(idx) = F1 + F*(t(idx)-T(2));
    
    idx = T(3)<t & t<T(4);
    F2 = F1 + F*dt(2);
    out(idx) = F2 + F*(t(idx)-T(3))/2 + F*sin(Wo2*(t(idx)-T(3)))/(Wo2*2);
    
    idx = t>=T(4);
    out(idx) = F2 + F*dt(3)/2;
  end

% 1/4 period functions
  function out = p4fun(t)
    out = zeros(size(t));
    
    idx = T(1)<t & t<T(2);
    out(idx) = F*sin(Wo1*(t(idx) - T(1)));
    
    idx = T(2)<=t & t<=T(3);
    out(idx) = F;
    
    idx = T(3)<t & t<T(4);
    out(idx) = F*(1 - sin(Wo2*(t(idx)-T(3))));
  end

  function out = dp4fun(t)
    out = zeros(size(t));
    
    idx = T(1)<=t & t<T(2);
    out(idx) = Wo1*F*cos(Wo1*(t(idx) - T(1)));
    
    idx = T(3)<=t & t<T(4);
    out(idx) = -Wo2*F*cos(Wo2*(t(idx)-T(3)));
  end

  function out = ip4fun(t)
    out = zeros(size(t));
    
    idx = T(1)<t & t<=T(2);
    out(idx) = 2*F/Wo1*sin(Wo1*(t(idx)-T(1))/2).^2;
    
    idx = T(2)<t & t<=T(3);
    out(idx) = F*(t(idx)-T(2)) + 2*F/Wo1*sin(Wo1*(T(2)-T(1))/2).^2;
    
    idx = T(3)<t & t<T(4);
    out(idx) = F*(t(idx)-T(3) +(cos(Wo2*(t(idx)-T(3)))-1)/Wo2)...
      + F*(T(3)-T(2)) + 2*F/Wo1*sin(Wo1*(T(2)-T(1))/2).^2;
    
    idx = t>=T(4);
    out(idx) = F*(T(4)-T(3) +(cos(Wo2*(T(4)-T(3)))-1)/Wo2)...
      + F*(T(3)-T(2)) + 2*F/Wo1*sin(Wo1*(T(2)-T(1))/2).^2;
  end

end