function s=margins(L)
%MARAGINS Stability Margins.
%  S = MARGINS(L) finds the stability margins associated with the system
%  defined by L, where L is a transfer function in tf, zpk, or ss form
%  as defined by the Control Toolbox.
%
%  S.Gm is the gain margin in dB
%  S.Wg is the gain crossover frequency in rad/s
%  S.Pm is the phase margin in degrees
%  S.Wp is the phase crossover frequency in rad/s
%  S.Vm is the vector margin
%  S.Wv is the vector margin frequecy in rad/s
%
% See also: MARGIN, TF, ZPK, SS

% D. Hanselman, 04/12/2013

if ~ismember(class(L),{'zpk','tf','ss'})
   error('L must be a transfer function object.')
end
if ~issiso(L)
   error('L must be an SISO system.')
end
if ~isproper(L)
   error('L must be proper.')
end
if ~isreal(L)
   error('The numerator or denominator of L must have real coefficients.')
end

warning('off','Control:analysis:MarginUnstable');
[Gm,Pm,Wgm,Wpm]= margin(L); % get traditional margins
s.Gm = 20*log10(Gm);        % save Gain margin in dB
s.Wg = Wgm;
s.Pm = Pm;
s.Wp = Wpm;
warning('on','Control:analysis:MarginUnstable');

% get vector margin

S = minreal(1/(1+L));       % sensitivity
[Smax,Wvm] = getPeakGain(S);% peak sensitivity
s.Vm = 1/Smax;              % vector margin
s.Wv = Wvm;
