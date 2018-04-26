function [Smax,Wsp]=PeakSens(L)
%PEAKSENS Peak Sensitivity.
%  Smax = PEAKSENS(L) finds the peak sensitivity for the system
%  defined by the loop transfer function L, where L is a transfer function
%  in tf, zpk, or ss form as defined by the Control Toolbox.
%
%  [Smax,Wsp] = PEAKSENS(L) also returns the frequency Wsp associated with Sp.
%
% See also: MARGIN, TF, ZPK, SS

% D. Hanselman, 04/06/2018

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

S = minreal(1/(1+L));        % sensitivity
[Smax,Wsp] = getPeakGain(S); % get peak value
