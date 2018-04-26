function G=ece414planttf(Month,Day,Alpha)
%ECE414PLANTTF return plant transfer function for design use
% G = ECE414PLANTTF(Month,Day,Alpha) returns the plant transfer function in
% zpk format based on the Month and Day of your birth.
% 0 <= Alpha <= 1 changes the output to reflect uncertainty in the plant.

% Duane Hanselman, 2018/03/20

if ~isnumeric(Month) || numel(Month)>1 || ...
              Month~=fix(Month) || Month<1 || Month>12
   error('Month must be a scalar between 1 and 12.')
end
if ~isnumeric(Day) || numel(Day)>1 || ...
              Day~=fix(Day) || Day<1 || Day>31
   error('Day must be a scalar between 1 and 31.')
end
if ~isnumeric(Alpha) || numel(Alpha)>1
   error('Alpha must be a scalar between 0 and 1')
end
Alpha = min(max(Alpha,0),1); % limit alpha range

Wn = 4*sqrt(125:25:400)/pi;
idxn = [6 3 11 7 8 5 1 2 4 9 10 12];
Z = linspace(0.3,0.8,12);
idxz=[10 4 5 2 9 7 3 11 6 12 1 8];

P = 20 + 12*sin(1:31);
wn = Wn(idxn(Month))*(1+ cos(2*pi*Alpha)/6);
z = Z(idxz(Month))*(1 + sin(2*pi*Alpha)/8);

p1 = P(Day) + 5*(Alpha-1);
k = 16+p1;
p = [-p1;roots([1 2*z*wn wn^2])];
G = zpk([],p,k);