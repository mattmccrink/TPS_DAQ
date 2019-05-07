function [X, dt ] = TIME_STEP(X,clkfreq)
% Convert Propeller clock counter to time steps
%   Based on 80 MHz clock speed
if nargin < 2;
    clkfreq = 96000000;
end

%Discard first 100 datapoints as they are likely corrupted by the save
%routine.
a = mod(diff(X(:,2)),2^32)/clkfreq;
il = find((a>30)|(a==0));

while ~isempty(il)
a = mod(diff(X(:,2)),2^32)/clkfreq;
il = find((a>30)|(a==0));
X(il,:) = [];
end

dt = cumsum(mod(diff(X(:,2)),2^32)/clkfreq);

end

