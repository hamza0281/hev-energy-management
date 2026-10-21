function use_cycle(name, reps)
% use_cycle('UDDS',2) puts UDDS repeated twice into data/cycle.mat
load(fullfile('data',['cycle_' name '.mat']),'cycle');
c = cycle; N = size(c,1); out = c;
for k = 2:reps
    out = [out; c(:,1) + (k-1)*N, c(:,2)];
end
cycle = out;
save(fullfile('data','cycle.mat'),'cycle');
fprintf('cycle.mat = %s x%d (%d s)\n', name, reps, cycle(end,1));
end