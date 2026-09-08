function output = dp_lqr_optimize(overrides)
%DP_LQR_OPTIMIZE Two-stage log-space LQR search on nonlinear dynamics.
if nargin < 1, overrides = struct(); end
P = apply_overrides(dp_config(), overrides);
rng(P.optimize.random_seed, 'twister');
[A, B, linearization] = dp_linearize(P, true);
try
    Kpaper = lqr(A, B, P.paper.Q, P.paper.R);
    paper_result = dp_evaluate_controller(Kpaper, P);
catch ME
    Kpaper = nan(1,6);
    paper_result = invalid_result(ME.message);
end

signature = sprintf(['v3_seed%d_c%d_r%d_dt%.12g_t%.12g_cases%d_' ...
    'plant_%.12g_%.12g_%.12g_%.12g_%.12g_%.12g_%.12g_%.12g'], ...
    P.optimize.random_seed, P.optimize.coarse_trials, ...
    P.optimize.refine_trials, P.optimize.dt, P.optimize.sim_time, ...
    size(P.optimize.test_initial_states,1), P.m1, P.m2, P.J1, P.J2, ...
    P.l1, P.l2, P.L1, P.L2);
checkpoint_file = fullfile(dp_output_dir(), 'lqr_opt_checkpoint.mat');
[candidates, search_params, stage, next_index] = load_or_initialize( ...
    checkpoint_file, signature, P);
fprintf('\nStarting LQR optimization: %d coarse + %d refinement trials.\n', ...
    P.optimize.coarse_trials, P.optimize.refine_trials);

while true
    stage_end = size(search_params,1);
    for idx = next_index:stage_end
        candidates(idx) = evaluate_candidate(search_params(idx,:), idx, stage, A, B, P);
        if mod(idx, P.optimize.progress_interval) == 0 || idx == stage_end
            costs = [candidates(1:idx).cost];
            [best_cost, ib] = min(costs);
            fprintf('Trial %d/%d | current %.6g | best %.6g | best maxX %.4f\n', ...
                idx, P.optimize.coarse_trials + P.optimize.refine_trials, ...
                candidates(idx).cost, best_cost, candidates(ib).max_abs_x);
        end
        if mod(idx, P.optimize.checkpoint_interval) == 0 || idx == stage_end
            next_index = idx + 1;
            save(checkpoint_file, 'signature', 'candidates', 'search_params', ...
                'stage', 'next_index', 'A', 'B', 'linearization', 'paper_result');
        end
    end

    if strcmp(stage, 'coarse') && P.optimize.refine_trials > 0
        coarse_costs = [candidates.cost];
        [~, order] = sort(coarse_costs);
        finite_order = order(isfinite(coarse_costs(order)));
        if isempty(finite_order)
            error('dp:NoValidCoarseCandidate', 'No valid controller was found in coarse search.');
        end
        top = finite_order(1:min(P.optimize.top_count,numel(finite_order)));
        refine = make_refinement(candidates(top), P);
        search_params = [search_params; refine]; %#ok<AGROW>
        candidates(end+1:size(search_params,1)) = empty_candidate();
        stage = 'refine';
        next_index = P.optimize.coarse_trials + 1;
        save(checkpoint_file, 'signature', 'candidates', 'search_params', ...
            'stage', 'next_index', 'A', 'B', 'linearization', 'paper_result');
    else
        break;
    end
end

all_scores = [candidates.cost]';
[~, best_idx] = min(all_scores);
best = candidates(best_idx);
best_Q = diag(best.q);
best_R = best.r;
best_K = best.K;
best_result = dp_evaluate_controller(best_K, P);
all_candidates = candidates;
summary_table = candidate_table(candidates);
save(fullfile(dp_output_dir(),'lqr_optimization_results.mat'), 'all_candidates', 'all_scores', ...
    'best_Q', 'best_R', 'best_K', 'best_result', 'A', 'B', ...
    'paper_result', 'Kpaper', 'linearization', 'summary_table', 'P');
writetable(summary_table, fullfile(dp_output_dir(),'lqr_optimization_summary.csv'));
write_best_controller(best_Q, best_R, best_K);
if P.optimize.make_plots, dp_plot_optimization(candidates, best_result); end
print_report(best_Q, best_R, best_K, eig(A-B*best_K), best_result, ...
    paper_result, numel(candidates));
output = struct('best_Q',best_Q,'best_R',best_R,'best_K',best_K, ...
    'best_result',best_result,'paper_result',paper_result,'A',A,'B',B, ...
    'all_candidates',candidates,'summary_table',summary_table);
end

function P = apply_overrides(P, overrides)
names = fieldnames(overrides);
for i = 1:numel(names)
    if isfield(P.optimize, names{i})
        P.optimize.(names{i}) = overrides.(names{i});
    else
        error('dp:UnknownOverride', 'Unknown optimization override: %s', names{i});
    end
end
end

function [candidates, params, stage, next_index] = load_or_initialize(file, signature, P)
if exist(file,'file') == 2
    C = load(file);
    if isfield(C,'signature') && strcmp(C.signature, signature) ...
            && isfield(C,'candidates') && isfield(C,'search_params')
        candidates = C.candidates; params = C.search_params;
        stage = C.stage; next_index = C.next_index;
        fprintf('Resuming checkpoint at trial %d.\n', next_index);
        return;
    end
end
params = latin_log_samples(P.optimize.coarse_trials, ...
    P.optimize.lower, P.optimize.upper);
candidates = repmat(empty_candidate(), size(params,1), 1);
stage = 'coarse'; next_index = 1;
end

function samples = latin_log_samples(n, lower, upper)
d = numel(lower); z = zeros(n,d);
for j = 1:d, z(:,j) = (randperm(n)' - rand(n,1)) / n; end
samples = 10.^(log10(lower) + z.*(log10(upper)-log10(lower)));
end

function samples = make_refinement(top, P)
n = P.optimize.refine_trials; d = 7; samples = zeros(n,d);
lo = log10(P.optimize.lower); hi = log10(P.optimize.upper);
rng(P.optimize.random_seed + 7919, 'twister');
for i = 1:n
    parent = top(mod(i-1,numel(top))+1);
    center = log10([parent.q parent.r]);
    proposal = center + P.optimize.refine_log10_sigma*randn(1,d);
    samples(i,:) = 10.^min(max(proposal,lo),hi);
end
end

function c = evaluate_candidate(param, trial, stage, A, B, P)
c = empty_candidate(); c.trial = trial; c.stage = stage;
c.q = param(1:6); c.r = param(7);
try
    [K,~,poles] = lqr(A,B,diag(c.q),c.r);
    if any(~isfinite(K)) || any(real(poles) >= 0)
        c.reason = 'invalid_linear_closed_loop'; return;
    end
    result = dp_evaluate_controller(K,P);
    c.K = K; c.poles = poles(:)'; c.cost = result.cost;
    c.success_count = result.success_count; c.success_rate = result.success_rate;
    c.max_abs_x = result.worst_max_abs_x; c.max_abs_u = result.max_abs_u;
    c.rms_u = result.rms_u; c.control_energy = result.control_energy;
    c.settling_time = result.settling_time; c.stable = result.stable;
    c.valid = isfinite(result.cost); c.reason = 'ok';
catch ME
    c.reason = ME.identifier;
end
end

function c = empty_candidate()
c = struct('trial',0,'stage','','q',nan(1,6),'r',nan,'K',nan(1,6), ...
    'poles',nan(1,6),'cost',Inf,'success_count',0,'success_rate',0, ...
    'max_abs_x',Inf,'max_abs_u',Inf,'rms_u',Inf,'control_energy',Inf, ...
    'settling_time',Inf,'stable',false,'valid',false,'reason','not_run');
end

function result = invalid_result(reason)
result = struct('cost',Inf,'success_count',0,'success_rate',0, ...
    'worst_max_abs_x',Inf,'max_abs_u',Inf,'rms_u',Inf, ...
    'control_energy',Inf,'settling_time',Inf,'stable',false, ...
    'cases',[],'reason',reason);
end

function T = candidate_table(c)
q = vertcat(c.q);
T = table([c.trial]', string({c.stage})', [c.cost]', q(:,1), q(:,2), ...
    q(:,3), q(:,4), q(:,5), q(:,6), [c.r]', [c.max_abs_x]', ...
    [c.max_abs_u]', [c.settling_time]', [c.stable]', ...
    'VariableNames', {'Trial','Stage','Cost','Qx','Qtheta1','Qtheta2', ...
    'Qxdot','Qtheta1dot','Qtheta2dot','R','MaxX','MaxU','SettlingTime','Stable'});
end

function write_best_controller(Q, R, K)
fid = fopen(fullfile(dp_output_dir(),'dp_best_lqr.m'),'w');
if fid < 0, error('dp:WriteBestFailed','Unable to create dp_best_lqr.m.'); end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'function L = dp_best_lqr()\n');
fprintf(fid, '%%DP_BEST_LQR Auto-generated by dp_lqr_optimize.\n');
fprintf(fid, 'L.Q = diag([%.17g %.17g %.17g %.17g %.17g %.17g]);\n', diag(Q));
fprintf(fid, 'L.R = %.17g;\n', R);
fprintf(fid, 'L.K = [%.17g %.17g %.17g %.17g %.17g %.17g];\n', K);
fprintf(fid, 'end\n');
end

function print_report(Q,R,K,poles,b,paper,ntrial)
fprintf('\n==================================================\n');
fprintf('LQR OPTIMIZATION FINISHED (%d trials)\n', ntrial);
fprintf('==================================================\n');
fprintf('Best Q diagonal:\n'); disp(diag(Q)');
fprintf('Best R: %.9g\nBest K (u = -K*X):\n',R); disp(K);
fprintf('Total cost: %.9g\nSuccess rate: %d / %d\n', ...
    b.cost,b.success_count,numel(b.cases));
fprintf('Worst max |x|: %.6f m\nMax |u|: %.6f m/s^2\n', ...
    b.worst_max_abs_x,b.max_abs_u);
fprintf('Worst settling time: %.6f s\nClosed-loop poles:\n',b.settling_time); disp(poles);
fprintf('Paper baseline: cost %.9g, maxX %.6f, maxU %.6f, settle %.6f, success %d/%d\n', ...
    paper.cost,paper.worst_max_abs_x,paper.max_abs_u,paper.settling_time, ...
    paper.success_count,numel(paper.cases));
fprintf('Candidate LQR and search results saved under matlab/output/double_pendulum; promote dp_best_lqr.m explicitly.\n');
fprintf('==================================================\n');
end
