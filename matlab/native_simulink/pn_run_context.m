function ctx=pn_run_context(order)
% A direct Run starts the selected physical pendulum hardware.
ctx=pn_context('get');
if isempty(ctx) || ctx.Finished
    ctx=PnHardwareSession(order,'hardware',inf);
    ctx.InputPreflight=true;
    pn_context('set',ctx);
end
end
