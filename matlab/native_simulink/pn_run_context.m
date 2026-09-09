function ctx=pn_run_context(order)
% A direct Run starts single-rod hardware; explicit launcher sessions take priority.
ctx=pn_context('get');
if isempty(ctx) || ctx.Finished
    mode='readonly';
    if order==1, mode='hardware'; end
    ctx=PnHardwareSession(order,mode,inf);
    ctx.InputPreflight=(order==1);
    pn_context('set',ctx);
end
end
