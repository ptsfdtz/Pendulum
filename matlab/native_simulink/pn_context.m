function value=pn_context(action,value)
% Handle registry for vendor resources; never stores controller state.
persistent current
switch action
    case 'get', value=current;
    case 'set', current=value;
    case 'clear', current=[]; value=[];
end
end
