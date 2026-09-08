classdef PhRunRecord < handle
    % Handle-owned cleanup state remains valid during Ctrl+C stack unwinding.
    properties
        Result; Log; Count=0; IO=[]; Folder; Finished=false;
    end
    methods
        function obj=PhRunRecord(order,C,folder)
            obj.Folder=folder;
            obj.Result=struct('order',order,'config',C,'status','initializing', ...
                'hardware_swingup_verified',false);
            obj.Log=zeros(ceil(C.duration/C.dt(order))+1,12);
        end
        function finish(obj)
            if obj.Finished, return; end
            obj.Finished=true;
            if ~isempty(obj.IO), obj.IO.stop(); delete(obj.IO); obj.IO=[]; end
            if strcmp(obj.Result.status,'running'), obj.Result.status='operator_interrupted'; end
            obj.Result.log=obj.Log(1:obj.Count,:);
            obj.Result.settled=false; obj.Result.stages_visited=[];
            if obj.Count>0
                data=obj.Result.log;
                obj.Result.stages_visited=unique(data(:,10))';
                tail=data(:,1)>=data(end,1)-2;
                angleColumns=3:(2+obj.Result.order);
                rateColumns=[5 6:(5+obj.Result.order)];
                obj.Result.settled=data(end,1)>=2 && ...
                    all(abs(data(tail,angleColumns))<0.05,'all') && ...
                    all(abs(data(tail,rateColumns))<0.25,'all');
                obj.Result.hardware_swingup_verified=strcmp(obj.Result.status,'completed') && ...
                    obj.Result.settled && all(ismember(1:(obj.Result.order+1),obj.Result.stages_visited));
            end
            obj.Result.columns={'t','x','theta1','theta2','xdot','omega1','omega2', ...
                'acceleration','voltage','stage','compute_seconds','lateness_seconds'};
            obj.Result.output_file=fullfile(obj.Folder,'run.mat');
            result=obj.Result; save(result.output_file,'result');
            fprintf('Outputs stopped. Log: %s\n',result.output_file);
        end
    end
end
