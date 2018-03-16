function [qstar, error, exitflag] = ikunceff(robot, T, q0, options)

    % check if Optimization Toolbox exists, we need it
    if ~exist('fminunc')
        error('rtb:ikunc:nosupport', 'Optimization Toolbox required');
    end
    
    % create output variables
    T_sz = size(T,3);
    qstar = zeros(T_sz,robot.n);
    error = zeros(T_sz,1);
    exitflag = zeros(T_sz,1);
    
    problem.solver = 'fminunc';
    problem.x0 = zeros(1, robot.n);
    problem.options = optimoptions('fminunc', ...
        'Algorithm', 'quasi-newton', ...
        'Display', 'off'); % default options for ikunc
    
    if nargin > 2
        problem.x0 = q0;
    end
    if nargin > 3
        problem.options = optimset(problem.options, options);
    end
    
    for t = 1:T_sz
        problem.objective = ...
            @(x) sumsqr(([T(:,:,t).t]-[robot.fkine(x).t]));
        
        [q_t, err_t, ef_t] = fminunc(problem);
        
        qstar(t,:) = q_t;
        error(t) = err_t;
        exitflag(t) = ef_t;
        
        problem.x0 = q_t;
    end
    
end

function s = sumsqr(A)
    s = sum(sum(A.^2));
end