% +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% ++++++++++++++++++++++++++++++++ JACOBIAN +++++++++++++++++++++++++++++++
% +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

% Load the data
% load('.\Experiments\Forces\MatlabInteractiveTests\JacobianTests.mat');
load('.\Experiments\Forces\MatlabInteractiveTests\JacobianUnitaryCube.mat');

% b = J*A

% Number of particles
N = length(particle_displacements)*3; % N*3 (x,y,z)
% Number of experiments
S = n_experiments;
% Number of Agents
Na = size(agent_actions,2); % A*3

% Action Inputs
A = zeros(N*S, Na*N); 

for s = 0:S-1
    for np = 0:N-1
        A((s*N)+(np+1),(np*Na)+1:(np*Na)+Na) = agent_actions(s+1,:);
    end
end

A = sparse(A);

% Outputs
b = [];
for i = 1:n_experiments
    slice = particle_displacements(:,:,i);

    slice = reshape(slice', [3*length(particle_displacements), 1]);

    b = [b;slice];
end


% J = (A' * A)^-1 * A' * b

% We try to compute the pseudo-inverse of A to compute the Jacobian
try
    % Fastest way
    Jvect = lsqminnorm(A,b);
catch
    % If not possible we try the traditional way
    invA = (A' * A)^-1 * A'; % It can result on a NaN value

    if any(isnan(invA(:)))

        try % We try the Moore-Penrose pseudo-inverse way
            invA = pinv(A);
            Jvect = invA * b;

        % If we run out of memory
        catch
            error("Not possible to generate the inverse of A");
        end
    
    % If invA has not NaN values we can compute the jacobian normally
    else
        Jvect = invA * b;
    end

end



J = reshape(Jvect, [N,Na]);

% save('.\Experiments\Forces\MatlabInteractiveTests\JacobianTests.mat',"J", "-append");
% save('.\Experiments\Forces\MatlabInteractiveTests\JacobianUnitaryCube.mat',"J", "-append");

%% ***** Tests *****
% Testing the first experiment
mean_error = 0;
for exp = 1:S
    % Jacobian Operation
    slice_b = J * agent_actions(exp,:)';
    
    % expected solution
    exp_slice_b = particle_displacements(:,:,exp);
    
    exp_slice_b = reshape(exp_slice_b', [3*length(particle_displacements), 1]);
    
    % Error
    e = abs(exp_slice_b - slice_b);
    curr_error = mean(e);
    mean_error = mean_error + curr_error;
    disp(strcat("Experiment ", num2str(exp), " has an average error of ",num2str(curr_error)));
    disp(max(e));
    histogram(e');

    
end
mean_error = mean_error/S;
disp(newline);
disp(strcat("There is an average error of ",num2str(mean_error)));