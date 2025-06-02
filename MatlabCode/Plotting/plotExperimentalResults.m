function plotExperimentalResults(matFileName)
%RUN_ALL_PLOTS Load .mat file and generate all relevant plots
%
%   RUN_ALL_PLOTS(matFileName) loads the .mat file and generates the plots:
%   Control outputs, standard data, CBF data, CBF differences,
%   h function values, gradient of h, individual velocity correction,
%   velocity differences, velocity comparison, Von Mises maximum,
%   stress differences, and stress comparisons.

% Validate input
arguments
    matFileName (1,:) char {mustBeFile(matFileName)}
end

% Load .mat file
data = load(matFileName);
disp(['Loaded data from: ', matFileName]);

%% Plot Control Outputs
if isfield(data, 'plot_ControlOutputs_3D') && data.plot_ControlOutputs_3D
    plot_control_outputs();
end

%% Plot Standard Data
if isfield(data, 'SimData2Struct_3DControl')
    SimData2Struct_3DControl();
end
if isfield(data, 'plot_CResults_3D') && data.plot_CResults_3D
    plotControlResults(data.data_standard);
end

%% Plot CBF Data
if isfield(data, 'plot_CResults_CBF') && data.plot_CResults_CBF
    data_cbf = data.data_standard;
    % Copy fields dynamically
    fieldsToCopy = {'plotTitleSufix','ps','nvs','vs','gammas_H','gammas_G','egs',...
                    'ess','eths','eth_xs','eth_ys','eth_zs'};
    for f = fieldsToCopy
        if isfield(data, [f{1}, '_cbf'])
            data_cbf.(f{1}) = data.([f{1}, '_cbf']);
        end
    end
    plotControlResults(data_cbf);
end

%% Plot CBF Differences
if isfield(data, 'plot_CBF_Conditions') && data.plot_CBF_Conditions
    figure;
    hold on;
    plot(max(data.bs),'r','LineWidth',2,'LineStyle',':','DisplayName','b_{max}');
    plot(min(data.bs),'b','LineWidth',2,'LineStyle',':','DisplayName','b_{min}');
    plot(mean(data.bs),'k','LineWidth',2,'LineStyle',':','DisplayName','b_{mean}');
    plot(max(data.As),'r','LineWidth',2,'DisplayName','(Au)_{max}');
    plot(min(data.As),'b','LineWidth',2,'DisplayName','(Au)_{min}');
    plot(mean(data.As),'k','LineWidth',2,'DisplayName','(Au)_{mean}');
    title('A x u_{cbf} <= b || gradH <= \alpha x h');
    xlabel('Iteration');
    ylabel('Unitless');
    legend('show');
    grid on;
    hold off;
end

%% Plot h Function Values
if isfield(data, 'plot_Hs_CBF') && data.plot_Hs_CBF
    max_stresses = ones(size(data.vm_stresses_cbf))*(data.yield_stress/data.SF);
    hs = max_stresses - data.vm_stresses_cbf;
    figure;
    plot(hs);
    title('CBF h - (\sigma_{max} - \sigma_{curr})');
    xlabel('Iteration');
    ylabel('VM Stress (Pa)');
    grid on;

    figure;
    hold on;
    plot(max(hs),'r','LineWidth',2,'DisplayName','h_{max}');
    plot(min(hs),'b','LineWidth',2,'DisplayName','h_{min}');
    plot(mean(hs),'k--','LineWidth',2,'DisplayName','h_{mean}');
    title('CBF h - (\sigma_{max} - \sigma_{curr})');
    xlabel('Iteration');
    ylabel('VM Stress (Pa)');
    legend('show');
    grid on;
    hold off;
end

%% Plot Gradient of h
if isfield(data, 'plot_GradsH_CBF') && data.plot_GradsH_CBF
    hs = ones(size(data.vm_stresses_cbf))*(data.yield_stress/data.SF) - data.vm_stresses_cbf;
    figure;
    plot(data.grads_hs);
    title('Grad_h - Au_{cbf}');
    xlabel('Iteration');
    ylabel('VM Stress (Pa)');
    grid on;

    figure;
    hold on;
    plot(max(hs),'r','LineWidth',2,'LineStyle',':','DisplayName','h_{max}');
    plot(min(hs),'b','LineWidth',2,'LineStyle',':','DisplayName','h_{min}');
    plot(mean(hs),'k--','LineWidth',2,'LineStyle',':','DisplayName','h_{mean}');
    plot(max(data.grads_hs),'r','LineWidth',2,'DisplayName','gradh_{max}');
    plot(min(data.grads_hs),'b','LineWidth',2,'DisplayName','gradh_{min}');
    plot(mean(data.grads_hs),'k','LineWidth',2,'DisplayName','gradh_{mean}');
    title('CBF h vs grad_h');
    xlabel('Iteration');
    ylabel('VM Stress (Pa)');
    legend('show');
    grid on;
    hold off;
end

%% Plot Individual Velocity Correction
if isfield(data, 'plot_CBF_VelCorr') && data.plot_CBF_VelCorr
    plotVelocityCorrections(data);
end

%% Plot Velocity Differences
if isfield(data, 'plot_CBF_VelDiff') && data.plot_CBF_VelDiff
    plotVelocityDifferences(data);
end

%% Plot Velocity Comparison
if isfield(data, 'plot_VelComp') && data.plot_VelComp
    figure;
    hold on;
    plot(data.nvs_3D(1,:));
    plot(data.nvs_cbf(1,:));
    if data.use_real_CBF
        plot(data.prop_nvs_cbf(1,:));
    end
    hold off;
    title('Velocity Norm Comparison');
    xlabel('Iteration');
    ylabel('m/s');
    if data.use_real_CBF
        legend({'Agent 1 - std', 'Agent 1 - cbf', 'Agent 1 - Proposed cbf'});
    else
        legend({'Agent 1 - std', 'Agent 1 - cbf'});
    end
end

%% Plot Von Mises Maximum
if isfield(data, 'plot_VonMisesMax') && data.plot_VonMisesMax
    figure;
    plot(max(data.vm_stresses_cbf),'b','LineWidth',2);
    title('Maximum Von Mises per Iteration');
    xlabel('Iteration');
    ylabel('von Mises Stress (Pa)');
    legend('MaxVonMises');
    grid on;
end

%% Plot Stress Differences
if isfield(data, 'plot_VMStressDiff') && data.plot_VMStressDiff
    vm_diff_matrix = reshape(data.vonMises_diffs, [], data.niters);
    [NodeGrid, IterGrid] = meshgrid(1:size(data.vonMises_diffs,1), 1:data.niters);
    figure;
    surf(IterGrid, NodeGrid, vm_diff_matrix');
    xlabel('Iteration');
    ylabel('Node Index');
    zlabel('Stress Difference (Pa)');
    title('3D Stress Difference Surface');
    shading interp;

    figure;
    plot(max(data.vonMises_diffs,[],1),'r','LineWidth',2); hold on;
    plot(min(data.vonMises_diffs,[],1),'b','LineWidth',2);
    plot(mean(data.vonMises_diffs,1),'k--','LineWidth',2);
    xlabel('Iteration');
    ylabel('Stress Difference (Pa)');
    legend({'Max','Min','Mean'});
    title('Aggregate Stress Differences');
    hold off;
end

%% Plot VM Stress Comparison
if isfield(data, 'plot_VMStressComp') && data.plot_VMStressComp
    figure;
    hold on;
    plot(max(data.vm_stresses_cbf),'b','LineWidth',2,'DisplayName','CBF - Max VMStress');
    plot(repelem(data.yield_stress/data.SF,size(data.vm_stresses_cbf,2)),'r','LineWidth',2,'DisplayName','Yield Stress');
    plot(max(data.pred_vmStress),'g','LineWidth',2,'DisplayName','CBF - Max Pred VMStress');
    plot(max(data.vm_stresses_3D),'Color',[1 0.5 0],'LineWidth',2,'DisplayName','3D Control - Max VMStress');
    xlabel('Iteration');
    ylabel('VM Stress');
    title('Von Mises Stress Comparison');
    legend('show');
    grid on;
    hold off;
end

disp('All requested plots have been generated.');

end
