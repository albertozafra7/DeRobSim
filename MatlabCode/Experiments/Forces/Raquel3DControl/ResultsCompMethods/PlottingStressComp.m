GlobalStressNumericalComp;

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% CBF %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fields_cbf = fieldnames(errors_cbf);
for i = 1:length(fields_cbf)
    [e_max, e_min, e_mean, e_std, e_values] = DecoupleErrorResults(errors_cbf,fields_cbf{i});
    plotErrorResults(e_max, e_min, e_mean, e_std, strcat(fields_cbf{i}, " - CBF"));
end

% [e_max, e_min, e_mean, e_std, e_values] = DecoupleErrorResults(errors_cbf,"pred_u_hat");
% plotErrorResults(e_max, e_min, e_mean, e_std, strcat("pred_u_hat", " - CBF"));

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% 3D %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fields_3D = fieldnames(errors_3D);
for i = 1:length(fields_3D)
    [e_max, e_min, e_mean, e_std, e_values] = DecoupleErrorResults(errors_3D,fields_3D{i});
    plotErrorResults(e_max, e_min, e_mean, e_std, strcat(fields_3D{i}, " - 3D"));
end

% [e_max, e_min, e_mean, e_std, e_values] = DecoupleErrorResults(errors_3D,"pred_u_hat");
% plotErrorResults(e_max, e_min, e_mean, e_std, strcat("pred_u_hat", " - 3D"));