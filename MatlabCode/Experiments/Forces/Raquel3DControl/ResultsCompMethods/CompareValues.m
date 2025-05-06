function [max_error,min_error,mean_error,std_error] = CompareValues(error_val,tobePrinted,var_name,method_name)
    % Optional parameters handling
    if ~exist("tobePrinted","var")
        tobePrinted = false;
    end
    if ~exist("var_name","var")
        var_name = "var_name";
    end
    if ~exist("method_name","var")
        method_name = "method_name";
    end

    % If the errors are different from 0
    if(nonzeros(error_val))
        % Compute max, min, mean and std
        max_error = max(error_val,[],"all");
        min_error = min(error_val,[],"all");
        [std_error,mean_error] = std(error_val,0,"all","omitmissing");
        if tobePrinted
            disp(' ');
            disp(strcat("[❌]",var_name, " results differ in ", method_name));
            disp(strcat("Maximum ", var_name," error - ", method_name, " = ", num2str(max_error)));
            disp(strcat("Minimum ", var_name," error - ", method_name, " = ", num2str(min_error)));
            disp(strcat("Mean ", var_name," error - ", method_name, " = ", num2str(mean_error)));
            disp(strcat("Standard Deviation ", var_name," error - ", method_name, " = ",num2str(std_error)));
        end
    else
        if tobePrinted
            disp(strcat("[✔]",var_name, " results are similar in ", method_name));
        end
        max_error = 0; min_error = 0; std_error = 0; mean_error = 0;
    end
end