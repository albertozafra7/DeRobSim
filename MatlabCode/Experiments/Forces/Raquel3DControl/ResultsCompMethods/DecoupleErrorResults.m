function [e_max, e_min, e_mean, e_std, e_values] = DecoupleErrorResults(var_struct,struct_field)
    e_max  = arrayfun(@(s) s.(struct_field).max, var_struct);
    e_min  = arrayfun(@(s) s.(struct_field).min, var_struct);
    e_mean = arrayfun(@(s) s.(struct_field).mean, var_struct);
    e_std  = arrayfun(@(s) s.(struct_field).std, var_struct);
    e_values = arrayfun(@(s) s.(struct_field).values, var_struct, 'UniformOutput', false);
end