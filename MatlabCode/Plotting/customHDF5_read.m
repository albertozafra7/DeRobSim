function [times, Matlab, Unity, unity_keys] = customHDF5_read(fullFileName)

    % Get info about the HDF5 file 
    info = h5info(fullFileName); 
    
    % Preallocate a structure to hold your data 
    data = struct();
    Unity = containers.Map();
    unity_keys = struct();
    unity_keys.Agents = containers.Map();
    unity_keys.DefObj = containers.Map();
    unity_keys.GameObj = containers.Map();
    Matlab = containers.Map();
    times = [];
    
    % Loop through each group 
    for i = 1:length(info.Groups) 
        group_name = info.Groups(i).Name; 
        data.(matlab.lang.makeValidName(group_name)) = struct(); % Create a struct for the group 
    
        % Loop through each subgroup within the group 
        for j = 1:length(info.Groups(i).Groups) 
            subgroup_name = info.Groups(i).Groups(j).Name;
            if((i == 1 && ~isempty(info.Groups(i).Groups)) || (i~=1 && isempty(info.Groups(1).Groups)))
                time_name = split(subgroup_name, '/');
                times = [times str2double(strrep(time_name(3), ',', '.'))];
            end
            
    
            data.(matlab.lang.makeValidName(group_name)).(matlab.lang.makeValidName(subgroup_name)) = struct(); % Create a struct for the subgroup          
            
            if(group_name == "/Matlab")
    
                % Loop through each dataset within the subgroup 
                for k = 1:length(info.Groups(i).Groups(j).Datasets) 
                    dataset_name = info.Groups(i).Groups(j).Datasets(k).Name;
                    % Use strcat to generate correct HDF5 path
                    dataset_path = strcat(subgroup_name, '/', dataset_name); 
        
                    
                    if(j == 1)
                        Matlab(dataset_name) = [h5read(fullFileName, dataset_path)];
                    else
                        Matlab(dataset_name) = [Matlab(dataset_name) h5read(fullFileName, dataset_path)];
                    end
                end
    
            else
                
                for k = 1:length(info.Groups(i).Groups(j).Groups)
                    subsubgroup_name = info.Groups(i).Groups(j).Groups(k).Name;
                    unityGroup_name = split(subsubgroup_name, '/');
    
                    for l = 1:length(info.Groups(i).Groups(j).Groups(k).Groups)
                        subsubgroup_name = info.Groups(i).Groups(j).Groups(k).Groups(l).Name;
                        object_name = split(subsubgroup_name, '/');
                        object_name = string(object_name(end));
                        if(~isKey(Unity, object_name))
                            Unity(object_name) = containers.Map();
                        end
    
                        for m = 1:length(info.Groups(i).Groups(j).Groups(k).Groups(l).Datasets) 
                            dataset_name = info.Groups(i).Groups(j).Groups(k).Groups(l).Datasets(m).Name;
                            % Use strcat to generate correct HDF5 path
                            dataset_path = strcat(subsubgroup_name, '/', dataset_name); 
                            if(j==1)
                                if(m==1)
                                    if(unityGroup_name(end) == "Agents")
                                        unity_keys.Agents(object_name) = {dataset_name};
                                    elseif (unityGroup_name(end) == "DeformableObjects")
                                        unity_keys.DefObj(object_name) = {dataset_name};
                                    else
                                        unity_keys.GameObj(object_name) = {dataset_name};
                                    end
                                else
                                    if(unityGroup_name(end) == "Agents")
                                        unity_keys.Agents(object_name) = [unity_keys.Agents(object_name), {dataset_name}];
                                    elseif (unityGroup_name(end) == "DeformableObjects")
                                        unity_keys.DefObj(object_name) = [unity_keys.DefObj(object_name), {dataset_name}];
                                    else
                                        unity_keys.GameObj(object_name) = [unity_keys.GameObj(object_name), {dataset_name}];
                                    end
                                end
                            end
        
                            if(j == 1)
                                outer_dict = Unity(object_name);
                                outer_dict(dataset_name) = [h5read(fullFileName, dataset_path)];
                            else
                                outer_dict = Unity(object_name);
                                outer_dict(dataset_name) = [outer_dict(dataset_name) h5read(fullFileName, dataset_path)];
                            end
                        end
                    end
                end
                
                % Read the dataset and store it in the corresponding struct 
                data.(matlab.lang.makeValidName(group_name)).(matlab.lang.makeValidName(subgroup_name)).(matlab.lang.makeValidName(dataset_name)) = h5read(fullFileName, dataset_path); 
            end 
        end
    end

    % Sort the times array and get sorting indices
    [times, sortIdx] = sort(times);
    
    Matlab_keys = keys(Matlab);
    
    for(i=1:length(Matlab_keys))
        outer_struct = Matlab(string(Matlab_keys(i)));
        Matlab(string(Matlab_keys(i))) = outer_struct(sortIdx);
    end
    
    Agent_keys = keys(unity_keys.Agents);
    for(i=1:length(Agent_keys))
        Agent_subkeys = unity_keys.Agents(string(Agent_keys(i)));
        outer_dict = Unity(string(Agent_keys(i)));
    
        for(j=1:length(Agent_subkeys))
            outer_struct = outer_dict(string(Agent_subkeys(j)));
            outer_dict(string(Agent_subkeys(j))) = outer_struct(sortIdx);
        end
        Unity(string(Agent_keys(i))) = outer_dict;
    end
    
    Def_keys = keys(unity_keys.DefObj);
    for(i=1:length(Def_keys))
        Def_subkeys = unity_keys.DefObj(string(Def_keys(i)));
        outer_dict = Unity(string(Def_keys(i)));
    
        for(j=1:length(Def_subkeys))
            outer_struct = outer_dict(string(Def_subkeys(j)));
            outer_dict(string(Def_subkeys(j))) = outer_struct(sortIdx);
        end
        Unity(string(Def_keys(i))) = outer_dict;
    end
    
    Game_keys = keys(unity_keys.GameObj);
    for(i=1:length(Game_keys))
        Game_subkeys = unity_keys.GameObj(string(Game_keys(i)));
        outer_dict = Unity(string(Game_keys(i)));
    
        for(j=1:length(Game_subkeys))
            outer_struct = outer_dict(string(Game_subkeys(j)));
            outer_dict(string(Game_subkeys(j))) = outer_struct(sortIdx);
        end
        Unity(string(Game_keys(i))) = outer_dict;
    end
end