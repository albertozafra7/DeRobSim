% Load the HDF5 File
[fileName, filePath] = uigetfile('*.h5', 'Select HDF5 file');
if fileName == 0
    return; % User canceled the file selection
end
fullFileName = fullfile(filePath, fileName);

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
        if(i == 1)
            time_name = split(subgroup_name, '/');
            times = [times str2double(time_name(3))];
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

% subfieldNames = fieldnames(data.x_Matlab);
% 
% % Preallocate an array to store the UD values
% U_D = [];
% 
% % Loop through each subfield and collect the UD values
% for i = 1:length(subfieldNames)
%     % Access the 'UD' field dynamically
%     U_D = [U_D data.x_Matlab.(subfieldNames{i}).U_D];
% end

% Display the time array
% disp('Time array (converted subgroup names):');
% disp(times);