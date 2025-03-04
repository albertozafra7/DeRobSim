function plotFilteredDataApp()
    %% Load the HDF5 File
    [fileName, filePath] = uigetfile({'*.h5';'*.mat'}, 'Select data file');
    if fileName == 0
        return; % User canceled the file selection
    end
    fullFileName = fullfile(filePath, fileName);

    shortfileName = split(fileName,".");
    shortfileName = string(shortfileName(end));

    if(strcmp(shortfileName,"h5"))
        % We load the hdf5 file
        [times, Matlab, Unity, unity_keys] = customHDF5_read(fullFileName);
        shortFileName = split(fileName,'.');
        shortFileName = string(shortFileName(1));
        % We save the content into a .mat file for faster readings
        save(strcat(shortFileName,".mat"),"times","Matlab","Unity","unity_keys",'-mat');

    else
        % If it is directly a .mat file, we just load the file
        load(fullFileName);
    end
    %% Create the main figure window
    fig = uifigure('Name', 'Filtered Data Plotter', 'Position', [100, 100, 800, 600]);

    % Create a panel for plot controls
    controlPanel = uipanel(fig, 'Position', [20, 50, 180, 450], 'Title', 'Controls');

    % Dropdown for selecting plot type
    lblPlotType = uilabel(controlPanel, 'Position', [10, 400, 160, 22], 'Text', 'Select Data Source:');
    dataTypeDropdown = uidropdown(controlPanel, 'Position', [10, 370, 160, 22], ...
        'Items', {'Control Params', 'Agents', 'Deformable Objects', 'Game Objects'}, 'Value', 'Control Params');

    % Dropdown for selecting data points (initially empty)
    lblDataFilter = uilabel(controlPanel, 'Position', [10, 330, 160, 22], 'Text', 'Select Object to Plot:');
    dataPointsDropdown = uidropdown(controlPanel, 'Position', [10, 300, 160, 22], ...
        'Items', {'Empty'}, 'Value', 'Empty');  % Initialize with placeholder

    % Create a hidden secondary dropdown for additional filtering
    lblSecondFilter = uilabel(controlPanel, 'Position', [10, 270, 160, 22], 'Text', 'Select Data to Plot:', 'Visible', 'off');
    secondDropdown = uidropdown(controlPanel, 'Position', [10, 240, 160, 22], ...
        'Items', {'Empty'}, 'Visible', 'off');  % Initially hidden

    % Create axes for plotting (three separate uiaxes for subplots)
    ax1 = uiaxes(fig, 'Position', [220, 400, 550, 150]);
    ax2 = uiaxes(fig, 'Position', [220, 230, 550, 150]);
    ax3 = uiaxes(fig, 'Position', [220, 60, 550, 150]);

    % Button to add selected data points to plot
    addButton = uibutton(controlPanel, 'Position', [10, 200, 160, 22], 'Text', 'Add to Plot', ...
        'ButtonPushedFcn', @(addButton, event) addDataToPlot(ax1, ax2, ax3, dataTypeDropdown, dataPointsDropdown, secondDropdown, Matlab, Unity, times, fig));

    % Create a list box to display plots
    lblPlotList = uilabel(controlPanel, 'Position', [10, 50, 160, 22], 'Text', 'Plots:');
    plotListBox = uilistbox(controlPanel, 'Position', [10, 10, 160, 100], ...
        'Items', {});

    % Button to remove selected plot from the list
    removeButton = uibutton(controlPanel, 'Position', [10, 150, 160, 22], 'Text', 'Remove Plot', ...
        'ButtonPushedFcn', @(removeButton, event) removePlotFromList(ax1, ax2, ax3, plotListBox, fig));

    % Create a Map to store plot data
    plotDataMap = containers.Map('KeyType', 'char', 'ValueType', 'any');

    % Store the plotListBox handle in the figure's app data
    guidata(fig, struct('plotListBox', plotListBox, 'ax1', ax1, 'ax2', ax2, 'ax3', ax3, ...
        'Matlab', Matlab, 'Unity', Unity, 'times', times, 'plotDataMap', plotDataMap));

    % Set up the callback to populate data points dropdown based on selected type
    dataTypeDropdown.ValueChangedFcn = @(src, event) updateDataPointsDropdown(src, dataPointsDropdown, Matlab, unity_keys, Unity, lblSecondFilter, secondDropdown);

    % Set up the second callback to populate the data subgroup points dropdown
    dataPointsDropdown.ValueChangedFcn = @(src, event) updateSecondDatapointsDropdown(src, secondDropdown, dataTypeDropdown, Unity);

    % Populate initial data points dropdown
    updateDataPointsDropdown(dataTypeDropdown, dataPointsDropdown, Matlab, unity_keys, Unity, lblSecondFilter, secondDropdown);
end

% Function to update dropdown based on selected data type
function updateDataPointsDropdown(src, dataPointsDropdown, Matlab, unity_keys, Unity, lblSecondFilter, secondDropdown)
    selectedType = src.Value;
    
    if strcmp(selectedType, 'Control Params')
        % Hide the second dropdown when "Control Params" is selected
        lblSecondFilter.Visible = 'off';
        secondDropdown.Visible = 'off';
        dataIndices = keys(Matlab);
    elseif strcmp(selectedType, 'Agents')
        % Show the second dropdown when "Agents" is selected
        lblSecondFilter.Visible = 'on';
        secondDropdown.Visible = 'on';
        dataIndices = keys(unity_keys.Agents);
        % Update the second Dropdown values
        seccondDataIndices = keys(Unity(string(dataIndices{1})));
        secondDropdown.Items = seccondDataIndices;
        secondDropdown.Value = seccondDataIndices{1}; % Set default value to first item
    elseif strcmp(selectedType, 'Deformable Objects')
        % Show the second dropdown when "Deformable Objects" is selected
        lblSecondFilter.Visible = 'on';
        secondDropdown.Visible = 'on';
        dataIndices = keys(unity_keys.DefObj);
        % Update the second Dropdown values
        seccondDataIndices = keys(Unity(string(dataIndices{1})));
        secondDropdown.Items = seccondDataIndices;
        secondDropdown.Value = seccondDataIndices{1}; % Set default value to first item
    else
        % Show the second dropdown when "Game Objects" is selected
        lblSecondFilter.Visible = 'on';
        secondDropdown.Visible = 'on';
        dataIndices = keys(unity_keys.GameObj);
        % Update the second Dropdown values
        seccondDataIndices = keys(Unity(string(dataIndices{1})));
        secondDropdown.Items = seccondDataIndices;
        secondDropdown.Value = seccondDataIndices{1}; % Set default value to first item
    end
    
    % Update dropdown list with available data points
    dataPointsDropdown.Items = dataIndices;
    dataPointsDropdown.Value = dataIndices{1}; % Set default value to first item
end

% Function to update the second dropdown based on data points
function updateSecondDatapointsDropdown(src, secondDropdown, dataTypeDropdown, Unity)
    selectedType = src.Value;
    if ~strcmp(dataTypeDropdown.Value, 'Control Params')
        dataIndices = keys(Unity(selectedType));
        % Update dropdown list with available data points
        secondDropdown.Items = dataIndices;
        secondDropdown.Value = dataIndices{1}; % Set default value to first item
    end
end

% Function to add selected data points to plot with 3 uiaxes (subplots)
function addDataToPlot(ax1, ax2, ax3, dataTypeDropdown, dataPointsDropdown, secondDropdown, Matlab, Unity, times, fig)
    selectedType = dataTypeDropdown.Value;
    selectedPointStr = dataPointsDropdown.Value;
    selectedSecondPointStr = secondDropdown.Value;

    if strcmp(selectedType, 'Control Params')
        % Retrieve data for all agents
        currentData = Matlab(selectedPointStr);
        
        % Ensure currentData is a cell array
        if ~iscell(currentData)
            currentData = num2cell(currentData);
        end
        
        % Preallocate arrays for storing agent data
        xData = [];
        yData = [];
        zData = [];
        
        % Loop through each agent and extract x, y, z data
        for i = 1:numel(currentData)
            agentData = currentData{i};
            xData = [xData; arrayfun(@(c) c.x, agentData, 'UniformOutput', false)'];
            yData = [yData; arrayfun(@(c) c.y, agentData, 'UniformOutput', false)'];
            zData = [zData; arrayfun(@(c) c.z, agentData, 'UniformOutput', false)'];
        end
        
        % Convert to matrices
        xData = cell2mat(xData');
        yData = cell2mat(yData');
        zData = cell2mat(zData');
        
        % Time vector
        n_agents = size(xData, 1);
        timeVec = repmat(times, n_agents, 1);

        % Plot data for each agent
        hold(ax1, 'on');
        hold(ax2, 'on');
        hold(ax3, 'on');

        for i = 1:n_agents
            if size(xData, 2) == length(times) % Ensure xData has correct dimensions
                plot(ax1, timeVec(i, :), xData(i, :), 'DisplayName', ['Agent ' num2str(i)], 'LineWidth', 1.5);
                plot(ax2, timeVec(i, :), yData(i, :), 'DisplayName', ['Agent ' num2str(i)], 'LineWidth', 1.5);
                plot(ax3, timeVec(i, :), zData(i, :), 'DisplayName', ['Agent ' num2str(i)], 'LineWidth', 1.5);
            end
        end

        % Set titles and labels for the axes
        title(ax1, 'X Data');
        xlabel(ax1, 'Time');
        ylabel(ax1, 'X-Axis');
        grid(ax1, 'on');
        legend(ax1, 'show'); % Update legend

        title(ax2, 'Y Data');
        xlabel(ax2, 'Time');
        ylabel(ax2, 'Y-Axis');
        grid(ax2, 'on');
        legend(ax2, 'show'); % Update legend

        title(ax3, 'Z Data');
        xlabel(ax3, 'Time');
        ylabel(ax3, 'Z-Axis');
        grid(ax3, 'on');
        legend(ax3, 'show'); % Update legend

    else
        % Handle other cases as before
        if strcmp(selectedType, 'Agents')
            outer_dict = Unity(selectedPointStr);
            currentData = outer_dict(selectedSecondPointStr);
        else
            outer_dict = Unity(selectedPointStr);
            currentData = outer_dict(selectedSecondPointStr);
        end

        % Ensure currentData is a struct array
        if isstruct(currentData)
            % Extract x, y, z data from the struct array
            xData = arrayfun(@(c) c.x, currentData);
            yData = arrayfun(@(c) c.y, currentData);
            zData = arrayfun(@(c) c.z, currentData);
        else
            error('Expected currentData to be a struct array.');
        end

        % Check if 'times' matches the length of the data to plot (if needed)
        if length(xData) ~= length(times)
            error('Length of times array does not match length of data to plot.');
        end
        
        % Sort data by time if needed
        [sorted_times, sortIdx] = sort(times);
        xData = xData(sortIdx);
        yData = yData(sortIdx);
        zData = zData(sortIdx);

        % Add data to the subplots
        hold(ax1, 'on');
        plot(ax1, sorted_times, xData, 'DisplayName', ['Plot: ' selectedPointStr], 'LineWidth', 1.5);
        hold(ax1, 'off');
        title(ax1, 'X Data');
        xlabel(ax1, 'Time');
        ylabel(ax1, 'X-Axis');
        grid(ax1, 'on');
        legend(ax1, 'show');  % Update legend

        hold(ax2, 'on');
        plot(ax2, sorted_times, yData, 'DisplayName', ['Plot: ' selectedPointStr], 'LineWidth', 1.5);
        hold(ax2, 'off');
        title(ax2, 'Y Data');
        xlabel(ax2, 'Time');
        ylabel(ax2, 'Y-Axis');
        grid(ax2, 'on');
        legend(ax2, 'show');  % Update legend

        hold(ax3, 'on');
        plot(ax3, sorted_times, zData, 'DisplayName', ['Plot: ' selectedPointStr], 'LineWidth', 1.5);
        hold(ax3, 'off');
        title(ax3, 'Z Data');
        xlabel(ax3, 'Time');
        ylabel(ax3, 'Z-Axis');
        grid(ax3, 'on');
        legend(ax3, 'show');  % Update legend
    end

    % Update the list box
    data = guidata(fig);
    plotListBox = data.plotListBox;
    plotListBox.Items{end+1} = ['Plot: ' selectedPointStr];
end



% Function to remove selected plot from the list
function removePlotFromList(ax1, ax2, ax3, plotListBox, fig)
    selectedPlot = plotListBox.Value;
    if isempty(selectedPlot)
        return;
    end

    % Retrieve plotListBox handle and other data from figure's app data
    data = guidata(fig);
    plotListBox = data.plotListBox;
    Matlab = data.Matlab;
    Unity = data.Unity;
    times = data.times;

    % Remove the selected plot from the list
    plotListBox.Items(strcmp(plotListBox.Items, selectedPlot)) = [];
    
    % Clear all subplots
    cla(ax1, 'reset');
    cla(ax2, 'reset');
    cla(ax3, 'reset');

    % Re-plot remaining items
    hold(ax1, 'on');
    hold(ax2, 'on');
    hold(ax3, 'on');

    keys = plotListBox.Items;
    
    for i = 1:length(keys)
        plotName = strrep(keys{i}, 'Plot: ', '');
        % Retrieve plot data based on plotName
        if isKey(plotDataMap, keys{i})
            dataToPlot = plotDataMap(keys{i});
            plot(ax1, dataToPlot.times, dataToPlot.xData, 'DisplayName', keys{i}, 'LineWidth', 1.5);
            plot(ax2, dataToPlot.times, dataToPlot.yData, 'DisplayName', keys{i}, 'LineWidth', 1.5);
            plot(ax3, dataToPlot.times, dataToPlot.zData, 'DisplayName', keys{i}, 'LineWidth', 1.5);
        end
    end

    hold(ax1, 'off');
    hold(ax2, 'off');
    hold(ax3, 'off');

    % Update legend
    legend(ax1, 'show');
    legend(ax2, 'show');
    legend(ax3, 'show');
end