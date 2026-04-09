% Define simulation parameters
clear;
clc;

simTime = 60; % Total simulation time in seconds
modelName = 'Suspension_DNN'; % Model name

% Define step input variations
finalValues = 0.01:0.001:0.016; % Final values from 0.01 to 0.016
initialValue = 0; % Initial step input

% Load and initialize the model
load_system(modelName);
set_param(modelName, 'FastRestart', 'off'); % Disable Fast Restart to avoid errors

% Specify signals to log
inputSignals = {'id_ref', 'iq_ref', 'id', 'iq', 'theta', 'wr'}; % Inputs
outputSignal = 'Sw'; % Output

% Initialize dataset storage
fullDataset = [];

% Run simulations for each final step input value
fprintf('Starting simulations...\n');

for idx = 1:length(finalValues)
    try
        fprintf('\nSimulation %d/%d - Step Value: %.4f\n', idx, length(finalValues), finalValues(idx));
        
        % Set step input values
        set_param([modelName '/Position'], 'Before', num2str(initialValue));
        set_param([modelName '/Position'], 'After', num2str(finalValues(idx)));
        
        % Run the simulation
        out = sim(modelName, 'StopTime', num2str(simTime));
        
        % Initialize data structure for this iteration
        data = struct();
        availableSignals = {};
        
        % Collect input signals from the 'out' structure
        for i = 1:length(inputSignals)
            try
                if isfield(out, inputSignals{i})
                    data.(inputSignals{i}) = out.(inputSignals{i}).Data;
                    availableSignals{end+1} = inputSignals{i};
                    fprintf('Successfully collected %s\n', inputSignals{i});
                else
                    fprintf('Signal %s not found in simulation output\n', inputSignals{i});
                end
            catch ME
                fprintf('Error collecting %s: %s\n', inputSignals{i}, ME.message);
            end
        end
        
        % Collect output signal
        try
            if isfield(out, outputSignal)
                data.Sw_output = out.(outputSignal).Data;
                fprintf('Successfully collected Sw\n');
            else
                fprintf('Output signal Sw not found in simulation output\n');
                continue;
            end
        catch ME
            fprintf('Error collecting Sw: %s\n', ME.message);
            continue;
        end
        
        % Check if we have any data
        if isempty(availableSignals)
            fprintf('No signals collected for this simulation\n');
            continue;
        end
        
        % Get the time vector
        timeVector = out.tout;
        
        % Ensure all signals have the same length
        signalLengths = cellfun(@(s) length(data.(s)), availableSignals);
        outputLength = length(data.Sw_output);
        numSamples = min([outputLength, signalLengths]);
        
        % Prepare dataset
        dataset = zeros(numSamples, length(availableSignals) + 1);
        for i = 1:numSamples
            for j = 1:length(availableSignals)
                dataset(i, j) = data.(availableSignals{j})(i);
            end
            dataset(i, end) = data.Sw_output(i);
        end
        
        % Append to full dataset
        fullDataset = [fullDataset; dataset];
        fprintf('Data collection complete for step value %.4f\n', finalValues(idx));
        
    catch ME
        fprintf('Error in simulation %d: %s\n', idx, ME.message);
    end
end

% Save the final dataset
if ~isempty(fullDataset)
    save('classification_dataset.mat', 'fullDataset', 'inputSignals', 'outputSignal', 'finalValues');
    fprintf('\nData collection complete!\n');
    fprintf('Dataset saved with %d samples and %d features\n', size(fullDataset, 1), size(fullDataset, 2));
else
    warning('No data was collected! Check model configuration.');
end

% Close the model
close_system(modelName, 0);
