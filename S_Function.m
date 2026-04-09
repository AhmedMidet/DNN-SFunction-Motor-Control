function sys = mdlOutputs(t,x,u)
    % Use global variables to maintain state between calls
    global DNN_MODEL DNN_MU DNN_SIGMA DNN_INITIALIZED;
    persistent last_valid_output switchMatrix lastSwitchTime prediction_counter error_count dnn_status last_model_check;
    
    % Initialize persistent variables if empty
    if isempty(switchMatrix)
        % Define the 8 possible switching states for a 3-phase inverter
        % [Sa, Sb, Sc] for each state
        switchMatrix = [
            0, 0, 0;  % State 1
            1, 0, 0;  % State 2
            1, 1, 0;  % State 3
            0, 1, 0;  % State 4
            0, 1, 1;  % State 5
            0, 0, 1;  % State 6
            1, 0, 1;  % State 7
            1, 1, 1   % State 8
        ];
        last_valid_output = [0, 0, 0];  % Default safe state
        lastSwitchTime = t;  % Initialize switch time
        prediction_counter = 0;  % Initialize counter for logging
        error_count = 0;  % Track consecutive errors
        dnn_status = 1;  % 1 = working, 0 = error state
        last_model_check = 0;  % Time of last model check
    end
    
    % Check if we need to initialize or verify the model is loaded properly
    % Only check every 1 second instead of every call
    if isempty(DNN_INITIALIZED) || ~DNN_INITIALIZED || (t - last_model_check > 1.0)
        try
            % Only reload if not initialized
            if isempty(DNN_INITIALIZED) || ~DNN_INITIALIZED
                disp(['Loading DNN model at simulation time t=', num2str(t), '...']);
                loaded = load('DNN_classification.mat');
                DNN_MODEL = loaded.net;
                
                % Load normalization parameters
                if isfield(loaded, 'mu') && isfield(loaded, 'sigma')
                    DNN_MU = loaded.mu;
                    DNN_SIGMA = loaded.sigma;
                else
                    % Default normalization if not available
                    warning('Normalization parameters not found. Using defaults.');
                    DNN_MU = zeros(1,6);
                    DNN_SIGMA = ones(1,6);
                end
                
                DNN_INITIALIZED = true;
                disp('DNN model loaded successfully!');
            else
                % Just verify model is valid without reloading
                if ~isobject(DNN_MODEL) || ~ismethod(DNN_MODEL, 'classify')
                    warning('DNN model appears invalid. Will reload on next check.');
                    DNN_INITIALIZED = false;
                end
            end
            last_model_check = t;  % Update time of last check regardless of outcome
        catch ME
            warning('Error checking/loading DNN model at t=%f: %s\nUsing fallback control.', t, ME.message);
            DNN_INITIALIZED = false;
            dnn_status = 0;
            % Use fallback control based on rotor position
            sys = implement_fallback_control(u(5));  % theta is u(5)
            last_valid_output = sys;
            return;
        end
    end
    
    % Extract inputs
    id_ref = u(1);
    iq_ref = u(2);
    id = u(3);
    iq = u(4);
    theta = u(5);
    wr = u(6);
    
    % Check for valid inputs (avoid NaN or Inf)
    if any(isnan([id_ref, iq_ref, id, iq, theta, wr])) || any(isinf([id_ref, iq_ref, id, iq, theta, wr]))
        % Don't flood console with warnings
        if mod(t, 0.1) < 1e-4
            warning('Invalid inputs detected at t=%f. Using fallback control.', t);
        end
        sys = implement_fallback_control(theta);
        last_valid_output = sys;
        return;
    end
    
    %----------------------------------------------------------------------
    % ADD THRUST LIMITING CODE HERE - START
    %----------------------------------------------------------------------
    % Calculate approximate torque
    % For PMSM: Torque = (3/2) * p * (λ_pm * iq + (Ld-Lq)*id*iq)
    % Simplified version (assuming constant flux):
    p = 4;  % Number of pole pairs - adjust for your motor
    lambda_pm = 0.1;  % Permanent magnet flux - adjust for your motor
    K_torque = (3/2) * p * lambda_pm;
    estimated_torque = K_torque * iq;

    % Convert torque to thrust using your system parameters
    % This is application specific - adjust constants for your system
    K_thrust = 1000;  % Torque to thrust conversion factor
    estimated_thrust = estimated_torque * K_thrust;

    % Limit thrust to 3000 N
    max_thrust = 3000;  % Desired thrust limit (N)
    if abs(estimated_thrust) > max_thrust
        scaling = max_thrust / abs(estimated_thrust);
        
        % Scale iq reference and actual value
        iq_ref_limited = iq_ref * scaling;
        iq_limited = iq * scaling;
        
        % Use limited current for DNN
        X = [id_ref, iq_ref_limited, id, iq_limited, theta, wr];
        
        % Periodic logging (every 0.5s)
        if mod(t, 0.5) < 1e-4
            fprintf('Thrust limiting active at t=%.2f: %.1f → %.1f N (scaling=%.3f)\n', ...
                    t, estimated_thrust, estimated_thrust * scaling, scaling);
        end
    else
        % Use original values if within limit
        X = [id_ref, iq_ref, id, iq, theta, wr];
    end
    %----------------------------------------------------------------------
    % ADD THRUST LIMITING CODE HERE - END
    %----------------------------------------------------------------------
    
    % Reset error count if we got valid inputs
    if error_count > 0
        error_count = error_count - 1;  % Gradually reduce error count with valid inputs
    end
    
    % Prevent too frequent switching - important for stability
    % Only change switch state if enough time has passed
    minSwitchInterval = 5e-5; % 50μs minimum between switches (typical for IGBT)
    if (t - lastSwitchTime) < minSwitchInterval
        sys = last_valid_output;
        return;
    end
    
    % Normalize inputs using the saved parameters
    % Note: Use X from the thrust limiting code instead of recreating it
    X_norm = (X - DNN_MU) ./ DNN_SIGMA;
    
    % Check if DNN is in working state and initialized
    if dnn_status == 1 && DNN_INITIALIZED
        try
            % Make prediction using the network
            [YPred, ~] = classify(DNN_MODEL, X_norm);
            
            % Convert categorical output to index
            YPredDouble = double(YPred);  % Will be either 1 or 2 (MATLAB indexing)
            
            % Normalize theta to [0, 2π)
            theta_norm = mod(theta, 2*pi);
            sector = floor(theta_norm / (pi/3)) + 1;  % 6 sectors for a 3-phase system
            
            % Choose switching state based on sector and prediction
            switchIndex = 1; % Default
            
            if YPredDouble == 2  % Class "1" (active state)
                % Choose active vector based on sector
                switch sector
                    case 1
                        switchIndex = 2;  % [1,0,0]
                    case 2
                        switchIndex = 3;  % [1,1,0]
                    case 3
                        switchIndex = 4;  % [0,1,0]
                    case 4
                        switchIndex = 5;  % [0,1,1]
                    case 5
                        switchIndex = 6;  % [0,0,1]
                    case 6
                        switchIndex = 7;  % [1,0,1]
                    otherwise
                        switchIndex = 1;  % Default to [0,0,0]
                end
            else  % Class "0" (zero state)
                % Choose between two zero states [0,0,0] or [1,1,1] based on current state
                % to minimize switching transitions
                if sum(last_valid_output) >= 2
                    switchIndex = 8;  % [1,1,1]
                else
                    switchIndex = 1;  % [0,0,0]
                end
            end
            
            % Ensure index is within bounds
            if switchIndex < 1 || switchIndex > size(switchMatrix, 1)
                switchIndex = 1;  % Use safe default
            end
            
            % Get the switching states
            sys = switchMatrix(switchIndex, :);
            
            % Update last valid output and switch time
            last_valid_output = sys;
            lastSwitchTime = t;
            
            % Periodic debugging (only every 1.0s to avoid flooding console)
            prediction_counter = prediction_counter + 1;
            if mod(t, 1.0) < 1e-4  % Print approximately every 1.0s
                fprintf('t=%.4f: in=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] -> Class=%d, Sector=%d, Sw=[%d,%d,%d], Count=%d\n', ...
                    t, X(1), X(2), X(3), X(4), X(5), X(6), YPredDouble, sector, sys(1), sys(2), sys(3), prediction_counter);
                prediction_counter = 0;  % Reset counter
            end
            
        catch ME
            % Print error message first time but then reduce frequency
            if error_count == 0 || mod(t, 1.0) < 1e-4
                warning('Error in DNN prediction at t=%f: %s\nUsing fallback control.', t, ME.message);
            end
            error_count = error_count + 1;
            
            if error_count > 100  % Only switch to fallback after persistent errors
                dnn_status = 0;  % Mark DNN as failed
                if mod(t, 1.0) < 1e-4  % Don't flood console
                    disp('DNN model appears to be failing consistently. Switching to fallback control.');
                end
                DNN_INITIALIZED = false;  % Force reload on next check
            end
            
            % Use fallback control based on rotor position
            sys = implement_fallback_control(theta);
            last_valid_output = sys;
        end
    else
        % DNN is in error state or not initialized, use fallback control
        sys = implement_fallback_control(theta);
        last_valid_output = sys;
        
        % Try to recover DNN periodically
        if mod(t, 2.0) < 1e-4  % Try recovery every 2 seconds
            disp(['Attempting to recover DNN at t=', num2str(t)]);
            DNN_INITIALIZED = false;  % Force reload on next check
            dnn_status = 1;  % Reset status to working
            error_count = 0;  % Reset error count
        end
    end
end