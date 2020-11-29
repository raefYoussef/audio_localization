close;
clear all;
clc;


%% Global Parameters

SPD_OF_SOUND = 343;                 % Speed of Sound, in m/s
Fs = 48000;                         % Sampling Rate, Hz
UPSAMPLING_FACTOR = 1;              % Upsample audio by this factor
FRAME_LEN = 1024;                   % Number of samples per capture
AMP = 500;                          % Amplification factor
ACTIVITY_THRESH = 1;                % Activity Threshold
REC_LEN = 30;                       % Recording length, in seconds
NMICS = 16;                         % Number of Microphones
MIC_SPACING =.042;                  % Microphone spacing , in Meters
MIC_COORDINATES = MIC_SPACING * [...
                    1,0,0;...       % MC1  
                    0,0,0;...       % MC2  
                    1,1,0;...       % MC3
                    0,1,0;...       % MC4
                    1,2,0;...       % MC5
                    0,2,0;...       % MC6
                    1,3,0;...       % MC7
                    0,3,0;...       % MC8
                    3,3,0;...       % MC9
                    2,3,0;...       % MC10
                    3,2,0;...       % MC11
                    2,2,0;...       % MC12
                    3,1,0;...       % MC13
                    2,1,0;...       % MC14
                    3,0,0;...       % MC15
                    2,0,0;...       % MC16
                    ].';       

DOA_SENSORS = [ 
                2,  15, 8   ;...
                9,  8,  15  ;... 
                2,  16, 6   ;...
                12, 6,  16  ;...
                1,  15, 5   ;...
                11, 5,  15  ;...
                4,  14, 8   ;...
                10, 8,  14  ;...
                3,  13, 7   ;...
                9,  7,  13  ;...
                ].';

DEBUG = true;

%% Derivative Parameters
[SENSOR1_INDX, SENSOR2_INDX] = sensor_comp_map(NMICS); 
SENSOR1_POS = MIC_COORDINATES(:,SENSOR1_INDX);
SENSOR2_POS = MIC_COORDINATES(:,SENSOR2_INDX);
MAX_LAGS = calc_max_lag(SENSOR1_POS, SENSOR2_POS, Fs);
INIT_POS = [0;-80;0];
INIT_COVAR = eye(3);
                
%% Setup Device

deviceReader = audioDeviceReader(...
 'Device', 'miniDSP ASIO Driver',...
 'Driver', 'ASIO', ...
 'SampleRate', Fs, ...
 'NumChannels', 16 ,...
 'OutputDataType','double',...
 'SamplesPerFrame', FRAME_LEN);

deviceWriter = audioDeviceWriter(...
            'SampleRate',Fs, ...
            'SupportVariableSizeInput', false);

%% Record Audio
tic;
acq_interp = zeros(UPSAMPLING_FACTOR * FRAME_LEN, NMICS);
Fs_interp = UPSAMPLING_FACTOR * Fs;

while toc < REC_LEN  
    acq = AMP * deviceReader();
    
    % measure activity
    if detect_activity(acq, ACTIVITY_THRESH)
        play(deviceWriter, acq(:,1));
        
        % upsample data
        for i = 1:NMICS
            acq_interp(:,i) = interp(acq(:,i), UPSAMPLING_FACTOR);
        end
        
        % Measure DOA 
        [doa_centers, doa_angles] = calc_DOA(acq_interp, Fs_interp, MIC_COORDINATES, DOA_SENSORS, DEBUG);
        avg_doa = mean(doa_angles);
        
        % Derive an initial estimate based on DOA
        doa_est = Moore_Penrose(doa_centers, doa_angles);
        
        % Measure TDOA / RDOA
        [tdoa, corr] = calc_TDOA(acq_interp, Fs_interp, MIC_COORDINATES, [SENSOR1_INDX; SENSOR2_INDX]);
        rdoa = tdoa * SPD_OF_SOUND;
        
        % Perform grid search (two loops with increasing resolution)
        grid_est1 = TDOA_grid_search(SENSOR1_POS, SENSOR2_POS, rdoa,...
                                     [0; 0], 12, .5,...
                                     false, true);
                                 
        grid_est2 = TDOA_grid_search(SENSOR1_POS, SENSOR2_POS, rdoa,...
                                     grid_est1, 3, .1,...
                                     false, true);
                                 
                                 
        % Derive final estimate using ILS
        [est_pos1, P1, conv_flag] = TDOA_ILS(rdoa, SENSOR1_POS, SENSOR2_POS, grid_est2, false, DEBUG);
        [est_pos2, P2, conv_flag] = TDOA_ILS(rdoa, SENSOR1_POS, SENSOR2_POS, doa_est, false, DEBUG);

        % Print Results
        if conv_flag
            fprintf('DOA: %.5f\n', avg_doa);
%             fprintf('Init Pos: \n');
%             disp(doa_est);
%             
%             fprintf('Pos: \n');
%             disp(est_pos);
%             fprintf('Covar Matrix: \n');
%             disp(P);
        else
%             fprintf('DOA: %.5f\n', avg_doa);
%             fprintf('Init Pos: \n');
%             disp(doa_est);
            
            fprintf('ILS did not converge\n');
        end
        
%         [est_pos, P] = TDOA_ILS(range_diff, SENSOR1_POS, SENSOR2_POS, INIT_POS, true)
%         [est_pos1, P1] = TDOA_EKF(range_diff, SENSOR1_POS, SENSOR2_POS, INIT_POS, INIT_COVAR)
    end
end