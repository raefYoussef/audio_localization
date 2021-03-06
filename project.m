close all;
clear;
clc;


%% Global Parameters

SPD_OF_SOUND = 343;                 % Speed of Sound, in m/s
Fs = 48000;                         % Sampling Rate, Hz
UPSAMPLING_FACTOR = 1;              % Upsample audio by this factor
FRAME_LEN = 1024;                   % Number of samples per capture
AMP = 500;                          % Amplification factor
ACTIVITY_THRESH = 1;                % Activity Threshold
REC_LEN = 60;                       % Recording length, in seconds
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

DOA_2D_SENSORS = [ 
                4,  14, 1,  5   ;...
                2,  12, 16, 6   ;...
                3,  13, 16, 12  ;...
                1,  11, 15, 5   ;...
                6,  12, 3,  7   ;...
                4,  10, 14, 8   ;...
                5,  11, 14, 10  ;...
                3,  9,  13, 7   ;...
                ].';
      
DOA_3D_SENSORS = [ 
                4,  14, 1,  5,  2,  12, 16, 6   ;...
                3,  13, 16, 12, 1,  11, 15, 5   ;...
                6,  12, 3,  7,  4,  10, 14, 8   ;...
                5,  11, 14, 10, 3,  9,  13, 7   ;...
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
        [doa_centers, doa_angles] = calc_3D_DOA(acq_interp, Fs_interp, MIC_COORDINATES, DOA_3D_SENSORS, true);
        
        % Derive an initial estimate based on DOA
        doa_est = Moore_Penrose(doa_centers, doa_angles);
        
        % Measure TDOA / RDOA
        [tdoa, corr] = calc_TDOA(acq_interp, Fs_interp, MIC_COORDINATES, [SENSOR1_INDX; SENSOR2_INDX]);
        rdoa_meas = tdoa * SPD_OF_SOUND;
        
        % Perform grid search
        [grid_est_l1, grid_est_l2] = TDOA_grid_search(SENSOR1_POS, SENSOR2_POS, rdoa_meas,...
                             [0; 0; 0], 1, .05,...
                             true, DEBUG);
                                 
%         % Perform grid search on DOA estimate
%         [comb_est_l1, comb_est_l2] = TDOA_grid_search(SENSOR1_POS, SENSOR2_POS, rdoa_meas,...
%                                      doa_est, 2, .05,...
%                                      false, DEBUG);
                                 
        % Derive final estimate using ILS
        [est_pos1, P1, conv_flag1] = TDOA_ILS(rdoa_meas, SENSOR1_POS, SENSOR2_POS, doa_est, true, DEBUG);
        [est_pos2, P2, conv_flag2] = TDOA_ILS(rdoa_meas, SENSOR1_POS, SENSOR2_POS, grid_est_l1, true, DEBUG);
        [est_pos3, P3, conv_flag3] = TDOA_ILS(rdoa_meas, SENSOR1_POS, SENSOR2_POS, grid_est_l2, true, DEBUG);
        
        % Print Results
        fprintf('---------------------------------\n');
        fprintf('DOA Est: (%.5f, %.5f, %.5f)\n', doa_est(1), doa_est(2), doa_est(3));
        fprintf('3D L1 Grid Est: (%.5f, %.5f, %.5f)\n', grid_est_l1(1), grid_est_l1(2), grid_est_l1(3));
        fprintf('3D L2 Grid Est: (%.5f, %.5f, %.5f)\n', grid_est_l2(1), grid_est_l2(2), grid_est_l2(3));
            
        if conv_flag1
            fprintf('DOA Pos: (%.5f, %.5f, %.5f)\n', est_pos1(1), est_pos1(2), est_pos1(3));
        else
            fprintf('DOA Pos: No Solution\n');
        end

        if conv_flag2
            fprintf('3D L1 Pos: (%.5f, %.5f, %.5f)\n', est_pos2(1), est_pos2(2), est_pos2(3));
        else
            fprintf('2D L1 Pos: No Solution\n');
        end
        
        if conv_flag3
            fprintf('3D L2 Pos: (%.5f, %.5f, %.5f)\n', est_pos3(1), est_pos3(2), est_pos3(3));
        else
            fprintf('3D L2 Pos: No Solution\n');
        end
            
        
%         [est_pos, P] = TDOA_ILS(range_diff, SENSOR1_POS, SENSOR2_POS, INIT_POS, true)
%         [est_pos1, P1] = TDOA_EKF(range_diff, SENSOR1_POS, SENSOR2_POS, INIT_POS, INIT_COVAR)
    end
end