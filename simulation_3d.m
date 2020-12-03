close all;
clear;
clc;


%% Global Parameters

SPD_OF_SOUND = 343;                 % Speed of Sound, in m/s
Fs = 48000;                         % Sampling Rate, Hz
UPSAMPLING_FACTOR = 1;              % Upsample audio by this factor
FRAME_LEN = 1024;                   % Number of samples per capture
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

TARGET_POS = [-1.1; 3.2; 2.7];
ERR_SIGMA = 0;
DEBUG = true;

%% Derivative Parameters
[SENSOR1_INDX, SENSOR2_INDX] = sensor_comp_map(NMICS); 
SENSOR1_POS = MIC_COORDINATES(:,SENSOR1_INDX);
SENSOR2_POS = MIC_COORDINATES(:,SENSOR2_INDX);
MAX_LAGS = calc_max_lag(SENSOR1_POS, SENSOR2_POS, Fs);
INIT_POS = [1;0;.5];
INIT_COVAR = eye(3);

%% Simulate TDOA Measurments 
true_rdoa = calc_range_diff(TARGET_POS, SENSOR1_POS, SENSOR2_POS);
noisy_rdoa = true_rdoa + ERR_SIGMA .* randn(size(true_rdoa));
true_lags = noisy_rdoa ./ SPD_OF_SOUND .* Fs;
rounded_lags = round(true_lags);
sim_rdoa = rounded_lags ./ Fs * SPD_OF_SOUND;

%% Grid Search
% Perform grid search
[grid_est_l1, grid_est_l2] = TDOA_grid_search(SENSOR1_POS, SENSOR2_POS, sim_rdoa,...
                             [0; 0; 0], 12, .5,...
                             true, DEBUG);

est_err_l1 = norm(TARGET_POS - grid_est_l1);
est_err_l2 = norm(TARGET_POS - grid_est_l2);

fprintf('L1 Estimate : (%.4f, %.4f, %.4f), Error = %.4f\n', ...
         grid_est_l1(1), grid_est_l1(2), grid_est_l1(3), est_err_l1);

fprintf('L2 Estimate : (%.4f, %.4f, %.4f), Error = %.4f\n', ...
         grid_est_l2(1), grid_est_l2(2), grid_est_l2(3), est_err_l2);

%% ILS
% Derive final estimate using ILS
[est_pos, P, conv_flag] = TDOA_ILS(true_rdoa, SENSOR1_POS, SENSOR2_POS, grid_est_l1, true, DEBUG);

