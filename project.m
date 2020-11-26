close;
clear all;


%% Global Parameters

SPD_OF_SOUND = 343e3;               % Speed of Sound, in mm/s
Fs = 48000;                         % Sampling Rate, Hz
FRAME_LEN = 1024;                   % Number of samples per capture
AMP = 500;                          % Amplification factor
ACTIVITY_THRESH = .4;               % Activity Threshold
REC_LEN = 30;                       % Recording length, in seconds
NMICS = 16;                         % Number of Microphones
MIC_SPACING = 42;                   % Microphone spacing , in Millimeters
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

while toc < REC_LEN  
    acq = AMP * deviceReader();
    
    % measure activity
    if detect_activity(acq, ACTIVITY_THRESH)
        play(deviceWriter, acq(:,1));
%         [tdoa, corr] = calc_tdoa(acq(:, [3 6 10 15]), Fs, SENSOR1_INDX, SENSOR2_INDX, MAX_LAGS);
        [tdoa, corr] = calc_tdoa(acq, Fs, SENSOR1_INDX, SENSOR2_INDX, MAX_LAGS);
        range_diff = tdoa * SPD_OF_SOUND;
        [est_pos, P] = TDOA_ILS(range_diff, SENSOR1_POS, SENSOR2_POS, INIT_POS, false)
%         [est_pos, P] = TDOA_ILS(range_diff, SENSOR1_POS, SENSOR2_POS, INIT_POS, true)
%         [est_pos1, P1] = TDOA_EKF(range_diff, SENSOR1_POS, SENSOR2_POS, INIT_POS, INIT_COVAR)
        continue
    end
end