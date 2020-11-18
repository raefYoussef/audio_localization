close;
clear all;


%% Global Parameters

SPD_OF_SOUND = 343;                 % Speed of Sound, in M/s
Fs = 48000;                         % Sampling Rate, Hz
FRAME_LEN = 1024;                   % Number of samples per capture
AMP = 500;                          % Amplification factor
ACTIVITY_THRESH = .2;               % Activity Threshold
REC_LEN = 30;                       % Recording length, in seconds
NMICS = 16;                         % Number of Microphones
MIC_SPACING = 0.042;                % Microphone spacing , in meters
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
                    2,0,0].';       % MC16
                
[SENSOR1_INDX, SENSOR2_INDX] = sensor_comp_map(NMICS); 
SENSOR1_POS = MIC_COORDINATES(:,SENSOR1_INDX);
SENSOR2_POS = MIC_COORDINATES(:,SENSOR2_INDX);
INIT_POS = [-1;-1;-1];
                
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
%     play(deviceWriter, acq(:,1));
    
    % measure activity
    if any(abs(acq) > ACTIVITY_THRESH, 'all')
        [tdoa, corr] = calc_tdoa(acq, Fs, SENSOR1_INDX, SENSOR2_INDX);
        [est_pos, P] = ILS(tdoa, SENSOR1_POS, SENSOR2_POS, INIT_POS)
    end
end