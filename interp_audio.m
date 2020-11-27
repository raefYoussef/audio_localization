function [new_audio] = interp_audio(audio_channels, factor)
% Interpolate Audio Data with a certain factor

    nchannels = size(audio_channels, 2);
    new_audio = zeros(size(audio_channels) .* [factor 1]);
    
    for i = 1:nchannels
        new_audio(:,i) = interp(audio_channels(:,i), factor);
    end
end

