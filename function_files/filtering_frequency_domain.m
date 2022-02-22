function [forceFFT, forceFFTlim, forceFFThann, forceFFThannlim, freqResReal, timeResReal, windowSize] = ...
                    filtering_frequency_domain(freqRes, Ts, force, f1, f2, f3, f4, endInd)

%   Fast-Fourier Transform over sliding window + Hann window,
%   implementation as Discrete-Fourier Transform
    
    %% parameters
    freq = 1/Ts; % frequency [Hz]
    log2N = floor(log2(freq/freqRes)); % make sure the windowsize is a multiple 
                                       % of log2N for the FFT implementation
    windowSize = 2^log2N; % windowsize in [nr of samples]
    freqResReal = freq/windowSize; % [Hz], different than the desired input resolution
    timeResReal = 1/freqResReal;   % [sec]

    %% FFT algorithm
    
    % initialize
    forceFFT = cell(2,1);
    forceFFTlim = cell(2,1);
    forceFFThann = cell(2,1);
    forceFFThannlim = cell(2,1);

    for i = 1:2
        forceFFT{i,1} = zeros(3, endInd);
        forceFFThann{i,1} = zeros(3, endInd);
        forceFFTlim{i,1} = zeros(3, endInd);
        forceFFThannlim{i,1} = zeros(3, endInd);
    end
    
    for h = 1:2
        for k = 1:3 % loop over three arm forces
            % start from the moment the window covers the complete start
            for i = (1+windowSize):(endInd) 

                % hann window
                w = hann(windowSize*2);

                % FFT: convert time domain signal to frequency domain
                FFT = fft(force{h,1}(k,(i-windowSize+1):i), windowSize);       

                % FFT with hann: convert time domain signal to frequency domain 
                FFThann = fft(force{h,1}(k,(i-windowSize+1):i).*w(1:windowSize)', windowSize);     

                % normalize (before computing one norm)
                FFT = FFT/windowSize;
                FFThann = FFThann/windowSize;

                % compute one norm
                accum = 0;
                for j = 1:windowSize
                    accum = accum + real(FFT(1,j))*real(FFT(1,j)) + imag(FFT(1,j))*imag(FFT(1,j));
                end
                oneNorm = sqrt(accum); % power in time domain
                forceFFT{h,1}(k,i) = oneNorm;

                % compute one norm limited spectrum
                accumLim = 0;      
                for j = f1:f2
                    accumLim = accumLim + real(FFT(1,j))*real(FFT(1,j)) + imag(FFT(1,j))*imag(FFT(1,j));
                end
                oneNormLim = sqrt(accumLim);  % power in time domain
                forceFFTlim{h,1}(k,i) = oneNormLim;

                % compute one norm hann
                accumHann = 0;
                for j = 1:windowSize
                    accumHann = accumHann + real(FFThann(1,j))*real(FFThann(1,j)) + imag(FFThann(1,j))*imag(FFThann(1,j));
                end
                oneNormHann = sqrt(accumHann);  % power in time domain
                forceFFThann{h,1}(k,i) = oneNormHann;

                % compute one norm hann limited spectrum
                accumHannLim = 0;
                for j = f3:f4
                    accumHannLim = accumHannLim + real(FFThann(1,j))*real(FFThann(1,j)) + imag(FFThann(1,j))*imag(FFThann(1,j));
                end
                oneNormHannLim = sqrt(accumHannLim);  % power in time domain
                forceFFThannlim{h,1}(k,i) = oneNormHannLim;
            end
        end
    end
end

