clear variables

s = serialport("COM4",115200)

input = [0,2,4,5,5,4,3,2,0];

fig = figure(1)
% Top two plots
nexttile

X = hankel(1:5, 5:7).';

title('Plot of ADC buffer')
p = plot(input);
% ylim([0 4096])
p.YDataSource = 'input';

Fs = 1000;            % Sampling frequency                    
T = 1/Fs;             % Sampling period

L = length(input)
Y = fft(input);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
ylim([0 4096])
%figure(2)
nexttile

fft_p = plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
% ylim([0 1])
fft_p.XDataSource = 'f';
fft_p.YDataSource = 'P1';
legend("init");

if 0
    while ishghandle(p)
        dataString = s.readline();
        disp(dataString);
        type = dataString(1);
        data = sscanf(dataString(3:end), '%g,', [1,inf]);

        L = length(data) 
        Y = fft(data);
        P2 = abs(Y/L);
        P1 = P2(1:L/2+1);
        P1(2:end-1) = 2*P1(2:end-1);
        f = Fs*(0:(L/2))/L;

        [maxy,idx] = max(P1);

        figure(1);
        refreshdata
        drawnow

%         figure(2);
%         legend(int2str(f(idx)));
%         refreshdata
%         drawnow
    end
else 
    while ishghandle(p)
        dataString = s.readline();
        
        if (contains(dataString,"i"))
            disp("Displaying input");
            
            dataString = s.readline();
            input = sscanf(dataString, '%g,', [1,inf]);
            %spectrogram(input(X));
%             disp(input);
            refreshdata
            drawnow
        end
        
        if (contains(dataString,"o"))
            disp("Displaying Output");
            dataString = s.readline();
            
            data = sscanf(dataString, '%g,', [1,inf]);
           
            L = length(data);
            Y = data;
            P2 = abs(Y/L);
            P1 = P2(1:L/2+1);
            P1(2:end-1) = 2*P1(2:end-1);
            P1(1) = 0;
            f = Fs*(0:(L/2))/L;
            [maxy,idx] = max(P1);
            refreshdata
            legend(int2str(f(idx)));
            drawnow
        end
        
        
% 
%         L = length(data)
%         Y = data;
%         P2 = abs(Y/L);
%         P1 = P2(1:L/2+1);
%         P1(2:end-1) = 2*P1(2:end-1);
%         P1(1) = 0;
%         f = Fs*(0:(L/2))/L;
% 
%         [maxy,idx] = max(P1);
% 
%         % figure(1);
%         refreshdata
%         legend(int2str(idx));
%         drawnow

%         figure(2);
%         legend(int2str(f(idx)));
%         refreshdata
%         drawnow
    end
end

delete(s);
clear s;
disp("Program Ended");