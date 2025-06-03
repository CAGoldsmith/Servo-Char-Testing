%Clarissa Goldsmith
%West Virginia University
%6/18/21

function [dampedFreq, logDec, posRad, t,fig] = processPendv2(fileTree,fileName)
%% 
% close all
% clear all

%------ Import the data
data = importdata(fileTree);
posRaw = data(:,2);
posRaw = posRaw - mean(posRaw(end-100:end)); %Normalize the position based on the mid-point of the servos
t = data(:,1)/1000;
t = t - t(1); %Normalize the time

%------ Get rid of the weird 1 position outliers throughout
posF(1) = posRaw(1); 
for i=2:1:length(posRaw)-1
    if posRaw(i-1) == posRaw(i+1)
        if abs(posRaw(i+1)-posRaw(i)) < 2
            posF(i) = posRaw(i+1);
        end
    else
        posF(i)= posRaw(i);
    end
end
posF(length(posRaw)) = posRaw(end);

%------ Smooth the filtered data
posFSm = smooth(t,posF,10);

%------ Get rid of the flat portion on the front of the data
frontFlat = 0;
i = 1;
while frontFlat == 0
    if abs(posFSm(i+100)-posFSm(i)) > 5
        posFSm = posFSm(i:end);
        t = t(i:end);
        frontFlat = 1;
    else
        posFSm = posFSm(i+100:end);
        t = t(i+100:end);
    end
    %plot(t,posFSm)
    i = i+1;
end
%------ Convert to radians
posFSmDeg = posFSm*0.088;
posRad = posFSmDeg*pi/180;

%% ------ Find peaks
%------ Determine if the first peak is a min or a max
posPeaks = posRad;
if posRad(1) > 0
    peakSign = 0; %Where 0 = - = min and 1 = + = max
else
    peakSign = 1;
end

peaks = [];
finding = 1;
j=1;
i = 1;
peakTol = 0.02;

%------ Start finding peaks
while finding == 1
    if peakSign == 0 %Peak is a minimum
        peakVal = min(posPeaks);
        peakSign = 1;
    else %Peak is a maximum
        peakVal = max(posPeaks);
        peakSign = 0;
    end

        %Find everywhere that the data hits that peak value
        peakApprox = find(peakVal-0.000001 < posRad & posRad < peakVal+0.000001);
        
        %Check if there are any weird outliers (they should all be roughly
        %sequential)
        outlier = 1;
        while outlier == 1
            if length(peakApprox)== 1
                outlier = 0;
            else
                peakL = length(peakApprox);
                peakLMid = round(peakL/2);
                if peakApprox(1)+ peakLMid + 2 < peakApprox(peakLMid) || peakApprox(end) - peakLMid - 2 > peakApprox(peakLMid)
                    if peakApprox(1)+ peakLMid + 2 < peakApprox(peakLMid)
                        peakApprox = peakApprox(2:end);
                    else
                        peakApprox = peakApprox(1:end-1);
                    end
                else
                    outlier = 0;
                end
            end
        end
        
        if abs(peakVal) <= 0.5
            %The peak position will be in the middle of the values found by
            %peakApprox
            peaks(j) = round(peakApprox(1) + (peakApprox(end)-peakApprox(1))/2);
            %Then cut the data we consider for the peaks so we don't find that peak
            %again
            posPeaks = posRad(peaks(j):end);
            j = j + 1;
            i = i +1;
            %Check if we've basically reached equilibrium
            if abs(peakVal) < peakTol 
                %Cut the data here to get rid of the flat bit at the end
                if peaks(end) > peaks(end-1)
                    posRad = posRad(1:peaks(end));
                    t = t(1:peaks(end));
                else
                    posRad = posRad(1:peaks(end-1));
                    t = t(1:peaks(end-1));
                end
                    peaks = peaks(1:end-1);
                    finding = 0;
            end
        else
            instPeak = round(peakApprox(1) + (peakApprox(end)-peakApprox(1))/2);
            posPeaks = posRad(instPeak:end);
            i = i+1;
        end

end



%% ------ Extract servo properties based on data
if length(peaks) <= 2
    warning('Error: Not enough peaks for calculation');
    T_damp = 0;
    omega_damp = 0;
    zeta_raw = 0;
    omega_nat_raw = 0;
    delta = 0;
else
    for i=1:length(peaks)-2
        dampedPer(i) = (t(peaks(i+2)) - t(peaks(i)));
        dampedFreq(i) = (2*pi)/dampedPer(i);
        logDec(i) = log(abs(posRad(peaks(i)))/abs(posRad(peaks(i+2))));
        dampRat(i) = logDec(i)/sqrt((2*pi)^2+logDec(i)^2);
        natFreq(i) = dampedFreq(i)/sqrt(1-dampRat(i)^2);
    end
    %Take the average of the parameters found via the peaks to get the usable
    %values
    T_damp = mean(dampedPer);
    omega_damp = mean(dampedFreq);
    zeta = mean(dampRat);
    omega_nat = mean(natFreq);
    delta = mean(logDec);
end

%% --- Plot shit
%------ Plot the peaks and the tolerance lines to check on things

fig = tiledlayout(2,3);
title(fig,fileName,'Interpreter', 'none');
nexttile(1,[1,3])
plot(t,posRad);
hold on
plot(t(peaks),posRad(peaks),'o')  
yline(peakTol,':','Color',[0.4 0.4 0.4]);
yline(-peakTol,':','Color',[0.4 0.4 0.4]);
yline (0.5, ':','Color',[0.4 0.4 0.4]);
yline (-0.5, ':','Color',[0.4 0.4 0.4]);
yline (0, '--k');
ylabel('Position (rad)');
xlabel('Time (s)');
%plot(t,charEq(vals));

%------ Plot all of the values over the peaks to see the variance
%Damped Period
nexttile
plot(dampedPer,'-o')
title('Damped Period'); 
d = 2;

perPlotMin = round(min(dampedPer),d, 'significant');
perPlotMax = round(max(dampedPer),d, 'significant'); 
if perPlotMax < max(dampedPer)
    perPlotMax = max(dampedPer);
elseif perPlotMin > min(dampedPer)
    perPlotMin = min(dampedPer);
end
if perPlotMax == perPlotMin
    perPlotMax = perPlotMax + 0.1;
end
axis([1 (length(peaks)-2) perPlotMin perPlotMax]);
grid on
yline(T_damp,'--');
xlabel(append('Averaged Value: ', num2str(T_damp))) 

%Damping Ratio
nexttile
plot(dampRat, '-o');
title('Damping Ratio')
grid on
dampRatPlotMin = round(min(dampRat),d, 'significant');
dampRatPlotMax = round(max(dampRat),d, 'significant'); 
if dampRatPlotMax == dampRatPlotMin
    dampRatPlotMax = dampRatPlotMax + 10^-(d);
end
if dampRatPlotMin > min(dampRat)
    dampRatPlotMin = dampRatPlotMin - 10^-(d);
end
if dampRatPlotMax < max(dampRat)
    dampRatPlotMax = dampRatPlotMax + 10^-(d);
end
axis([1 (length(peaks)-2) dampRatPlotMin dampRatPlotMax]);
yline(zeta,'--');
xlabel(append('Averaged Value: ', num2str(zeta)));

%Frequencies

nexttile
plot(dampedFreq, '-o');
hold on
plot(natFreq,'-o');

title('Frequencies')
grid on
freqPlotMin = min(round(min(natFreq),d, 'significant'),round(min(dampedFreq),d, 'significant'));
if freqPlotMin > min(natFreq)
    freqPlotMin = freqPlotMin - 10^-(d-1);
elseif freqPlotMin > min(dampedFreq)
    freqPlotMin = freqPlotMin - 10^-(d-1);
end
freqPlotMax = max(round(max(natFreq),d, 'significant'),round(max(dampedFreq),d, 'significant')); 
if freqPlotMax < max(natFreq)
    freqPlotMax = freqPlotMax + 10^-(d-1);
elseif freqPlotMax < max(dampedFreq)
    freqPlotMax = freqPlotMax + 10^-(d-1);
end
if freqPlotMax == freqPlotMin
    freqPlotMax = freqPlotMax + 10^-(d-1);
end
axis([1 (length(peaks)-2) freqPlotMin freqPlotMax]);
yline(omega_damp,'--','Color',[0 0.4470 0.7410]);
yline(omega_nat,'--','Color',[0.8500 0.3250 0.0980]);
legend('Damped Frequency','Natural Frequency')
xlabel({append('Averaged Values: \omega_{damp} = ',num2str(omega_damp),', ','\omega_{nat} = ',num2str(omega_nat))})
%keyboard
end



