function [kServoRadAvg,Rsq] = StiffnessDataProcessing()
%%
clear all
close all
stiff = [16 32 48 64 80 96 112 128 144 160 224 254]; %Stiffness values in bits
trials = 5; %Number of trials per set of commanded positions
servo = 'mx-28';
errors.scale = .001;
errors.posBits = 1; %In bits

%Lift data from text files
thetaComm = {};
thetaAct = {};
weights = {};
for k = 1:length(stiff)
    filename = [servo ' kp=' num2str(stiff(k)) '.txt']; 
    raw = importdata(filename);
    thetaComm{k} = raw(1,:);
    thetaAct{k} = raw(2,:);
    weights{k} = raw(3:end, :);
end

for j=1:length(stiff)
    [kServoDeg(j,:),kServoRad(j,:),errors, kRadError, kDegError, Rsq{j}] = ProcessTrials(weights{j}, thetaComm{j}, thetaAct{j}, stiff(j), trials, errors);
    kServoDegAvg(j) = sum(kServoDeg(j,:))/sum(kServoDeg(j,:)~=0);
    kServoRadAvg(j) = sum(kServoRad(j,:))/sum(kServoRad(j,:)~=0);
    errors.kDeg{j} = kDegError;
    errors.kRad{j} = kRadError;
    errors.kDegAvg(j) = norm(errors.kDeg{j})/trials;
    errors.kRadAvg(j) = norm(errors.kRad{j})/trials;

end
figure
kError = ones(1,length(kServoRadAvg)).*errors.kRadAvg;
plot(stiff, kServoRadAvg,'o');
hold on
pLin = polyfit(stiff, kServoRadAvg,1)
p2nd = polyfit(stiff, kServoRadAvg,2);
fLin = polyval(pLin,stiff);
f2nd = polyval(p2nd,stiff);

SStot = sum((kServoRadAvg-mean(kServoRadAvg)).^2); 
SSres = sum((kServoRadAvg-fLin).^2);
RsqFinal = 1-SSres/SStot;

plot(stiff, fLin);
plot(stiff, f2nd);
xlabel('K_p Value (bits)');
ylabel('K_{servo} Value (Nm/rad)');
grid on
legend('Data Points','Linear Fit','Quadratic Fit');
title(['Rsq = ' num2str(RsqFinal)])
keyboard
end

function [kServoDeg, kServoRad, errors, kRadError, kDegError,Rsq] = ProcessTrials(weights, thetaComm, thetaAct, stiffness, trials, errors)

torques = weights*9.81*.1103; %Where .1103 is the length of the presser
errors.torque = errors.scale*9.81*.1103;
[r,c] = size(weights);

%Convert thetas from bits to degrees and radians
thetaComm = thetaComm*.088;
thetaAct = thetaAct*.088;
errors.posDeg = errors.posBits*.088;
thetaCommRad = thetaComm*(pi/180);
thetaActRad = thetaAct*(pi/180);
errors.posRad = errors.posDeg*(pi/180);
delTheta = thetaComm - thetaAct;
errors.delThetaDeg = sqrt(errors.posDeg^2+errors.posDeg^2);
delThetaRad = thetaCommRad - thetaActRad;
errors.delThetaRad = sqrt(errors.posRad^2+errors.posRad^2);

%Define the error bar lengths
yneg = ones(1,length(thetaComm))*errors.torque/2;
ypos = yneg;
xneg = ones(1,length(thetaComm))*errors.delThetaDeg/2;
xpos = xneg;
errors.stiffNum = sqrt(errors.torque^2+errors.torque^2);
errors.stiffDegDen = sqrt(errors.delThetaDeg^2 + errors.delThetaDeg^2);
errors.stiffRadDen = sqrt(errors.delThetaRad^2 + errors.delThetaRad^2);

figure
t = tiledlayout(2,3,'TileSpacing','Compact');

torquesAll = [];
delThetaAll = [];

for i=1:r
    nexttile

    %Plot the data points
    plot(delThetaRad,torques(i,:),'o');
    hold on
    %Find p values for line of best fit for the degree and radian cases
    p(i,:) = polyfit(delTheta,torques(i,:),1);
    pRad(i,:) = polyfit(delThetaRad,torques(i,:),1);
    %Use p values to create line and plot it on the figure
    f(i,:) = polyval(pRad(i,:),delThetaRad);
    plot(delThetaRad, f(i,:));
    grid on
    %The slope of the line of best fit is k
    kServoDeg(i) = p(i,1);
    kServoRad(i) = pRad(i,1);
    %The error of k
    kDegError(i) = kServoDeg(i)*sqrt((errors.stiffDegDen/(delTheta(end)-delTheta(1)))^2+(errors.stiffNum/((p(i,2)+p(i,1)*delTheta(end))-(p(i,2)+p(i,1)*delTheta(1))))^2);
    kRadError(i) = kServoRad(i)*sqrt((errors.stiffRadDen/(delThetaRad(end)-delThetaRad(1)))^2+(errors.stiffNum/((pRad(i,2)+pRad(i,1)*delThetaRad(end))-(pRad(i,2)+pRad(i,1)*delThetaRad(1))))^2);

    %Find R^2 value for each fit
    SStot = sum((torques(i,:)-mean(torques(i,:))).^2); 
    SSres = sum((torques(i,:)-f(i,:)).^2);
    Rsq(1,i) = 1-SSres/SStot;
    title(['Trial' ' ' num2str(i)])
    subtitle(['R^2 = ' num2str(Rsq(1,i))])

    torquesAll = [torquesAll torques(i,:)];
    delThetaAll = [delThetaAll delThetaRad];
end
%     if stiffness == 254
%     keyboard
%     end
title(t, ['K_p = ' num2str(stiffness)]);
xlabel(t,'\Delta\theta (rad)');
ylabel(t,'Torque (Nm)');
if r < trials
    kServoDeg(r+1:trials) = 0;
    kServoRad(r+1:trials) = 0;
end

%Plot figure for dissertation
figure
hold on
for i=1:c
    plot(delThetaRad(i)*ones(1,r),f(:,i),'ko')
end
tauAvg = polyval([mean(pRad(:,1)) mean(pRad(:,2))],delThetaRad);
tauAvgAll = polyval([mean(pRad(:,1)) mean(pRad(:,2))],delThetaAll);

SStot = sum((torquesAll-mean(torquesAll)).^2); 
SSres = sum((torquesAll-tauAvgAll).^2);
RsqAll = 1-SSres/SStot;

plot(delThetaRad, tauAvg, '-r')
title(['K_p = ' num2str(stiffness)])
subtitle(['Rsq = ' num2str(RsqAll)])
grid on
xlabel('\Delta\theta (rad)');
xlim([0 0.16])
ylabel('Torque (Nm)');
ylim([0 1.6])
end
