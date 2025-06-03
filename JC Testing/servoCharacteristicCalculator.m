%Clarissa Goldsmith
%West Virginia University
%6/18/21

clear all
%close all

%------ CHANGE THESE VALUES
currFolder = pwd;
numJTrials = [10;0]; 
numCTrials = [10];
JFileNameBase = 'NoTNoPos_0.5kg_';
JFileLoc = {[currFolder '\Trials\No Nylon Washer\J Trials\6.22.21'],...
    [currFolder '\Trials\No Nylon Washer\J Trials\6.24.21']};
CFileLoc = {[currFolder '\Trials\No Nylon Washer\C Trials']};
CFileNameBase = 'YesTNoPos_2kg_';

%------ 
%For the J tests, use the 500g pendulum. For the c tests, use the 2000g
%pendulum.
JSetup.mass = (500+71.22)/1000; %In kg
CSetup.mass = (2000+71.22)/1000;

JSetup.length = 10.213*0.0254; %In m
CSetup.length = 11.6321*0.0254;


%------ Calculate J_pend and k_pend for both setups
JSetup.pendJ = JSetup.mass*JSetup.length^2;
CSetup.pendJ = CSetup.mass*CSetup.length^2;

JSetup.kPend = JSetup.mass*9.81*JSetup.length;
CSetup.kPend = CSetup.mass*9.81*CSetup.length;



%------ Process J trials (if any) and find values
tr = 0;
JOmegaDamp = [];
JDelta = [];
if norm(numJTrials) >= 1
    %------ Process the J trial with processPendv2
    for ii = 1:length(JFileLoc)
        for i=1:numJTrials(ii)
            tr = tr + 1;
            JFileName = append(JFileNameBase, 'Trial', string(tr));
            JFileTree = append(JFileLoc{ii}, '\', JFileName, '.txt');
            if isfile(JFileTree)
                figure
                [JTrials(tr).dampedFreq, JTrials(tr).logDec, JTrials(tr).pos, JTrials(tr).t, JTrials(tr).fig] = processPendv2(JFileTree,JFileName);
                JOmegaDamp = [JOmegaDamp JTrials(tr).dampedFreq];
                JDelta = [JDelta JTrials(tr).logDec];
            end
        end
    end
    %------ Check if there are any outlier property values and remove them
    %Basically checking if there are any bins in a histogram that would
    %contain less values than would have come out of one trial, then
    %removing them from consideration
    [omegaDN,omegaDEdges] = histcounts(JOmegaDamp);
    [deltaN,deltaEdges] = histcounts(JDelta);
    avgTrialVals = length(JDelta)*.1;
    outliers = 1;
    while outliers == 1
        if omegaDN(1) < avgTrialVals || omegaDN(end) < avgTrialVals || deltaN(1) < avgTrialVals || deltaN(end) < avgTrialVals
            if omegaDN(1) < avgTrialVals
                omegaDN = omegaDN(2:end);
                omegaDEdges = omegaDEdges(2:end);
                vals = find(JOmegaDamp < omegaDEdges(1));
                for i=1:length(vals)
                    JOmegaDamp(vals(i)) = NaN;
                    JDelta(vals(i)) = NaN;
                end
            end
            
            if omegaDN(end) < avgTrialVals
                omegaDN = omegaDN(1:end-1);
                omegDEdges = omegaDEdges(1:end-1);
                vals = find(JOmegaDamp > omegaDEdges(end));
                for i=1:length(vals)
                    JOmegaDamp(vals(i)) = NaN;
                    JDelta(vals(i)) = NaN;
                end
            end
            
            if deltaN(1) < avgTrialVals
                deltaN = deltaN(2:end);
                deltaEdges = deltaEdges(2:end);
                vals = find(JDelta < deltaEdges(1));
                for i=1:length(vals)
                    JOmegaDamp(vals(i)) = NaN;
                    JDelta(vals(i)) = NaN;
                end
            end
            
            if deltaN(end) < avgTrialVals
                deltaN = deltaN(1:end-1);
                deltaEdges = deltaEdges(1:end-1);
                vals = find(JDelta > deltaEdges(end));
                for i=1:length(vals)
                    JOmegaDamp(vals(i)) = NaN;
                    JDelta(vals(i)) = NaN;
                end
            end
        else
            outliers = 0;
        end
        JOmegaDamp = rmmissing(JOmegaDamp);
        JDelta = rmmissing(JDelta);
    end
    JZeta = 1./sqrt(1+((2*pi)./JDelta).^2);
    JOmegaNat = JOmegaDamp./sqrt(1-JZeta.^2);
   
    % Figure out error bars for J
    delJOmegaD = max(abs(mean(JOmegaDamp) - JOmegaDamp));
    delJDelta = max(abs(mean(JDelta) - JDelta));
    delJZeta = sqrt(((4*pi^2)/(((4*pi^2)/mean(JDelta)^2+1)^(3/2)*mean(JDelta)^3))^2*delJDelta^2);
    delJOmegaNat = sqrt((1/sqrt(1-mean(JZeta)^2))^2*(delJOmegaD)^2 + ((mean(JOmegaDamp)*mean(JZeta))/((1-mean(JZeta)^2)^(3/2)))^2*(delJZeta)^2);
    servoJ = JSetup.kPend/mean(JOmegaNat)^2 - JSetup.pendJ;
    delJ = sqrt(((2*JSetup.kPend)/mean(JOmegaNat)^3)^2*(delJOmegaNat)^2);
    disp(['Servo Moment of Inertia: ', num2str(servoJ), ' ',char(177), ' ', num2str(delJ), ' (', num2str(delJ/servoJ*100),'% error)'])
    
    % What if zeta was zero?
    servoJNoDamp = JSetup.kPend/mean(JOmegaDamp)^2 - JSetup.pendJ;
    delJNoDamp = sqrt(((2*JSetup.kPend)/mean(JOmegaDamp)^3)^2*(delJOmegaD)^2);
    disp(['With zeta = 0: ',num2str(servoJNoDamp), ' ',char(177), ' ', num2str(delJNoDamp) ' (', num2str(delJNoDamp/servoJNoDamp*100),'% error)']);
end

%------ Process c trials (if any) and find values
tr = 0;
COmegaDamp = [];
CDelta = [];
if norm(numCTrials) >= 1
    %------ Process the c trial with processPendv2
    for ii = 1:length(CFileLoc)
        for i=1:numCTrials(ii)
            tr = tr + 1;
            CFileName = append(CFileNameBase, 'Trial', string(i));
            CFileTree = append(CFileLoc{ii}, '\', CFileName, '.txt');
            if isfile(CFileTree)
                figure
                [CTrials(tr).dampedFreq, CTrials(tr).logDec, CTrials(tr).pos, CTrials(tr).t, CTrials(tr).fig] = processPendv2(CFileTree, CFileName);
                COmegaDamp = [COmegaDamp CTrials(tr).dampedFreq];
                CDelta = [CDelta CTrials(tr).logDec];
            end
        end
    end
    if norm(numCTrials) > 1
        %------ Check if there are any outlier property values and remove them
        %Basically checking if there are any bins in a histogram that would
        %contain less values than would have come out of one trial, then
        %removing them from consideration
        [omegaDN,omegaDEdges] = histcounts(COmegaDamp);
        [deltaN,deltaEdges] = histcounts(CDelta);
        avgTrialVals = length(CDelta)/tr;
        outliers = 1;
        while outliers == 1
            if omegaDN(1) < avgTrialVals || omegaDN(end) < avgTrialVals || deltaN(1) < avgTrialVals || deltaN(end) < avgTrialVals
                if omegaDN(1) < avgTrialVals
                    omegaDN = omegaDN(2:end);
                    omegaDEdges = omegaDEdges(2:end);
                    vals = find(COmegaDamp < omegaDEdges(1));
                    for i=1:length(vals)
                        COmegaDamp(vals(i)) = NaN;
                        CDelta(vals(i)) = NaN;
                    end
                end

                if omegaDN(end) < avgTrialVals
                    omegaDN = omegaDN(1:end-1);
                    omegDEdges = omegaDEdges(1:end-1);
                    vals = find(COmegaDamp > omegaDEdges(end));
                    for i=1:length(vals)
                        COmegaDamp(vals(i)) = NaN;
                        CDelta(vals(i)) = NaN;
                    end
                end

                if deltaN(1) < avgTrialVals
                    deltaN = deltaN(2:end);
                    deltaEdges = deltaEdges(2:end);
                    vals = find(CDelta < deltaEdges(1));
                    for i=1:length(vals)
                        COmegaDamp(vals(i)) = NaN;
                        CDelta(vals(i)) = NaN;
                    end
                end

                if deltaN(end) < avgTrialVals
                    deltaN = deltaN(1:end-1);
                    deltaEdges = deltaEdges(1:end-1);
                    vals = find(CDelta > deltaEdges(end));
                    for i=1:length(vals)
                        COmegaDamp(vals(i)) = NaN;
                        CDelta(vals(i)) = NaN;
                    end
                end
            else
                outliers = 0;
            end
            COmegaDamp = rmmissing(COmegaDamp);
            CDelta = rmmissing(CDelta);
        end
    end
    
    %Figure out error bars for C
    delCOmegaD = max(abs(mean(COmegaDamp) - COmegaDamp));
    delCDelta = max(abs(mean(CDelta) - CDelta));
    servoC = (mean(CDelta)*mean(COmegaDamp)*2*CSetup.mass)/(2*pi);
    delC = sqrt(delCDelta^2*((mean(COmegaDamp)*2*CSetup.mass)/(2*pi))+ delCOmegaD^2*((mean(CDelta)*2*CSetup.mass)/(2*pi))^2);
    disp(['Servo Damping Coefficient: ', num2str(servoC), ' ',char(177), ' ', num2str(delC), ' (', num2str(delC/servoC*100),'% error)'])
end

    