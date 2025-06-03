clear all
errorVal = [];
errorPerc = [];
magBits = [];
magMom = [];
masses = [];
%%
% clearvars -except errorVal errorPerc magBits magMom masses 
clear all
close all

pause('on')
lib_name = '';

if strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h');
end

% Control table address
ADDR_TORQUE_ENABLE          = 64;                 % Control table address is different in Dynamixel model
ADDRGOAL_POSITION           = 116;
ADDR_PRESENT_POSITION       = 132;
ADDR_PRESENT_LOAD           = 126;
ADDR_PROFILE_VELOCITY       = 112;

% Data Byte Length
LEN_GOAL_POSITION       = 4;
LEN_PRESENT_POSITION    = 4;
LEN_PRESENT_LOAD        = 2;
LEN_PROFILE_VELOCITY    = 4;

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID                     = 1;            % Dynamixel#1 ID: 1
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM4';       % Check which port is being used on your controller
% ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 5;           % Dynamixel moving status threshold
DXL_PROFILE_VELOCITY        = 62;
% DXL_PROFILE_VELOCITY        = 80;

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

port_num = portHandler(DEVICENAME);

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

% Initialize Groupsyncwrite Structs
groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDRGOAL_POSITION, LEN_GOAL_POSITION);

% Initialize Groupsyncread Structs for Present Position
groupread_num = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD);
groupread_num2 = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_addparam_result = false;              % AddParam result
dxl_getdata_result = false;               % GetParam result
dxl_goal_position = 2048;
tStep = 0.05;

if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end

if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel #%d has been successfully connected \n', DXL_ID);
end

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PROFILE_VELOCITY, DXL_PROFILE_VELOCITY);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Set velocity in Dynamixel #%d successfully \n', DXL_ID);
end

dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL_ID);
if dxl_addparam_result ~= true
    fprintf('[ID:%03d] groupSyncRead addparam failed', DXL_ID);
    return;
end

dxl_addparam_result = groupSyncReadAddParam(groupread_num2, DXL_ID);
if dxl_addparam_result ~= true
    fprintf('[ID:%03d] groupSyncRead addparam failed', DXL_ID);
    return;
end

% Add Dynamixel goal position value to the Syncwrite storage
dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL_ID, typecast(int32(dxl_goal_position), 'uint32'), LEN_GOAL_POSITION);
if dxl_addparam_result ~= true
    fprintf('[ID:%03d] groupSyncWrite addparam failed', DXL_ID);
    return;
end

% Syncwrite goal position
groupSyncWriteTxPacket(groupwrite_num);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
end

% Clear syncwrite parameter storage
groupSyncWriteClearParam(groupwrite_num);

if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
end

index = 1;
tVec = [];
tVec(1) = 0;
t_s = tic;
while tVec(end) <= 5
        % Syncread present load
        groupSyncReadTxRxPacket(groupread_num);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        end

        groupSyncReadTxRxPacket(groupread_num2);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        end
    tVec(index) = toc(t_s);
    % Get each Dynamixel present position and load value
    dxl_present_load(index) = groupSyncReadGetData(groupread_num, DXL_ID, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD);
    dxl_present_pos(index) = groupSyncReadGetData(groupread_num2, DXL_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    fprintf('%.2f Load:%03d \n',tVec(index),dxl_present_load(index));
    index = index+1;
end

if input('Press any key to close port.\n', 's') == ESC_CHARACTER
end
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

fprintf('Closed port successfully. \n')

tVec = tVec-tVec(1);

dxl_present_load_raw = dxl_present_load;
dxl_present_pos_raw = dxl_present_pos;

dxl_present_pos = dxl_present_pos - 2048;
dxl_present_pos = dxl_present_pos.*.088;
dxl_present_pos = dxl_present_pos.*(pi/180);
m = 2; %kg
l = 39/1000; %m
F = m*9.81; %N

convEq = @(bits) 0.0029*bits + .1892;

for i=1:length(tVec)
    if dxl_present_load(i) > 60000
        dxl_present_load(i) = dxl_present_load(i) - 2^16;
    end
end
dxl_load = (dxl_present_load*.001*2.5)/.60;

% dxl_load = convEq(dxl_load);

dLoaddt = dxl_load(2:end)-dxl_load(1:end-1);
for i=1:length(dLoaddt)
    if dLoaddt(i) > .005
        onsetID = i;
        break
    end
end

dt = round(mean(tVec(2:end)-tVec(1:end-1)),3);
tVecIdeal = 0:dt:dt*(length(tVec)-1);

for i=1:length(tVec)
    theta(i) = abs(dxl_present_pos(i));
    if i <= onsetID
        torqueIdeal(i) = 0;
    else
        torqueIdeal(i) = l*cos(theta(i))*F*cos(theta(i));
    end
end

display(['Ideal Torque: ' num2str(torqueIdeal(end),'%.3f')])

plot(tVec,dxl_load)
hold on
plot(tVec,torqueIdeal);


ts = tVec(onsetID);
loadNorm = dxl_load./torqueIdeal(end);
thres = loadNorm(end)*.63;
[~,thresID] = min(abs(loadNorm-thres));
tau = tVecIdeal(thresID)-ts;
h = diff(loadNorm)./diff(tVecIdeal);
hMax = max(h);
c = exp(-tVecIdeal./tau);
input = ifft(fft(dxl_load)./fft(c)).*sum(c);

plot(tVecIdeal,input(1:length(tVecIdeal)))

% errorVal = [errorVal torqueIdeal(end) - dxl_load(end)];
% errorPerc = [errorPerc [errorVal(end)/dxl_load(end); errorVal(end)/torqueIdeal(end)]];
% magBits = [magBits dxl_present_load(end)];
% magMom = [magMom dxl_load(end)];
% masses = [masses 4];
