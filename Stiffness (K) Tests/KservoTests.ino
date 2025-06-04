#include <DynamixelSDK.h>

// AX-series Control table address
#define ADDR_AX_TORQUE_ENABLE           24
#define ADDR_AX_PRESENT_POSITION        36
#define ADDR_AX_GOAL_POSITION           30
#define ADDR_AX_P_GAIN                  28
#define ADDR_AX_PRESENT_TEMP            43        

#define PROTOCOL_VERSION                1.0

#define DXL_ID                          1
#define BAUDRATE                        1000000
#define DEVICENAME                      "1"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0

#define ESC_ASCII_VALUE                 0x1b

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);

  Serial.println("Start..");

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  unsigned long t = 0;
  //int dxl_comm_position[10] = {2060, 2150, 2070, 2140, 2080, 2130, 2090, 2120, 2100, 2110}; //K = 16
  //int dxl_comm_position[10] = {2080, 2161, 2089, 2152, 2098, 2143, 2107, 2134, 2116, 2125}; //K = 32
  //int dxl_comm_position[10] = {2080, 2134, 2086, 2128, 2092, 2122, 2098, 2116, 2104, 2110}; //K = 48
  //int dxl_comm_position[10] = {2062, 2116, 2068, 2110, 2074, 2104, 2080, 2098, 2086, 2092}; //K = 64
  //int dxl_comm_position[10] = {2068, 2086, 2104, 2092, 2074, 2050, 2062, 2098, 2080, 2056}; //K = 80
  int dxl_comm_position[10] = {2045, 2087, 2093, 2075, 2081, 2069, 2051, 2063, 2057}; //K = 96
  //int dxl_comm_position[10] = {2090, 2072, 2060, 2042, 2048, 2078, 2084, 2066, 2054}; //K = 112
  //int dxl_comm_position[10] = {2045, 2090, 2050, 2085, 2055, 2080, 2060, 2075, 2065, 2070}; //K = 128
  //int dxl_comm_position[10] = {2075, 2051, 2069, 2057, 2063, 2045, 2081, 2087}; //K = 144
  //int dxl_comm_position[10] = {2040, 2085, 2045, 2080, 2050, 2075, 2055, 2070, 2060, 2065}; //K = 160
  //int dxl_comm_position[10] = {2081, 2057, 2075, 2063, 2051, 2069, 2045}; //K = 224
  //int dxl_comm_position[10] = {2045, 2090, 2050, 2085, 2055, 2080, 2060, 2075, 2065, 2070}; //K = 254
  int dxl_rest_position = 2020;
  int dxl_p_gain = 96;

  uint8_t dxl_error = 0;
  int16_t dxl_present_position = 0;
  uint8_t dxl_present_temp = 0;

  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(BOARD_LED_PIN, OUTPUT);

  // Open port
  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }
  //dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
    Serial.print("Dynamixel ");
    Serial.print("has NOT connected \n");
  }
  else
  {
    Serial.print("Dynamixel ");
    Serial.print("has been successfully connected \n");
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_P_GAIN, dxl_p_gain, &dxl_error);
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_GOAL_POSITION, dxl_rest_position, &dxl_error);
  
  while(digitalRead(BOARD_BUTTON_PIN)==LOW);

  Serial.print("Comm.");
  Serial.print("\t");
  Serial.print("Actual");
  Serial.println(" ");
  
  int ii = 0;
  while(ii < 9) {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_GOAL_POSITION, dxl_comm_position[ii], &dxl_error);  
    delay(5000);
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);
  
    Serial.print(dxl_comm_position[ii]);
    Serial.print('\t');
    Serial.print(dxl_present_position);
    Serial.println(" ");
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_GOAL_POSITION, dxl_rest_position, &dxl_error);
    while(digitalRead(BOARD_BUTTON_PIN)==LOW);    

    ii++;
  }
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  Serial.print("End");
  Serial.println(" ");
  dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID, ADDR_AX_PRESENT_TEMP, (uint8_t*)&dxl_present_temp, &dxl_error);
  Serial.print("Present Temp: ");
  Serial.print(dxl_present_temp);
}

void loop() {
  // put your main code here, to run repeatedly:


}
