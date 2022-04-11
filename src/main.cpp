/*********************************************************************

*********************************************************************/
#include <arduino.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Wire.h>
#include <SPI.h>

// https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/nrf52-adc
#define VBAT_MV_PER_LSB       (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER          (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP     (1.403F)        // Compensation factor for the VBAT divider
#define REAL_VBAT_MV_PER_LSB  (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

#define ADS8881_MAX_SCALE     (131071.0)
#define ADS8881_MV_PER_LSB    (3300.0 / ADS8881_MAX_SCALE)

const byte interruptPin = 15;

/* data read from the DATA register, byte 0*/
uint8_t data_reg_spi_0;
/* data read from the DATA register, byte 1*/
uint8_t data_reg_spi_1;
/* data read from the DATA register, byte 2*/
uint8_t data_reg_spi_2;
/* data read from the DATA register, byte 3*/
uint8_t data_reg_spi_3;
/* flag for data ready from MAX11270*/
bool data_reg_ready = false;

// MAX11210 8-bit Command unsigned char
// ------------------------------------------------------------------------
// name     B7         B6         B5     B4     B3     B2     B1     B0 
// ------------------------------------------------------------------------
// COMMAND  START = 1  MODE = 0   CAL   IMPD   RATE3  RATE2  RATE1  RATE0
// COMMAND  START = 1  MODE = 1   RS4   RS3    RS2    RS1    RS0    R/!W
// MAX11210 Status & control registers

// ------------------------------------------------------------------------
//     name     B7     B6     B5     B4     B3     B2     B1     B0
// ------------------------------------------------------------------------
// 0x0 STAT   INRESET  ERROR  —     —   PDSTAT1 PDSTAT0 RDERR AOR 
//			   RATE3  RATE2 RATE1 RATE0  SYSGOR  DOR    MSTAT   RDY 
// 0x1 CTRL1    EXTCK SYNCMODE PD1 PD0   U/~B   FORMAT  SCYCLE  CONTSC 
// 0x2 CTRL2    DGAIN1 DGAIN0 BUFEN LPMODE PGAEN  PGAG2  PGAG1   PGAG0 
// 0x3 CTRL3     —     —   ENMSYNC MODBITS DATA32  —     —       —

// LED_BLUE
// LED_RED

// Experimental:
// #include <FreeRTOSConfig.h>
// #define configTICK_RATE_HZ     100

/*
    Flow Sensor UUID
    a7ea14cf-val-43ba-ab86-1d6e136a2e9e
    Service val = 1000
    Characteristic val = 1100
*/
#define FLOW_SENSOR_UUID(val) \
  (const uint8_t[]) { \
    0x9e, 0x2e, 0x6a, 0x13, 0x6e, 0x1d, 0x86, 0xab, \
    0xba, 0x43, (uint8_t) (val & 0xff), (uint8_t) (val >> 8), 0xcf, 0x14, 0xea, 0xa7  \
  }

/* Flow Sensor Service */
BLEService        service                   (FLOW_SENSOR_UUID(0x1000));
/* Flow Sensor Characteristic */
BLECharacteristic characteristic            (FLOW_SENSOR_UUID(0x1100));

/* A7 for feather nRF52832 */
uint32_t vbat_pin = PIN_VBAT;
/* Battery voltage in miliVolts */
float vbat_mv;
/* Estimated battery percentage */
uint8_t vbat_per;
/* Last battery percentage transmitted */
uint8_t last_vbat_per;

// BLE Services
/* OTA DFU service */
// BLEDfu  bledfu;
/* device information */
BLEDis  bledis;
/* UART over ble */
BLEUart bleuart;
/* battery service */
BLEBas  blebas;

uint32_t ADS_value;

const int chipSelectPin = 7;

uint32_t counter = 0;

bool notify_enabled = false;

float values_50_Hz[5];
uint8_t counter_50_Hz = 0;

void request_continuous_read();
void request_stop_continuous_read();

float average(float * array, int len)  // assuming array is int.
{
  double sum = 0.0 ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}

/* Sensor Measurement Task Handler */
TaskHandle_t sensor_task_handle; 

/* Callback function when changing the notify of the Characteristic */
void flow_sensor_notify_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t value)
{
    Serial.print("Notify callback:  -  ");
    // Serial.print("Value: ");
    // Serial.println(value);
    notify_enabled = (value == 0x0001);
    Serial.println(notify_enabled);

    if (notify_enabled)
    {
        request_continuous_read();
    }
    else
    {
        request_stop_continuous_read();
    }

}

/* Function to init the service and the characteristic */
int setupBLEFlowSensor(void)
{
    VERIFY_STATUS( service.begin() );

    characteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY | CHR_PROPS_INDICATE);
    characteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    characteristic.setCccdWriteCallback(flow_sensor_notify_callback);
    characteristic.setMaxLen(10 * sizeof(float));

    characteristic.setUserDescriptor("voltage (mV)");
    VERIFY_STATUS( characteristic.begin() );
    
    // uint16_t len = 20;
    // char buf[len];
    // String("Teste").toCharArray(buf, len);
    // float val = 0.0;
    characteristic.writeFloat(0.0);

    return 0;
}

/* Helper function to print binary */
void printBinary(byte inByte)
{
    for (int b = 7; b >= 0; b--)
    {
        Serial.print(bitRead(inByte, b));
    }
}

/* Helper function to print binary */
void printBinary16(uint16_t inByte)
{
    for (int b = 15; b >= 0; b--)
    {
        Serial.print(bitRead(inByte, b));
    }
}

/* Helper function to print binary */
void printBinary32(uint32_t inByte)
{
    for (int b = 31; b >= 0; b--)
    {
        Serial.print(bitRead(inByte, b));
    }
}

void MAX11270_calibrate()
{
    digitalWrite(chipSelectPin, LOW);
    uint8_t data_request_CAL = B10100000;
    SPI.transfer(data_request_CAL);
    digitalWrite(chipSelectPin, HIGH);
    delay(200);
}

void print_STAT()
{
    Serial.print("STAT:  ");
    digitalWrite(chipSelectPin, LOW);
    uint8_t data_request_STAT = B11000001;
    SPI.transfer(data_request_STAT);
    uint8_t data_spi_2 = SPI.transfer(0x00);
    printBinary(data_spi_2);
    Serial.print(" ");
    byte data_spi_3 = SPI.transfer(0x00);
    printBinary(data_spi_3);
    Serial.print(" ");
    byte data_spi_4 = SPI.transfer(0x00);
    printBinary(data_spi_4);
    digitalWrite(chipSelectPin, HIGH);
    Serial.println("");
}

void print_CTRL1()
{
    Serial.print("CTRL1: ");
    digitalWrite(chipSelectPin, LOW);
    uint8_t data_request_CTRL1 = B11000011;
    SPI.transfer(data_request_CTRL1);
    uint8_t data_spi_2 = SPI.transfer(0x00);
    printBinary(data_spi_2);
    Serial.print(" ");
    byte data_spi_3 = SPI.transfer(0x00);
    printBinary(data_spi_3);
    Serial.print(" ");
    byte data_spi_4 = SPI.transfer(0x00);
    printBinary(data_spi_4);
    digitalWrite(chipSelectPin, HIGH);
    Serial.println("");
}

void print_CTRL2()
{
    Serial.print("CTRL2: ");
    digitalWrite(chipSelectPin, LOW);
    uint8_t data_request_CTRL2 = B11000101;
    SPI.transfer(data_request_CTRL2);
    uint8_t data_spi_2 = SPI.transfer(0x00);
    printBinary(data_spi_2);
    Serial.print(" ");
    byte data_spi_3 = SPI.transfer(0x00);
    printBinary(data_spi_3);
    Serial.print(" ");
    byte data_spi_4 = SPI.transfer(0x00);
    printBinary(data_spi_4);
    digitalWrite(chipSelectPin, HIGH);
    Serial.println("");
}

void print_CTRL3()
{
    Serial.print("CTRL3: ");
    digitalWrite(chipSelectPin, LOW);
    uint8_t data_request_CTRL3 = B11000111;
    SPI.transfer(data_request_CTRL3);
    uint8_t data_spi_2 = SPI.transfer(0x00);
    printBinary(data_spi_2);
    Serial.print(" ");
    byte data_spi_3 = SPI.transfer(0x00);
    printBinary(data_spi_3);
    Serial.print(" ");
    byte data_spi_4 = SPI.transfer(0x00);
    printBinary(data_spi_4);
    digitalWrite(chipSelectPin, HIGH);
    Serial.println("");
}

void print_CTRL4()
{
    Serial.print("CTRL4: ");
    digitalWrite(chipSelectPin, LOW);
    uint8_t data_request_CTRL4 = B11001001;
    SPI.transfer(data_request_CTRL4);
    uint8_t data_spi_2 = SPI.transfer(0x00);
    printBinary(data_spi_2);
    Serial.print(" ");
    byte data_spi_3 = SPI.transfer(0x00);
    printBinary(data_spi_3);
    Serial.print(" ");
    byte data_spi_4 = SPI.transfer(0x00);
    printBinary(data_spi_4);
    digitalWrite(chipSelectPin, HIGH);
    Serial.println("");
}

void print_CTRL5()
{
    Serial.print("CTRL5: ");
    digitalWrite(chipSelectPin, LOW);
    uint8_t data_request_CTRL5 = B11001011;
    SPI.transfer(data_request_CTRL5);
    uint8_t data_spi_2 = SPI.transfer(0x00);
    printBinary(data_spi_2);
    Serial.print(" ");
    byte data_spi_3 = SPI.transfer(0x00);
    printBinary(data_spi_3);
    Serial.print(" ");
    byte data_spi_4 = SPI.transfer(0x00);
    printBinary(data_spi_4);
    digitalWrite(chipSelectPin, HIGH);
    Serial.println("");
}

void request_single_read()
{
    //Requesting single reading from MAX11270
    digitalWrite(chipSelectPin, LOW);
    byte data_request_single_read = B10000000;
    SPI.transfer(data_request_single_read);
    digitalWrite(chipSelectPin, HIGH);
}

void request_continuous_read(void)
{
    /*
     * Change Control 1 register
     * CTRL1    EXTCK SYNCMODE PD1 PD0   U/~B   FORMAT  SCYCLE  CONTSC 
     *            0       0     0   0      1       1       1      1
    */
    digitalWrite(chipSelectPin, LOW);
    uint8_t data_request_write_ctrl1 = B11000010;
    SPI.transfer(data_request_write_ctrl1);
    uint8_t data_to_write_ctrl1 = B00001111;
    SPI.transfer(data_to_write_ctrl1);
    digitalWrite(chipSelectPin, HIGH);
    print_CTRL1();
    request_single_read();
}

void request_stop_continuous_read(void)
{
    /*
     * Change Control 1 register
     * CTRL1    EXTCK SYNCMODE PD1 PD0   U/~B   FORMAT  SCYCLE  CONTSC 
     *            0       0     0   0      1       1       0      1
    */
    digitalWrite(chipSelectPin, LOW);
    uint8_t data_request_write_ctrl1 = B11000010;
    SPI.transfer(data_request_write_ctrl1);
    uint8_t data_to_write_ctrl1 = B00001110;
    SPI.transfer(data_to_write_ctrl1);
    digitalWrite(chipSelectPin, HIGH);
    print_CTRL1();
}

/* Read battery voltage (mV) */
float readVBAT(void) {
    float raw;

    // Set the analog reference to 3.0V (default = 3.6V)
    analogReference(AR_INTERNAL_3_0);

    // Set the resolution to 12-bit (0..4095)
    analogReadResolution(12); // Can be 8, 10, 12 or 14

    // Let the ADC settle
    delay(1);

    // Get the raw 12-bit, 0..3000mV ADC value
    raw = analogRead(vbat_pin);

    // Set the ADC back to the default settings
    analogReference(AR_DEFAULT);
    analogReadResolution(10);

    // Convert the raw value to compensated mv, taking the resistor-
    // divider into account (providing the actual LIPO voltage)
    // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
    return raw * REAL_VBAT_MV_PER_LSB;
}

/* 
    Convert Voltage (mV) to percentage
    based on LiPo battery behaviour.
 */
uint8_t mvToPercent(float mvolts) {
  if(mvolts<3300)
    return 0;

  if(mvolts <3600) {
    mvolts -= 3300;
    return mvolts/30;
  }

  mvolts -= 3600;

  if ((10 + (mvolts * 0.15F )) > 100)
  {
    return 100;
  }

  return 10 + (mvolts * 0.15F );  // thats mvolts /6.66666666
}

/* callback invoked when central connects */
void connect_callback(uint16_t conn_handle)
{
    // Get the reference to current connection
    BLEConnection* connection = Bluefruit.Connection(conn_handle);

    char central_name[32] = { 0 };
    connection->getPeerName(central_name, sizeof(central_name));

    Serial.print("Connected to ");
    Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
    (void) conn_handle;
    (void) reason;

    Serial.print("Disconnected, reason = 0x");
    Serial.println(reason, HEX);

    notify_enabled = false;
}

/* Function to init BLE advertisement */
void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
//   Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.addService(service);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

/* Sensor measurement Task
   * - runs every 100ms (10Hz)
   * - Applies a digital filter on the data collected.
   * 
   * 
   */
void Flow_vTaskFunction( void * pvParameters )
{
    Serial.println("Thread started");
    TickType_t xLastWakeTime;
    TickType_t xFrequency = 10;
    // 102 / 1024 =~ 99.6 ms
    // 10 / 1024 =~ 9.76 ms
    // 1 / 1024 =~ 0,976 ms

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for( ;; )
    {
        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if(data_reg_ready)
        {   
            data_reg_ready = false;

            uint32_t data_reg_spi = data_reg_spi_0;
            data_reg_spi = data_reg_spi << 8;
            data_reg_spi = data_reg_spi | data_reg_spi_1;
            data_reg_spi = data_reg_spi << 8;
            data_reg_spi = data_reg_spi | data_reg_spi_2;
            
            float data_reg_mili_volts = ((float) data_reg_spi / pow(2, 24)) * 3300;
            
            values_50_Hz[counter_50_Hz++] = data_reg_mili_volts;
            if (counter_50_Hz > 4)
            {
                counter_50_Hz = 0;
                data_reg_mili_volts = average(values_50_Hz, 5);
                Serial.println((double) data_reg_mili_volts, 3);
                if (notify_enabled)
                {
                    characteristic.notify32(data_reg_mili_volts);
                }
            }
        }
    }
}

void data_ready()
{
    digitalWrite(chipSelectPin, LOW);
    uint8_t data_request_read_data = B11001101;
    SPI.transfer(data_request_read_data);
    data_reg_spi_0 = SPI.transfer(0x00);
    data_reg_spi_1 = SPI.transfer(0x00);
    data_reg_spi_2 = SPI.transfer(0x00);
    data_reg_spi_3 = SPI.transfer(0x00);
    digitalWrite(chipSelectPin, HIGH);
    data_reg_ready = true;
}

void setup()
{
    Serial.begin(115200);

    delay_ns(50);
    SPI.begin();
    SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
    pinMode(chipSelectPin, OUTPUT);
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), data_ready, FALLING);

#if CFG_DEBUG
    // Blocking wait for connection when debug mode is enabled via IDE
    while ( !Serial ) yield();
#endif

    // vNopDelayMS(1000); // prevents usb driver crash on startup, do not omit this
    // while(!Serial);

    Serial.println("");
    Serial.println("BLE Flow Sensor Device");
    Serial.println("----------------------\n");
    Serial.print("configCPU_CLOCK_HZ: ");
    Serial.println(configCPU_CLOCK_HZ);
    Serial.print("configTICK_RATE_HZ: ");
    Serial.println(configTICK_RATE_HZ); 
    Serial.print("portTICK_PERIOD_MS: ");
    Serial.println(portTICK_PERIOD_MS);
    Serial.print("Calculated portTICK_PERIOD_MS: ");
    Serial.println(1000.0 / (float)configTICK_RATE_HZ);


    print_STAT();
    print_CTRL1();
    print_CTRL2();
    print_CTRL3();
    print_CTRL4();
    print_CTRL5();

    Serial.println("");
    Serial.println("Changing the registers.");

    /*
     * Change Control 1 register
     * CTRL1    EXTCK SYNCMODE PD1 PD0   U/~B   FORMAT  SCYCLE  CONTSC 
     *            0       0     0   0      1       1       1      0
    */
    digitalWrite(chipSelectPin, LOW);
    uint8_t data_request_write_ctrl1 = B11000010;
    SPI.transfer(data_request_write_ctrl1);
    uint8_t data_to_write_ctrl1 = B00001110;
    SPI.transfer(data_to_write_ctrl1);
    digitalWrite(chipSelectPin, HIGH);

    print_CTRL1();
    
    /*
     * Change Control 2 register
     CTRL2    DGAIN1 DGAIN0 BUFEN LPMODE PGAEN  PGAG2  PGAG1   PGAG0 
                0       0     0     0      0       0       0      0
    */
    digitalWrite(chipSelectPin, LOW);
    uint8_t data_request_write_ctrl2 = B11000100;
    SPI.transfer(data_request_write_ctrl2);
    uint8_t data_to_write_ctrl2 = B00100000;
    SPI.transfer(data_to_write_ctrl2);
    digitalWrite(chipSelectPin, HIGH);

    print_CTRL2();
    Serial.println("");
    Serial.println("Calling self calibration.");
    MAX11270_calibrate();
    Serial.println("");

    // Setup the BLE LED to be enabled on CONNECT
    // Note: This is actually the default behavior, but provided
    // here in case you want to control this LED manually via PIN 19
    Bluefruit.autoConnLed(true);

    // Config the peripheral connection with maximum bandwidth 
    // more SRAM required by SoftDevice
    // Note: All config***() function must be called before begin()
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

    Bluefruit.begin();
    Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
    //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    Bluefruit.setName("Flow Sensor");

    // // To be consistent OTA DFU should be added first if it exists
    // bledfu.begin();

    // Configure and Start Device Information Service
    bledis.setManufacturer("RUG");
    bledis.setModel("Sencilia Sensor");
    bledis.begin();

    // Setup custom service and characteristic
    setupBLEFlowSensor();

    // Configure and Start BLE Uart Service
    bleuart.begin();

    // Start BLE Battery Service
    readVBAT();
    vbat_mv = readVBAT();
    vbat_per = mvToPercent(vbat_mv);
    last_vbat_per = vbat_per;
    blebas.begin();
    blebas.write(vbat_per);

    // Set up and start advertising
    startAdv();

    char buf[64];
    int count = snprintf(buf, 64, "%ld", counter++);
    bleuart.write( buf, count );

    xTaskCreate(Flow_vTaskFunction,       "BLE Flow Task", 256, NULL,    tskIDLE_PRIORITY + 3,    &sensor_task_handle);
}

void loop()
{
    // Get a raw ADC reading
    vbat_mv = readVBAT();
    // Convert from raw mv to percentage (based on LIPO chemistry)
    vbat_per = mvToPercent(vbat_mv);
    // Write to BLE battery service
    blebas.write(vbat_per);
    
    // request_single_read();

    // Serial.println("");
    delay(500);
}

