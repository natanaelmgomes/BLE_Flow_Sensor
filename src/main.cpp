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
// const int MOSI_pin = 11;
uint32_t counter = 0;

bool notify_enabled = false;

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

/* Function to read the ADS8881 over SPI */
float read_ADS()
{
    digitalWrite(chipSelectPin, HIGH);
    delay_ns(20);
    digitalWrite(chipSelectPin, LOW);
    delay_ns(10);

    byte ADS_data_high = SPI.transfer(0x00);
    // ADS_data_high &= 0b00000011; //you only needs bits 1 and 0
    byte ADS_data_mid = SPI.transfer(0x00);
    byte ADS_data_low = SPI.transfer(0x00);

    // printBinary(ADS_data_high);
    // Serial.print(" ");
    // printBinary(ADS_data_mid);
    // Serial.print(" ");
    // printBinary(ADS_data_low);
    // Serial.print(" - ");

    //combine the three parts into one 18-bit number:
    int32_t result = ((ADS_data_high << 10) | (ADS_data_mid << 2) | (ADS_data_low >> 6));

    for (int b = 31; b >= 0; b--)
    {
        Serial.print(bitRead(result, b));
    }
    Serial.print(" - ");

    if (result > 131071)
    {
        result = result - 262143;
    }

    
    Serial.print("result ");
    Serial.println(result);

    float result_mv = (float) ((double) result * ADS8881_MV_PER_LSB);

    return result_mv;
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
    TickType_t xFrequency = 10000;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for( ;; )
    {
        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency );

        // int32_t raw_value = 
        float value_mv = read_ADS();
        while (value_mv < 0)
        {
            value_mv = read_ADS();
        }

        if (notify_enabled)
        {
            
            characteristic.notify32(value_mv);
        }

        if (digitalRead(11))
        {
            xFrequency = 1000;
        }
        else
        {
            xFrequency = 10000;
        }

    }
}

void setup()
{
    Serial.begin(115200);

    delay_ns(50);
    SPI.begin();
    SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE3));
    pinMode(chipSelectPin, OUTPUT);
    pinMode(11, INPUT);

#if CFG_DEBUG
    // Blocking wait for connection when debug mode is enabled via IDE
    while ( !Serial ) yield();
#endif

    // vNopDelayMS(1000); // prevents usb driver crash on startup, do not omit this
    // while(!Serial);

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

    blebas.write(vbat_per);


    delay(1000);
}

