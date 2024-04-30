
## MCC Configuration Screen Shots ##
*Figure 10. MCC Configuration Pin-Out*
<img width="946" alt="image" src="https://github.com/Team-310/Team-310.github.io/assets/156128630/3b848997-cb70-4b8c-9b33-3c823dd8d3c9">

*Figure 11. MCC Configuration ESUART2*
<img width="428" alt="image" src="https://github.com/Team-310/Team-310.github.io/assets/156128630/12c0e922-4bf9-47a0-937f-61a7483d6fc9">

*Figure 12. MCC Configuration MSSP1*
<img width="442" alt="image" src="https://github.com/Team-310/Team-310.github.io/assets/156128630/53f86935-41dc-457d-9737-ef89d4f4a04f">

*Figure 13. MCC Configuration MSSP2*
<img width="446" alt="image" src="https://github.com/Team-310/Team-310.github.io/assets/156128630/489744d5-5f60-48a8-8dd3-54777dab13f9">

*Figure 14. MCC Configuration TMR2*
<img width="405" alt="image" src="https://github.com/Team-310/Team-310.github.io/assets/156128630/ebcaae43-fcd0-4646-afd2-e707fc111393">

*Figure 15. MCC Configuration Interrupt Module*
<img width="343" alt="image" src="https://github.com/Team-310/Team-310.github.io/assets/156128630/6ca37d53-3179-4cc2-97ab-32fbbb392805">

## MPLAB X Main Code ##

```c
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/i2c2_master.h"
#include "mcc_generated_files/examples/i2c2_master_example.h"
#include "mcc_generated_files/eusart2.h"
#include "application.h"
#include "bme280.h"
#include <string.h>
#include <stdio.h>

#define TC74 0x4C
#define TC74_Address 0x00

uint8_t byteArray[] = {0b11101111, 0b11101000, 0b11101101};
uint8_t byteValue;
uint8_t byteValue2;
uint8_t byteValue3;

void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();
    I2C2_Initialize();
    EUSART2_Initialize();
    SPI1_Initialize();
    SPI1_Open(SPI1_DEFAULT);

    uint8_t temp = 0;
    char dataStr[50]; // Buffer to hold the formatted sensor data string

    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();

    while (1)
    {
        WeatherStation_initialize();
        WeatherStation_Print();

        float humidity = BME280_getHumidity();
        float pressure = BME280_getPressure();

        if (humidity <= 27) {
            Press_LED_SetHigh();
        } else {
            Press_LED_SetLow();
        }
        __delay_ms(100);

        temp = I2C2_Read1ByteRegister(TC74, TC74_Address); // Read temperature from TC74

        if (temp <= 27) { //25.5
            Temp_LED_SetHigh();

            __delay_ms(50);
            CSNpin_SetLow();
            SPI1_ExchangeByte(0b11101111); // Command to control motor in one direction         
            CSNpin_SetHigh();
            __delay_ms(50);
        } else {
            Temp_LED_SetLow();

            __delay_ms(50);
            CSNpin_SetLow();
            SPI1_ExchangeByte(byteArray[2]); // Command to control motor in another direction        
            CSNpin_SetHigh();
            __delay_ms(50);
        }

        // Format the sensor data into a string
        sprintf(dataStr, "T:%u H:%.2f P:%.2f", temp, humidity, pressure);
        
        // Transmit formatted data to ESP32 over EUSART2
        for (char *ptr = dataStr; *ptr != '\0'; ptr++) {
            EUSART2_Write(*ptr);
        }
        EUSART2_Write('\n'); // Send newline for better readability at the receiver end
    }
}
```


## MPLAB X BME280 Source File ##

```c
#include "bme280.h"
#include "mcc_generated_files/examples/i2c2_master_example.h"
#include <math.h>

/**
  Section: Driver APIs
 */

uint8_t BME280_getID(void) {
    return I2C2_Read1ByteRegister(BME280_ADDR, BME280_ID_REG);
}

void BME280_reset(void) {
    I2C2_Write1ByteRegister(BME280_ADDR, BME280_RESET_REG, BME280_SOFT_RESET);
}

void BME280_sleep(void) {
    bme280_ctrl_meas.mode = BME280_SLEEP_MODE;
    I2C2_Write1ByteRegister(BME280_ADDR, BME280_CTRL_MEAS_REG, bme280_ctrl_meas.ctrlMeasReg);
}

void BME280_readFactoryCalibrationParams(void) {
    uint8_t paramBuff[24];
    I2C2_ReadDataBlock(BME280_ADDR, BME280_CALIB_DT1_LSB_REG, paramBuff, 24);
    calibParam.dig_T1 = (((uint16_t) paramBuff[1]) << 8) + paramBuff[0];
    calibParam.dig_T2 = (((int) paramBuff[3]) << 8) + paramBuff[2];
    calibParam.dig_T3 = (((int) paramBuff[5]) << 8) + paramBuff[4];
    calibParam.dig_P1 = (((uint16_t) paramBuff[7]) << 8) + paramBuff[6];
    calibParam.dig_P2 = (((int) paramBuff[9]) << 8) + paramBuff[8];
    calibParam.dig_P3 = (((int) paramBuff[11]) << 8) + paramBuff[10];
    calibParam.dig_P4 = (((int) paramBuff[13]) << 8) + paramBuff[12];
    calibParam.dig_P5 = (((int) paramBuff[15]) << 8) + paramBuff[14];
    calibParam.dig_P6 = (((int) paramBuff[17]) << 8) + paramBuff[16];
    calibParam.dig_P7 = (((int) paramBuff[19]) << 8) + paramBuff[18];
    calibParam.dig_P8 = (((int) paramBuff[21]) << 8) + paramBuff[20];
    calibParam.dig_P9 = (((int) paramBuff[23]) << 8) + paramBuff[22];
    calibParam.dig_H1 = (uint8_t) I2C2_Read1ByteRegister(BME280_ADDR, BME280_CALIB_DH1_REG);

    I2C2_ReadDataBlock(BME280_ADDR, BME280_CALIB_DH2_LSB_REG, paramBuff, 7);
    calibParam.dig_H2 = (((int) paramBuff[1]) << 8) + paramBuff[0];
    calibParam.dig_H3 = (uint8_t) paramBuff[2];
    calibParam.dig_H4 = (((int) paramBuff[3]) << 4) | (paramBuff[4] & 0xF);
    calibParam.dig_H5 = (((int) paramBuff[5]) << 4) | (paramBuff[4] >> 4);
    calibParam.dig_H6 = (short) paramBuff[6];
}

void BME280_config(uint8_t sbtime, uint8_t coeff) {
    bme280_config.t_sb = sbtime; // Set standby time;
    bme280_config.filter = coeff; // Set filter coefficient;
}

void BME280_ctrl_meas(uint8_t osrs_T, uint8_t osrs_P, uint8_t mode) {
    bme280_ctrl_meas.osrs_T = osrs_T; // Set oversampling temperature;
    bme280_ctrl_meas.osrs_P = osrs_P; // Set oversampling pressure;
    bme280_ctrl_meas.mode = mode; // Set sensor mode;
}

void BME280_ctrl_hum(uint8_t osrs_H) {
    bme280_ctrl_hum = osrs_H; // Set oversampling humidity;
}

void BME280_initializeSensor(void) {
    I2C2_Write1ByteRegister(BME280_ADDR, BME280_CONFIG_REG, bme280_config.configReg);
    I2C2_Write1ByteRegister(BME280_ADDR, BME280_CTRL_HUM_REG, bme280_ctrl_hum);
    I2C2_Write1ByteRegister(BME280_ADDR, BME280_CTRL_MEAS_REG, bme280_ctrl_meas.ctrlMeasReg);
}

void BME280_startForcedSensing(void) {
    bme280_ctrl_meas.mode = BME280_FORCED_MODE;
    I2C2_Write1ByteRegister(BME280_ADDR, BME280_CTRL_MEAS_REG, bme280_ctrl_meas.ctrlMeasReg);
}

void BME280_readMeasurements(void) {
    uint8_t sensorData[BME280_DATA_FRAME_SIZE];

    I2C2_ReadDataBlock(BME280_ADDR, BME280_PRESS_MSB_REG, sensorData, BME280_DATA_FRAME_SIZE);

    adc_H = ((uint32_t) sensorData[BME280_HUM_MSB] << 8) |
            sensorData[BME280_HUM_LSB];

    adc_T = ((uint32_t) sensorData[BME280_TEMP_MSB] << 12) |
            (((uint32_t) sensorData[BME280_TEMP_LSB] << 4) |
            ((uint32_t) sensorData[BME280_TEMP_XLSB] >> 4));

    adc_P = ((uint32_t) sensorData[BME280_PRESS_MSB] << 12) |
            (((uint32_t) sensorData[BME280_PRESS_LSB] << 4) |
            ((uint32_t) sensorData[BME280_PRESS_XLSB] >> 4));
}

float BME280_getTemperature(void) {
    float temperature = (float) BME280_compensateTemperature() / 100;
    return temperature;
}

float BME280_getPressure(void) {
    float pressure = (float) BME280_compensatePressure() / 100; // measured in hPa (equivalent to millibar)
 
    // Note: Atmospheric pressure changes with elevation. 
    // The following code finds the equivalent pressure at sea level to give accurate readings
    // in accordance with the international Standard Atmosphere.
    // The equation is: P0 = P1 (1 - (0.0065h/ (T + 0.0065h + 273.15))^(-5.257)
    // where:   P0 = calculated mean sea level pressure (hPa)   
    //          P1 = actual measured pressure (hPa))
    //          h = elevation (m) 
    //          T = temp is degrees C
    float temp = BME280_getTemperature();
    double mantissa = 1 - (0.0065 * ELEVATION / (temp + (0.0065 * ELEVATION) + 273.15));
    double adjustment = pow(mantissa, -5.257);
    float press_adj = adjustment * pressure;

    return press_adj;
}

float BME280_getHumidity(void) {
    float humidity = (float) BME280_compensateHumidity() / 1024;
    return humidity;
}

static uint32_t BME280_compensateTemperature(void) {
    long long tempV1, tempV2, t;

    tempV1 = ((((adc_T >> 3) - ((long long) calibParam.dig_T1 << 1))) * ((long long) calibParam.dig_T2)) >> 11;
    tempV2 = (((((adc_T >> 4) - ((long long) calibParam.dig_T1)) * ((adc_T >> 4) - ((long long) calibParam.dig_T1))) >> 12)*((long long) calibParam.dig_T3)) >> 14;
    t_fine = tempV1 + tempV2;
    t = (t_fine * 5 + 128) >> 8;

    return t;
}

static uint32_t BME280_compensatePressure(void) {
    long pressV1, pressV2;
    uint32_t p;

    pressV1 = (((long) t_fine) >> 1) - (long) 64000;
    pressV2 = (((pressV1 >> 2) * (pressV1 >> 2)) >> 11) * ((long) calibParam.dig_P6);
    pressV2 = pressV2 + ((pressV1 * ((long) calibParam.dig_P5)) << 1);
    pressV2 = (pressV2 >> 2)+(((long) calibParam.dig_P4) << 16);
    pressV1 = (((calibParam.dig_P3 * (((pressV1 >> 2) * (pressV1 >> 2)) >> 13)) >> 3) +
            ((((long) calibParam.dig_P2) * pressV1) >> 1)) >> 18;
    pressV1 = ((((32768 + pressV1))*((long) calibParam.dig_P1)) >> 15);

    if (pressV1 == 0) {
        return 0; // avoid exception caused by division by zero
    }

    p = (((uint32_t) (((long) 1048576) - adc_P)-(pressV2 >> 12)))*3125;
    if (p < 0x80000000) {
        p = (p << 1) / ((uint32_t) pressV1);
    } else {
        p = (p / (uint32_t) pressV1) * 2;
    }

    pressV1 = (((long) calibParam.dig_P9) * ((long) (((p >> 3) * (p >> 3)) >> 13))) >> 12;
    pressV2 = (((long) (p >> 2)) * ((long) calibParam.dig_P8)) >> 13;
    p = (uint32_t) ((long) p + ((pressV1 + pressV2 + calibParam.dig_P7) >> 4));

    return p;
}

static uint32_t BME280_compensateHumidity(void) {
    long humV;
    uint32_t h;

    humV = (t_fine - ((long) 76800));
    humV = (((((adc_H << 14) - (((long) calibParam.dig_H4) << 20) - (((long) calibParam.dig_H5) * humV)) + ((long) 16384)) >> 15) * (((((((humV * ((long) calibParam.dig_H6)) >> 10) * (((humV * ((long) calibParam.dig_H3)) >> 11) + ((long) 32768))) >> 10) + ((long) 2097152)) * ((long) calibParam.dig_H2) + 8192) >> 14));
    humV = (humV - (((((humV >> 15) * (humV >> 15)) >> 7) * ((long) calibParam.dig_H1)) >> 4));
    humV = (humV < 0 ? 0 : humV);
    humV = (humV > 419430400 ? 419430400 : humV);
    h = (uint32_t) (humV >> 12);

    return h;
}
```


## MPLAB X Application Source Code ##

```c
#include "application.h"

/**
  Section: Variable Definitions
 */

#define DEFAULT_STANDBY_TIME    BME280_STANDBY_HALFMS
#define DEFAULT_FILTER_COEFF    BME280_FILTER_COEFF_OFF
#define DEFAULT_TEMP_OSRS       BME280_OVERSAMP_X1
#define DEFAULT_PRESS_OSRS      BME280_OVERSAMP_X1
#define DEFAULT_HUM_OSRS        BME280_OVERSAMP_X1
#define DEFAULT_SENSOR_MODE     BME280_FORCED_MODE

bool weather_initialized = 0;

bool label_initial = false;

/**
  Section: Driver APIs
 */

void WeatherClick_readSensors(void) {
    if (DEFAULT_SENSOR_MODE == BME280_FORCED_MODE) {
        BME280_startForcedSensing();
    }
    BME280_readMeasurements();
}

void WeatherStation_initialize(void) {
    BME280_reset();
    __delay_ms(50);
    BME280_readFactoryCalibrationParams();
    BME280_config(BME280_STANDBY_HALFMS, BME280_FILTER_COEFF_OFF);
    BME280_ctrl_meas(BME280_OVERSAMP_X1, BME280_OVERSAMP_X1, BME280_FORCED_MODE);
    BME280_ctrl_hum(BME280_OVERSAMP_X1);
    BME280_initializeSensor();
    weather_initialized = 1;
}

void WeatherStation_Print(void) {
    float temp_string, press_string;
    uint8_t humid_string;
    char str_temp[16], str_press[16], str_hum[16];

    WeatherClick_readSensors();

    temp_string = BME280_getTemperature();
    press_string = BME280_getPressure();    // using float
    humid_string = (uint8_t) BME280_getHumidity();
    
    sprintf(str_temp, "      %.1fC", temp_string); // Temperature to String Conversion;
    sprintf(str_press, "       %u hPa", (unsigned int) press_string); // Pressure to String Conversion;
    sprintf(str_hum, "          %u%%", humid_string); // Humidity to String Conversion;
    

    printf("\nTemperature: %.1fC\r\n", temp_string);
    printf("Pressure: %u hPa\r\n", (unsigned int) press_string);
    printf("Relative Humidity: %u%%\r\n", humid_string);
}
```
