//
// Combined LTC6804 Battery Monitor and LXSR 6-NPS Current Sensor System
//
#include "driverlib.h"
#include "device.h"


// LTC6804 Command Definitions
#define LTC6804_WRCFG   0x0001                     // Write Configuration Register Group
#define LTC6804_RDCFG   0x0002                     // Read Configuration Register Group  
#define LTC6804_RDCVA   0x0004                     // Read Cell Voltage Register Group A
#define LTC6804_RDCVB   0x0006                     // Read Cell Voltage Register Group B
#define LTC6804_RDCVC   0x0008                     // Read Cell Voltage Register Group C
#define LTC6804_RDCVD   0x000A                     // Read Cell Voltage Register Group D
#define LTC6804_ADCV    0x0360                     // Start Cell Voltage ADC Conversion (Filtered Mode)
#define LTC6804_ADSTAT  0x0468                     // Start Status ADC Conversion
#define LTC6804_PLADC   0x0714                     // Poll ADC Conversion Status


// Current Sensor Defines
#define RESULTS_BUFFER_SIZE     256
#define SENSOR_SENSITIVITY      104.2f             // mV/A (typical)
#define SENSOR_VREF             2.5f               // V (Vcc/2, zero current output)
#define ADC_VREF                3.0f               // V (ADC reference voltage, LAUNCHXL-F28379D's ADC)
#define ADC_MAX_VALUE           4095               // 12-bit ADC max value (2^12 - 1)
#define OFF_SET                 0.011f             // Hall Voltage's Offset

// Battery spec
#define DT                      1.0f               // Coulomb Counting interval
#define BAT_CAPACITY_Ah         4.0f               // Battery Capacity 

// CPU TIMER
#define TIMER_FREQ_MHZ          100.0f
#define TIMER_PERIOD_SEC        1.0f

// LTC6804 Global Variables
volatile uint16_t rxBuffer[12];                    // Receive buffer for LTC6804 data
volatile uint16_t txBuffer[8];                     // Transmit buffer for LTC6804 commands
volatile bool spiTransactionComplete = false;
volatile bool adcConversionComplete = false;
const uint16_t CRC15_POLY = 0x4599;
volatile uint16_t cell1;
volatile float cell1_voltage;

// Current Sensor Global Variables
uint16_t adcAResults[RESULTS_BUFFER_SIZE];         // Raw ADC results buffer
float currentValues[RESULTS_BUFFER_SIZE];          // Calculated current values
uint16_t currentIndex;                             // Index into result buffer
volatile uint16_t bufferFull;                      // Flag to indicate buffer is full
volatile float averageCurrent;                     // Running average current
float voltageValues[RESULTS_BUFFER_SIZE];
volatile float adc_voltage;
volatile float current;
volatile float soc;


// PEC15 Table for LTC6804
static uint16_t pec15Table[256] = {0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,
    0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
    0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
    0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
    0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
    0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
    0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
    0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
    0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
    0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
    0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
    0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
    0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
    0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
    0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
    0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
    0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
    0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
    0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
    0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
    0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
    0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
    0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
};

// OCV - SOC table, OCV - SOC linear interpolation
float ocv_table[] = {3.4399f, 3.3332f, 3.3323f, 3.3319f, 3.3313f, 3.3309f, 3.3300f, 3.3122f, 3.2959f, 3.2929f, 3.2918f, 3.2909f, 3.2903f, 3.2891f, 3.2814f, 3.2646, 3.2475f, 3.2224f, 3.2078f, 3.1673f, 2.8719f};
float soc_table[] = {1.00f,0.95f, 0.90f, 0.85f, 0.80f, 0.75f, 0.70f, 0.65f, 0.60f, 0.55f, 0.50f, 0.45f, 0.40f, 0.35f, 0.30f, 0.25f, 0.20f, 0.15f, 0.10f, 0.05f, 0.00f};
#define TABLE_SIZE 21

float OCV_to_SOC(float ocv) {
    if (ocv >= ocv_table[0])
        return soc_table[0];

    if (ocv <= ocv_table[TABLE_SIZE - 1])
        return soc_table[TABLE_SIZE - 1]; 

    for (int i = 0; i < TABLE_SIZE - 1; i++) {
        float x1 = ocv_table[i];
        float x2 = ocv_table[i+1];
        float y1 = soc_table[i];
        float y2 = soc_table[i+1];

        if (ocv <= x1 && ocv > x2) {
            float ratio = (ocv - x1) / (x2 - x1);  // x2 - x1 < 0 → ratio < 0
            return y1 + ratio * (y2 - y1);
        }
    }

    return -1.0f;
}

//
// Function Prototypes
//

// LTC6804 Functions
void initSPIMaster(void);
void initSpiGpio(void);
void ltc6804_sendCommand(uint16_t command);
void ltc6804_readCellVoltages(void);
void ltc6804_startADCConversion(void);
bool ltc6804_pollADCStatus(void);
void LTC6804_write_config(void);
void init_PEC15_Table(void);
void LTC6804_read_config(void);
void LTC6804_wakeup(void);
uint16_t pec15_calc(uint8_t *data, uint8_t len);

// Current Sensor Functions
void initADC(void);
void initEPWM(void);
void initADCSOC(void);
void calculateCurrent(void);
float adcToVoltage(uint16_t adcValue);
float voltageToCurrent(float voltage);


// Set up CPU TIMER Functions
void initCPUTimer1(void);
void configCPUTimer1(void);
void startCPUTimer1(void);

// Current and voltage measurement & Coulomb Counting ISR
__interrupt void adcA1ISR(void);
__interrupt void cpuTimer1ISR(void);

//
// Main Function
//
void main(void)
{
    // Initialize device clock and peripherals
    Device_init();
    Device_initGPIO();

    // initial SOC estimation
    float ocv = 3.2901f;
    soc = OCV_to_SOC(ocv);
    
    // Initialize PIE and clear PIE registers
    Interrupt_initModule();
    Interrupt_initVectorTable();
    
    // Register interrupt handlers
    Interrupt_register(INT_TIMER1, &cpuTimer1ISR);
    Interrupt_register(INT_ADCA1, &adcA1ISR);
    
    // Initialize SPI for LTC6804
    initSpiGpio();
    initSPIMaster();
    
    // Initialize ADC system for current sensor
    initADC();
    initEPWM();
    initADCSOC();
    
    // Initialize current sensor variables
    for(currentIndex = 0; currentIndex < RESULTS_BUFFER_SIZE; currentIndex++)
    {
        adcAResults[currentIndex] = 0;
        currentValues[currentIndex] = 0.0f;
    }
    
    currentIndex = 0;
    bufferFull = 0;
    averageCurrent = 0.0f;
    
    // Enable interrupts
    Interrupt_enable(INT_TIMER1);
    Interrupt_enable(INT_ADCA1);
    EINT;
    ERTM;
    
    // Initialize LTC6804
    LTC6804_wakeup();
    LTC6804_write_config();
    LTC6804_read_config();
    
    // Set up CPU TIMER
    initCPUTimer1();
    configCPUTimer1();
    startCPUTimer1();
    
    // Start current & voltage measurement
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP);
}

//
// LTC6804 Functions
//

// Initialize SPI Master for LTC6804 Communication
void initSPIMaster(void)
{
    // Put SPI into reset before configuration
    SPI_disableModule(SPIA_BASE);
    
    // SPI configuration for LTC6804
    // LTC6804 supports up to 1MHz SPI clock
    // Use SPI Mode 0 (CPOL=1, CPHA=0)
    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL1PHA0,
                  SPI_MODE_MASTER, 1000000, 8);
    
    SPI_disableLoopback(SPIA_BASE);
    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_FREE_RUN);
    
    // Enable FIFO mode
    SPI_enableFIFO(SPIA_BASE);
    
    // Enable SPI module
    SPI_enableModule(SPIA_BASE);
}

// Configure GPIO pins for SPI communication with LTC6804
void initSpiGpio(void)
{
    // MOSI (SPI SIMO A) - GPIO 58
    GPIO_setMasterCore(58, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_58_SPISIMOA);
    GPIO_setDirectionMode(58, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(58, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(58, GPIO_QUAL_ASYNC);

    // MISO (SPI SOMI A) - GPIO 59
    GPIO_setMasterCore(59, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_59_SPISOMIA);
    GPIO_setDirectionMode(59, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(59, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(59, GPIO_QUAL_ASYNC);

    // SCLK (SPI CLK A) - GPIO 60
    GPIO_setMasterCore(60, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_60_SPICLKA);
    GPIO_setDirectionMode(60, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(60, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(60, GPIO_QUAL_ASYNC);

    // STE (SPI STE A) - GPIO 61
    GPIO_setMasterCore(61, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_61_SPISTEA);
    GPIO_setPadConfig(61, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(61, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(61, GPIO_DIR_MODE_OUT);
}

// Start ADC Conversion for Cell Voltages
void ltc6804_startADCConversion(void)
{
    ltc6804_sendCommand(LTC6804_ADCV);
}

// Poll ADC Conversion Status
bool ltc6804_pollADCStatus(void)
{
    ltc6804_sendCommand(LTC6804_PLADC);
    return adcConversionComplete;
}

// Read Cell Voltage Data from LTC6804
void ltc6804_readCellVoltages(void)
{
    // Read Cell Voltage Register Group A
    ltc6804_sendCommand(LTC6804_RDCVA);
    for(int i = 0; i < 12; i++){
        SPI_writeDataBlockingFIFO(SPIA_BASE, 0x00);
        rxBuffer[i] = SPI_readDataBlockingFIFO(SPIA_BASE);
    }
    
    // Read cell 1 voltage
    cell1 = ((rxBuffer[5] << 8 & 0xFF00) | (rxBuffer[4] & 0x00FF));
    cell1_voltage = cell1 * 0.0001;
}

// Send Command to LTC6804
void ltc6804_sendCommand(uint16_t command)
{
    uint8_t cmdBuffer[4];
    
    // Save command 
    cmdBuffer[0] = (command >> 8) & 0xFF;  // 상위 바이트
    cmdBuffer[1] = command & 0xFF;         // 하위 바이트
    
    // Calculate PEC
    uint16_t pec = pec15_calc(cmdBuffer, 2);
    cmdBuffer[2] = (pec >> 8) & 0xFF;     // PEC 상위
    cmdBuffer[3] = pec & 0xFF;            // PEC 하위
    
    // Transmit 4 bytes
    for(int i = 0; i < 4; i++) {
        SPI_writeDataBlockingFIFO(SPIA_BASE, cmdBuffer[i] << 8);
        SPI_resetRxFIFO(SPIA_BASE);
    }
}

// LTC6804 write configuration data
void LTC6804_write_config(void)
{  
    // Send WRCFG
    ltc6804_sendCommand(LTC6804_WRCFG);

    // Save configuration data
    uint8_t cfgBuffer[8];
    cfgBuffer[0] = 0x04; // CFGR0
    cfgBuffer[1] = 0x00; // CFGR1
    cfgBuffer[2] = 0xF0; // CFGR2
    cfgBuffer[3] = 0xFF; // CFGR3
    cfgBuffer[4] = 0x00; // CFGR4
    cfgBuffer[5] = 0x00; // CFGR5

    // Calculate PEC & save
    uint16_t data_pec = pec15_calc(cfgBuffer, 6);
    cfgBuffer[6] = (data_pec >> 8) & 0xFF;
    cfgBuffer[7] = data_pec & 0xFF;

    // Transmit 8 bytes
    spiTransactionComplete = false;
    for(int i = 0; i < 8; i++) {
        SPI_writeDataBlockingFIFO(SPIA_BASE, cfgBuffer[i] << 8);
        SPI_resetRxFIFO(SPIA_BASE);
    }

    // Wait for configuration register save
    DEVICE_DELAY_US(500000); 
}

// LTC6804 read configuration data
void LTC6804_read_config(void)
{
    ltc6804_sendCommand(LTC6804_RDCFG);
    SPI_resetRxFIFO(SPIA_BASE);

    uint16_t rawdata[12];
    for(int i = 0; i < 12; i++) {
        SPI_writeDataBlockingFIFO(SPIA_BASE, 0x0000);
        rawdata[i] = SPI_readDataBlockingFIFO(SPIA_BASE);
    }
}

// PEC calculation function
void init_PEC15_Table(void) 
{
    for (int i = 0; i < 256; i++) {
        uint16_t remainder = i << 7;
        for (int bit = 0; bit < 8; bit++) {
            if (remainder & 0x4000) {
                remainder = (remainder << 1) ^ CRC15_POLY;
            } else {
                remainder = (remainder << 1);
            }
        }
        pec15Table[i] = remainder;
    }
}

uint16_t pec15_calc(uint8_t *data, uint8_t len)
{
    uint16_t remainder, addr;

    remainder = 16; // Initialize the PEC
    for (uint8_t i = 0; i < len; i++) {
        addr = ((remainder >> 7) ^ data[i]) & 0xff;
        remainder = (remainder << 8) ^ pec15Table[addr];
    }
    return (remainder * 2); // The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

// Wake up LTC6804
void LTC6804_wakeup(void)
{
    // Wait ≥70 µs 
    DEVICE_DELAY_US(100);
    
    // Transmit dummy (8 clock toggle)
    SPI_writeDataBlockingFIFO(SPIA_BASE, 0x00 << 8);
    
    // Wait ≥tREADY (10 µs) 
    DEVICE_DELAY_US(15);
}

//
// Current Sensor Functions
//

// Function to configure and power up ADCA
void initADC(void)
{
    // Set ADCDLK divider to /4
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);

    // Set resolution and signal mode (12-bit single-ended)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);

    // Set pulse positions to late
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);

    // Power up the ADC and then delay for 1 ms
    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000);
}

// Function to configure ePWM1 to generate the SOC
// Setting Frequency : 10kHz
void initEPWM(void)
{
    // Disable SOCA
    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    // Configure the SOC to occur on the first up-count event
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 15);

    // Set for 10kHz sampling rate
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 5000);
    EPWM_setTimeBasePeriod(EPWM1_BASE, 9999);

    // Set the local ePWM module clock divider to /1
    EPWM_setClockPrescaler(EPWM1_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    // Freeze the counter
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP);
}

// Function to configure ADCA's SOC0 to be triggered by ePWM1
void initADCSOC(void)
{
    // Configure SOC0 of ADCA to convert pin A0 (LKSR 25-NPS sensor output)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN0, 15);

    // Set SOC0 to set the interrupt 1 flag
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}

// Function to convert ADC value to voltage
float adcToVoltage(uint16_t adcValue)
{
    return ((float)adcValue * ADC_VREF) / ADC_MAX_VALUE;
}

// Function to convert sensor voltage to current
float voltageToCurrent(float voltage)
{
    float current;
    
    // Convert voltage to current using sensor characteristics
    current = (voltage - SENSOR_VREF) / (SENSOR_SENSITIVITY / 1000.0f);
    
    return current;
}

// Function to calculate current values from ADC results
void calculateCurrent(void)
{
    uint16_t i;
    float sum = 0.0f;
    
    // Convert all ADC results to current values
    for(i = 0; i < RESULTS_BUFFER_SIZE; i++)
    {
        // Convert ADC value to voltage
        adc_voltage = adcToVoltage(adcAResults[i]);
        voltageValues[i] = adc_voltage;

        // Convert voltage to current
        current = voltageToCurrent(adc_voltage);
        
        // Store calculated current
        currentValues[i] = current;
        
        // Sum for average calculation
        sum += current;
    }
    
    // Calculate average current
    averageCurrent = sum / RESULTS_BUFFER_SIZE;
}

// Initialize CPU Timer
void initCPUTimer1(void)
{
    // Initialize CPU Timer 1
    CPUTimer_setPeriod(CPUTIMER1_BASE, 0xFFFFFFFF);
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
}

// Set up CPU Timer
void configCPUTimer1(void)
{
    uint32_t timerPeriod;

    //
    // Calculate timer period for 1 second
    // Timer period = (CPU Freq in Hz) * (Period in seconds)
    //
    timerPeriod = (uint32_t)(TIMER_FREQ_MHZ * 1000000 * TIMER_PERIOD_SEC);
    
    CPUTimer_setPeriod(CPUTIMER1_BASE, timerPeriod);
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    CPUTimer_setEmulationMode(CPUTIMER1_BASE, CPUTIMER_EMULATIONMODE_RUNFREE);
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);
}

// Start CPU Timer
void startCPUTimer1(void)
{
    CPUTimer_startTimer(CPUTIMER1_BASE);
}

//
// ADC A Interrupt 1 ISR
//
__interrupt void adcA1ISR(void)
{
    EINT;

    // Send command (ADCV) & Perform analog-to-digital conversion while calculating current
    ltc6804_startADCConversion();

    // Add the latest result to the buffer
    adcAResults[currentIndex++] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);

    // Set the bufferFull flag if the buffer is full
    if(RESULTS_BUFFER_SIZE <= currentIndex)
    {
        currentIndex = 0;
        bufferFull = 1;
    }

    // Check if overflow has occurred
    if(true == ADC_getInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    }
    
    // Current calculate
    calculateCurrent();

    // Send commnad (RDCV)
    ltc6804_readCellVoltages();


    // Clear the interrupt flag
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    // Acknowledge the interrupt
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void cpuTimer1ISR(void)
{
    // Coulomb counting on a 1-second basis
    float delta_soc = (-averageCurrent * DT) / (BAT_CAPACITY_Ah * 3600.0f);
    soc += delta_soc;

    // Clear timer interrupt flag
    CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);
    
    // Acknowledge interrupt
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
