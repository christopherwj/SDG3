/*
PWM Tester for Gate Driver

PWM high is on pin 45, PWM low is on pin 7.
Analog in (audio) is on pin A0
Be aware the Due runs at 3.3V.


Serial interface instructions
To change the amplitude, send it "a ##" whatever value between 0 and 952. 
    The setting below ADC, controls whether it listens to the ADC or serial interface
Deadtime is changed using "d ##" with some value between 0 and 100. I think it "should be" 
    around 14 based on what timing we need I remember from the datasheet. You can set both 
    at the same time using "d ## a ##" or "a ## d ##". 

Amplitude has a range between 0 and 952. This is mapped to a range between
-1 and 1 with 476 mapping to 0.
*/

#include <Arduino.h>

#define DEFAULT_DEADTIME 14 // I think this should be the 14 for our gate driver. Adjust to determine
#define DEFAULT_PERIOD 952   // 84E6/952/2 = 44,117.64khz
#define DEFAULT_DUTY DEFAULT_PERIOD/3     

#define MAX_AMP 952
#define MAX_DEADTIME 100 // Arbitrary, should be lower like probably in the 
#define MAX_SAMPLE_FREQ 44100
#define MAX_WAVE_FREQ 20000
#define MAX_ADC 4095
#define DEFAULT_WAVE flat

enum waveTypes {flat, square, triangle, sine} waveType;
char waveTypeNames[][10] = {"flat\0", "square\0", "triangle\0", "sine\0"};

char string[100];
uint16_t amplitude, deadtime, sampleFreq, waveFreq;
uint32_t AdcResult = 0;
uint32_t channel;
bool adcEnable = false;

// Parses a string with commands to control the PWM outputs for testing.
//
// Inputs:
//      string: The input string
//      amplitude: Pointer to the variable for the amplitude.
//      ... etc
// Returns: If the input string was valid
//
// Uses: parseSerial("a 100", &amp, &dt, null, null, null);
//       Parse the input string and set amplitude to 100.
// Uses: parseSerial("d 57", &amp, &dt, null, null, null);
//       Parse the input string and set the dead time to 57 cycles.
// Uses: parseSerial("d 20 a 120", &amp, &dt, null, null, null);
//       Parse the input string and set deadtime to 20, amplitude to 120
bool parseSerial(char *string, uint16_t *amplitude, uint16_t *deadtime, uint16_t *sampleFreq, enum waveTypes *waveType, uint16_t *waveFreq){
    uint8_t currentChar = 0;
    uint8_t action;
    long long int tempValue;
    while(*string != 0){
        action = *string++;
        do{currentChar = *string++;} while(currentChar != ' ');
        tempValue = atoll(string);
        switch(action){
            case 'A':
            case 'a':
                *amplitude = tempValue <= MAX_AMP ? tempValue : MAX_AMP; break;
            case 'D':
            case 'd':
                *deadtime = tempValue <= MAX_DEADTIME ? tempValue : MAX_DEADTIME; break;
            case 'F':
            case 'f':
                *waveFreq = tempValue <= MAX_WAVE_FREQ ? tempValue : MAX_WAVE_FREQ; break;
            case 'S':
            case 's':
                *sampleFreq = tempValue <= MAX_SAMPLE_FREQ ? tempValue : MAX_SAMPLE_FREQ; break;
            case 'T':
            case 't':
                *waveType = tempValue < sizeof(enum waveTypes) ? (enum waveTypes)tempValue : DEFAULT_WAVE;
                break;
            default:
                return false;
        }
    }
    return true;
}

// Sets up the ADC on the A0 pin.
void setupADC(){
    ADC->ADC_WPMR &= ~(ADC_WPMR_WPEN); // Disable Write Protect Mode
    ADC->ADC_CHER |= ADC_CHER_CH7; // Enable A0 pin
    ADC->ADC_MR = 0; 
    ADC->ADC_MR = ADC_MR_PRESCAL(4);    //ADC Clock set to 8MHz 
    ADC->ADC_MR |= ADC_MR_TRACKTIM(3); 
    ADC->ADC_MR |= ADC_MR_STARTUP_SUT8; 
    ADC->ADC_EMR = 0; // TODO: Figure out this register
    return;
}

// PWM Interrupt Handler
// Every time the PWM completes a cycle it triggers an interrupt.
// This reads the value in the ADC, starts a new ADC reading, and sets the PWM duty to the maped value from the ADC.
void PWM_Handler() {
    PWM->PWM_ISR1;                          // Clear flag by reading register
    AdcResult = ADC->ADC_CDR[7];            // Read the previous result
    ADC->ADC_CR |= ADC_CR_START;            // Begin the next ADC conversion. 
    PWMC_SetDutyCycle(PWM, channel, (uint16_t)map(AdcResult, 0, MAX_ADC, 0, MAX_AMP));
    return;
}

// Sets up PWM at 44117 Hz, Differential Inputs, and Deadtime
void setupPWM(){
    uint32_t ulPin = 7;

    // PWM Startup code

    // Starts the PWM clocks. Required to enable PWM.
    pmc_enable_periph_clk(PWM_INTERFACE_ID);
    // We'll be using the main clock without any dividers. This allows us the highest resolution.
 
    // Some Arduino library code to help with determining the channel
    channel = g_APinDescription[ulPin].ulPWMChannel;
    
    // Setup PWM for this pin and complementary pin
    PIO_Configure(g_APinDescription[ulPin].pPort,
            g_APinDescription[ulPin].ulPinType,
            g_APinDescription[ulPin].ulPin | PIO_PC18,      // In addition to the pin enabled from this function,
            g_APinDescription[ulPin].ulPinConfiguration);   // also enable the complementary pin.
  
    // Configure the channel
    PWM->PWM_CH_NUM[0].PWM_CMR = 1;
    // Disable ul_channel (effective at the end of the current period)
    if ((PWM->PWM_SR & (1 << channel)) != 0) {
        PWM->PWM_DIS = 1 << channel;
        while ((PWM->PWM_SR & (1 << channel)) != 0);
    }
    // Configure ul_channel   
    PWM->PWM_CH_NUM[channel].PWM_CMR = PWM_CMR_DTE | PWM_CMR_CALG; // Enable Deadtime and center align

    // Deadtime value set
    PWM->PWM_CH_NUM[channel].PWM_DT = DEFAULT_DEADTIME << PWM_DT_DTL_Pos | DEFAULT_DEADTIME;
    // Period value set
    PWMC_SetPeriod(PWM, channel, DEFAULT_PERIOD);
    PWMC_SetDutyCycle(PWM_INTERFACE, channel, DEFAULT_DUTY);
    PWMC_EnableChannel(PWM_INTERFACE, channel);

    PWMC_EnableChannelIt(PWM, channel);
    NVIC_EnableIRQ(PWM_IRQn);

    g_pinStatus[ulPin] = (g_pinStatus[ulPin] & 0xF0) | PIN_STATUS_PWM;
    return;
}

// Sets the deadtime in the update register
//
// Inputs: dtTime, The number of cycles there exists deadtime.
//
// Currently set up so that the deadtime between the HI to LO and LO to HI waveforms are the same.
// This can be changed if needed.
void setDT(uint16_t dtTime){
    PWM->PWM_CH_NUM[channel].PWM_DTUPD = dtTime << 16 | dtTime;
    return;
}


// Sets the period at which the PWM completes a cycle. 
//
// Inputs: period, The number of cycles in a period.
//
// This shouldn't be needed as it's set at 44117 Hz already.
void setPeriod(uint16_t period){
    PWMC_SetPeriod(PWM, channel, period);
    return;
}

// Sets the duty cycle of the PWM
// 
// Inputs: duty, The number of cycles the HI side of the input is enabled.
//               Note: The LO side is complementary
void setDuty(uint16_t duty){
    PWMC_SetDutyCycle(PWM, channel, duty);
    return;
}

void setup() {
  Serial.begin(115200);
  setupADC();
  setupPWM();
  amplitude = DEFAULT_DUTY;
  deadtime = DEFAULT_DEADTIME;
  sampleFreq = DEFAULT_PERIOD;
  waveFreq = 0;
  waveType = flat;
  while(!Serial);
  Serial.println("ready");
}

void loop() {
    if(Serial.available() > 0){
        Serial.readBytesUntil(0, string, 100);
        if(!parseSerial(string, &amplitude, &deadtime, &sampleFreq, &waveType, &waveFreq)){
            sprintf(string, "Amplitude: %u Deadtime: %u", amplitude, deadtime);
            Serial.println(string);
            PWMC_DisableChannelIt(PWM, channel);
            NVIC_DisableIRQ(PWM_IRQn);
            setDuty(amplitude);
            setDT(deadtime);
        }
    }
}
