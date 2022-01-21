/*
PWM Tester for Gate Driver

PWM high is on pin 45, PWM low is on pin 7.
Be aware the Due runs at 3.3V.

To change the amplitude, send it "a ##" whatever value between 0 and 952, less the deadtime. 
Deadtime is changed using "d ##" with some value between 0 and 100. I think it "should be" 
around 14 based on what timing we need I remember from the datasheet. You can set both at the 
same time using "d ## a ##" or "a ## d ##". 
*/

#include <Arduino.h>

#define DEFAULTDEADTIME 14 // I think this should be the 14 for our gate driver. Adjust to determine
#define DEFAULTPERIOD 952   // 84E6/952/2 = 44,117.64khz
#define DEFAULTDUTY DEFAULTPERIOD/3     

#define MAXAMP 952
#define MAXDEADTIME 100 // Arbitrary, should be lower like probably in the 
#define MAXSAMPLEFREQ 44100
#define MAXWAVEFREQ 20000
#define DEFAULTWAVE flat

enum waveTypes {flat, square, triangle, sine} waveType;
char waveTypeNames[][10] = {"flat\0", "square\0", "triangle\0", "sine\0"};

char string[100];
uint16_t amplitude, deadtime, sampleFreq, waveFreq;
uint32_t adcResult = 0;
uint32_t channel;

void parseSerial(char *string, uint16_t *amplitude, uint16_t *deadtime, uint16_t *sampleFreq, enum waveTypes *waveType, uint16_t *waveFreq){
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
                *amplitude = tempValue <= MAXAMP ? tempValue : MAXAMP; break;
            case 'D':
            case 'd':
                *deadtime = tempValue <= MAXDEADTIME ? tempValue : MAXDEADTIME; break;
            case 'F':
            case 'f':
                *waveFreq = tempValue <= MAXWAVEFREQ ? tempValue : MAXWAVEFREQ; break;
            case 'S':
            case 's':
                *sampleFreq = tempValue <= MAXSAMPLEFREQ ? tempValue : MAXSAMPLEFREQ; break;
            case 'T':
            case 't':
                *waveType = tempValue < sizeof(enum waveTypes) ? (enum waveTypes)tempValue : DEFAULTWAVE;
                break;
            default:
                break;
        }
    }
    return;
}

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

// I think the timer can be replaced by the PWM creating an interrupt.
/*
void tc_setup() {
  PMC->PMC_PCER0 |= PMC_PCER0_PID29;                      // TC2 power ON : Timer Counter 0 channel 2 IS TC2
  TC0->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK2  // MCK/8, clk on rising edge
                              | TC_CMR_WAVE               // Waveform mode
                              | TC_CMR_WAVSEL_UP_RC        // UP mode with automatic trigger on RC Compare
                              | TC_CMR_ACPA_CLEAR          // Clear TIOA2 on RA compare match
                              | TC_CMR_ACPC_SET;           // Set TIOA2 on RC compare match

  TC0->TC_CHANNEL[2].TC_RC = 238;  //<*********************  Frequency = (Mck/8)/TC_RC  Hz = 44.117 Hz
  TC0->TC_CHANNEL[2].TC_RA = 40;  //<********************   Any Duty cycle in between 1 and 874

  TC0->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Software trigger TC2 counter and enable
}
*/

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
    /* Disable ul_channel (effective at the end of the current period) */
    if ((PWM->PWM_SR & (1 << channel)) != 0) {
        PWM->PWM_DIS = 1 << channel;
        while ((PWM->PWM_SR & (1 << channel)) != 0);
    }
    /* Configure ul_channel */   
    PWM->PWM_CH_NUM[channel].PWM_CMR = PWM_CMR_DTE | PWM_CMR_CALG; // Enable Deadtime and center align
    
    // Deadtime value set
    PWM->PWM_CH_NUM[channel].PWM_DT = DEFAULTDEADTIME << PWM_DT_DTL_Pos | DEFAULTDEADTIME;
    // Period value set
    PWMC_SetPeriod(PWM, channel, DEFAULTPERIOD);
    PWMC_SetDutyCycle(PWM_INTERFACE, channel, DEFAULTDUTY);
    PWMC_EnableChannel(PWM_INTERFACE, channel);
    g_pinStatus[ulPin] = (g_pinStatus[ulPin] & 0xF0) | PIN_STATUS_PWM;
    return;
}

void setDT(uint16_t dtTime){
    PWM->PWM_CH_NUM[channel].PWM_DTUPD = dtTime << 16 | dtTime;
    return;
}

// Shouldn't need this one
void setPeriod(uint16_t period){
    PWMC_SetPeriod(PWM, channel, period);
    return;
}

void setDuty(uint16_t duty){
    PWMC_SetDutyCycle(PWM, channel, duty);
    return;
}

void setup() {
  Serial.begin(115200);
  setupADC();
  //tc_setup();
  setupPWM();
  amplitude = DEFAULTDUTY;
  deadtime = DEFAULTDEADTIME;
  sampleFreq = DEFAULTPERIOD;
  waveFreq = 0;
  waveType = flat;
  while(!Serial);
  Serial.println("ready");
}

void loop() {
    if(Serial.available() > 0){
        Serial.readBytesUntil(0, string, 100);
        parseSerial(string, &amplitude, &deadtime, &sampleFreq, &waveType, &waveFreq);
        sprintf(string, "Amplitude: %u\nDeadtime: %u", amplitude, deadtime);
        Serial.println(string);
        
        setDuty(amplitude);
        setDT(deadtime);
    }
}