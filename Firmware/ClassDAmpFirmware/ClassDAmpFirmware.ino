/*
Class D Amp Firmware

PWM high is on pin 45, PWM low is on pin 7.
Analog in (audio) is on pin A0
Be aware the Due runs at 3.3V.
*/

#include <Arduino.h>

#define DEFAULT_DEADTIME 10 // I think this should be the 14 for our gate driver. Adjust to determine
#define DEFAULT_PERIOD 952   // 84E6/952/2 = 44,117.64khz
#define DEFAULT_DUTY DEFAULT_PERIOD/3     

#define MAX_AMP 952
#define MAX_DEADTIME 10 // Arbitrary, should be lower like probably in the 10's
#define MAX_SAMPLE_FREQ 44100
#define MAX_ADC 4095
#define SAMPLES_QUIET 100
#define FILTER_SAMPLES 4
#define FILTER_INPUT_OUTPUT_ENABLE
#define FILTER_INPUT_OUTPUT_SAMPLES 1000

uint32_t channel;
bool dacEnable = true;

#ifdef FILTER_INPUT_OUTPUT_ENABLE
uint16_t samples[2][1000];
uint16_t sampleIndex = 0;
bool sampleEn = false;
#endif

uint32_t logApprox(uint32_t input, uint32_t order){
    uint32_t output = input;
    for(uint32_t i = 0; i < order; i++){
        output *= input;
        output >>= 12;
    }    
    return output;
}
/*
int16_t filter(int32_t signal){
    #define POLES 3
    //static double y[3] = {0};
    static uint16_t x[3] = {0};
    
    // Shift old inputs so that the index is their n-m where n is the current
    // sample and m is the m'th after the current one.
    //memmove(x+1, x, POLES*sizeof(x[0]));
    //x[0] = signal;

    // Shift old outputs, same as above.
    //memmove(y+1, y, (POLES-1)*sizeof(y[0]));
    //y[0] = 0;

    // LPF at 20khz.
    // Found using MatLab, [b,a] = butter(2,fc/(fs/2)) where fc = 20k, fs=44117
    //const double a[3] = {0, -1.9497, 0.9509};
    //const double b[3] = {0.00030912, 0.00061824, 0.00030912};

    // Calculate the filtered output and save result for next sample
    //y[0] = b[0]*x[0]+b[1]*x[1]+b[2]*x[2]-a[1]*y[1]-a[2]*y[2];
    
    // Direct-Form II at 10kHz
    /*
    const double a[3] = {1, -0.171886271172, 0.176800554};
    const double b[3] = {1, 2, 1};
    x[0] = signal - a[1]*x[1] - a[2]*x[2];
    y[0] = b[0] * x[0] + b[1] * x[1] + b[2]*x[2];
    */
    /*
    for(uint8_t i = 0; i < POLES; i++){
        y[0] += b[i]*x[i]-a[i]*y[i];
    }
    

    // Return most recent result.
    return x[0];
}
*/

uint16_t filter(uint16_t input){
    static int32_t v[3] = {0};
    v[0] = v[1];
    v[1] = v[2];
    v[2] = ((((input * 263432L) >>  2) + ((v[0] * -370778L) >> 3) + ((v[1] * 360472L) >> 3)	)+131072) >> 18; 
    return (short)(((v[0] + v[2])+2 * v[1]));
}

// Sets up the ADC on the A0 pin.
void setupADC(){
    ADC->ADC_WPMR &= ~(ADC_WPMR_WPEN); // Disable Write Protect Mode
    ADC->ADC_CHER |= ADC_CHER_CH7; // Enable A0 pin
    //ADC->ADC_CHER |= ADC_CHER_CH6;// g_APinDescription[1].ulADCChannelNumber; // Enable A1 pin
    //ADC->ADC_CHER |= ADC_CHER_CH5;
    ADC->ADC_MR = 0; 
    ADC->ADC_MR = ADC_MR_PRESCAL(4);    //ADC Clock set to 8MHz 
    ADC->ADC_MR |= ADC_MR_TRACKTIM(3); 
    ADC->ADC_MR |= ADC_MR_STARTUP_SUT8; 
    ADC->ADC_EMR = 0; // TODO: Figure out this register
    return;
}

void setupDAC () {
  PIOB->PIO_PDR |= PIO_PDR_P15 | PIO_PDR_P16;  // Disable GPIO on corresponding pins DAC0 and DAC1
  PMC->PMC_PCER1 |= PMC_PCER1_PID38 ;     // DACC power ON
  DACC->DACC_CR = DACC_CR_SWRST ;         // reset DACC

  DACC->DACC_MR = DACC_MR_REFRESH (1)
                  | DACC_MR_STARTUP_0
                  | DACC_MR_MAXS
                  | DACC_MR_USER_SEL_CHANNEL1;

  DACC->DACC_CHER =  DACC_CHER_CH1;      // enable DAC  channel 1
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

void setupTimer(){
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

// Sets the duty cycle of the PWM
// 
// Inputs: duty, The number of cycles the HI side of the input is enabled.
//               Note: The LO side is complementary
void setDuty(uint16_t duty){
    PWMC_SetDutyCycle(PWM, channel, duty);
    return;
}

int16_t mapRange(uint16_t input, uint16_t inLow, uint16_t inHigh, uint16_t outLow, uint16_t outHigh){
    return outLow+(input-inLow)*(double)(outHigh-outLow)/(double)(inHigh-inLow);
}

int16_t rollingAverage(int16_t input){
    static uint16_t values[FILTER_SAMPLES] = {0};
    static uint32_t index = 0;
    if(index == FILTER_SAMPLES) index = 0;
    static int32_t sum = 0;
    sum -= values[index];
    sum += input;
    values[index++] = input;
    return sum/FILTER_SAMPLES;
}

int16_t rollingAverageQuiet(int16_t input){
    static uint16_t values[SAMPLES_QUIET] = {0};
    static uint32_t index = 0;
    if(index == SAMPLES_QUIET) index = 0;
    static int32_t sum = 0;
    sum -= values[index];
    sum += input;
    values[index++] = input;
    return sum/SAMPLES_QUIET;
}

// PWM Interrupt Handler
// Every time the PWM completes a cycle it triggers an interrupt.
// This reads the value in the ADC, starts a new ADC reading, and sets the PWM duty to the maped value from the ADC.
void PWM_Handler() {
    PWM->PWM_ISR1;                          // Clear flag by reading register
    uint32_t Adc0Result = ADC->ADC_CDR[7];            // Read the previous result
    uint32_t Adc1Result = ADC->ADC_CDR[6];
    ADC->ADC_CR |= ADC_CR_START;            // Begin the next ADC conversion. 
    //int16_t filteredReading = Adc0Result;
    //int16_t filteredReading = rollingAverage(Adc0Result);
    int16_t filteredReading = filter(Adc0Result);
    /*
    if(Adc0Result >= filteredReading - 10 && Adc0Result <= filteredReading + 10){
        filteredReading = 2047;
    }
    */
/*
#ifdef FILTER_INPUT_OUTPUT_ENABLE
    if(sampleEn == true){
        samples[0][sampleIndex] = Adc0Result;
        samples[1][sampleIndex++] = filteredReading;
    }
#endif
*/
    //int16_t filteredReading = filter(AdcResult-2048.0);
    //uint16_t filteredReading = logApprox(AdcResult,5);
    if(filteredReading > 4095) filteredReading = 4095;
    if(filteredReading < 0) filteredReading = 0;
    uint16_t mappedResult = mapRange(filteredReading, 0, MAX_ADC, 0, MAX_AMP);
    //int16_t mappedResult = mapRange(filteredReading, 1302, 2730, 0, MAX_AMP);
    if(mappedResult > 952) mappedResult = 952;
    if(mappedResult < 0) mappedResult = 0;
    PWMC_SetDutyCycle(PWM, channel, (uint16_t)mappedResult);
    //PWMC_SetDutyCycle(PWM, channel, 100);
    if(dacEnable == true){
        DACC->DACC_CDR = (uint32_t) filteredReading;
    }
    
    return;
}

// Sine wave generator
/*
void PWM_Handler() {
    PWM->PWM_ISR1;                          // Clear flag by reading register
    static uint32_t timeStep = 0;
    if(timeStep++ == 44116) timeStep = 0;
#define FREQUENCY 4300
    int16_t filteredReading = 1500*sin(2*FREQUENCY*(double)timeStep/44117.0)+2048;
    if(filteredReading > 4095) filteredReading = 4095;
    if(filteredReading < 0) filteredReading = 0;
    uint16_t mappedResult = map(filteredReading, 0, MAX_ADC, 0, MAX_AMP);
    PWMC_SetDutyCycle(PWM, channel, mappedResult);
    DACC->DACC_CDR = (uint32_t) filteredReading;
    return;
}
*/
void setup() {
  setupADC();
  setupPWM();
  setupTimer();
  if(dacEnable == true) setupDAC();
  Serial.begin(9600);
  delay(5000);
  Serial.println("boo");
  sampleEn = true;
}

void loop() {
#ifdef FILTER_INPUT_OUTPUT_ENABLE
/*
    if(sampleIndex == 1000){
        sampleEn = false;
        sampleIndex = 0;
        for(int i = 0; i < 1000; i++){
           Serial.print(i);
           Serial.print(",");
           Serial.print(samples[0][i]);
           Serial.print(",");
           Serial.println(samples[1][i]);
        }
    }
*/
#endif
}
