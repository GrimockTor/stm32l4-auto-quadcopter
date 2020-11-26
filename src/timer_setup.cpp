#include "flight_controller.h"

volatile uint32_t PulseWidth;
volatile uint32_t CurrentRise_Capture, CurrentFall_Capture;
volatile uint32_t measured_time_start;
uint32_t timer_freq, timer_freq_adj;
uint8_t check_byte, channel_select_counter;
HardwareTimer *Tim15_input, *Tim1_output1, *Tim1_output2, *Tim1_output3, *Tim1_output4; 
uint32_t InputChannel, OutputChannel1, OutputChannel2, OutputChannel3, OutputChannel4;

void Input_Capture_Rising_IT_callback(void) {
  
  CurrentRise_Capture = Tim15_input->getCaptureCompare(InputChannel);
 // SerialUSB.println((String) "Rising_IT_Callback: CurrentCapture = " + Current_Capture);
}

void Input_Capture_Falling_IT_callback(void) {
  
  CurrentFall_Capture = Tim15_input->getCaptureCompare(InputChannel);
  PulseWidth = CurrentFall_Capture - measured_time_start;
  if (PulseWidth < 0) PulseWidth += 0xFFFF;
  measured_time_start = CurrentRise_Capture;
  if (PulseWidth > 4000)channel_select_counter = 0;
  else channel_select_counter++;

  if (channel_select_counter == 1)channel[CH_1] = PulseWidth;
  if (channel_select_counter == 2)channel[CH_2] = PulseWidth;
  if (channel_select_counter == 3)channel[CH_3] = PulseWidth;
  if (channel_select_counter == 4)channel[CH_4] = PulseWidth;
  if (channel_select_counter == 5)channel[CH_5] = PulseWidth;
  if (channel_select_counter == 6)channel[CH_6] = PulseWidth;
  //SerialUSB.println((String) "Channel " + channel + ": PulseWidth = " + PulseWidth);
}

void set_esc_outputs (int32_t channel1, int32_t channel2, int32_t channel3, int32_t channel4){

    // Configure and start PWM
    Tim1_output1->setPWM(OutputChannel1, TIM1_CH1, 250, (100 * channel1 / 4000)); // 250 Hertz = 4000us, 25% dc = 1000us
    Tim1_output2->setPWM(OutputChannel2, TIM1_CH2, 250, (100 * channel2 / 4000)); // 250 Hertz = 4000us, 25% dc = 1000us
    Tim1_output3->setPWM(OutputChannel3, TIM1_CH3, 250, (100 * channel3 / 4000)); // 250 Hertz = 4000us, 25% dc = 1000us
    Tim1_output4->setPWM(OutputChannel4, TIM1_CH4, 250, (100 * channel4 / 4000)); // 250 Hertz = 4000us, 25% dc = 1000us
}

void timer_setup(void){

  // SETUP PWM INPUT DETECTION 
  TIM_TypeDef *Instance_input = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(TIM15_CH1), PinMap_PWM);
  InputChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(TIM15_CH1), PinMap_PWM));
  Tim15_input = new HardwareTimer(Instance_input);

  // Compute this input scale factor only once
  timer_freq = Tim15_input->getTimerClkFreq();
  timer_freq_adj = timer_freq / Tim15_input->getPrescaleFactor();
  
  // Configure Input measurement
  Tim15_input->setMode(InputChannel, TIMER_INPUT_FREQ_DUTY_MEASUREMENT, TIM15_CH1);

  uint32_t PrescalerFactor = 84;  // Max AHB1 clock
  Tim15_input->setPrescaleFactor(PrescalerFactor);
  Tim15_input->setOverflow(0xFFFF); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  Tim15_input->attachInterrupt(CH_RISING, Input_Capture_Rising_IT_callback);
  Tim15_input->attachInterrupt(CH_FALLING, Input_Capture_Falling_IT_callback);
  //Tim2_input->attachInterrupt(Rollover_IT_callback);
  HAL_NVIC_EnableIRQ(TIM15_IRQn);   //TODO: CHECK IF THIS IS NEEDED
  Tim15_input->resume();
  delay(100);

  TIM_TypeDef *Instance1 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(TIM1_CH1), PinMap_PWM);
  OutputChannel1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(TIM1_CH1), PinMap_PWM));
  TIM_TypeDef *Instance2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(TIM1_CH2), PinMap_PWM);
  OutputChannel2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(TIM1_CH2), PinMap_PWM));
  TIM_TypeDef *Instance3 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(TIM1_CH3), PinMap_PWM);
  OutputChannel3 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(TIM1_CH3), PinMap_PWM));
  TIM_TypeDef *Instance4 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(TIM1_CH4), PinMap_PWM);
  OutputChannel4 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(TIM1_CH4), PinMap_PWM));

  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  HardwareTimer *Tim1_output1 = new HardwareTimer(Instance1);
  HardwareTimer *Tim1_output2 = new HardwareTimer(Instance2);
  HardwareTimer *Tim1_output3 = new HardwareTimer(Instance3);
  HardwareTimer *Tim1_output4 = new HardwareTimer(Instance4);
  
  set_esc_outputs(1000, 1000, 1000, 1000);
  
} 