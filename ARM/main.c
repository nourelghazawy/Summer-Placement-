/** This Code is Using the control loop for Just the Right wheel and it's much faster than 
		Setting the timer counter and  reading it's value after a fixed time the Right_Speed variable
		is the input speed and it's a value from 0 to 1000 however the ouput speed (Encoder Reading) will not
		be that accurate This variable can be incremented or decremented using bluetooth by sending + or - **///


#include "main.h"

int snprintf(char *str, unsigned int size, const char *format, char receive);
void SystemClock_Config(void);
void Configure_Timer2(void);
void Configure_Timer4(void);
void Right_Motor_Duty_Cycle(unsigned int Right_duty_cycle, unsigned char Direction);
void Left_Motor_Duty_Cycle(unsigned int left_duty_cycle, unsigned char Direction );
void LED_Init(void);
void UserButton_Init(void);
void Configure_Left_Motor(void);
void Motor_Enable(void);
void Motor_Disable(void);
void Motor_Direction_bits(void);
void Motor_Enable_Disable(void);
void Configure_USART6(void);
void Configure_USART2(void);
void Configure_USART1(void);
void RGB_LED_Init(void);
void Red_LED_On(void);
void Green_LED_On(void);
void Blue_LED_On(void);
void duty_cycle_decrement(void);
void duty_cycle_increment(void);
void Left_Speed_Increment(void);
void Left_Speed_Decrement(void);
void Right_Speed_Increment(void);
void Right_Speed_Decrement(void);
void Configure_PB2(void);
volatile unsigned int uart6_buffer_counter = 0;
volatile unsigned char uart6_buffer[1000];
volatile int duty_Cycle=100;
volatile int Right_duty_Cycle = 100;
volatile int Left_duty_Cycle = 100;
volatile int current_buffer_value=0;
volatile unsigned char Left_Direction = 0;
volatile unsigned char Right_Direction = 0;
volatile int Left_Encoder_RPM = 0;
volatile int Right_Encoder_RPM = 0;
volatile unsigned char flag=0;
float left_power_difference=0;
float new_right_power_difference=0;
float old_right_power_difference=0;
float right_derivative=0;
float right_kd=0;
float right_proportional=0;
float right_integral=0;
float right_ki=0;
volatile int Left_Speed=70;
volatile int Right_Speed=700;
volatile int counter = 0;
volatile unsigned char flag_left_byte=0;
volatile unsigned char flag_right_byte=0;
volatile int Left_byte=0;
volatile int Right_byte=0;
volatile float kp =0.09;
volatile float kd =0;
volatile float derivative=0;
volatile float previous_power_differnce=0;
volatile float factor=2.85;
volatile float offset=2.8;
#define TIM_DUTY_CYCLES_NB 11
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Duty cycles: D = T/P * 100%                                                */
/* where T is the pulse duration and P  the period of the PWM signal          */
static uint32_t aDutyCycle[TIM_DUTY_CYCLES_NB] = {
  0,    /*  0% */
  1,   /* 10% */
  2,   /* 20% */
  3,   /* 30% */
  4,   /* 40% */
  5,   /* 50% */
  6,   /* 60% */
  7,   /* 70% */
  8,   /* 80% */
  9,   /* 90% */
  10,  /* 100% */
};


uint8_t iDutyCycle = 0;                        /* Duty cycle index */
uint32_t uwMeasuredDutyCycle = 0;              /* Measured duty cycle */
uint32_t TimOutClock = 1;                      /* TIM2 Clock */


int main(void)
{
  /* Configure the system clock to 84 MHz */
  SystemClock_Config();
	
	/* Configure USART6 for Bluetooth Communciation */
	Configure_USART6();
	
	/* Configure USART2 to communicate with pc */
	Configure_USART2();
	
		/* Configure USART1 to communicate with the PIC */
	Configure_USART1();
	
	/* Congiguring PB2 as input to destinguish between the two encoders */
	Configure_PB2();
	
	/* Configuring User Button */
	UserButton_Init();
	
	/* Initialising the LED */
	LED_Init();
	
	/* Configuring timer 2 For PWM */
  Configure_Timer2();
	
	/* Configuring Timer 3 for PWM */
	Configure_Timer4();
	
	/* Configuring the Direction Bits as Outputs */
	Motor_Direction_bits();
	
	/* Configuring the Enable Bit as Output */
	Motor_Enable_Disable();
	
	/* Enabling the two Motors */
  Motor_Enable();

  /* Infinite loop  */
  while (1)
  { 	
				
		Left_Motor_Duty_Cycle(1000, Left_Direction);
		Right_Motor_Duty_Cycle(Right_duty_Cycle, Right_Direction);
	  
  }
}
void Motor_Direction_bits(void)
{
	/* Left Motor Direction Bit GPIO PC2 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_2, LL_GPIO_PULL_NO);
	
	/* Right Motor Direction Bit GPIO PC10 */
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_10, LL_GPIO_PULL_NO);
	
}

void Motor_Enable_Disable(void)
{ 
	/* Configuring the Motor Enable pin to be output PC3  */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_3, LL_GPIO_PULL_NO);
}

void Motor_Enable(void)
{ 
	/* Function to Enable the motors  */
	LL_GPIO_SetOutputPin(GPIOC,   LL_GPIO_PIN_3);
}
void Motor_Disable(void)
{
	/* Function to Disable the motors  */
	LL_GPIO_ResetOutputPin(GPIOC,   LL_GPIO_PIN_3);
}

void Configure_Timer4(void)
{
	/* Enable the peripheral clock of GPIO PB1 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
  /* GPIO TIM3_CH4 configuration */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_1, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_1, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_1, LL_GPIO_AF_2);
	
	NVIC_SetPriority(TIM3_IRQn, 0);
  NVIC_EnableIRQ(TIM3_IRQn);
	
	// Timer 4 Configuration
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3); // Enable clock to Timer 3 peripheral
	//Set the pre-scaler value to have TIM3 counter clock equal to 10 kHz */
  LL_TIM_SetPrescaler(TIM3, __LL_TIM_CALC_PSC(SystemCoreClock/100, 10000));
	LL_TIM_EnableARRPreload(TIM3);
	/* Set the auto-reload value to have a counter frequency of 100 Hz */
  /* TIM4CLK = SystemCoreClock / (APB prescaler & multiplier)               */
  TimOutClock = SystemCoreClock/1;
	LL_TIM_SetAutoReload(TIM3, __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM3), 1000));
	//Set Timer Output mode
	LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
	/* Set compare value to half of the counter period (50% duty cycle ) */
  LL_TIM_OC_SetCompareCH4(TIM3, ( (LL_TIM_GetAutoReload(TIM3) + 1 ) / 2));
	/* Enable TIM3_CCR4 register preload. Read/Write operations access the      */
  /* preload register. TIM3_CCR4 preload value is loaded in the active        */
  /* at each update event.                                                    */
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH4);
	/* Enable the capture/compare interrupt for channel 4*/
  LL_TIM_EnableIT_CC4(TIM3);
	/* Enable output channel 4 */
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
  /* Enable counter */
  LL_TIM_EnableCounter(TIM3);
  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIM3);
	
}

void Configure_Timer2(void)
{
	/* Enable the peripheral clock of GPIO PA_15 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	
  /* GPIO TIM2_CH1 configuration */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_15, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_15, LL_GPIO_AF_1);
	
	NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);
	
	// Timer 2 Configuration
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2); // Enable clock to Timer 2 peripheral
	//Set the pre-scaler value to have TIM2 counter clock equal to 10 kHz */
  LL_TIM_SetPrescaler(TIM2, __LL_TIM_CALC_PSC(SystemCoreClock/100, 10000));
	LL_TIM_EnableARRPreload(TIM2);
	/* Set the auto-reload value to have a counter frequency of 100 Hz */
  /* TIM2CLK = SystemCoreClock / (APB prescaler & multiplier)               */
  TimOutClock = SystemCoreClock/1;
	LL_TIM_SetAutoReload(TIM2, __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM2), 1000));
	//Set Timer Output mode
	LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
	/* Set compare value to half of the counter period (50% duty cycle ) */
  LL_TIM_OC_SetCompareCH1(TIM2, ( (LL_TIM_GetAutoReload(TIM2) + 1 ) / 2));
	/* Enable TIM2_CCR1 register preload. Read/Write operations access the      */
  /* preload register. TIM2_CCR1 preload value is loaded in the active        */
  /* at each update event.                                                    */
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
	/* Enable the capture/compare interrupt for channel 1*/
  LL_TIM_EnableIT_CC1(TIM2);
	/* Enable output channel 1 */
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
  /* Enable counter */
  LL_TIM_EnableCounter(TIM2);
  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIM2);
	
}

void Right_Motor_Duty_Cycle(unsigned int Right_duty_cycle, unsigned char Direction)
{
  /*Changing the Direction of the Motor according to the Argument Direction */
	if (Direction==1)
	{
		LL_GPIO_SetOutputPin(GPIOC,   LL_GPIO_PIN_10);
	}else{
		LL_GPIO_ResetOutputPin(GPIOC,   LL_GPIO_PIN_10);
	}
	
	/* Making sure the Duty cycle is below 100 */
	if (Right_duty_cycle>1000)
	{
		Right_duty_cycle=1000;
		LL_GPIO_SetOutputPin(GPIOC,   LL_GPIO_PIN_10);
	}
	
  uint32_t P;    /* Pulse duration                                            */
  uint32_t T;    /* PWM signal period                                         */
  /* PWM signal period is determined by the value of the auto-reload register */
  T = LL_TIM_GetAutoReload(TIM3) + 1;
  /* Pulse duration is determined by the value of the compare register.       */
  /* Its value is calculated in order to match the requested duty cycle.      */
  P = (Right_duty_cycle*T)/1000;
  LL_TIM_OC_SetCompareCH4(TIM3, P);
}



void Left_Motor_Duty_Cycle(unsigned int left_duty_cycle, unsigned char Direction )
{
	/*Changing the Direction of the Motor according to the Argument Direction */
	if (Direction ==1)
	{
		LL_GPIO_ResetOutputPin(GPIOC,   LL_GPIO_PIN_2);		
	}else{
		LL_GPIO_SetOutputPin(GPIOC,   LL_GPIO_PIN_2);
	}
	
	/* Making sure the Duty cycle is below 100 */
	if (left_duty_cycle>1000)
	{
		left_duty_cycle =1000;
	}
		
  uint32_t P;    /* Pulse duration                                            */
  uint32_t T;    /* PWM signal period                                         */
  /* PWM signal period is determined by the value of the auto-reload register */
  T = LL_TIM_GetAutoReload(TIM2) + 1;
  /* Pulse duration is determined by the value of the compare register.       */
  /* Its value is calculated in order to match the requested duty cycle.      */
  P = (left_duty_cycle*T)/1000;
  LL_TIM_OC_SetCompareCH1(TIM2, P);
}

void TIM2_IRQHandler(void)
{
  /* Check whether CC1 interrupt is pending */
  if(LL_TIM_IsActiveFlag_CC1(TIM2) == 1)
  {
    /* Clear the update interrupt flag*/
    LL_TIM_ClearFlag_CC1(TIM2);

		uwMeasuredDutyCycle = (LL_TIM_GetCounter(TIM2) * 100) / ( LL_TIM_GetAutoReload(TIM2) + 1 );
  }
}

void TIM3_IRQHandler(void)
{
  /* Check whether CC2 interrupt is pending */
  if(LL_TIM_IsActiveFlag_CC4(TIM3) == 1)
  {
    /* Clear the update interrupt flag*/
    LL_TIM_ClearFlag_CC4(TIM3);

    /* TIM4 capture/compare interrupt processing(function defined in main.c) */
    uwMeasuredDutyCycle = (LL_TIM_GetCounter(TIM3) * 100) / ( LL_TIM_GetAutoReload(TIM3) + 1 );
  }
}

void LED_Init(void)
{
  LED2_GPIO_CLK_ENABLE();
  /* Configure IO in output push-pull mode to drive external LED2 */
  LL_GPIO_SetPinMode(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_MODE_OUTPUT); 
}

void UserButton_Init(void)
{
  /* Enable the BUTTON Clock */
  USER_BUTTON_GPIO_CLK_ENABLE();
  
  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);
  
  /* Connect External Line to the GPIO*/
  USER_BUTTON_SYSCFG_SET_EXTI();
    
  /* Enable a rising trigger EXTI line 13 Interrupt */
  USER_BUTTON_EXTI_LINE_ENABLE();
  USER_BUTTON_EXTI_FALLING_TRIG_ENABLE();
    
  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn); 
  NVIC_SetPriority(USER_BUTTON_EXTI_IRQn,0x03);  
}

void UserButton_Callback(void)
{
	  /* Set new duty cycle, cycle though the 11  combinations 0-100% */
  iDutyCycle = (iDutyCycle + 1) % TIM_DUTY_CYCLES_NB;

  /* Change PWM signal duty cycle */
 Left_Motor_Duty_Cycle(aDutyCycle[iDutyCycle],0);//**commented**//
}


void Configure_USART6(void)
{
  /* (1) Enable GPIO clock and configures the USART pins *********************/
  // USART2 PINS: TX = PA11,  RX = PA12
  /* Enable the peripheral clock of GPIO Port */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_11, LL_GPIO_AF_8);        //See AF Mapping PDF in Useful Information Folder
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_11,LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_12, LL_GPIO_AF_8);        //See AF Mapping PDF in Useful Information Folder
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_12, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_12, LL_GPIO_PULL_UP);

	NVIC_SetPriority(USART6_IRQn, 0);  
  NVIC_EnableIRQ(USART6_IRQn);
	
  /* Enable USART peripheral clock and clock source ***********************/
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);

  /* Configure USART functional parameters ********************************/
  /* Note: Commented as corresponding to Reset value */
  LL_USART_Disable(USART6);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USART6, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 2 stop bit, no parity */
  LL_USART_ConfigCharacter(USART6, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_2);
 
  LL_USART_SetBaudRate(USART6,SystemCoreClock, LL_USART_OVERSAMPLING_16, 9600);  //this correspondts to baud rate of 115200

  /* (4) Enable USART *********************************************************/
  LL_USART_Enable(USART6);
	
	LL_USART_EnableIT_RXNE(USART6);
  LL_USART_EnableIT_ERROR(USART6);
}

void Configure_USART2(void)
{
  /* (1) Enable GPIO clock and configures the USART pins *********************/
  // USART2 PINS: TX = PA2,  RX = PA3
  /* Enable the peripheral clock of GPIO Port */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7);        //See AF Mapping PDF in Useful Information Folder
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2,LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_7);        //See AF Mapping PDF in Useful Information Folder
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);

	NVIC_SetPriority(USART2_IRQn, 0);  
  NVIC_EnableIRQ(USART2_IRQn);
	
  /* Enable USART peripheral clock and clock source ***********************/
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  /* Configure USART functional parameters ********************************/
  /* Note: Commented as corresponding to Reset value */
  LL_USART_Disable(USART2);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 2 stop bit, no parity */
  LL_USART_ConfigCharacter(USART2, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_2);
 
  LL_USART_SetBaudRate(USART2,SystemCoreClock, LL_USART_OVERSAMPLING_16, 9600);  //this correspondts to baud rate of 115200

  /* (4) Enable USART *********************************************************/
  LL_USART_Enable(USART2);
	
	LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableIT_ERROR(USART2);
}

void USART6_IRQHandler(void)
{
  /* Check RXNE flag value in SR register */
  if(LL_USART_IsActiveFlag_RXNE(USART6) && LL_USART_IsEnabledIT_RXNE(USART6))
  {
    /* RXNE flag will be cleared by reading of DR register (done in call) */
    /* Call function in charge of handling Character reception */
    __IO unsigned char received_char;		// changed from __IO uint32_t to char

  /* Read Received character. RXNE flag is cleared by reading of DR register */
  received_char = LL_USART_ReceiveData8(USART6);
	
  /* Check if received value is corresponding to specific one : S or s */
  if ((received_char == 'S') || (received_char == 's'))
  {

  }

  /* Echo received character on TX */
  //LL_USART_TransmitData8(USART2, received_char);
	Green_LED_On();
	uart6_buffer[uart6_buffer_counter]=received_char;
	
			switch(uart6_buffer[uart6_buffer_counter])
		{
			case 119:
				Motor_Enable();			//Move Forward by sending w and pressing enter in PUTTY
				Left_Direction=1;
				Right_Direction=1;		
				break;
			case 115:
				Motor_Enable();
				Left_Direction=0;		//Move Backward by sending s and pressing enter in PUTTY
				Right_Direction=0;
				break;
			case 100:
				Motor_Enable();
				Left_Direction=0;		//Turning Right by sending d and pressing enter in PUTTY
				Right_Direction=1;	
				break;
			case 97:
				Motor_Enable(); 		//Turning Left by sending a and pressing enter in PUTTY
				Left_Direction=1;
				Right_Direction=0;		
				break;
			case 112:
				Motor_Disable();		//Disabling the motors by sending P and pressing enter in PUTTY
				break;
			case 43:
				Right_Speed_Increment(); //incrementing the speed by sending + and pressing enter in PUTTY 
				break;
			case 45:
				Right_Speed_Decrement(); //Decrementing the speed by sending - and pressing enter in PUTTY
			  break;

		}		
			uart6_buffer_counter++;
  }
}

void Right_Speed_Increment(void)
{
	
	Right_Speed=Right_Speed+30;
	
}
void Right_Speed_Decrement(void)
{
	
	Right_Speed=Right_Speed-30;
	
}

void USART2_IRQHandler(void)
{
  /* Check RXNE flag value in SR register */
  if(LL_USART_IsActiveFlag_RXNE(USART2) && LL_USART_IsEnabledIT_RXNE(USART2))
  {
    /* RXNE flag will be cleared by reading of DR register (done in call) */
    /* Call function in charge of handling Character reception */
    __IO uint32_t received_char;

  /* Read Received character. RXNE flag is cleared by reading of DR register */
  received_char = LL_USART_ReceiveData8(USART2);
	
  /* Check if received value is corresponding to specific one : S or s */
  if ((received_char == 'S') || (received_char == 's'))
  {

  }

  /* Echo received character on TX */
  LL_USART_TransmitData8(USART6, received_char);
	LL_USART_TransmitData8(USART2, received_char);
  }
}
void Configure_PB2(void)
{	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	/* Configuring PB2 this is needed to read the encoders of the left Motor */
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);

	/* Configuring PB12 this is needed to read the encoders of the Right Motor */
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_12, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_12, LL_GPIO_PULL_UP);

}
void Configure_USART1(void)
{	
  /* (1) Enable GPIO clock and configures the USART pins *********************/
  // USART2 PINS: TX = PB6,  RX = PB7
  /* Enable the peripheral clock of GPIO Port */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_7);        //See AF Mapping PDF in Useful Information Folder
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6,LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7, LL_GPIO_AF_7);        //See AF Mapping PDF in Useful Information Folder
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);

	NVIC_SetPriority(USART1_IRQn, 0);  
  NVIC_EnableIRQ(USART1_IRQn);
	
  /* Enable USART peripheral clock and clock source ***********************/
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  /* Configure USART functional parameters ********************************/
  /* Note: Commented as corresponding to Reset value */
  LL_USART_Disable(USART1);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
 
  LL_USART_SetBaudRate(USART1,SystemCoreClock, LL_USART_OVERSAMPLING_16, 9600);  //changed baud rate to 115200

  /* (4) Enable USART *********************************************************/
  LL_USART_Enable(USART1);
	
	LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_ERROR(USART1);
}



void USART1_IRQHandler(void)
{
  /* Check RXNE flag value in SR register */
  if(LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))
  {
    /* RXNE flag will be cleared by reading of DR register (done in call) */
    /* Call function in charge of handling Character reception */
    __IO unsigned char received_char;		// changed from __IO uint32_t to char

  /* Read Received character. RXNE flag is cleared by reading of DR register */
  received_char = LL_USART_ReceiveData8(USART1);

			
		if (flag_right_byte==1){
			Right_byte = ((int)received_char);
			flag_right_byte=0;
		}
		if (flag_left_byte==1){
			Left_byte =((int)received_char);
			Right_Encoder_RPM = (Left_byte<<8)|Right_byte;
			Right_Encoder_RPM=	(60/((Right_Encoder_RPM)*0.00001610565*2))*factor; //this is the real RPM of the wheel Multpilied by a factor to make it from 0 to 1000 i think this factor will change when the buggy is on the floor
			flag_left_byte=0;
			LL_USART_TransmitData8(USART2, (Right_Encoder_RPM/10));	// trasmitting the RPM to Bluetooth as Percentage
			LL_mDelay(1);
			//LL_USART_TransmitData8(USART6, 13);		// transmitting the carriage return for Bluetooth to transmit it over UART
			//LL_mDelay(1);
		}
	
	if (received_char==250)
	{
		flag_left_byte=1;
	}
	if (received_char==251)
	{
		flag_right_byte=1;
	}

		new_right_power_difference = Right_Speed-(Right_Encoder_RPM);// Right_Encoder RPM is the value read from the Pic, Right_speed is the desired output speed 
		right_proportional=new_right_power_difference*kp;
		right_derivative= kd*(new_right_power_difference-old_right_power_difference);
		right_integral=right_ki*(new_right_power_difference+old_right_power_difference);
		if (Right_Speed>(Right_Encoder_RPM))
		{
			if (new_right_power_difference>old_right_power_difference)
			{
			Right_duty_Cycle=(Right_duty_Cycle)-(right_proportional)+right_derivative;	//changing the Duty cycle to achieve the desired output speed
			}else{
			Right_duty_Cycle=(Right_duty_Cycle)-(right_proportional)-right_derivative;	
			}
		}else{
			
			if (new_right_power_difference>old_right_power_difference)
			{
			Right_duty_Cycle=(Right_duty_Cycle)-(right_proportional)-right_derivative;	//changing the Duty cycle to achieve the desired output speed
			}else{
			Right_duty_Cycle=(Right_duty_Cycle)-(right_proportional)+right_derivative;	
			}
		}
		if (Right_duty_Cycle<0)	
		{
			Right_duty_Cycle=0;
		}
		if(Right_duty_Cycle>1000)
		{
			Right_duty_Cycle=1000;
		}
		old_right_power_difference=new_right_power_difference;
  /* Check if received value is corresponding to specific one : S or s */
  if ((received_char == 'S') || (received_char == 's'))
  {

  }
	
	Green_LED_On();
  }
}

void RGB_LED_Init(void)
{
  // LED Red = PB_4 (D5)	LED Green = PC_7 (D9), LED Blue = PA_9 (D8)
	
  /* Configure Red LED */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);
	
	/* Configure Green LED */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_7, LL_GPIO_PULL_NO);
	
	/* Configure Blue LED */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_NO);
}

void Red_LED_On(void)
{
	LL_GPIO_ResetOutputPin(GPIOB,   LL_GPIO_PIN_4); //Red   ON
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);     //Green OFF
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);     //Blue  OFF
}

void Blue_LED_On(void)
{
	LL_GPIO_ResetOutputPin(GPIOA,   LL_GPIO_PIN_9); //Blue  ON
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);     //Red   OFF
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);     //Green OFF
}

void Green_LED_On(void)
{
	LL_GPIO_ResetOutputPin(GPIOC,   LL_GPIO_PIN_7); //Green  ON
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);     //Blue OFF
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);     //Red   OFF
}
//////////////////////////////Encoders Stuff//////////////////////////////////

/**
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 84000000
  *            HCLK(Hz)                       = 84000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 2
  */
void SystemClock_Config(void)
{
  /* Enable HSE oscillator */
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() != 1)
  {
  };

  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 336, LL_RCC_PLLP_DIV_4);
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set APB1 & APB2 prescaler */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms */
  SysTick_Config(84000000 / 1000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  SystemCoreClock = 84000000;
}


