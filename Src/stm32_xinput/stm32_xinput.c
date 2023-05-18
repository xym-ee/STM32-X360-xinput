/*
	Interface between Racing wheel and XINPUT Controller library
  Compatible w/ PC
  
	Developer: Daniel Nesvera
	
	WTFPL lincense

*/

#include "stm32_xinput.h"
#include "xinput.h"

// Initiate 
int16_t wheelEncoderValue = 0;
uint16_t handbrakeValue = 0;
int16_t leftTriggerValue_ADC = 0;
int16_t rightTriggerValue_ADC = 0;
int16_t xLeftStickValue_ADC = 0;
int16_t yLeftStickValue_ADC = 0;
int16_t xRightStickValue_ADC = 0;
int16_t yRightStickValue_ADC = 0;

int8_t adcValueReady = 0;
int16_t leftTriggerValue = 0;
int16_t rightTriggerValue = 0;
int16_t xLeftStickValue = 0;
int16_t yLeftStickValue = 0;
int16_t xRightStickValue = 0;
int16_t yRightStickValue = 0;

struct _pin digitalArray[NUM_BUTTONS];
struct _pin analogArray[NUM_ANALOG];
struct _pin encoderPins[2];

/* Declare port, pin and state of all buttons
*
*/
void declareButtonPins(void){
	digitalArray[0].port  = GPIOB;		digitalArray[0].pin  = GPIO_PIN_7;		digitalArray[0].state  = GPIO_PIN_RESET;		// A
	digitalArray[1].port  = GPIOB;		digitalArray[1].pin  = GPIO_PIN_13;		digitalArray[1].state  = GPIO_PIN_RESET;		// B
	digitalArray[2].port  = GPIOB;		digitalArray[2].pin  = GPIO_PIN_14;		digitalArray[2].state  = GPIO_PIN_RESET;		// X
	digitalArray[3].port  = GPIOB;		digitalArray[3].pin  = GPIO_PIN_15;		digitalArray[3].state  = GPIO_PIN_RESET;		// Y
	digitalArray[4].port  = GPIOB;		digitalArray[4].pin  = GPIO_PIN_10;		digitalArray[4].state  = GPIO_PIN_RESET;		// LB	
	digitalArray[5].port  = GPIOB;		digitalArray[5].pin  = GPIO_PIN_11;		digitalArray[5].state  = GPIO_PIN_RESET;		// RB
	digitalArray[6].port  = GPIOB;		digitalArray[6].pin  = GPIO_PIN_5;		digitalArray[6].state  = GPIO_PIN_RESET;		// L3
	digitalArray[7].port  = GPIOB;		digitalArray[7].pin  = GPIO_PIN_6;		digitalArray[7].state  = GPIO_PIN_RESET;		// R3
	digitalArray[8].port  = GPIOB;		digitalArray[8].pin  = GPIO_PIN_3;		digitalArray[8].state  = GPIO_PIN_RESET;		// START
	digitalArray[9].port  = GPIOB;		digitalArray[9].pin  = GPIO_PIN_4;		digitalArray[9].state  = GPIO_PIN_RESET;		// BACK
	digitalArray[10].port = GPIOB;		digitalArray[10].pin = GPIO_PIN_12;		digitalArray[10].state = GPIO_PIN_RESET;		// XBOX-LOGO
	digitalArray[11].port = GPIOA;		digitalArray[11].pin = GPIO_PIN_8;		digitalArray[11].state = GPIO_PIN_RESET;		// D-UP	
	digitalArray[12].port = GPIOA;		digitalArray[12].pin = GPIO_PIN_9;		digitalArray[12].state = GPIO_PIN_RESET;		// D-DOWN
	digitalArray[13].port = GPIOA;		digitalArray[13].pin = GPIO_PIN_10;		digitalArray[13].state = GPIO_PIN_RESET;		// D-LEFT
	digitalArray[14].port = GPIOA;		digitalArray[14].pin = GPIO_PIN_15;		digitalArray[14].state = GPIO_PIN_RESET;		// D-RIGHT
}

/* Declare port, pin of the analog inputs
*
*/
void declareAnalogPins(void){
	analogArray[0].port = GPIOA;		analogArray[1].pin = GPIO_PIN_0;		analogArray[1].state = GPIO_PIN_RESET;			// LEFT TRIGGER
	analogArray[1].port = GPIOA;		analogArray[2].pin = GPIO_PIN_1;		analogArray[2].state = GPIO_PIN_RESET;			// RIGHT TRIGGER
	analogArray[2].port = GPIOA;		analogArray[5].pin = GPIO_PIN_2;		analogArray[5].state = GPIO_PIN_RESET;			// X RIGHT STICK
	analogArray[3].port = GPIOA;		analogArray[6].pin = GPIO_PIN_3;		analogArray[6].state = GPIO_PIN_RESET;			// Y RIGHT STICK
	analogArray[4].port = GPIOA;		analogArray[3].pin = GPIO_PIN_4;		analogArray[3].state = GPIO_PIN_RESET;			// X LEFT STICK		
	analogArray[5].port = GPIOA;		analogArray[4].pin = GPIO_PIN_5;		analogArray[4].state = GPIO_PIN_RESET;			// Y LEFT STICK		
	
}

/*	Declare port, pin of the encoder
*
*/
void declareEncoderPins(void){
	encoderPins[0].port = NULL;		encoderPins[0].pin = NULL;		encoderPins[0].state = GPIO_PIN_RESET;			// Encoder input A - interrupt pin
	encoderPins[1].port = NULL;		encoderPins[1].pin = NULL;		encoderPins[1].state = GPIO_PIN_RESET;			// Encoder input B - normal input
}

/*	Read/update buttons and potentiometer of the handrake
*
*/
void readButtons(void){
	int i = 0;
	int state = 1;
	uint8_t buttonArray[11] = {0,0,0,0,0,0,0,0,0,0,0};				// initialize array of buttons
	uint8_t dpadArray[4] = {0,0,0,0};													// initialize array from dpad
		
	while( i < NUM_BUTTONS ){
		
		if( digitalArray[i].port != NULL ){
			
			// 0 = PRESSED		1 = NOT PRESSED
			state = HAL_GPIO_ReadPin( digitalArray[i].port, digitalArray[i].pin );	// read buttons, button is active-low
			
			if( i <= 10 ){		// Buttons
				buttonArray[i] = !state;
				
			}else{					// D-PAD
				//dpadArray[i-11] = state;
			}
		}
		
		i++;
	}
			
	XINPUT_buttonArrayUpdate( buttonArray );																					// update buttons
	
	dpadArray[0] = !HAL_GPIO_ReadPin( digitalArray[11].port, digitalArray[11].pin );
	dpadArray[1] = !HAL_GPIO_ReadPin( digitalArray[12].port, digitalArray[12].pin );
	dpadArray[2] = !HAL_GPIO_ReadPin( digitalArray[13].port, digitalArray[13].pin );
	dpadArray[3] = !HAL_GPIO_ReadPin( digitalArray[14].port, digitalArray[14].pin );
	
	XINPUT_dpadUpdate( dpadArray[0], dpadArray[1], dpadArray[2], dpadArray[3] );			// update dpad

}

/*	Read input values from sticks and triggers
*
*/

#include "sbus.h"

void readAdcValues(void){
  
  
		rightTriggerValue = (uint8_t)map( sbus.rs, SBUS_CH_MIN, SBUS_CH_MAX, 0, UINT8_MAX );
		leftTriggerValue 	= (uint8_t)map( sbus.ls, SBUS_CH_MIN, SBUS_CH_MAX, 0, UINT8_MAX );
			

//		rightTriggerValue = 0;
//		leftTriggerValue 	= 0;

		xRightStickValue = (int16_t)map( sbus.rh, SBUS_CH_MIN, SBUS_CH_MAX, INT16_MIN, INT16_MAX );
		yRightStickValue = (int16_t)map( sbus.rv, SBUS_CH_MIN, SBUS_CH_MAX, INT16_MIN, INT16_MAX );							// 4040 is the max value that my adc presents with potentiometers
		
    xLeftStickValue = (int16_t)map( sbus.lh, SBUS_CH_MIN, SBUS_CH_MAX, INT16_MIN, INT16_MAX );
		yLeftStickValue = (int16_t)map( sbus.lv, SBUS_CH_MIN, SBUS_CH_MAX, INT16_MIN, INT16_MAX );
	
}

/*	Update brake and throttle triggers
*
*/
void updateTriggers(void){
		
	XINPUT_triggerUpdate(leftTriggerValue, rightTriggerValue);
}

/*	Update right and left sticks
*			LeftStick X-axis comes from steering wheel
*			RightStick comes from potentiometers
*/
void updateSticks(void){

	XINPUT_stickUpdate(STICK_LEFT, xLeftStickValue, yLeftStickValue);
	XINPUT_stickUpdate(STICK_RIGHT, xRightStickValue, yRightStickValue );
	
}

/*	Re-maps a number from one range to another
*
*/
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
