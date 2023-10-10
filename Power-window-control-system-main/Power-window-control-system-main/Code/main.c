#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "macros.h"
#include "TM4C123GH6PM.h"

/*********************** Ports IRQ Number ***********************/
/*-------------------------------------------------------------*/

#define PortD_IRQn 3
#define PortE_IRQn 4
#define PortF_IRQn 30

/*********************** Ports initilization Prototypes ***********************/
/*---------------------------------------------------------------------------*/

void PortA_Init(void);
void PortD_Init(void);
void PortE_Init(void);
void PortF_Init(void);

/***************************** Semaphore Handle *****************************/
/*-------------------------------------------------------------------------*/

/********************* Driver Semaphore Handles **************************/

SemaphoreHandle_t xUPAutomaticSemaphore;
SemaphoreHandle_t xDOWNAutomaticSemaphore;
SemaphoreHandle_t xUPManualSemaphore;
SemaphoreHandle_t xDOWNManualSemaphore;

/******************** Shotgun Semaphore Handles ********************/
SemaphoreHandle_t xUPSemaphore_P;
SemaphoreHandle_t xDOWNSemaphore_P;

/******************** Jam Semaphore Handle ********************/
SemaphoreHandle_t xJamSemaphore;

/****************** Lock Semaphore Handle ******************/
SemaphoreHandle_t xLOCKSemaphore;

xTaskHandle Auto_up_Handle;
xTaskHandle P_up_Handle;
xTaskHandle Manual_up_Handle;


/***************************** Jamming Task *****************************/
/*---------------------------------------------------------------------*/

void v_JamTask(void* pvParameters) 
{
  xSemaphoreTake(xJamSemaphore, 0);									  // Take Semaphore
	for(;;)
	{
    xSemaphoreTake(xJamSemaphore, portMAX_DELAY);			// Take xJamSemaphore
		vTaskSuspend(Auto_up_Handle);											// Suspend Driver’s Automatic UP Task whenever obstacle is detected
		vTaskSuspend(Manual_up_Handle);										// Suspend Driver’s Manual UP Task whenever obstacle is detected							
		vTaskSuspend(P_up_Handle);												// Suspend Shotgun’s Auto/Manual UP Task whenever obstacle is detected
		int i=0;
		
		// Reverse Motor Direction for 0.5 second
		while (i<16000*15)																
		{
			GPIO_PORTA_DATA_R |= (1<<7);
	    GPIO_PORTA_DATA_R &=~ (1<<6);	
			GPIO_PORTA_DATA_R |=(1<<5);
			i++;
		}
		
		// Stopping the Motor after reversing it’s direction
		GPIO_PORTA_DATA_R &=~(1<<5);	
		
  } 
}

/***************************** DRIVER AUTOMATIC (UP) Task *****************************/
/*-------------------------------------------------------------------------------------*/

void v_DUPAutomaticTask(void *pvParameters)
{
	  xSemaphoreTake(xUPAutomaticSemaphore,0); 											// Take Semaphore
	 
	for(;;)
	{
		xSemaphoreTake(xUPAutomaticSemaphore,portMAX_DELAY); 				// Take xUPAutomaticSemaphore
		
		// check while the upper limit switch and Driver’s Pushbuttons (Auto (Down) / Manual (UP)/ Auto (Down)) are not pressed
     while ((GPIO_PORTF_DATA_R & (1<<0)) && (GPIO_PORTF_DATA_R & (1<<2)) && (GPIO_PORTF_DATA_R & (1<<1)) && (GPIO_PORTF_DATA_R & (1<<3))  && (GPIO_PORTF_DATA_R & (1<<4)))
		 { 
		  //Rotates Motor Up Automatically
			GPIO_PORTA_DATA_R |= (1<<6);
	    GPIO_PORTA_DATA_R &=~ (1<<7);	
			GPIO_PORTA_DATA_R |=(1<<5);
		 }
		 //Stop the Motor if upper limit switch or Driver’s Pushbuttons (Auto (Down) / Manual (UP)/ Manual (Down)) are pressed
			GPIO_PORTA_DATA_R &=~(1<<5);	
   }
}

/***************************** DRIVER AUTOMATIC (DOWN) Task *****************************/
/*-------------------------------------------------------------------------------------*/

void v_DDOWNAutomaticTask(void *pvParameters)
{
	  xSemaphoreTake(xDOWNAutomaticSemaphore,0); 											// Take Semaphore
	 
	for(;;)
	{
		xSemaphoreTake(xDOWNAutomaticSemaphore,portMAX_DELAY);				// Take xDOWNAutomaticSemaphore

		// check while the lower limit switch and Driver’s Pushbuttons (Auto (UP) / Manual (UP)/ Auto (Down)) are not pressed
    while ((GPIO_PORTE_DATA_R & (1<<1)) && (GPIO_PORTF_DATA_R & (1<<4)) && (GPIO_PORTF_DATA_R & (1<<1)) && (GPIO_PORTF_DATA_R & (1<<3))  && (GPIO_PORTF_DATA_R & (1<<2)))
		{ 
		  //Rotates Motor Down Automatically
			GPIO_PORTA_DATA_R |= (1<<7);
	    GPIO_PORTA_DATA_R &=~ (1<<6);	
			GPIO_PORTA_DATA_R |=(1<<5);
	  }
		 
		 //Stop the Motor if lower limit switch or Driver’s Pushbuttons (Auto (UP) / Manual (UP)/ Manual (Down)) are pressed
		 GPIO_PORTA_DATA_R &=~(1<<5);	
		 
		// Resume the Auto Driver and Manual and Driver Auto/Manual Shotgun up tasks if there they are suspended during jamming
		vTaskResume(Auto_up_Handle);
	  vTaskResume(Manual_up_Handle);
	  vTaskResume(P_up_Handle);

   }
}

/***************************** DRIVER MANUAL (UP) Task *****************************/
/*-----------------------------------------------------------------------------------*/

void v_DUPManualTask(void *pvParameters)
{
	  xSemaphoreTake(xUPManualSemaphore,0);											// Take Semaphore
	 
	for(;;)
	{
		xSemaphoreTake(xUPManualSemaphore,portMAX_DELAY);					// Take xUPManualSemaphore					
	  
		// check while the upper limit switch and Driver’s Pushbuttons (Auto (UP) / Auto (Down)/ Auto (Down)) are not pressed
		while ((GPIO_PORTF_DATA_R & (1<<0)) && (GPIO_PORTF_DATA_R & (1<<3))&&(!(GPIO_PORTF_DATA_R & (1<<1))) && (GPIO_PORTF_DATA_R & (1<<4)) && (GPIO_PORTF_DATA_R & (1<<2)))
	  { 
		  //Rotates Motor Up Manually
		 GPIO_PORTA_DATA_R |= (1<<6);
		 GPIO_PORTA_DATA_R &=~ (1<<7);	
		 GPIO_PORTA_DATA_R |=(1<<5);
	  }
		
		//Stop the Motor if upper limit switch or Driver’s Pushbuttons (Auto (UP) / Auto (Down)/ Manual (Down)) are pressed
		GPIO_PORTA_DATA_R &=~(1<<5);	
	}
}

/***************************** DRIVER MANUAL (DOWN) Task *****************************/
/*------------------------------------------------------------------------------------*/

void v_DDOWNManualTask(void *pvParameters)
{
	  xSemaphoreTake(xDOWNManualSemaphore,0);											// Take Semaphore
	for(;;)
	{
		xSemaphoreTake(xDOWNManualSemaphore,portMAX_DELAY);					// Take xDOWNManualSemaphore	
		
		// check while the lower limit switch and Driver’s Pushbuttons (Auto (UP) / Auto (Down)/ Manual (UP)) are not pressed
    while ((GPIO_PORTE_DATA_R & (1<<1)) && (GPIO_PORTF_DATA_R & (1<<1)) && (!(GPIO_PORTF_DATA_R & (1<<3))) && (GPIO_PORTF_DATA_R & (1<<4)) && (GPIO_PORTF_DATA_R & (1<<2)))
		 { 
		  //Rotates Motor Down Manually			 
			GPIO_PORTA_DATA_R |= (1<<7);
	    GPIO_PORTA_DATA_R &=~ (1<<6);	
			GPIO_PORTA_DATA_R |=(1<<5);
		 }
		//Stop the Motor if lower limit switch or Driver’s Pushbuttons (Auto (UP) / Auto (Down)/ Manual (UP)) are pressed
		GPIO_PORTA_DATA_R &=~(1<<5);	
		 
		// Resume the Auto Driver and Manual and Driver Auto/Manual Shotgun up tasks if there they are suspended during jamming
	  vTaskResume(Auto_up_Handle);
	  vTaskResume(Manual_up_Handle);
	  vTaskResume(P_up_Handle);
	}
}

/***************************** LOCK Task *****************************/
/*------------------------------------------------------------------*/

void vLOCKTask(void *pvParameters){
	xSemaphoreTake(xLOCKSemaphore,0);											// Take Semaphore
	for(;;)
	{
		xSemaphoreTake(xLOCKSemaphore,portMAX_DELAY);				// Take xLOCKSemaphore
		
		// Stopping the Motor while the switch is ON whether the shotgun is pressing on the pushbuttons or not
		while(((GPIOD->DATA & (1<<2)) ==0) && ( ((GPIOD->DATA & (1<<0)) == 0) || ((GPIOD->DATA & (1<<1)) == 0) || (!((GPIOD->DATA & (1<<0)) == 0)) || (!((GPIOD->DATA & (1<<1)) == 0))) )
		{
  		GPIO_PORTA_DATA_R &=~ (1<<5);
		}
	}		
}


/***************************** SHOTGUN Auto/Manual UP Task *****************************/
/*------------------------------------------------------------------------------------*/

void v_PUPTask(void *pvParameters){
	xSemaphoreTake(xUPSemaphore_P,0);											// Take Semaphore
	for(;;)
	{
		xSemaphoreTake(xUPSemaphore_P,portMAX_DELAY);				// Take xUPSemaphore_P
		int counter = 0;
		
		// check if the Pushbutton (UP) is pressed and the upper Limit switch is not pressed
    if ((((GPIOD->DATA & (1<<0)) == 0)) && (!((GPIOF->DATA & (1<<0)) == 0)))
		{
			// count for 100 msec to check either  Auto/Manual state
		  while (counter < 100*3180)
		  {
				// Rotate the Motor in the (up) direction
		    GPIO_PORTA_DATA_R |= (1<<5)|(1<<6);
	      GPIO_PORTA_DATA_R &=~ (1<<7);
        counter ++;		
		  }
	  }
		
		// check during the Pushbutton (UP) is pressed and the upper Limit switch is not pressed		
    while (((GPIOD->DATA & (1<<0)) == 0)&& (!((GPIOF->DATA & (1<<0)) == 0)))
		{
			// check if the Pushbutton (UP) is pressed and the upper Limit switch is not pressed
			if (((GPIOD->DATA & (1<<0)) == 0) && (!((GPIOF->DATA & (1<<0)) == 0)))	
			{
				// Rotate the Motor in the (up) direction
				GPIO_PORTA_DATA_R |= (1<<5)|(1<<6);
	      GPIO_PORTA_DATA_R &=~ (1<<7);	
			}
			
			// if the Pushbutton (UP) is released stop the Motor
			GPIO_PORTA_DATA_R &=~ (1<<5);
		}
		
	}		
}

/***************************** SHOTGUN Auto/Manual Down Task *****************************/
/*--------------------------------------------------------------------------------------*/

void v_PDOWNTask(void *pvParameters){
	xSemaphoreTake(xDOWNSemaphore_P,0);											// Take Semaphore
	for(;;)
	{
		xSemaphoreTake(xDOWNSemaphore_P,portMAX_DELAY);				// Take xDOWNSemaphore_P
		int counter = 0;
		
		// check if the Pushbutton (Down) is pressed and the upper Limit switch is not pressed
    if ((((GPIOD->DATA & (1<<1)) == 0)) && (!((GPIOE->DATA & (1<<1)) == 0)))
		{
			// count for 100 msec to check either  Auto/Manual state
		  while (counter < 100*3180)
		  {
				// Rotate the Motor in the (Down) direction
		    GPIO_PORTA_DATA_R |= (1<<5)|(1<<7);
	      GPIO_PORTA_DATA_R &=~ (1<<6);
        counter ++;		
		  }
	  }
		
		// check during the Pushbutton (Down) is pressed and the Lower Limit switch is not pressed		
    while (((GPIOD->DATA & (1<<1)) == 0) && (!((GPIOE->DATA & (1<<1)) == 0)))
		{
			// check if the Pushbutton (Down) is pressed and the lower Limit switch is not pressed
			if (((GPIOD->DATA & (1<<1)) == 0) && (!((GPIOE->DATA & (1<<1)) == 0)))	
			{
				// Rotate the Motor in the (Down) direction
				GPIO_PORTA_DATA_R |= (1<<5)|(1<<7);
	      GPIO_PORTA_DATA_R &=~ (1<<6);	
			}
			// if the Pushbutton (Down) is released stop the Motor
			GPIO_PORTA_DATA_R &=~ (1<<5);
		}
		
		// Resume the Auto Driver and Manual and Driver Auto/Manual Shotgun up tasks if there they are suspended during jamming
	  vTaskResume(P_up_Handle);
	  vTaskResume(Auto_up_Handle);
	  vTaskResume(Manual_up_Handle);
	}		
}



/***************************** main function *****************************/
/*----------------------------------------------------------------------*/
int main( void )
{
	
	/******************** Call Ports initialization ********************/
		PortA_Init();
    PortD_Init();
    PortE_Init();
    PortF_Init();
	
	/********************* Enable Interrupts *********************/
		__ASM("CPSIE i");
		
	/************* Create Semaphore for each Task *************/
	
		vSemaphoreCreateBinary(xUPAutomaticSemaphore);				// Create Driver Automatic up Semaphore
	  vSemaphoreCreateBinary(xDOWNAutomaticSemaphore);			// Create Driver Automatic down Semaphore
		vSemaphoreCreateBinary(xUPManualSemaphore);						// Create Driver Manual up Semaphore
	  vSemaphoreCreateBinary(xDOWNManualSemaphore);					// Create Driver Manual down Semaphore
		vSemaphoreCreateBinary(xUPSemaphore_P);								// Create shotgun Auto/Manual up Semaphore
		vSemaphoreCreateBinary(xDOWNSemaphore_P);							// Create shotgun Auto/Manual down Semaphore
		vSemaphoreCreateBinary(xJamSemaphore);     						// Create Jam Semaphore
		vSemaphoreCreateBinary(xLOCKSemaphore);					  		// Create Lock Semaphore
 
	// Check if all the semaphores are created
	if( (xUPAutomaticSemaphore && xDOWNAutomaticSemaphore &&xDOWNManualSemaphore &&xUPManualSemaphore && xJamSemaphore && xUPSemaphore_P && xDOWNSemaphore_P &&  xLOCKSemaphore)!= NULL )
		{
			// Create Jamming Task and configure it as highest priority
  		xTaskCreate( v_JamTask, "JAM", 150, NULL, 4, NULL );
			
			// Create Driver’s Tasks and configure it’s priority
			xTaskCreate( v_DUPAutomaticTask, "UP_A", 150, NULL, 3, &Auto_up_Handle);
  		xTaskCreate( v_DDOWNAutomaticTask, "DOWN_A", 150, NULL, 3, NULL );
			xTaskCreate( v_DUPManualTask, "UP_M", 150, NULL, 3, &Manual_up_Handle );
  		xTaskCreate( v_DDOWNManualTask, "DOWN_M", 150, NULL, 3, NULL );
			
			// Create Lock Task and configure it’s priority
			xTaskCreate( vLOCKTask, "LOCK", 150, NULL, 2, NULL );
			
			// Create shotgun’s Task and configure it’s priority as lowest priority
			xTaskCreate( v_PUPTask, "UP Driver", 150, NULL, 1, &P_up_Handle );
			xTaskCreate( v_PDOWNTask, "DOWN Driver", 150, NULL, 1, NULL );
			
			vTaskStartScheduler();
		}
		
    for( ;; );
}

/***************************** Ports initilization *****************************/
/*----------------------------------------------------------------------------*/


/***************************** PortA initilization **************************/

void PortA_Init(void) {
	
	SYSCTL->RCGCGPIO |= (1<<0);               							// Enable portA clock
	GPIO_PORTA_DIR_R |= (1 << 5) | (1 << 6) | (1 << 7);			// Configure portA pins as Outputs 
	GPIO_PORTA_DEN_R |= (1 << 5) | (1 << 6) | (1 << 7);			// enable digital pins PA5 , PA6, PA7
	GPIO_PORTA_DATA_R &= ~ (1<<5)&(1<<6)&(1<<7);						// configure initial values on PA5 , PA6, PA7
}

/***************************** PortD initilization **************************/

void PortD_Init(void){ 

	SYSCTL->RCGCGPIO |= (1<<3);               // Enable portD clock
	GPIOD->LOCK = 0x4C4F434B;  				        // unlock PortD  to enable to write on it
  GPIOD->CR |= (1<<0) |(1<<1) |(1<<2);      // Allow changes on PD0, PD1, PD2       
  GPIOD->AMSEL= 0x00;       				        // Disable analog function
  GPIOD->DIR |= 0x00;         				      // Configure portD pins as inputs   
  GPIOD->AFSEL = 0x00;      				        // no alternate function
  GPIOD->PUR |= (1<<0) |(1<<1) |(1<<2);     // enable pullup resistors on PD0, PD1, PD2       
  GPIOD->DEN |= (1<<0) |(1<<1) |(1<<2);     // enable digital pins PD0, PD1, PD2 
	GPIOD->DATA = 0x00;												// configure initial values on PD0, PD1, PD2 to zeros
	
	GPIOD->ICR = (1<<0)|(1<<1)|(1<<2);        // Clear any Previous Interrupt 
	GPIOD->IM |= (1<<0)|(1<<1)|(1<<2);        // Unmask the interrupts for PD0, PD1, PD2
	GPIOD->IS &=~ (1<<0)&(1<<1) &(1<<2);      // Make bits PD0, PD1, PD2 edge sensitive
	GPIOD->IEV &=~ (1<<0)&(1<<1) &(1<<2);     // Sense on Rising Edge
	
	NVIC_EnableIRQ(PortD_IRQn);               // Enable the Interrupt for PortD in NVIC
	NVIC_PRI0_R = 0xe0000000;									//configure portD  Interrupt Priority equals to 7

}

/***************************** PortE initilization **************************/

void PortE_Init(void){ 
	
	SYSCTL->RCGCGPIO |= 0x00000010;          	// Enable portE clock
	GPIOE->LOCK = 0x4C4F434B;  				       	// unlock PortE  to enable to write on it 
  GPIOE->CR = 0x1F;          				       	// Allow changes on PE1 and PE2      
  GPIOE->AMSEL= 0x00;       				       	// Disable analog function
  GPIOE->DIR |= 0x00;         				     	// Configure portE pins as inputs    
  GPIOE->AFSEL = 0x00;      				       	// no alternate function
  GPIOE->PUR |= (1<<1)|(1<<2);       		   	// enable pullup resistors on PE1 and PE2        
  GPIOE->DEN = 0x1F;       				         	// enable digital pins PE1 and PE2 
	GPIOE->DATA = 0x00;												// configure initial values on PE1 and PE2 to zeros
	
	GPIOE->ICR = (1<<2)|(1<<1);              	// Clear any Previous Interrupt 
	GPIOE->IM |= (1<<2)|(1<<1);               // Unmask the interrupts for PE1 and PE2
	GPIOE->IS &=~(1<<2)&(1<<1);               // Make bits PE1 and PE2 edge sensitive
	GPIOE->IEV &=~(1<<1);                     // Sense on Rising Edge
	GPIOE->IEV |=(1<<2);                      // Sense on Falling Edge
	
	
	NVIC_EnableIRQ(PortE_IRQn);              // Enable the Interrupt for PortE in NVIC
}


/***************************** PortF initilization **************************/

void PortF_Init(void){ 

  SYSCTL->RCGCGPIO |= 0x00000020;        												  // Enable portF clock
  GPIOF->LOCK = 0x4C4F434B;  				      											 	// unlock PortF  to enable to write on it  
  GPIOF->CR = 0x1F;          				     												  // Allow changes on PF0 to PF4    
  GPIOF->AMSEL= 0x00;       				     												  // Disable analog function 
  GPIOF->DIR = 0x00;         				      											  // Configure portF pins as inputs    
  GPIOF->AFSEL = 0x00;      				      											  // no alternate function
  GPIOF->PUR =  (1<<0) | (1<<1) | (1<<2) | (1<<3) |(1<<4) ;       // enable pullup resistors on PF0 to PF4    
  GPIOF->DEN = 0x1F;       				         												// enable digital pins PF0 to PF4						 
	GPIOF->DATA = 0x00;																							// configure initial values on PF0 to PF4 to zeros
	

	GPIOF->ICR = (1<<4)|(1<<2) |(1<<1)|(1<<3)|(1<<0);      					// Clear any Previous Interrupt 
	GPIOF->IM |= (1<<4)|(1<<2)|(1<<1) |(1<<3)|(1<<0);      					// Unmask the interrupts for PF0 and PF4
	GPIOF->IS &=~ (1<<4)&(1<<2)&(1<<1) &(1<<3)|(1<<0);    				  // Make bits PF0 and PF4 edge sensitive
	GPIOF->IEV &=~ (1<<3)&(1<<1)|(1<<0);                            // Sense on Rising Edge
	GPIOF->IEV |= (1<<4)|(1<<2);                                    // Sense on Falling Edge
	
	
	NVIC_EnableIRQ(PortF_IRQn);             											 // Enable the Interrupt for PortF in NVIC
	NVIC_PRI7_R = 0xC00000;																				 //configure portF  Interrupt Priority equals to 6

}


/***************************** Interrupts Handler *****************************/
/*---------------------------------------------------------------------------*/


/***************************** PORTD ISR *****************************/
void GPIOD_Handler(void)
 {
	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	 
	/***** checking if the shotgun clicks on the Automatic/Manual (UP) pushbutton*****/
 
	if (GPIOD->MIS & (1<<0))
	{	
  xSemaphoreGiveFromISR(xUPSemaphore_P,&xHigherPriorityTaskWoken);   		//Give the xUPSemaphore_P to the Task 
	GPIOD->ICR = (1<<0);                  																// clear the interrupt flag of PORTF
  i= GPIOD->ICR ;                       																// Reading the register to force the flag to be cleared
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );										//Switching to the highest priority task taht has taken its semaphore
	}
	
	/***** checking if the shotgun clicks on the Automatic/Manual (Down) pushbutton*****/

	if (GPIOD->MIS & (1<<1))
	{	
  xSemaphoreGiveFromISR(xDOWNSemaphore_P,&xHigherPriorityTaskWoken);  	 //Give the xDOWNSemaphore_P to the Task 
	GPIOD->ICR = (1<<1);                  															   // clear the interrupt flag of PORTF
  i= GPIOD->ICR ;                       																 // Reading the register to force the flag to be cleared
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );										 //Switching to the highest priority task taht has taken its semaphore
	}
	
	/***** checking if the shotgun is locked *****/

	if (GPIOD->MIS & (1<<2))
	{	
  xSemaphoreGiveFromISR(xLOCKSemaphore,&xHigherPriorityTaskWoken);			//Give the xLOCKSemaphore to the Task 
	GPIOD->ICR = (1<<2);                  																// clear the interrupt flag of PORTF
  i= GPIOD->ICR ;                      																	// Reading the register to force the flag to be cleared
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );										//Switching to the highest priority task taht has taken its semaphore
	}
	
 }

 /***************************** PORTE ISR *****************************/

void GPIOE_Handler(void){
	
	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	/***** checking if there is an obstacle *****/
	
	if (GPIOE->MIS &(1<<2))
	{
  xSemaphoreGiveFromISR(xJamSemaphore,&xHigherPriorityTaskWoken);		//Give the xJamSemaphore to the Task 
	GPIOE->ICR = (1<<2);                  														// clear the interrupt flag of PORTF
  i= GPIOE->ICR ;                       														// Reading the register to force the flag to be cleared
	for(int i = 0; i<= 16000*75; i++){}																// Delay due to switch debouncing				
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );								//Switching to the highest priority task taht has taken its semaphore
	}
	
	/***** checking if the lower Limit switch is pressed *****/

	if ((GPIOE->MIS & (1<<1)))
	{
	  GPIOE->ICR = (1<<1);               															// clear the interrupt flag of PORTF
    i= GPIOE->ICR ;                    															// Reading the register to force the flag to be cleared
	  GPIO_PORTA_DATA_R &=~ (1<<5);			 															//Stopping Motor by disable PIN5
	}
}


/***************************** PORTF ISR *****************************/

void GPIOF_Handler(void){
	
	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	/***** checking if the upper Limit switch is pressed *****/
	
	if (GPIOF->MIS & (1<<0))
	{
	  GPIOF->ICR = 0x01;                    												// clear the interrupt flag of PF0
    i= GPIOF->ICR ;                       												// Reading the register to force the flag to be cleared
		GPIO_PORTA_DATA_R &=~ (1<<5);																	//Stopping Motor by disable PIN5
	}	

	/***** checking if the Driver clicks on the Automatic (Down) pushbutton*****/
	
	if (GPIOF->MIS &(1<<2))
	{
  xSemaphoreGiveFromISR(xDOWNAutomaticSemaphore,&xHigherPriorityTaskWoken);			//Give the xDOWNAutomaticSemaphore to the Task 
	GPIOF->ICR = (1<<2);                  								 												// clear the interrupt flag of PF2
  i= GPIOF->ICR ;                        								 												// Reading the register to force the flag to be cleared
	for(int i = 0; i<= 16000*75; i++){}										 												// Delay due to switch debouncing
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken ); 		 												//Switching to the highest priority task taht has taken its semaphore
	}
	
	/***** checking if the Driver clicks on the Automatic (UP) pushbutton*****/

	if (GPIOF->MIS &(1<<4))
	{
  xSemaphoreGiveFromISR(xUPAutomaticSemaphore,&xHigherPriorityTaskWoken);			//Give the xUPAutomaticSemaphore to the Task 
	GPIOF->ICR = 0x10;                    																			// clear the interrupt flag of PF4
  i= GPIOF->ICR ;                        																			// Reading the register to force the flag to be cleared
	for(int i = 0; i<= 16000*75; i++){}																					// Delay due to switch debouncing
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );	  											//Switching to the highest priority task taht has taken its semaphore
	}
	
	/***** checking if the Driver clicks on the Manual (UP) pushbutton*****/

	if (GPIOF->MIS &(1<<1))
	{
  xSemaphoreGiveFromISR(xUPManualSemaphore,&xHigherPriorityTaskWoken);			 //Give the xUPManualSemaphore to the Task 
	GPIOF->ICR = (1<<1);                   																		 // clear the interrupt flag of PF1
  i= GPIOF->ICR ;                        									                   // Reading the register to force the flag to be cleared
	for(int i = 0; i<= 16000*75; i++){}											                   // Delay due to switch debouncing
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );			                   //Switching to the highest priority task taht has taken its semaphore
	}
	
	/***** checking if the Driver clicks on the Manual (Down) pushbutton*****/
	if (GPIOF->MIS &(1<<3))
	{
  xSemaphoreGiveFromISR(xDOWNManualSemaphore,&xHigherPriorityTaskWoken);		//Give the xDOWNManualSemaphore to the Task 
	GPIOF->ICR = (1<<3);                   																		// clear the interrupt flag of PF3
  i= GPIOF->ICR ;                        																		// Reading the register to force the flag to be cleared
	for(int i = 0; i<= 16000*75; i++){}																				// Delay due to switch debouncing
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );												//Switching to the highest priority task taht has taken its semaphore
	}
	
}


