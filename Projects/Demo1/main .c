/**
  ******************************************************************************
  * @file    IO_Toggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f429i_discovery.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"


/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStructure;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void Hardware_Init(void);
void Red_LED_On(void);
void Red_LED_Off(void);
void Green_LED_On(void);
void Green_LED_Off(void);


void Sender_Task(void*);//Send data
void Reciver_Task(void*);//Reive data
void Monitor_Task(void*)//monite data
xQueue xQueueCreat(10000,sizeof(int));//Queue for Sender_Task and Receiver_Task
//xQueueS_M xQueueCreat(10000,sizeof(int));//Queue for Sender_Task and Monitor_Task
//xQueueR_M xQueueCreat(10000,sizeof(int));//Queue for Monitor_Task and Receiver_Task
unsigned int Sum_Sender;//Sum for Sender
unsigned int Sum_Receiver;//Sum for Receiver





/**
 * Red_LED_On: 
 */
void Red_LED_On(void)
{
//    GPIO_SetBits(GPIOG, GPIO_Pin_14);
    GPIOG->ODR |= 0x4000;
}

/**
 * Red_LED_Off: 
 */
void Red_LED_Off(void)
{
//    GPIO_ResetBits(GPIOG, GPIO_Pin_14);
  GPIOG->ODR &= 0xBFFF;
}

/**
 * Green_LED_On: 
 */
void Green_LED_On(void)
{
//    GPIO_SetBits(GPIOG, GPIO_Pin_13);
    GPIOG->ODR |= 0x2000;
}

/**
 * Green_LED_Off: 
 */
void Green_LED_Off(void)
{
//    GPIO_ResetBits(GPIOG, GPIO_Pin_13);
    GPIOG->ODR &= 0xDFFF;
}
/**





/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */
       Hardware_Init();
 
       /* Init and start tracing*/
        vTraceEnable(TRC_INIT);
        vTraceEnable(TRC_START);

       /* Create tasks */
       xTaskCreate(
		  Sender_Task,                 /* Function pointer */
		  "Task_LED1",                          /* Task name - for debugging only*/
		  configMINIMAL_STACK_SIZE,         /* Stack depth in words */
		  (void*) NULL,                     /* Pointer to tasks arguments (parameter) */
		  tskIDLE_PRIORITY + 3UL,           /* Task priority*/
		  NULL                              /* Task handle */
       );

       xTaskCreate(
		  Reciver_Task,                 /* Function pointer */
		  "Task_LED2",                          /* Task name - for debugging only*/
		  configMINIMAL_STACK_SIZE,         /* Stack depth in words */
		  (void*) NULL,                     /* Pointer to tasks arguments (parameter) */
		  tskIDLE_PRIORITY + 2UL,           /* Task priority*/
		  NULL                              /* Task handle */
       );

	 xTaskCreate(
		  Monitor_Task,                 /* Function pointer */
		  "Task_LED3",                          /* Task name - for debugging only*/
		  configMINIMAL_STACK_SIZE,         /* Stack depth in words */
		  (void*) NULL,                     /* Pointer to tasks arguments (parameter) */
		  tskIDLE_PRIORITY + 1UL,           /* Task priority*/
		  NULL                              /* Task handle */
       );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for( ;; );

}


/**
 * Hardware_Init: 
 */
void Hardware_Init(void)
{
        /* GPIOG Periph clock enable */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

        /* Configure PG13, PG14 in output pushpull mode */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13| GPIO_Pin_14;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOG, &GPIO_InitStructure);

}

 /* Sender_Task: Toggle LED1 via RTOS Timer
 */




void Sender_Task(void *pvParameters)
{
	unsigned int i=1,sum=0;
	
	while(1)
	{		
		printf("i=%u\n",i);
		if(i>10000)
		i-=10000;
		xQueueSend(xQueueS_R,void*(&i),0);
		//xQueueSend(xQueueS_M,i,0);
		Sum_Sender+=i;
		
		 vTaskDelay(2);
		i++;
	}



/*    int led = 0;  

    while (1) 
    {
       if(led == 0)
        {
            Red_LED_On();
            led = 1;
        } 
        else
        {
            Red_LED_Off();
            led = 0;
*/         }
        /*
        Delay for a period of time. vTaskDelay() places the task into
        the Blocked state until the period has expired.
        The delay period is spacified in 'ticks'. We can convert
        yhis in milisecond with the constant portTICK_RATE_MS.
        */
 //       vTaskDelay(1000 / portTICK_RATE_MS);
  //}
}

/**
 * Reciver_Task: Toggle LED2 via RTOS Timer
 */


void Reciver_Task(void *pvParameters)

{
	
	unsigned int j=0,sum_2=0;//ii from 1 to 10000;sum_2 is the total
	while(1)
	{	
		
		for(;xQueueReceive(xQueue,&j,1000/portTICK_RATE_MS)==pdTRUE;)
		j=xQueueRecive(xQueue,&valueToSend,0);
		
		{
		Sum_Receiver+=j;
		}
		vTaskDelay(1000);
		
	}









/*    int led = 0;  
    while (1) 
    {
        if(led == 0)
        {
            Green_LED_On();
            led = 1;
        } 
        else
        {
            Green_LED_Off();
            led = 0;
*/    //     }
        /*
        Delay for a period of time. vTaskDelay() places the task into
        the Blocked state until the period has expired.
        The delay period is spacified in 'ticks'. We can convert
        yhis in milisecond with the constant portTICK_RATE_MS.
        */
 //       vTaskDelay(2000 / portTICK_RATE_MS);
 //}
}




void Monitor_Task(void *pvParameters)

{
	Green_LED_Off();
	Red_LED_Off();
	
	
	while(1)
	{
		
		if(Sum_Sender==Sum_Receiver)
		{
		Green_LED_On();
		vTaskDeley(1000);
		}		
		else
		{
		Red_LED_On();
		vTaskDeley(1000);
		}
		
		vTaskDelay(10000);
		
	}



}








void vApplicationTickHook( void )
{
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
