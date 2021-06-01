/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * @Author         : Yann Jacquet, Maxence Huart, Marcello Ternullo
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* include of librairies usefull for our purpose*/
#include "USBH_HID_generic/usbh_hid.h"
#include "stdio.h"
#include "Dualshock3_driver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* declaration of variable to take care of huart 2 (debug system) and 3 (communication from this 
 *   stm32 to another stm32
*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* variable of "USBH_HID_generic/usbh_hid.h" */
extern USBH_HID_DriverTypeDef dualshock3_Driver;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 128 * 4
};
/* Definitions for usbhTask */
osThreadId_t usbhTaskHandle;
const osThreadAttr_t usbhTask_attributes = {
		.name = "usbhTask",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 128 * 4
};
/* Definitions for processReport */
osThreadId_t processReportHandle;
const osThreadAttr_t processReport_attributes = {
		.name = "processReport",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 1024 * 4
};
/* Definitions for stateMachineThread */
osThreadId_t stateMachineThreadHandle;
const osThreadAttr_t stateMachineThread_attributes = {
		.name = "stateMachine",
		.priority = (osPriority_t) osPriorityAboveNormal,
		.stack_size = 128 * 4
};
/* Definitions for idleThread */
osThreadId_t idleThreadHandle;
const osThreadAttr_t idleThread_attributes = {
		.name = "idle",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 128 * 4
};
/* Definitions for translatorThread */
osThreadId_t translatorThreadHandle;
const osThreadAttr_t translatorThread_attributes = {
		.name = "translator",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 1024 * 4
};
/* Definitions for reportQueue */
osMessageQueueId_t reportQueueHandle;
const osMessageQueueAttr_t reportQueue_attributes = {
		.name = "reportQueue"
};
/* Definitions for usbhMutex */
osMutexId_t usbhMutexHandle;
const osMutexAttr_t usbhMutex_attributes = {
		.name = "usbhMutex"
};
/* Definitions for uart2Mutex */
osMutexId_t uart2MutexHandle;
const osMutexAttr_t uart2Mutex_attributes = {
		.name = "uart2Mutex"
};

/* Definitions for uart3Mutex */
osMutexId_t uart3MutexHandle;
const osMutexAttr_t uart3Mutex_attributes = {
		.name = "uart3Mutex"
};

/* Declarations of semaphores */
osSemaphoreId_t idleOn;
osSemaphoreId_t translatorOn;
osSemaphoreId_t deviceConnected;
osSemaphoreId_t deviceDisconnected;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void usbhTaskFunction(void *argument);
void processReportFunction(void *argument);

void idleThread(void *argument);		//Idle state thread
void translatorThread(void *argument);		//Translator state thread
void stateMachineThread(void *argument);	//State machine gestion thread

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//uint32_t lastReportTime = 0;

void Dualshock3_newReport_CB(DS3_report* report)
{
        // check if free space is available in the queue
	if(osMessageQueueGetSpace(reportQueueHandle) == 0)
	{
                // if there are no more space, the queue is resetted 
		osMessageQueueReset(reportQueueHandle);
	}
        // data put inside the queue
	osMessageQueuePut(reportQueueHandle, report, 0, 0);

        // debug :
	/*char Uart_Buf[100];
	sprintf(Uart_Buf, "got Report \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)Uart_Buf, strlen(Uart_Buf), 100);*/
	//lastReportTime = osKernelGetTickCount();
}

void Dualshock3_connected_CB(){
/* callback called when the initialisation of usb is finished */
  
	// use semaphore to start report task
	osSemaphoreRelease(deviceConnected);
}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	MX_USB_HOST_Init();

	// stops the broken auto generated thread for the usbh process
	MX_USB_HOST_SuspendThread();

	// loads the dualshock 3 driver
	USBH_HID_GenericRegisterDriver(dualshock3_Driver);

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();
	/* Create the mutex(es) */
	/* creation of usbhMutex */
	usbhMutexHandle = osMutexNew(&usbhMutex_attributes);

	/* creation of uart2Mutex */
	uart2MutexHandle = osMutexNew(&uart2Mutex_attributes);

	/* creation of uart3Mutex */
	uart3MutexHandle = osMutexNew(&uart3Mutex_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	idleOn = osSemaphoreNew(1U, 0, NULL);
	translatorOn = osSemaphoreNew(1U, 0, NULL);

	deviceConnected = osSemaphoreNew(1U, 0, NULL);
	deviceDisconnected = osSemaphoreNew(1U, 0, NULL);
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of reportQueue */
	reportQueueHandle = osMessageQueueNew (2, sizeof(DS3_report), &reportQueue_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of usbhTask */
	usbhTaskHandle = osThreadNew(usbhTaskFunction, NULL, &usbhTask_attributes);

	/* creation of processReport */
	//processReportHandle = osThreadNew(processReportFunction, NULL, &processReport_attributes);

	/* creation of idleThread */
	idleThreadHandle = osThreadNew(idleThread, NULL, &idleThread_attributes);

	/* creation of translatorThread */
	translatorThreadHandle = osThreadNew(translatorThread, NULL, &translatorThread_attributes);

	/* creation of stateMachineThread */
	stateMachineThreadHandle = osThreadNew(stateMachineThread, NULL, &stateMachineThread_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */

	osThreadSuspend(defaultTaskHandle);
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	//huart2.Init.WordLength = 64;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure onboard led */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* init code for USB_HOST */
	MX_USB_HOST_Init();
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_usbhTaskFunction */
/**
 * @brief Function implementing the usbhTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_usbhTaskFunction */
void usbhTaskFunction(void *argument)
{
	/* USER CODE BEGIN usbhTaskFunction */
	/* Infinite loop */
	for(;;)
	{
		osMutexAcquire(usbhMutexHandle, 500U);
		MX_USB_HOST_Process();
		osMutexRelease(usbhMutexHandle);

		osDelay(1);
	}
	/* USER CODE END usbhTaskFunction */
}




/* USER CODE BEGIN Header_processReportFunction */
/**
 * @brief Function implementing the processReport thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_processReportFunction */
void processReportFunction(void *argument)
{
	/* USER CODE BEGIN processReportFunction */
	/* Infinite loop */

	//char Uart_Buf[192] = "";

	for(;;)
	{
		/*DS3_report report;
		osMessageQueueGet(reportQueueHandle, &report, NULL, osWaitForever);

		sprintf(Uart_Buf, "Buttons: %2X %2X %2X\r\n", ((uint8_t*)&report)[2], ((uint8_t*)&report)[3], ((uint8_t*)&report)[4]);

		HAL_UART_Transmit(&huart2, (uint8_t *)Uart_Buf, strlen(Uart_Buf), 100);

		sprintf(Uart_Buf, "X: %4d, Y: %4d, Z: %4d, G: %4d\r\n",
				decodeRawData(report.AccX_Raw_HI, report.AccX_Raw_LO),
				decodeRawData(report.AccY_Raw_HI, report.AccY_Raw_LO),
				decodeRawData(report.AccZ_Raw_HI, report.AccZ_Raw_LO),
				decodeRawData(report.Gyro_Raw_HI, report.Gyro_Raw_LO));


		HAL_UART_Transmit(&huart2, (uint8_t *)Uart_Buf, strlen(Uart_Buf), 100);
		HAL_UART_Transmit(&huart2, (uint8_t *)"\033[F\033[F", strlen("\033[F\033[F"), 100);*/

	}
	/* USER CODE END processReportFunction */
}




/**
 * @brief  Thread managing the state 1 of the translator.
 * @param  argument: Not used
 * @retval None
 */
void idleThread(void *argument){
/* COMMENT: 
   This is the first state that the state machine calls. This state allows to loop if any controller is connected. 
   Now if a controller is detected and connected on the usb, then the semaphore is released
 */ 
        // variable used for the uart2 (debug)
	char Uart_Buf[100];

	for(;;){
		// acquiring of semaphore
		osStatus_t tmp = osSemaphoreAcquire(idleOn,50U);
		if(tmp == osOK){
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

			// showing up of a message to the console. This ressource is protected by a mutex
			sprintf(Uart_Buf, "in idle\r\n");
			osMutexAcquire(uart2MutexHandle, 50U);
			HAL_UART_Transmit(&huart2, (uint8_t *)Uart_Buf, strlen(Uart_Buf), 100);
			osMutexRelease(uart2MutexHandle);
		}

		osDelay(50);
	}
}


/**
 * @brief  Thread managing the state 2 of the translator.
 * @param  argument: Not used
 * @retval None
 */
void translatorThread(void *argument){
/* This function allows to send all the data through uart3. This uart 
   is the communication from this stm32 to another stm32(emulator) 
*/
	// report variable contents all controller changes
	DS3_report report;
	// Arrays of characters to manage debug and the data transmission (controller report)
	char Uart_Buf[128];
	char Uart_Buf1[128];
	char Uart_Buf2[128];
	char Uart3_buf[128];
	//char Uart3_buf1[130];

	// Run
	for(;;){
               
		// semaphore acquire a token
		osStatus_t tmp = osSemaphoreAcquire(translatorOn,50U);
		
		// if the token is acquired, then the process of the task could start
		if(tmp == osOK){

			// debug :  state status displayed to the console with a mutex protection on the uart
			sprintf(Uart_Buf, "in Translator\r\n");
			osMutexAcquire(uart2MutexHandle, 50U);  
			HAL_UART_Transmit(&huart2, (uint8_t *)Uart_Buf, strlen(Uart_Buf), 100);
			osMutexRelease(uart2MutexHandle);

			// Get all change from the controller by the queue
			osStatus_t gotMess = osMessageQueueGet(reportQueueHandle, &report, NULL, 50U);

			// If the status is correct and has all we need then the debug and transmission will start
			if(gotMess == osOK){
				
				/* DEBUG : 
				sprintf(Uart_Buf, "Buttons: %2X %2X %2X\r\n", ((uint8_t*)&report)[2], ((uint8_t*)&report)[3], ((uint8_t*)&report)[4]);
                                osMutexAcquire(uart2MutexHandle, 50U);
				HAL_UART_Transmit(&huart2, (uint8_t *)Uart_Buf, strlen(Uart_Buf), 100);
				osMutexRelease(uart2MutexHandle);
				osDelay(5);  */

				// formatting the frame to the console debug. Displaying all data from accelerometer and gyroscope
				sprintf(Uart_Buf1, "X: %4d, Y: %4d, Z: %4d, G: %4d\r\n",
						decodeRawData(report.AccX_Raw_HI, report.AccX_Raw_LO),
						decodeRawData(report.AccY_Raw_HI, report.AccY_Raw_LO),
						decodeRawData(report.AccZ_Raw_HI, report.AccZ_Raw_LO),
						decodeRawData(report.Gyro_Raw_HI, report.Gyro_Raw_LO));

				// displaying data to the console with a mutex protection on the uart2
				/*osMutexAcquire(uart2MutexHandle, 50U);
				HAL_UART_Transmit(&huart2, (uint8_t *)Uart_Buf, strlen(Uart_Buf), 100);
				// end of transmission
				HAL_UART_Transmit(&huart2, (uint8_t *)"\033[F\033[F", strlen("\033[F\033[F"), 100);
				osMutexRelease(uart2MutexHandle);*/

				// Formatting the frame to the console debug. Displaying all data from the stick
				sprintf(Uart_Buf2, "LX: %4d, LY: %u, RX: %u, RY: %u\r\n",
						report.LeftStickX,
						report.LeftStickY,
						report.RightStickX,
						report.RightStickY);

				// Displaying all data contained in buf-1-2 to the console with a mutex protection on the uart2
				osMutexAcquire(uart2MutexHandle, 50U);
				HAL_UART_Transmit(&huart2, (uint8_t *)Uart_Buf, strlen(Uart_Buf), 100);
				HAL_UART_Transmit(&huart2, (uint8_t *)Uart_Buf1, strlen(Uart_Buf1), 100);
				HAL_UART_Transmit(&huart2, (uint8_t *)Uart_Buf2, strlen(Uart_Buf2), 100);
				osMutexRelease(uart2MutexHandle);

				// Translation managing
				uint8_t byte2Received = ((uint8_t*)&report)[2];
				uint8_t byte3Received = ((uint8_t*)&report)[3];
				uint8_t byte4Received = ((uint8_t*)&report)[4];

				uint8_t byte2ToSend = ((byte2Received & 0b00010000)>>4) | ((byte2Received & 0b01000000)>>5) |
									  ((byte2Received & 0b10000000)>>5) | ((byte2Received & 0b00100000)>>2) |
									  ((byte2Received & 0b00001000)<<1) | ((byte2Received & 0b00000001)<<5) |
									  ((byte2Received & 0b00000010)<<5) | ((byte2Received & 0b00000100)<<5);

				uint8_t byte3ToSend = ((byte3Received & 0b00000100)>>2) | ((byte3Received & 0b00001000)>>2) |
									  ((byte4Received & 0b00000001)<<2) | ((byte3Received & 0b00000000)<<3) |
									  ((byte3Received & 0b01000000)>>2) | ((byte3Received & 0b00100000)<<0) |
									  ((byte3Received & 0b10000000)>>1) | ((byte3Received & 0b00010000)<<3);


				/*DEBUG : 
					sprintf(Uart3_buf,"Report:");
					for(uint8_t i = 2; i < sizeof(report); i ++)
				     	{
				        	sprintf(Uart3_buf,"%s%2X", Uart3_buf, ((uint8_t*)&report)[i]); // ici on concatene: %s est remplacé par le contenu actuel du buffer, et on ajoute 2 caractères hexa pour le byte data[i]
				     	}
						*/

                                
				// sending all data to the another stm32. A total of 6 decimal bytes is sent on uart3
				sprintf(Uart3_buf,"%d%d%d%d%d%d", byte2ToSend, byte3ToSend,
									       report.LeftStickX,report.LeftStickY,
									       report.RightStickX,report.RightStickY);
                                
				// sending data on uart3. A protection is used on the uart3 ressource
				//sprintf(Uart3_buf,"%d\0", Uart3_buf);                                                           //A changer ICI !!!!!!!!!!!!!!!!
				osMutexAcquire(uart3MutexHandle, 50U);
				HAL_UART_Transmit(&huart3, (uint8_t *)Uart3_buf, strlen(Uart3_buf), 100);
				osMutexRelease(uart3MutexHandle);
			}
			else{
				// if the connection is lost, a semaphore send the signal
				osSemaphoreRelease(deviceDisconnected);
			}
		}
                
		osDelay(10);
	}
}


/**
 * @brief  Thread allowing to manage the state machine of translator app
 * @note   This function is called every time by the scheduler
 * @param  void : *argument
 * @retval None
 */
void stateMachineThread(void *argument){
	int state = 0;           // variable that manages states
	char Uart_Buf[100];      // variable for displaying the debug to the console

        // Run
	for(;;){
                
                
	if(osSemaphoreAcquire(deviceConnected,50U) == osOK){                    // this semaphore allows to manage the connection. if the connection is established then, state equals 1 (state translation)
			state = 1;
			//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);    // Debug
		}
		if(osSemaphoreAcquire(deviceDisconnected,50U) == osOK){				// this semaphore allows to manage the connection. if the connection is lost then, state equals 0 (state idle)
                 	state = 0;
		}

		// showing up of a message to the console. This ressource is protected by a mutex
		sprintf(Uart_Buf, "in State : %u\r\n", state);
		osMutexAcquire(uart2MutexHandle, 50U);
		HAL_UART_Transmit(&huart2, (uint8_t *)Uart_Buf, strlen(Uart_Buf), 100);
		osMutexRelease(uart2MutexHandle);


		/* DEBUG :
		   sprintf(Uart_Buf, "Ticks : %u\r\n", osKernelGetTickCount());
		   osMutexAcquire(uart2MutexHandle, 50U);
		   HAL_UART_Transmit(&huart2, (uint8_t *)Uart_Buf, strlen(Uart_Buf), 100);
		   osMutexRelease(uart2MutexHandle);*/

		// according to the value of the state the thread is selected
		switch(state){
		case 0:
			osSemaphoreRelease(idleOn);
			break;
		case 1:
			osSemaphoreRelease(translatorOn);
			break;
		}
		osDelay(10);
	}
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
