/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "complex.h"
#include "kiss_fft.h"

#include "C:\Users\User\STM32Cube\Repository\STM32Cube_FW_F4_V1.27.1\Drivers\BSP\STM32F429I-Discovery/stm32f429i_discovery.h"
#include "C:\Users\User\STM32Cube\Repository\STM32Cube_FW_F4_V1.27.1\Drivers\BSP\STM32F429I-Discovery/stm32f429i_discovery_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

uint16_t adc_buf[N];
uint16_t dac_buf[N];
float samples[N];
uint16_t count = 0;
float hamming[N];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
void fft_rec(double complex *x, int n, double complex *tw);
void FFT(int *x_in, double complex *x_out, double complex *tw);

void getSamples(float[]);
float hammingFunc(int, int);
void generateHamming(); //uint16_t hamming[]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void startScreen() {
	int data[32][32] = { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0,
			0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1 }, { 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 0, 0, 0, 0, 0, 1, 0,
			1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1 }, { 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
			{ 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1,
					1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 0, 0, 1, 0, 0, 0, 1, 1,
					1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 1 }, { 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1,
					1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 0, 1,
					1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
					0, 1, 1, 1, 1, 1, 1, 1, 1 }, { 1, 1, 0, 1, 0, 1, 1, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1,
					1, 1 }, { 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 1, 0, 1, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
					1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
					1 }, { 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 1, 1, 1, 1,
					0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1,
					1, 1, 1, 1, 1, 1, 1 },
			{ 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0,
					1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 0, 0, 0, 0, 0, 0, 0, 0,
					1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 1 }, { 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 0, 0,
					1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
					1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
					0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
					1, 1 }, { 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0,
					1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 0, 0, 0,
					1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1,
					1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1,
					0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
					1 }, { 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1,
					1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 1, 1, 0, 1,
					1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 1, 1, 1, 1, 1 },
			{ 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 1 }, { 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 1, 1, 1,
					0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1,
					1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 1, 1, 1, 0, 1, 0, 0, 1, 0, 1,
					1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
					1, 1 }, { 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0,
					0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1 }, { 1, 1, 1, 0,
					0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1,
					1, 1, 1, 1, 1, 0, 1, 1 } };

	BSP_LCD_Clear(LCD_COLOR_WHITE);
	for (int i = 0; i < 32; i++) {
		for (int j = 0; j < 32; j++) {
			BSP_LCD_DrawPixel(200 + i, 8 + j, data[j][i] * LCD_COLOR_BLACK);
			HAL_Delay(1);
		}
	}
}

void graph(kiss_fft_cpx *X, int len) {
	int y[len];
	for (int k = 0; k < len; k++) {
		y[k] = (int) 5 * abs(20 * log10(X[k].r));
		BSP_LCD_DrawVLine(k + 50 + 1, 280 - y[k], y[k]);
		HAL_Delay(1);
	}
}

void calculateTwiddleFactors(double complex *twiddleFactor) {
	for (int k = 0; k < N; k++) {
		twiddleFactor[k] = cexp(-1 * I * 2 * M_PI * k / N);
	}
}

//double FFT(int* x, int N, double complex * twiddleFactor, double* X){
//	// A recursive implementation of the 1D Cooley-STukey FFT,
//	// the input should have a length of power of 2.
//    if (N == 1) {
//        return x[0] ;
//    } else {
//        int X_even_arr[128] ;
//        int X_odd_arr[128] ;
//    	for (int k=0;  k<N; k++){
//    		if (k %2 == 0){
//    			X_even_arr[k/2] = x[k] ;
//    		} else {
//    			X_odd_arr[(int)ceil(k/2)] = x[k] ;
//    		}
//    	}
//
//	double X_even = FFT(X_even_arr, (int)N/2, twiddleFactor, X) ;
//	double X_odd = FFT(X_odd_arr, (int)N/2, twiddleFactor, X) ;
//
//	int index = 0 ;
//	for (int k=0; (int)N/2; k++){
//		X[index++] = X_even+twiddleFactor[k] *X_odd ;
//	}
//	for (int k=(int)N/2; k<N; k++){
//		X[index++] = X_even+twiddleFactor[k]*X_odd ;
//	}
//	return 1.0 ;
//    }
//}

void fft(int *x_in, double complex *x_out, double complex *tw) {

	// Make copy of array and apply window
	for (int i = 0; i < N; i++) {
		x_out[i] = (double complex) x_in[i];
		x_out[i] *= 1; // Window
	}

	// Start recursion
	fft_rec(x_out, N, tw);
}

void fft_rec(double complex *x, int n, double complex *tw) {
	// Check if it is splitted enough
	if (n <= 1) {
		return;
	}

	// Split even and odd
	double complex odd[n / 2];
	double complex even[n / 2];
	for (int i = 0; i < n / 2; i++) {
		even[i] = x[i * 2];
		odd[i] = x[i * 2 + 1];
	}

	// Split on tasks
	fft_rec(even, n / 2, tw);
	fft_rec(odd, n / 2, tw);

	// Calculate DFT
	for (int k = 0; k < n / 2; k++) {
		x[k] = ((even[k] + tw[k] * odd[k]));
		x[n / 2 + k] = ((even[k] - tw[k] * odd[k]));
	}
}

//void fft(double complex *x, int n, double complex *tw) {
//    if (n == 1)
//    	return;
//
//    double complex even[n/2], odd[n/2];
//    for(int i=0; i<n/2; i++) {
//        even[i] = x[2*i];
//        odd[i] = x[2*i+1];
//    }
//
//    fft(even, n/2, tw);
//    fft(odd, n/2, tw);
//
//    for(int k=0; k<n/2; k++){
//        double complex t = cexp(-2*M_PI * k / n) * odd[k];
//        x[k] = even[k] + t;
//        x[k+n/2] = even[k] - t;
//    }
//}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
	BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER);
	BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
	BSP_LCD_DisplayOn();
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font24);
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CRC_Init();
	MX_DMA2D_Init();
	MX_FMC_Init();
	MX_LTDC_Init();
	MX_SPI5_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	MX_I2C3_Init();
	MX_ADC1_Init();
	MX_TIM14_Init();
	/* USER CODE BEGIN 2 */
	// Start ADC Timer
	HAL_ADC_Start(&hadc1);
//	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
//	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) &dac_buf, N,
//			DAC_ALIGN_12B_R);
//	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_Base_Start_IT(&htim14);

	getSamples(samples); // Burn first adc values

	generateHamming(hamming);

	// Graph Heading
	startScreen();
	HAL_Delay(2000);

	BSP_LCD_DisplayStringAt(0, 0, (uint8_t*) "FFT", CENTER_MODE);

	// Axis Headings and Divisions
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(0, 300, (uint8_t*) "Frequency (kHz)", CENTER_MODE);
	for (int x = 50; x < 230 - 50; x += 16) {
		BSP_LCD_DrawPixel(x, 281, LCD_COLOR_BLACK);

//	  BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY) ;
//	  BSP_LCD_DrawVLine(x, 30, 250) ;

		float hz = (x - 50) * 156.25 / 1000;
		char buffer[50];
		if (hz == (int) hz) {
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_SetFont(&Font12);
			sprintf(buffer, "%.0f", hz);
			BSP_LCD_DisplayStringAt(x - 5, 285, (uint8_t*) buffer, LEFT_MODE);
//	  } else {
//		  BSP_LCD_SetFont(&Font8) ;
//		  sprintf(buffer, "%.1f", hz);
//		  BSP_LCD_DisplayStringAt(x-5, 285, (uint8_t *)buffer, LEFT_MODE) ;
		}
	}

	BSP_LCD_DrawHLine(50, 280, 180);
	HAL_Delay(5);

	for (int y = 280; y > 30; y -= 25) {
		BSP_LCD_DrawPixel(49, y, LCD_COLOR_BLACK);
//	  BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY) ;
//	  BSP_LCD_DrawHLine(50, y, 180) ;
	}

	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawVLine(50, 30, 250);

//	int N = 128;

	// Calculate Twiddle Factors
	double complex twiddleFactors[N];
	calculateTwiddleFactors(twiddleFactors);

	// Reguit Lyn
//  int arr[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128} ;
//  graph(arr, 129) ;

	// Sample obtained from ideal 10kHz signal
	double complex X_10khz[] = { 2048, 3496, 4096, 3496, 2048, 600, 0, 600,
			2048, 3496, 4096, 3496, 2048, 600, 0, 600, 2048, 3496, 4096, 3496,
			2048, 600, 0, 600, 2048, 3496, 4096, 3496, 2048, 600, 0, 600, 2048,
			3496, 4096, 3496, 2048, 600, 0, 600, 2048, 3496, 4096, 3496, 2048,
			600, 0, 600, 2048, 3496, 4096, 3496, 2048, 600, 0, 600, 2048, 3496,
			4096, 3496, 2048, 600, 0, 600, 2048, 3496, 4096, 3496, 2048, 600, 0,
			600, 2048, 3496, 4096, 3496, 2048, 600, 0, 600, 2048, 3496, 4096,
			3496, 2048, 600, 0, 600, 2048, 3496, 4096, 3496, 2048, 600, 0, 600,
			2048, 3496, 4096, 3496, 2048, 600, 0, 600, 2048, 3496, 4096, 3496,
			2048, 600, 0, 600, 2048, 3496, 4096, 3496, 2048, 600, 0, 600, 2048,
			3496, 4096, 3496, 2048, 600, 0, 600 };
	//  double complex X[128] ;

	////////////////////////////////////////
//  int n = 8;
//  kiss_fft_cpx cin[n];
//  kiss_fft_cpx cout[n];
//
//  float samples[] = {0.3535, 0.3535, 0.6464, 1.0607, 0.3535, -1.0607, -1.3535, -0.3535} ;
//  // initialize input sequence
//  for(int i=0; i<n; i++) {
//	  cin[i].r = samples[i] ;
//	  cin[i].i = 0.0 ;
//  }
//
//  kiss_fft_cfg cfg = kiss_fft_alloc(n, 0, NULL, NULL );
//  kiss_fft(cfg, cin, cout);
//  free(cfg);
//
////  fft(x_real1, n, X) ;
//  graph(cout, n) ;
	/////////////////////////////////////////

	kiss_fft_cpx cin[N];
	kiss_fft_cpx cout[N];

	float samples_20khz[] = { 2048, 4096, 2048, 0, 2048, 4096, 2048, 0, 2048,
			4096, 2048, 0, 2048, 4096, 2048, 0, 2048, 4096, 2048, 0, 2048, 4096,
			2048, 0, 2048, 4096, 2048, 0, 2048, 4096, 2048, 0, 2048, 4096, 2048,
			0, 2048, 4096, 2048, 0, 2048, 4096, 2048, 0, 2048, 4096, 2048, 0,
			2048, 4096, 2048, 0, 2048, 4096, 2048, 0, 2048, 4096, 2048, 0, 2048,
			4096, 2048, 0, 2048, 4096, 2048, 0, 2048, 4096, 2048, 0, 2048, 4096,
			2048, 0, 2048, 4096, 2048, 0, 2048, 4096, 2048, 0, 2048, 4096, 2048,
			0, 2048, 4096, 2048, 0, 2048, 4096, 2048, 0, 2048, 4096, 2048, 0,
			2048, 4096, 2048, 0, 2048, 4096, 2048, 0, 2048, 4096, 2048, 0, 2048,
			4096, 2048, 0, 2048, 4096, 2048, 0, 2048, 4096, 2048, 0, 2048, 4096,
			2048, 0 };
	float samples_15khz[] = { 2048, 3940, 3496, 1264, 0, 1264, 3496, 3940, 2048,
			156, 600, 2832, 4096, 2832, 600, 156, 2048, 3940, 3496, 1264, 0,
			1264, 3496, 3940, 2048, 156, 600, 2832, 4096, 2832, 600, 156, 2048,
			3940, 3496, 1264, 0, 1264, 3496, 3940, 2048, 156, 600, 2832, 4096,
			2832, 600, 156, 2048, 3940, 3496, 1264, 0, 1264, 3496, 3940, 2048,
			156, 600, 2832, 4096, 2832, 600, 156, 2048, 3940, 3496, 1264, 0,
			1264, 3496, 3940, 2048, 156, 600, 2832, 4096, 2832, 600, 156, 2048,
			3940, 3496, 1264, 0, 1264, 3496, 3940, 2048, 156, 600, 2832, 4096,
			2832, 600, 156, 2048, 3940, 3496, 1264, 0, 1264, 3496, 3940, 2048,
			156, 600, 2832, 4096, 2832, 600, 156, 2048, 3940, 3496, 1264, 0,
			1264, 3496, 3940, 2048, 156, 600, 2832, 4096, 2832, 600, 156 };
	float samples_10khz[] = { 2048, 3496, 4096, 3496, 2048, 600, 0, 600, 2048,
			3496, 4096, 3496, 2048, 600, 0, 600, 2048, 3496, 4096, 3496, 2048,
			600, 0, 600, 2048, 3496, 4096, 3496, 2048, 600, 0, 600, 2048, 3496,
			4096, 3496, 2048, 600, 0, 600, 2048, 3496, 4096, 3496, 2048, 600, 0,
			600, 2048, 3496, 4096, 3496, 2048, 600, 0, 600, 2048, 3496, 4096,
			3496, 2048, 600, 0, 600, 2048, 3496, 4096, 3496, 2048, 600, 0, 600,
			2048, 3496, 4096, 3496, 2048, 600, 0, 600, 2048, 3496, 4096, 3496,
			2048, 600, 0, 600, 2048, 3496, 4096, 3496, 2048, 600, 0, 600, 2048,
			3496, 4096, 3496, 2048, 600, 0, 600, 2048, 3496, 4096, 3496, 2048,
			600, 0, 600, 2048, 3496, 4096, 3496, 2048, 600, 0, 600, 2048, 3496,
			4096, 3496, 2048, 600, 0, 600 };
	// initialize input sequence
	for (int i = 0; i < N; i++) {
		cin[i].r = samples_15khz[i];
		cin[i].i = 0.0;
	}

	kiss_fft_cfg cfg = kiss_fft_alloc(N, 0, NULL, NULL);
	kiss_fft(cfg, cin, cout);
	free(cfg);

	graph(cout, N / 2);

//  int dX[128] ;
//  for (int i=0; i<128; i++) {
//	  dX[i] = creal(X[i]) ;
//  }

//  graph(dX, 128) ;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		getSamples(samples);

		for (int i = 0; i < N; i++) {
			cin[i].r = samples[i];
			cin[i].i = 0.0;
		}

		kiss_fft_cfg cfg = kiss_fft_alloc(N, 0, NULL, NULL);
		kiss_fft(cfg, cin, cout);
		free(cfg);

		graph(cout, N);
		HAL_Delay(200);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_FillRect(51, 30, 128, 250);
//		BSP_LCD_Clear(LCD_COLOR_WHITE) ;
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
//	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin) ;
//	HAL_Delay(400) ;
//	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin) ;
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief DMA2D Initialization Function
 * @param None
 * @retval None
 */
static void MX_DMA2D_Init(void) {

	/* USER CODE BEGIN DMA2D_Init 0 */

	/* USER CODE END DMA2D_Init 0 */

	/* USER CODE BEGIN DMA2D_Init 1 */

	/* USER CODE END DMA2D_Init 1 */
	hdma2d.Instance = DMA2D;
	hdma2d.Init.Mode = DMA2D_M2M;
	hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
	hdma2d.Init.OutputOffset = 0;
	hdma2d.LayerCfg[1].InputOffset = 0;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
	hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[1].InputAlpha = 0;
	if (HAL_DMA2D_Init(&hdma2d) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DMA2D_Init 2 */

	/* USER CODE END DMA2D_Init 2 */

}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.ClockSpeed = 100000;
	hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}

/**
 * @brief LTDC Initialization Function
 * @param None
 * @retval None
 */
static void MX_LTDC_Init(void) {

	/* USER CODE BEGIN LTDC_Init 0 */

	/* USER CODE END LTDC_Init 0 */

	LTDC_LayerCfgTypeDef pLayerCfg = { 0 };

	/* USER CODE BEGIN LTDC_Init 1 */

	/* USER CODE END LTDC_Init 1 */
	hltdc.Instance = LTDC;
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
	hltdc.Init.HorizontalSync = 9;
	hltdc.Init.VerticalSync = 1;
	hltdc.Init.AccumulatedHBP = 29;
	hltdc.Init.AccumulatedVBP = 3;
	hltdc.Init.AccumulatedActiveW = 269;
	hltdc.Init.AccumulatedActiveH = 323;
	hltdc.Init.TotalWidth = 279;
	hltdc.Init.TotalHeigh = 327;
	hltdc.Init.Backcolor.Blue = 0;
	hltdc.Init.Backcolor.Green = 0;
	hltdc.Init.Backcolor.Red = 0;
	if (HAL_LTDC_Init(&hltdc) != HAL_OK) {
		Error_Handler();
	}
	pLayerCfg.WindowX0 = 0;
	pLayerCfg.WindowX1 = 240;
	pLayerCfg.WindowY0 = 0;
	pLayerCfg.WindowY1 = 320;
	pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
	pLayerCfg.Alpha = 255;
	pLayerCfg.Alpha0 = 0;
	pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
	pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
	pLayerCfg.FBStartAdress = 0xD0000000;
	pLayerCfg.ImageWidth = 240;
	pLayerCfg.ImageHeight = 320;
	pLayerCfg.Backcolor.Blue = 0;
	pLayerCfg.Backcolor.Green = 0;
	pLayerCfg.Backcolor.Red = 0;
	if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN LTDC_Init 2 */

	/* USER CODE END LTDC_Init 2 */

}

/**
 * @brief SPI5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI5_Init(void) {

	/* USER CODE BEGIN SPI5_Init 0 */

	/* USER CODE END SPI5_Init 0 */

	/* USER CODE BEGIN SPI5_Init 1 */

	/* USER CODE END SPI5_Init 1 */
	/* SPI5 parameter configuration*/
	hspi5.Instance = SPI5;
	hspi5.Init.Mode = SPI_MODE_MASTER;
	hspi5.Init.Direction = SPI_DIRECTION_2LINES;
	hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi5.Init.NSS = SPI_NSS_SOFT;
	hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi5.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi5) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI5_Init 2 */

	/* USER CODE END SPI5_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 0;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 1125;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */

	/* USER CODE END TIM14_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void) {

	/* USER CODE BEGIN FMC_Init 0 */

	/* USER CODE END FMC_Init 0 */

	FMC_SDRAM_TimingTypeDef SdramTiming = { 0 };

	/* USER CODE BEGIN FMC_Init 1 */

	/* USER CODE END FMC_Init 1 */

	/** Perform the SDRAM1 memory initialization sequence
	 */
	hsdram1.Instance = FMC_SDRAM_DEVICE;
	/* hsdram1.Init */
	hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
	hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
	hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
	hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
	hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
	hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
	hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
	hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
	hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
	/* SdramTiming */
	SdramTiming.LoadToActiveDelay = 2;
	SdramTiming.ExitSelfRefreshDelay = 7;
	SdramTiming.SelfRefreshTime = 4;
	SdramTiming.RowCycleDelay = 7;
	SdramTiming.WriteRecoveryTime = 3;
	SdramTiming.RPDelay = 2;
	SdramTiming.RCDDelay = 2;

	if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN FMC_Init 2 */

	/* USER CODE END FMC_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin | CSX_Pin | OTG_FS_PSO_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, RDX_Pin | WRX_DCX_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, LD3_Pin | LD4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
	GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin | CSX_Pin | OTG_FS_PSO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : B1_Pin MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
	GPIO_InitStruct.Pin = B1_Pin | MEMS_INT1_Pin | MEMS_INT2_Pin | TP_INT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ACP_RST_Pin */
	GPIO_InitStruct.Pin = ACP_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OC_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_HS_ID_Pin OTG_HS_DM_Pin OTG_HS_DP_Pin */
	GPIO_InitStruct.Pin = OTG_HS_ID_Pin | OTG_HS_DM_Pin | OTG_HS_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : VBUS_HS_Pin */
	GPIO_InitStruct.Pin = VBUS_HS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VBUS_HS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : TE_Pin */
	GPIO_InitStruct.Pin = TE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
	GPIO_InitStruct.Pin = RDX_Pin | WRX_DCX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : LD3_Pin LD4_Pin */
	GPIO_InitStruct.Pin = LD3_Pin | LD4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void getSamples(float samples[]) {
	while (count < N) {
	}

	count = 0;
	for (int i = 0; i < N; i++) {
		samples[i] = (adc_buf[i] / 2048.0 - 1) * hamming[i];
	}
}

//void setSamples(float samples[]) {
//	for (int i = 0; i < N; i++) {
//		dac_buf[i] = samples[i] * 2048 + 2048;
//	}
//}

float hammingFunc(int n, int m) {  // n-> index; m-> length of window/samples
	return (0.54 - 0.46 * cos(2.0 * M_PI * (float) n / (float) (m - 1)));
}

void generateHamming() {  //float hamming[]) {
	for (int i = 0; i < N; i++) {
		hamming[i] = hammingFunc(i, N);
	}
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */
	if (htim == &htim14) {
		if (count < N) {
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 10);
			adc_buf[count] = HAL_ADC_GetValue(&hadc1);
			count++;
		}
	}
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
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
