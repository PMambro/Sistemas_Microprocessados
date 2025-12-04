/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "funcoes_SPI_display.h"
#include <stdint.h>

/* Private define ------------------------------------------------------------*/

/* Cronômetro: conta décimos de segundo usando SysTick */
#define DT_CRONO   100u   // 100 ms → 0,1 s
#define DT_DEB_LOW  30u   // debouncing borda de descida
#define DT_DEB_HIGH 30u   // debouncing borda de subida
#define DT_VARRE     5u   // tempo entre varreduras do display (ms)

#define NDGDSP       4    // 4 dígitos no display

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* Cronômetro: Crono[0]=décimos, [1]=seg, [2]=dezenas de seg, [3]=min */
volatile int8_t  Crono[NDGDSP] = {0,0,0,0};

/* Vetor que realmente vai para o display */
int8_t           DspHex[NDGDSP] = {8,8,8,8};

/* Máscara de pontos decimais (ligar o que quiser, aqui: min e dez seg) */
uint8_t          ptDec = 0b1010;

/* Flag: cronômetro está rodando (sempre 1 neste exemplo) */
volatile uint8_t fRun = 1;

/* Flag: chegou no limite (se quiser usar depois) */
volatile uint8_t fFim = 0;

/* Flag: 0 = mostrar 8888 (estado inicial / pós-RESET)
         1 = mostrar o conteúdo de Crono[] no display              */
volatile uint8_t fShowCrono = 0;

/* Estados para debouncing */
typedef enum { DB_NORMAL = 0, DB_FALL, DB_LOW, DB_RISING } DBState;

/* Debouncing para A1 e RESET (PA3) */
DBState sttBTA1   = DB_NORMAL;
DBState sttBTRESET = DB_NORMAL;

/* Variáveis de tempo (base HAL_GetTick) */
uint32_t tNow       = 0;
uint32_t tIN_A1     = 0;
uint32_t tIN_RESET  = 0;
uint32_t tIN_varre  = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* MAIN ----------------------------------------------------------------------*/
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

  /* Estado inicial:
   * - Crono zerado
   * - Display mostra 8888
   * - Cronômetro já rodando (fRun=1), mas ainda oculto (fShowCrono=0)
   */
  for (int i = 0; i < NDGDSP; i++) {
    Crono[i]  = 0;
    DspHex[i] = 8;
  }
  fRun       = 1;
  fFim       = 0;
  fShowCrono = 0;
  ptDec      = 0b1010;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN 3 */
  while (1)
  {
    tNow = HAL_GetTick();

    /* ---------------- tarefa #1: botão A1 (PA1) -> liberar display --------- */
    switch (sttBTA1)
    {
      case DB_NORMAL:
        /* botão pressionado (nível 0) */
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) {
          tIN_A1   = tNow;
          /* ação no momento da borda de descida:
             libera exibição do cronômetro no display              */
          fShowCrono = 1;
          sttBTA1 = DB_FALL;
        }
        break;

      case DB_FALL:
        if ((tNow - tIN_A1) > DT_DEB_LOW) {
          sttBTA1 = DB_LOW;
        }
        break;

      case DB_LOW:
        /* esperar botão soltar (subida do sinal) */
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) {
          tIN_A1  = tNow;
          sttBTA1 = DB_RISING;
        }
        break;

      case DB_RISING:
        if ((tNow - tIN_A1) > DT_DEB_HIGH) {
          sttBTA1 = DB_NORMAL;
        }
        break;
    }

    /* --------- tarefa #2: botão RESET (PA3) -> voltar pra 8888 e zerar ----- */
    switch (sttBTRESET)
    {
      case DB_NORMAL:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
          tIN_RESET = tNow;

          /* AÇÃO DO RESET:
             - zerar Crono[]
             - voltar a mostrar 8888
             - continuar contando a partir de 0
             - ocultar novamente o cronômetro (fShowCrono = 0)                  */
          Crono[0] = Crono[1] = Crono[2] = Crono[3] = 0;
          DspHex[0] = DspHex[1] = DspHex[2] = DspHex[3] = 8;
          fRun       = 1;
          fFim       = 0;
          fShowCrono = 0;

          sttBTRESET = DB_FALL;
        }
        break;

      case DB_FALL:
        if ((tNow - tIN_RESET) > DT_DEB_LOW) {
          sttBTRESET = DB_LOW;
        }
        break;

      case DB_LOW:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
          tIN_RESET  = tNow;
          sttBTRESET = DB_RISING;
        }
        break;

      case DB_RISING:
        if ((tNow - tIN_RESET) > DT_DEB_HIGH) {
          sttBTRESET = DB_NORMAL;
        }
        break;
    }

    /* ------ tarefa #3: atualizar vetor do display periodicamente ----------- */
    if ((tNow - tIN_varre) >= DT_VARRE) {
      tIN_varre = tNow;

      if (fShowCrono == 0) {
        /* Mostrar 8888 (ligou ou acabou de dar RESET) */
        DspHex[0] = 8;
        DspHex[1] = 8;
        DspHex[2] = 8;
        DspHex[3] = 8;
      } else {
        /* Mostrar o conteúdo real do cronômetro */
        for (int i = 0; i < NDGDSP; i++) {
          DspHex[i] = (uint8_t)Crono[i];
        }
      }

      /* Envia DspHex para o display (função da prática 05/06) */
      mostrar_no_display(DspHex, ptDec);
    }

    /* Por enquanto, não há mais tarefas (A2, LEDs, buzzer, etc. estão desligados) */
  }
  /* USER CODE END 3 */
}

/* System Clock Configuration ------------------------------------------------*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* GPIO Initialization Function ----------------------------------------------*/
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB12 PB13 PB14
                           PB15 PB5 PB6 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}


/* USER CODE BEGIN 4 */

/* Callback do SysTick: cronômetro sempre contando se fRun == 1
   (baseado na prática 06, modo "UP" – cronômetro crescente). */
void HAL_SYSTICK_Callback(void)
{
  static uint16_t contaCRN = 0;

  if (fRun == 0) return;

  if (contaCRN >= DT_CRONO) {
    contaCRN = 0;
    fFim     = 0;

    /* Cronômetro crescente: 0.00.0 → 9.59.9 (por exemplo) */
    ++Crono[0];                 // décimos
    if (Crono[0] > 9) {
      Crono[0] = 0;
      ++Crono[1];               // segundos
      if (Crono[1] > 9) {
        Crono[1] = 0;
        ++Crono[2];             // dezenas de segundos
        if (Crono[2] > 5) {
          Crono[2] = 0;
          ++Crono[3];           // minutos
          if (Crono[3] > 9) {
            // Se quiser, pode travar aqui e setar fFim = 1;
            Crono[3] = 9;
            Crono[2] = 5;
            Crono[1] = 9;
            Crono[0] = 9;
            fFim     = 1;
          }
        }
      }
    }
  } else {
    ++contaCRN;
  }
}

/* A implementação de mostrar_no_display() está no arquivo funcoes_SPI_display.c.
   Aqui só declaramos o protótipo (feito lá em cima em PFP). */

/* USER CODE END 4 */

void Error_Handler(void)
{
  /* Opcional: você pode colocar um breakpoint aqui para ver se entrou no erro */
  __disable_irq();
  while (1)
  {
    // Trava aqui se der erro de clock ou algo crítico
  }
}
