#include "delay_us.h"

uint32_t DWT_Delay_Init(void)
{
  // DWT가 초기화 되어 있으면 그대로 통과
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }

  // 사이클 카운터 활성화
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 활성화 되었는지 확인
  return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) ? 0 : 1;
}

void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  // 주파수 기반 사이클 수 계산
  uint32_t cycles = us * (HAL_RCC_GetHCLKFreq() / 1000000);

  while ((DWT->CYCCNT - start) < cycles);
}
