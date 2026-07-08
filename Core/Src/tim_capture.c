#include "tim_capture.h"
#include "main.h"
#include "rc_input.h"
#include "wheel_speed.h"

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    RC_Input_OnCapture(htim);
  }
  else if (htim->Instance == TIM2)
  {
    WheelSpeed_CaptureCallback(htim);
  }
}
