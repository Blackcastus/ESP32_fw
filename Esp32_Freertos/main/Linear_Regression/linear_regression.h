#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void Linear_Regression(void *arg);
void Linear_Regression_calc(float *a, float *b, float *arry_x, float *arry_y, int point);
