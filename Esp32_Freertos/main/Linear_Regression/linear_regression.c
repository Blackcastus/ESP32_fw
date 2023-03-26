#include "linear_regression.h"


static float pointX[5] = {14.0, 26.0, 28.0, 42.0, 47.0};
static float pointY[5] = {15.0, 25.0, 30.0, 40.0, 50.0};
static float a, b;

void Linear_Regression(void *arg)
{
    while (1)
    {
        Linear_Regression_calc(&a, &b, pointX, pointY, sizeof(pointX));
        printf("a: %d | b: %d\n",2, 3);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
}


void Linear_Regression_calc(float *a, float *b, float *arry_x, float *arry_y, int point)
{
    float sumX = 0, sumY = 0, sumX2 = 0, sumXY = 0;
    float pb;

    for(int i = 0;i < point; i++){
        // printf("x = %2.2f | y = %2.2f\n", arry_x[i], arry_y[i]);
        sumX += arry_x[i];
        sumX2 += arry_x[i] * arry_x[i];
        sumY += arry_y[i];
        sumXY += arry_x[i] * arry_y[i];
    }
    // printf("%2.2f | %2.2f | %2.2f | %2.2f\n", sumX, sumX2, sumY, sumXY);
    pb = (point * sumXY - sumX * sumY) / (point * sumX2 - sumX * sumX);
    *b = (sumY - pb * sumX) / point;
    *a = pb;
}