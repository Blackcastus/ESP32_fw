# FreeRTOS
   
## xTaskCreate() 
    cho FreeROTOS tự chọn core để thực thi nhiệm vụ. 
    Linh hoạt chuyển đổi task giữa các core.
    làm tăng độ trễ thực thi.
## xTaskCreatePinnedToCore() 
    Task sẽ được thực thi cố định trên core chỉ định
    Giúp giảm độ trễ thực thi do không phải chuyển đổi giữa các core
## vTaskDelete(myTaskHandle)
    Delete task myTaskHandle
## vTaskSuspend(myTaskHandle)
    Pause task myTaskHandle
## vTaskResume(myTaskHandle)
    Continue task myTaskHandle