#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>

/* Static memory allocation for idle task */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

/* Static memory allocation for timer task */
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

/* Required by FreeRTOS when static allocation is enabled */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   configSTACK_DEPTH_TYPE *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* Required by FreeRTOS when static allocation is enabled and timers are used */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    configSTACK_DEPTH_TYPE *pulTimerTaskStackSize) {
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

/* Hook function called when malloc fails */
void vApplicationMallocFailedHook(void) {
    /* Called if a call to pvPortMalloc() fails because there is insufficient
       free memory available in the FreeRTOS heap. */
    for(;;);
}

/* Hook function called on each tick */
void vApplicationTickHook(void) {
    /* This function will be called by each tick interrupt if
       configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. */
}

/* Hook function called when idle task runs */
void vApplicationIdleHook(void) {
    /* This function will be called on each cycle of the idle task if
       configUSE_IDLE_HOOK is set to 1 in FreeRTOSConfig.h. */
}

/* Hook function called on stack overflow */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    /* This function will get called if a task overflows its stack. */
    (void) xTask;
    (void) pcTaskName;
    for(;;);
}