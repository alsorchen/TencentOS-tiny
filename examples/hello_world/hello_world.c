#include "cmsis_os.h"
#include "mcu_init.h"

#define TASK1_STK_SIZE          1024
void task1(void *arg);
osThreadDef(task1, osPriorityNormal, 1, TASK1_STK_SIZE);

#define TASK2_STK_SIZE          1024
void task2(void *arg);
osThreadDef(task2, osPriorityNormal, 1, TASK2_STK_SIZE);

#if TOS_CFG_TASK_DYNAMIC_CREATE_EN > 0u
#define TASK3_STK_SIZE          512
void task3(void *arg);
#endif

void task1(void *arg)
{
	int j = 0;
#if TOS_CFG_TASK_DYNAMIC_CREATE_EN > 0u
    osThreadId task_dyn_created;

    osThreadDynamicDef(task3, osPriorityNormal, 1, TASK3_STK_SIZE);
    task_dyn_created = osThreadCreate(osThread(task3), NULL);

    int count = 0;
#endif

    while (1) {
        printf("###I am task1,%3d\r\n",j++);
		SetBeeper(1);
        osDelay(100);
		SetBeeper(0);
        osDelay(3900);

#if TOS_CFG_TASK_DYNAMIC_CREATE_EN > 0u
        if (count++ == 3) {
            printf("###I am task1, kill the task3(dynamic created)\r\n");
            osThreadTerminate(task_dyn_created);
        }
#endif
    }
}

void task2(void *arg)
{
	uint8_t i;
	int j=0;
	Start4Phase();
    while (1) {
#if TOS_CFG_TASK_STACK_DRAUGHT_DEPTH_DETACT_EN > 0u
        k_err_t rc;
        int depth;

        rc = tos_task_stack_draught_depth(K_NULL, &depth);
        printf("%d  %d\n", rc, depth);
#endif

        printf("***I am task2,%3d\r\n",j);
		for(i=0;i<8;i++)
		{
			osDelay(250);
			ToggleLed();
		}
//		if(j<10)
		FeedIWDG();
		j++;
    }
}

#if TOS_CFG_TASK_DYNAMIC_CREATE_EN > 0u
void task3(void *arg)
{
    while (1) {
        printf("$$$I am task3(dynamic created)\r\n");
        osDelay(2000);
    }
}
#endif

void application_entry(void *arg)
{
    printf("***I am task\r\n");
    osThreadCreate(osThread(task1), NULL); // Create task1
    osThreadCreate(osThread(task2), NULL); // Create task2
}

