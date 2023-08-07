# STM32_workspace
This repo contains C scripts for the STM32 module of a course project CE/CZ3004 Multi-Disciplinary Project(MDP).
These scripts leverage FreeRTOS to achieve real-time multi-tasking among tasks that collectively contribute to the system functionality of a robot car.
- [Primary C script file](MDP_STM32/Core/Src/main.c)
- [constant values and macros definition](MDP_STM32/Core/Inc/main.h)


## Task structure
By default, the STM32CubeIDE generates a task template for every predefined task:
```
void runXXXTask(void *argument) {
  for (;;) {
    osDelay(10);
  }
}
```
When a task is given chance to be run, it will run infinitely within the `for` loop scope until voluntarily release the control via `osDelay()` or
when an `interrrupt` occurs.
With this information, a reusable task structure is came out to leverage the multi-tasking feature:
```
void runXXXTask(void *arguement) {
  for (;;) {
    if (curTask != TASK_XXX) osDelay(1000);
    else {
      // eg. move until y distance task
      while (curDistance < targetYDistance) {
        // run move robot car code
        // update curDistance value as robot move

        osDelay(1); // give other task chance to run but would like to get back to this task very soon
      }

      osDelay(1000); // task is completed here since already exit the while loop scope
    }
  }
}
```
If condition `curTask != TASK_XXX` is met, implies the current task is not this runXXXTask,
therefore immediately release the control to other task via `osDelay(1000)`,
a big delay value is set (1000ms) such that the task won't be frequently scheduled again soon.

If condition `curTask != TASK_XXX` is not met, implies runXXXTask is indeed the current task to be run,
therefore run it infinitely until voluntarily release, for instance, keep the task logic within a `while` loop.

the variable `curTask` can either be updated in an
- an interrupt callback that receive instruction message from RPi, or
- another `runXXXTask` that process message received from RPi, but has more complex logic to determine what value `curTask` need to be next.

