Basic task/stage progression follows the `ITAASQE` structure of:
```yaml
1: I nit
2: T ask_buffer
3: A ctive_task
4: A ctive_stage
5: S tart
6: Q uery
7: E nd
```

### This is from the perspective of the stage

From the perspective of the stage, it moves through these five stages.

Every stage (unless interrupted) will make their way through these 7 processes.


```yaml
  Method | (Location)     | [Trigger]
---------|----------------|-----------
         |                | [new task required]
         | (Unassigned)   |
1:  Init |                |
         | (Task_buffer)  |
         |                | [task @ task_buffer head]
         | (Active_task)  |
         |                | [stage @ stage_list head]
         | (Active_stage) |
2: Start |                |
3: Query |                |
4:   End |                |
         | (Deleted)      |
```
















