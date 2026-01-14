基于stm32f103c8t6的hal库制作的两轮平衡小车
在6.文件之后如果使用STM32CUBEMX生成代码的时候记得将gpio.c中的	
HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);转移到mpu_dmp_init();
之后，原理是因为mpu6050初始化很慢，如果在mpu6050初始化完成之前将中断初始化，那么系统会崩溃导致mpu605读取不出数据

调PID参数基本思路
确定Vertical_kp的极性（比如给100是向前，小车轮就是向前，反之给负号），确定好之后调整Vertical_kp，从100开始向上增到小车可以幅度大，频率低，像不倒翁一样大幅度晃动然后保留Vertical_kp值*0.6
Vertical_kd，调到小车短时间平衡
Velocity_ki，这个时候也要确认极性，一般跟Vertical_kp极性相同，一般给0.几的值就可以了，调到推小车，小车可以来回晃荡变成原来的位置就行
Turn_kd调整不走弯就行

硬件 
stm32f103c8t6
520电机
tb6612
12V电池
MP1584EN降压模块
mpu6050陀螺仪模块
