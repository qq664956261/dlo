time timestamp
uint8 turn_motor_ctl     #!< 0 刹车； 1 运动； 2 滑行 
int16 turn_motor_l      #!< 水面前进控制 喷口转向电机 范围(-180~180度) 左侧喷口转向电机控制
int16 turn_motor_r      #!< 水面前进控制 范围(-180~180度)右侧喷口转向电机控制

uint8 pump_ctl       #!< 0 停止； 1 运行 水泵控制          
uint8 pump_speed       #!< 占空比 0到100 水泵PWM控制