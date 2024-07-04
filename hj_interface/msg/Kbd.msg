uint8 type
int32 left_msg
int32 right_msg

uint8 turn_motor_status     #!< 0 刹车； 1 运动； 2 滑行
int16 turn_motor_l      #!< 水面前进控制 喷口转向电机
int16 turn_motor_r      #!< 水面前进控制

uint8 pump_status       #!< 0 刹车； 1 运行；水泵控制
uint8 pump_speed        #!< 占空比  0到100

uint8 airbag_status     #!< 0 全放气； 1 全充气； 2 左充气； 3 右充气
uint16 airbag_time      #!< 充/放气时间  单位10ms

uint8 fan_status        #!< 0 刹车； 1 运行； 自清洁电机,目前没有转速

int16 flip_cover_angle     #!< 翻盖电机  位置  单位0.1度 上吸口开门
