std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Point r_tag
int16 direction
float64 start_time  # 此为开始时间(切换模式时相机打开的时间,publishing image的时间),最小二乘拟合是以该时间戳为基准的。程序开始运行的时间点信息
float64 b_a
float64 b_omega
float64 b_t0
float64 b_b
float64 b_c


