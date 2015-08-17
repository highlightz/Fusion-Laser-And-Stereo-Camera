/*
融合航点规划模块和视觉里程计模块的设计思路：

S1：实例化hokuyo_wrapper对象，启动激光雷达；

S2：实例化DirectionGenerator对象；

S3：实例化bb2_wrapper对象，启动双目相机；

S4：实例化libviso2_wrapper对象；

S5：进入主循环体
	S1：获取激光序列

	S1：调用DirectionGenerator对象的genWaypoint方法，确定下一个航点的坐标值

	S2：启动视觉里程计，初始化坐标系，将航点坐标值映射到里程计坐标系；

	S3：进入航点跟踪次循环体

		S1：监控里程计读数，判断是否到达航点

		S2：达到航点后，退出次循环体
*/

#include "hokuyo_wrapper.h"
#include "directiongenerator.h"

#include "bb2_wrapper.h"
#include "libviso2_wrapper.h"

int main( )
{
	return 0;
}

