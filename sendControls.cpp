/*
控制指令描述：
长度：10字节
帧头：fffe
控制字段：两个float数据，一个是速度，一个是方向。
游程：-1到1
*/

// Prepare controls
float spd =  0.3f;
float ang = -0.1f;

// Pre-define the control header	
char send[10];
send[0] = 0xff;
send[1] = 0xfe;

// Speed control
const int *p = (int *)&spd;
send[2] = (*p>>24)&0xff;
send[3] = (*p>>16)&0xff;
send[4] = (*p>>8)&0xff;
send[5] = (*p>>0)&0xff;

// Angle control
p = (int *)&ang;
send[6] = (*p>>24)&0xff;
send[7] = (*p>>16)&0xff;
send[8] = (*p>>8)&0xff;
send[9] = (*p>>0)&0xff;
