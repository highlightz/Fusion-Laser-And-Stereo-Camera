// By Xiaojia Xie
// Aug. 26, 2015
// Function: sends controls to MCU to control the car
// Decription:
// Lengh of instruction: 10 bytes
// Frame header: fffe
// Control segment: two float number, one for velocity, and the other for direction.
// Range: [-1, 1]

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
