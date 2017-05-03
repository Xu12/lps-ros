/*
//Title: UWB tag driver (IoT Lab.)
//Author: Chen Chun-Lin
//Data: 2017/05/02
//Update:
*/

#include <fcntl.h>
#include <termios.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
//#include <cereal_port/CerealPort.h>
#include <sensor_msgs/Imu.h>
//#include <uwbtag_link/UWBNode.h>
using namespace std;

//ROS Parameter
#define MAIN_FREQ           5000.0       //Controller main frequency
#define TF_REV_DELAY_T      150         //TF of MAP_ODOM frame receive start delay units * ts
#define USE_REV_ENCODE      1
#define DEBUG_FLAG          1
#define ANCHOR_NUM          6

#define REPLY_SIZE 1
#define TIMEOUT 10
#define TX_ONE_BYTE_DELAY 0.5
#define TX_LEN 3
#define RX_LEN 22

#define PI           3.14159
#define RTD          180.0/PI           //Radian to Degree
#define DTR          PI/180.0           //Degree to Radian
#define TS 0.1                          //unit:s

#define AXON_ROBOT_R 0.1075             //unit:m
#define AXON_ROBOT_L 0.2680             //unit:m
#define AXON_WHEEL_MINSPEED 0.0065      //(rad/s)
#define AXON_WHEEL_MAXSPEED 7.5         //(rad/s)
#define AXON_WHEEL_CMD_LIMIT 45         //AXON Command 0~127, 45:wheel velocity limit in 3.0(rad/s)

#define ONE_R_ENCODER_PLUES     25600.0                    //unit:pluse
#define RAD_PLUSE               PI/ONE_R_ENCODER_PLUES     //unit:rad/pluse

#define GETSTR_PORT "/uwbtag_parameter/SerialPort"
#define GETSTR_RATE "/uwbtag_parameter/BaudRate"

//------------------------------------------
double main_freq_=MAIN_FREQ;
double tf_rev_delay_t_=TF_REV_DELAY_T;
int use_rev_encode_=USE_REV_ENCODE;
int debug_flag_=DEBUG_FLAG;

typedef struct
{
    double roll;
    double pitch;
    double yaw;
} Pose;

//----------------------------------------------------------
ros::Subscriber axon_info_sub_;
//Publish the AXON info. over ROS
ros::Time current_time, last_time, odom_current_time, odom_last_time;

int baud_rate=9600;
std::string serial_port;
//cereal::CerealPort device;
char reply[REPLY_SIZE];
int cmd_type=0;
unsigned char rx_buff[RX_LEN];
int rx_count=0;
char rx_timeout_flag=0;
int rx_timeout_cnt=0;

double total_time=0;
int rx_stop_flag=0;
int spin_cnt=0;
int tx_cmd_send_cnt=0;

//Quaternion to All Euler angle
Pose Quat_to_AllEuler(geometry_msgs::Quaternion msg_quat)
{
    tf::Quaternion tf_quat;
    Pose pose;
    tf::quaternionMsgToTF(msg_quat, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(pose.roll, pose.pitch, pose.yaw);
    return pose;
}

void Sat_Value_uc(unsigned char *In, unsigned char Upp, unsigned char Low)
{
    unsigned char data;
    data=*In;
    if (data>Upp) data=Upp;
    if (data<Low) data=Low;
    *In=data;
}

void Sat_Value_ui(unsigned int *In, unsigned int Upp, unsigned int Low)
{
    unsigned int data;
    data=*In;
    if (data>Upp) data=Upp;
    if (data<Low) data=Low;
    *In=data;
}

void Sat_Value(double *In, double Upp, double Low)
{
    double data;
    data=*In;
    if (data>Upp) data=Upp;
    if (data<Low) data=Low;
    *In=data;
}

void Print_HEXString(char s)
{
    unsigned int num;
    unsigned int num1,num2;
    char* str;
    char str1[3];
    if (s<0)
        num=s+256;
    else
        num=s;
    num1=num/16;
    num2=num%16;

    if (num1>=10)
        num1=num1+'A'-10;
    else
        num1=num1+'0';
    if (num2>=10)
        num2=num2+'A'-10;
    else
        num2=num2+'0';

    str1[0]=num1;
    str1[1]=num2;
    str1[2]='\0';
    str=str1;
    printf("0x%s ", str);
}

int main(int argc, char** argv)
{    
    ros::init(argc, argv, "uwbtag_link_node");
    ros::NodeHandle n;

    current_time = ros::Time::now();
    last_time = ros::Time::now();
    odom_current_time = ros::Time::now();
    odom_last_time = ros::Time::now();

    int rx_count=0;
    char rx_flag=0;
    int rx_id=0;
    double ranges[ANCHOR_NUM];
    unsigned char rx_buff_num[6];
    char rx_sign=0;
    double rx_num=0;
    double rx_exp=1;

    int fd;
    int wr, rd;
    char buff;
    struct termios options;

    //==================================================
    //load ROS Parameter
    n.getParam("main_freg", main_freq_);
    n.getParam("tf_rev_delay_t", tf_rev_delay_t_);
    n.getParam("use_rev_encode", use_rev_encode_);
    n.getParam("debug_flag", debug_flag_);
    n.getParam(GETSTR_PORT, serial_port);
    ROS_INFO("I get SerialPort: %s", serial_port.c_str());
    n.getParam(GETSTR_RATE, baud_rate);
    ROS_INFO("I get BaudRate: %d", baud_rate);

    serial_port="/dev/ttyACM0";
    baud_rate=9600;

    /*
    try{ device.open(serial_port.c_str(), baud_rate); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port!!!");
        ROS_BREAK();
    }
    ROS_INFO("The serial port is opened.");
    */

    rx_stop_flag=0;

    /* open the port */
    fd=open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1 )
    {
        perror("open_port: Unable to open serial port\n");
        ROS_FATAL("Failed to open the serial port!!!");
        ROS_BREAK();
    }
    else
    {
        fcntl(fd, F_SETFL,0);
        /* get the current options */
        tcgetattr(fd, &options);

        /* set raw input, 1 second timeout */
        options.c_cflag     |= (CLOCAL | CREAD);
        options.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag     &= ~OPOST;
        options.c_cc[VMIN]  = 0;
        options.c_cc[VTIME] = 10;

        /* set the baud rates to 9600... */
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);

        /* set the options */
        tcsetattr(fd, TCSANOW, &options);
        printf("Port 1 has been sucessfully opened and %d is the file description\n", fd);
    }

    ros::Rate r(main_freq_);
    while(ros::ok())
    {
        //Main loop----------------------------------------------------------
        // Get the reply, the last value is the timeout in ms
        /*
        try{ device.read(reply, REPLY_SIZE, TIMEOUT);
        }
        catch(cereal::TimeoutException& e)
        {
            ROS_ERROR("Timeout!");
        }
        ROS_INFO("Got this reply: %s", reply);
        */

        //wr=write(fd,"TEST\r",5);
        rd=read(fd, &buff, 1);
        //printf("Bytes sent are %d %s \n",rd, &buff);
        if (rd>=1)
            reply[0]=buff;

        //Packet
        if (reply[0]=='s')
        {
            rx_count=0;
            rx_buff[rx_count]=reply[0];
            rx_count++;
            rx_flag=1;
        }
        else if (rx_flag==1)
        {
            rx_buff[rx_count]=reply[0];
            rx_count++;
            if (rx_count>15)
            {
                if ( (rx_buff[0]=='s') && (rx_buff[1]=='t') && (rx_buff[2]=='a') && (rx_buff[3]=='n') && (rx_buff[4]=='c') && (rx_buff[5]=='e') )
                {
                    if (rx_buff[6]==' ')
                        rx_id=(rx_buff[7]-0x30);
                    else
                        rx_id=(rx_buff[6]-0x30)*10+(rx_buff[7]-0x30);

                    rx_sign=0;
                    for (int i=0;i<6;i++)
                    {
                        if ( ((rx_buff[9+i]<='9') && (rx_buff[9+i]>='0')) || (rx_buff[9+i]==' ') || (rx_buff[9+i]=='-')  )
                        {
                            if (rx_buff[9+i]==' ')
                                rx_buff_num[i]=0;
                            else if (rx_buff[9+i]=='-')
                            {
                                rx_sign=1;
                                rx_buff_num[i]=0;
                            }
                            else
                                rx_buff_num[i]=rx_buff[9+i]-0x30;
                        }
                        else
                            rx_buff_num[i]=0;
                    }

                    printf("1=%d, 2=%d, 3=%d, 4=%d, 5=%d, 6=%d\n", rx_buff_num[0], rx_buff_num[1], rx_buff_num[2] ,rx_buff_num[3], rx_buff_num[4], rx_buff_num[5]);

                    rx_num=0.0;
                    rx_exp=1.0;
                    for (int i=0;i<6;i++)
                    {
                        rx_num=rx_num+(double)rx_buff_num[5-i]*rx_exp;
                        rx_exp=rx_exp*10.0;
                    }
                    if ((rx_id<=ANCHOR_NUM)&&(rx_id>=1))
                    {
                        if (rx_sign==1)
                            ranges[rx_id-1]=-rx_num;
                        else
                            ranges[rx_id-1]=rx_num;
                        printf("ID%d=%f\n", rx_id, ranges[rx_id-1]);
                    }
                }//end if ( (rx_buff[0]=='s') && ...
                rx_flag=0;
            }
        }
        //loop----------------------------------------------------------
        ros::spinOnce();
        r.sleep();
    }
}
