/*
//Title: UWB tag driver (IoT Lab.)
//Author: Chen Chun-Lin
//Data: 2017/05/02
//Update: 2017/05/05
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
#include <sensor_msgs/Imu.h>
#include <bitcraze_lps_estimator/UwbRange.h>
using namespace std;

//ROS Parameter
#define MAIN_FREQ           5000.0      //Controller main frequency
#define TF_REV_DELAY_T      150         //TF of MAP_ODOM frame receive start delay units * ts
#define DEBUG_FLAG          1
#define ANCHOR_NUM          6
#define TAG_ID              8

#define REPLY_SIZE 1
#define TIMEOUT 10
#define TX_ONE_BYTE_DELAY 0.5
#define TX_LEN 3
#define RX_LEN 22

#define PI           3.14159
#define RTD          180.0/PI           //Radian to Degree
#define DTR          PI/180.0           //Degree to Radian

#define GETSTR_PORT "/UWBTag_SerialPort"
#define GETSTR_RATE "/UWBTag_BaudRate"

//------------------------------------------
double main_freq_=MAIN_FREQ;
double tf_rev_delay_t_=TF_REV_DELAY_T;
int debug_flag_=DEBUG_FLAG;
int tag_id=TAG_ID;

bool use_fit_flag_=true;
double fit_coffa_=0.9475;
double fit_coffb_=0.4762;

typedef struct
{
    double roll;
    double pitch;
    double yaw;
} Pose;

//----------------------------------------------------------
//Publish the UWB Range info. over ROS
ros::Time current_time, last_time;

std::string serial_port;
int baud_rate=9600;
char reply[REPLY_SIZE];
unsigned char rx_buff[RX_LEN];
int rx_count=0;
char rx_timeout_flag=0;
int rx_timeout_cnt=0;

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

    ros::Publisher uwb_pub = n.advertise<bitcraze_lps_estimator::UwbRange>("uwbrange", 100);

    bitcraze_lps_estimator::UwbRange uwb_msg;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    int rx_count=0;
    char rx_flag=0;
    int rx_id=0;
    double ranges[ANCHOR_NUM];
    unsigned char rx_buff_num[6];
    char rx_sign=0;
    double rx_num=0;
    double rx_exp=0.001;

    int fd;
    int rd;
    //int wr;
    char buff;
    struct termios options;

    serial_port="/dev/ttyACM0";
    baud_rate=9600;

    double distance_err[ANCHOR_NUM];
    std::vector<double> anchor_pos[ANCHOR_NUM];

    //==================================================
    //load ROS Parameter
    n.getParam("main_freg", main_freq_);
    n.getParam("tf_rev_delay_t", tf_rev_delay_t_);
    n.getParam("debug_flag", debug_flag_);
    n.getParam(GETSTR_PORT, serial_port);
    n.getParam(GETSTR_RATE, baud_rate);
    n.getParam("anchor0_pos", anchor_pos[0]);
    n.getParam("anchor1_pos", anchor_pos[1]);
    n.getParam("anchor2_pos", anchor_pos[2]);
    n.getParam("anchor3_pos", anchor_pos[3]);
    n.getParam("anchor4_pos", anchor_pos[4]);
    n.getParam("anchor5_pos", anchor_pos[5]);
    n.getParam("distance0_err", distance_err[0]);
    n.getParam("distance1_err", distance_err[1]);
    n.getParam("distance2_err", distance_err[2]);
    n.getParam("distance3_err", distance_err[3]);
    n.getParam("distance4_err", distance_err[4]);
    n.getParam("distance5_err", distance_err[5]);
    n.getParam("use_fit_flag", use_fit_flag_);
    n.getParam("fit_coffa", fit_coffa_);
    n.getParam("fit_coffb", fit_coffb_);
    /*
    try{ device.open(serial_port.c_str(), baud_rate); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port!!!");
        ROS_BREAK();
    }
    ROS_INFO("The serial port is opened.");
    */

    fd=open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); /* open the port */
    if (fd == -1 )
    {
        perror("open_port: Unable to open serial port\n");
        ROS_FATAL("UWB: Failed to open the serial port!!!");
        ROS_BREAK();
    }
    else
    {
        fcntl(fd, F_SETFL,0);
        tcgetattr(fd, &options); /* get the current options */
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

                    //printf("1=%d, 2=%d, 3=%d, 4=%d, 5=%d, 6=%d\n", rx_buff_num[0], rx_buff_num[1], rx_buff_num[2] ,rx_buff_num[3], rx_buff_num[4], rx_buff_num[5]);

                    rx_num=0.0;
                    rx_exp=0.001;
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

                        if (use_fit_flag_==1)
                            ranges[rx_id-1]=fit_coffa_*ranges[rx_id-1]+fit_coffb_;

                        printf("ID%d=%f, use_fit=%s, A=%f, B=%f\n", rx_id, ranges[rx_id-1], use_fit_flag_ ? "true" : "false", fit_coffa_, fit_coffb_);
                        current_time = ros::Time::now();
                        uwb_msg.header.stamp = current_time;
                        uwb_msg.header.frame_id = "range";
                        uwb_msg.requester_id=tag_id;
                        uwb_msg.responder_id=rx_id-1;
                        uwb_msg.distance=ranges[rx_id-1];
                        uwb_msg.distance_err=distance_err[rx_id-1];
                        uwb_msg.antenna=0;
                        uwb_msg.responder_location.x=anchor_pos[rx_id-1][0];
                        uwb_msg.responder_location.y=anchor_pos[rx_id-1][1];
                        uwb_msg.responder_location.z=anchor_pos[rx_id-1][2];
                        uwb_pub.publish(uwb_msg);
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
