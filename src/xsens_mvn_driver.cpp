#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "xsens_mvn_driver.h"

#define SERVER_PORT 9763 //udp ports, should be matched to window's port
#define BUFF_LEN 1024 // buffe zone, must be large
#define PI 3.141592 

// header: defined in the .h file
// buf: buff's first point
// time_stamp_sec: tf time msg need
// time_stamp_nsec: tf time msg need
void parse_header(Header *header, char *buf, int *time_stamp_sec, int *time_stamp_nsec)
{
    int hour = 0, min = 0,sec = 0,nanosec = 0;
    float a = 0.0;
    int to_stamp_sec = 0;
    float nano_1000;
    float temp;
    float float_time_code;
    double b = 0.0;
    memcpy(header->ID_String, buf, 6);
    memcpy(&(header->sample_counter), buf+6, 4);
    header->sample_counter = ntohl(header->sample_counter);
    header->datagram_counter = buf[11];
    header->number_of_items = buf[12];
    memcpy(&(header->time_code), buf+12, 4);
    header->time_code = ntohl(header->time_code);
    float_time_code = (float)header->time_code;
    //decoding time_code;
    if (header->time_code <= 99999) { //if time_code < 99.999;
	float_time_code /= 1000;
	//(debug) printf("float_time_code = header->time_code/1000 = %f\n", float_time_code);
	sec = (int)float_time_code;
	//(debug) printf("sec = (int)float_time_code = %d\n", sec);
	nanosec = (int)((float_time_code - sec)*1000);
	//(debug)printf("nanosec = %d\n", nanosec);
	to_stamp_sec = sec;
		
	//**convert to time stamp**
	*time_stamp_sec = to_stamp_sec;
	*time_stamp_nsec = nanosec;
    } else if (header->time_code > 99999 && header->time_code <= 9999999) { //if 99.999 < time_code <= 99:99.999;
	float_time_code = float_time_code/100000;
	min = (int)float_time_code;
	a = (int)((float_time_code - min)*100000);
	float_time_code = (float)a;
	float_time_code /= 1000;
	sec = (int)float_time_code;
	nanosec = (int)((float_time_code - sec)*1000);
	to_stamp_sec = sec + min*100;
	//**convert to time stamp**
	*time_stamp_sec = to_stamp_sec;
	*time_stamp_nsec = nanosec;
    } else if (header->time_code > 9999999 && header->time_code <= 999999999) { //if 99:99.999 < time_code < 99:99:99.999;
	float_time_code /= 10000000;
	hour = (int)float_time_code;
	a = (int)((float_time_code - hour)*10000000);
	float_time_code = (float)a;
	float_time_code = float_time_code/100000;
	min = (int)float_time_code;
	a = (int)((float_time_code - min)*100000);
	float_time_code = (float)a;
	float_time_code /= 1000;
	sec = (int)float_time_code;
	nanosec = (int)((float_time_code - sec)*1000);
	to_stamp_sec = sec + min*100 + hour*100*100;

	//convert to time stamp
	*time_stamp_sec = to_stamp_sec;
	*time_stamp_nsec = nanosec;
    }
    header->character_ID = buf[17];
    memcpy(header->reserved_for_future_use, buf+18, 7);
    /* printf("ID_String = %s\n", header->ID_String);
    printf("sample_counter = %d\n", header->sample_counter);   //endian conversion
    printf("datagram_counter = %d\n", header->datagram_counter);  
    printf("number_of_items = %d\n", header->number_of_items);
    printf("time_code = %d\n", (int)header->time_code);
    printf("time = %d:%d:%d.%d\n", hour, min, sec, nanosec);
    printf("time stamp for rosmsg: %d.%d\n", *stamp_sec, *stamp_nsec);
    printf("character_ID = %d\n", header->character_ID); */
	
}

// if don't use this function, coordinate won't be parsed correctly
float parse_coordinates(float coordinate, int count, char *buf)
{
    // check Endianess
    //**** Intel processors use the order of "Little Endian" byte!
	// Little_Endian and Big_Endian; dif cpu has diff mode
    int num = 1;
    int Endianess;
    if(*(char *)&num == 1) {
	Endianess = 0; //Little_Endian;
    } else {
	Endianess = 1; //Big_Endian;
    }
   
    unsigned char byte_array[4];
    memcpy(&byte_array, buf+count, sizeof(coordinate));
    
    if (Endianess == 0) {
	*((unsigned char*)(&coordinate) + 3) = byte_array[0];
	*((unsigned char*)(&coordinate) + 2) = byte_array[1];
	*((unsigned char*)(&coordinate) + 1) = byte_array[2];
	*((unsigned char*)(&coordinate) + 0) = byte_array[3];
    } else if (Endianess == 1) {
	*((unsigned char*)(&coordinate) + 3) = byte_array[3];
	*((unsigned char*)(&coordinate) + 2) = byte_array[2];
	*((unsigned char*)(&coordinate) + 1) = byte_array[1];
	*((unsigned char*)(&coordinate) + 0) = byte_array[0];
    }
    return coordinate;
}

// transform coordinate, y pointing up to z pointing up
void convertFromYupToZup(float *x, float *y, float *z)
{
    float x1 = *x;
    float y1 = *y;
    float z1 = *z;
	
    *x = z1;
    *y = x1;
    *z = y1;
}

// covert rad to deg
float convertFromRadToDeg(float rad)
{
    float deg = rad*180/PI;
    return deg;
}

/*
4 bytes segment ID See 2.5.9
4 bytes x–coordinate of segment position
4 bytes y–coordinate of segment position
4 bytes z–coordinate of segment position
4 bytes q1 rotation – segment rotation quaternion component 1 (re)
4 bytes q2 rotation – segment rotation quaternion component 1 (i)
4 bytes q3 rotation – segment rotation quaternion component 1 (j)
4 bytes q4 rotation – segment rotation quaternion component 1 (k)
*/
void parse_body(char *buf, int *segment_id, float *x, float *y, float *z, float *re, float *i, float *j, float *k)
{
    int seg_id = 0;
    float x_p, y_p, z_p = 0.0;
    float q1_r, q2_r, q3_r, q4_r = 0.0;
    memcpy(&seg_id, buf, 4); // copy buff's first 4 byte to seg_id 
    seg_id = ntohl(seg_id);
    *segment_id = seg_id;
    x_p = parse_coordinates(x_p, 4, buf);
    *x = x_p;

    y_p = parse_coordinates(y_p, 8, buf);
    *y = y_p;

    z_p = parse_coordinates(z_p, 12, buf);
    *z = z_p;

    //convertFromYupToZup(&x_p, &y_p, &z_p);
    //need to convert from Y up tp Z up in type 01
	
    q1_r = parse_coordinates(q1_r, 16, buf);
    q1_r = convertFromRadToDeg(q1_r);
    *re = q1_r;
   
    q2_r = parse_coordinates(q2_r, 20, buf);
    q2_r = convertFromRadToDeg(q2_r);
    *i = q2_r;
   
    q3_r = parse_coordinates(q3_r, 24, buf);
    q3_r = convertFromRadToDeg(q3_r);
    *j = q3_r;
   
    q4_r = parse_coordinates(q4_r, 28, buf);
    q4_r = convertFromRadToDeg(q4_r);
    *k = q4_r;

    /********
    printf("Segment ID: %d\n", seg_id);
    printf("Segment Position: (%f, %f, %f)\n", x_p, y_p, z_p);
    printf("Segment Rotation Quaternion: (re:%f, i:%f, j:%f, k:%f)\n", q1_r, q2_r, q3_r, q4_r);
    ********/
}

/*
two index: 'read_bytes' for checking the index of received data, 
'cur_index' for checking the index of parsed data

*/
void handle_udp_msg(int fd, int argc, char* argv[])
{
    char *buf = (char*)malloc(1024);
    int read_bytes = 0; 
    int cur_index = 0;
    int count;
    int time_stamp_sec;
    int time_stamp_nsec;
    int segment_id;
    float x, y, z;
    float re, i, j, k;
    struct sockaddr_in client_addr; // udp protocol
    socklen_t len = sizeof(client_addr);
    Header header;
    int round = 0;

    ros::init(argc, argv, "xsens_data_publisher_tf"); //name of the node
    ros::NodeHandle n;
    //ros::Publisher data_publisher = n.advertise<sensor_msgs::JointState>("position_data", 50);
    tf::TransformBroadcaster pelvis_xsens, l5_abdomen_down_xsens, l3_abdomen_up_xsens, t12_sternum_down_xsens, t8_sternum_up_xsens, neck_xsens, head_xsens, right_shoulder_xsens, right_upper_arm_xsens, right_forearm_xsens, right_hand_xsens, left_shoulder_xsens, left_upper_arm_xsens, left_forearm_xsens, left_hand_xsens, right_upper_leg_xsens, right_lower_leg_xsens, right_foot_xsens, right_toe_xsens, left_upper_leg_xsens, left_lower_leg_xsens, left_foot_xsens, left_toe_xsens;
   
    while(ros::ok) 
    {   
	    memset(buf, 0, BUFF_LEN);
		// return how many data has received
	    count = recvfrom(fd, buf+read_bytes, BUFF_LEN, 0, (struct sockaddr*)&client_addr, &len); 
	    if(count == -1)
		{
		    printf("receieve data fail!\n");
		    return;
		}
	    read_bytes += count;
		// len of header is 24 byte, parse data after receving complete header
	    if (read_bytes >= 24) {
		parse_header(&header, buf+cur_index, &time_stamp_sec, &time_stamp_nsec);
		cur_index += 24;
		// compare if received data is "MXTP02" which is Quaternion type
		if (strncmp(header.ID_String, "MXTP02", 6) == 0) {
		    //Message type 02 - Pose data (Quaternion)
		    if (read_bytes >= 24+32*header.datagram_counter) {
			int p = 0;
			//printf("Message type 02: Pose data (Quaternion)\n"); 
			// len of header is 24 byte, parse data after receving complete header
			for (p = 0; p < header.datagram_counter; p++) {
                if (ros::isShuttingDown()){
                 ROS_INFO("ros shutdown is called");}
			    parse_body(buf+cur_index+p*32, &segment_id,  &x, &y, &z, &re, &i, &j, &k);
			    printf("re = %f, i = %f, j = %f, k = %f\n", re, i, j, k);
			    ros::Time temp;	    
			    switch (segment_id) {
			    case 1:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
				pelvis_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "pelvis_xsens"));
				break;
			    case 2:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
				l5_abdomen_down_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "l5_abdomen_down_xsens"));
				break;
			    case 3:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    l3_abdomen_up_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "l3_abdomen_up_xsens"));
				break;
			    case 4:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    t12_sternum_down_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "t12_sternum_down_xsens"));
				break;
			    case 5:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    t8_sternum_up_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "t8_sternum_up_xsens"));
				break;
			    case 6:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    neck_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "neck_xsens"));
				break;
			    case 7:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    head_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "head_xsens"));
				break;
			    case 8:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    right_shoulder_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "right_shoulder_xsens"));
				break;
			    case 9:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    right_upper_arm_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "right_upper_arm_xsens"));
				break;
			    case 10:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    right_forearm_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "right_forearm_xsens"));
				break;
			    case 11:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    right_hand_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "right_hand_xsens"));
				break;
			    case 12:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    left_shoulder_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "left_shoulder_xsens"));
				break;
			    case 13:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    left_upper_arm_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "left_upper_arm_xsens"));
				break;
			    case 14:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    left_forearm_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "left_forearm_xsens"));
				break;
			    case 15:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    left_hand_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "left_hand_xsens"));
				break;
				break;
			    case 16:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    right_upper_leg_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "right_upper_leg_xsens"));
				break;
			    case 17:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    right_lower_leg_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "right_lower_leg_xsens"));
				break;
			    case 18:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    right_foot_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "right_foot_xsens"));
				break;
			    case 19:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			    right_toe_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "right_toe_xsens"));
				break;
			    case 20:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
				left_upper_leg_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "left_upper_leg_xsens"));
				break;
			    case 21:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
				left_lower_leg_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "left_lower_leg_xsens"));
				break;
			    case 22:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
				left_foot_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "left_foot_xsens"));
				break;
			    case 23:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
				left_toe_xsens.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sensor", "left_toe_xsens"));
				break;
			    }
			}
			cur_index += 32*header.datagram_counter;
			int left = read_bytes - cur_index;
			if (left > 0)
			    memcpy(buf, buf+cur_index, left);
			read_bytes = left;
			cur_index = 0;
		    } else {
			cur_index -= 24;
		    }
		} else {
		    //TODO
		    printf("other type\n");
		}
		//break;
	    }	
        if (ros::isShuttingDown()) {
             ROS_INFO("ros shutdown is called");
             break;
        }	
	    round++;
	    printf("round = %d\n", round);
    }
    free(buf);
}


/*
  server:
  socket-->bind-->recvfrom-->sendto-->close
*/

int main(int argc, char* argv[])
{
    //ros::init(argc, argv, "xsens_data_publisher"); //name of the node
    //ros::NodeHandle n;
    //ros::Publisher data_publisher = n.advertise<sensor_msgs::JointState>("position_data", 50);
    
    int server_fd, ret;
    struct sockaddr_in ser_addr;
	
    server_fd = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET: IPV4; SOCK_DGRAM: UDP
    if(server_fd < 0)
	{
	    printf("create socket fail!\n");
	    return -1;
	}
	
    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    ser_addr.sin_port = htons(SERVER_PORT);
	
    ret = bind(server_fd, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
    if(ret < 0)
	{
	    printf("socket bind fail!\n");
	    return -1;
	}
		
    handle_udp_msg(server_fd, argc, argv);
	
    close(server_fd);

}
	
