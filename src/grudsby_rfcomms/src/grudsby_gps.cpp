//Reference
//https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
#include <ros/ros.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <grudsby_rfcomms/emlid_reach_gps.h>


////////////////////////////////////////////////////////////////////////////////
///Outgoing messages////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//#define LLH_MESSAGE_LENGTH 141
#define LLH_MESSAGE_LENGTH 140
//#define LLH_MESSAGE_LENGTH 500

std::vector<std::string> split_string_on_spaces(char * buf, int buf_size);
bool validate_llh_format(std::vector<std::string> llh_vector);
void format_gps_message(grudsby_rfcomms::emlid_reach_gps & gps_msg, std::vector<std::string> llh_vector);


////////////////////////////////////////////////////////////////////////////////
///Configure UART TTY///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
///?????????////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    //tty.c_cc[VTIME] = 1;    //tenths of a second  timeout
    tty.c_cc[VTIME] = 0;    //tenths of a second  timeout

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}


////////////////////////////////////////////////////////////////////////////////
///You spin me right round//////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    grudsby_rfcomms::emlid_reach_gps gps_msg;
    char *portname = "/dev/ttyACM0";
    int fd;
    int wlen;
    std::vector<std::string> strings;
    ros::init(argc, argv, "grudsby_rfcomms");
    ros::NodeHandle n;
    ros::Publisher  pub = n.advertise<grudsby_rfcomms::emlid_reach_gps>("grudsby_positioning_system",1000);
    ros::Rate loop_rate(5);

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) { printf("Error opening %s: %s\n", portname, strerror(errno)); return -1; }
    set_interface_attribs(fd, B57600); //baudrate 115200, 8 bits, no parity, 1 stop bit
    set_mincount(fd, 0);                /* set to pure timed read */

    while(ros::ok()){
      //unsigned char buf[LLH_MESSAGE_LENGTH+1];
      char buf[LLH_MESSAGE_LENGTH+1];
      int rdlen;
      rdlen = read(fd, buf, sizeof(buf) - 1);
      //if (rdlen > 0) {
      std::cout << buf << '\n';
      strings = split_string_on_spaces(buf,sizeof(buf));
      if (validate_llh_format(strings)){ 
        format_gps_message(gps_msg,strings);
        strings.clear();
        pub.publish(gps_msg);
      }
      /*
      if (rdlen >= LLH_MESSAGE_LENGTH-5) {
          buf[rdlen] = 0;
          printf("Read %d: \"%s\"\n", rdlen, buf);
      } else if (rdlen < 0) {
          printf("Error from read: %d: %s\n", rdlen, strerror(errno));
      }*/
      
      /* repeat read to get full message */
      tcflush(fd, TCIOFLUSH);
      ros::spinOnce();
      loop_rate.sleep();
    }
}



////////////////////////////////////////////////////////////////////////////////
///GPS UART MESSAGE PARSING AND VALIDATION//////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

std::vector<std::string> split_string_on_spaces(char * buf, int buf_size){
  //Split a char buf array on spaces, each sub-string added as a string vector entry
  int i=0;
  std::vector<std::string> output_string_vector;
  std::string tmp_string;

  for(i=0;i<buf_size;i++){
    if(buf[i] == ' '){
      if(tmp_string.size() > 0){
        output_string_vector.push_back(tmp_string);
      }
      tmp_string = "";
    }
    else if(buf[i] == '\0'){
      if(tmp_string.size() > 0){
        output_string_vector.push_back(tmp_string);
      }
      break;
    }
    else {
      tmp_string += buf[i];
    }
  }
  return output_string_vector;
}

bool validate_llh_format(std::vector<std::string> llh_vector){
  bool valid;
  bool numeric_fields_ok;
  bool date_ok;
  if (llh_vector.size() != 15) return false;     //validate packet size
  if (!llh_vector.at(0).find('/')) return false; //validate date format
  if (!llh_vector.at(1).find(':')) return false; //validate timestamp format
  return true;
}

void format_gps_message(grudsby_rfcomms::emlid_reach_gps & gps_msg, std::vector<std::string> llh_vector){
  gps_msg.header.stamp = ros::Time::now();
  gps_msg.gps_date = llh_vector.at(0);
  gps_msg.gps_time = llh_vector.at(1);
  gps_msg.latitude = std::atof(llh_vector.at(2).c_str());
  gps_msg.longitude = std::atof(llh_vector.at(3).c_str());
  gps_msg.height = std::atof(llh_vector.at(4).c_str());

  gps_msg.quality_flag = std::atof(llh_vector.at(5).c_str()); //int8
  gps_msg.n_satellites = std::atof(llh_vector.at(6).c_str()); //int8

  gps_msg.sdn = std::atof(llh_vector.at(7).c_str());
  gps_msg.sde = std::atof(llh_vector.at(8).c_str());
  gps_msg.sdu = std::atof(llh_vector.at(9).c_str());
  gps_msg.sdne = std::atof(llh_vector.at(10).c_str());
  gps_msg.sdeu = std::atof(llh_vector.at(11).c_str());
  gps_msg.sdun = std::atof(llh_vector.at(12).c_str());
  gps_msg.differential_age = std::atof(llh_vector.at(13).c_str());
  gps_msg.ratio = std::atof(llh_vector.at(14).c_str());
}
