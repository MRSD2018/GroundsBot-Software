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
#define LLH_MESSAGE_LENGTH 140

std::vector<std::string> split_string_on_spaces(char * buf, int buf_size);
bool validate_llh_format(std::vector<std::string> llh_vector);
void format_gps_message(std::vector<std::string> llh_vector);

grudsby_rfcomms::emlid_reach_gps gps_msg;


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
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}


////////////////////////////////////////////////////////////////////////////////
///You spin me right round//////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//int main()
int main(int argc, char **argv)
{
    char *portname = "/dev/ttyACM1";
    int fd;
    int wlen;
    ros::init(argc, argv, "grudsby_rfcomms");
    ros::NodeHandle n;
    ros::Publisher  pub = n.advertise<grudsby_rfcomms::emlid_reach_gps>("grudsby_positioning_system",1000);
    ros::Rate loop_rate(10);

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) { printf("Error opening %s: %s\n", portname, strerror(errno)); return -1; }
    set_interface_attribs(fd, B57600); //baudrate 115200, 8 bits, no parity, 1 stop bit
    //set_mincount(fd, 0);                /* set to pure timed read */

    while(ros::ok()){
      unsigned char buf[LLH_MESSAGE_LENGTH+1];
      int rdlen;
      rdlen = read(fd, buf, sizeof(buf) - 1);
      //if (rdlen > 0) {
      if (rdlen >= LLH_MESSAGE_LENGTH-5) {
          buf[rdlen] = 0;
          printf("Read %d: \"%s\"\n", rdlen, buf);
      } else if (rdlen < 0) {
          printf("Error from read: %d: %s\n", rdlen, strerror(errno));
      }
      
      /* repeat read to get full message */
      pub.publish(gps_msg);
      ros::spinOnce();
      loop_rate.sleep();
    }
}


/*
#include <chatbot_node/reply_msg.h>
#include <message_ui/sent_msg.h>
#include <counter_node/counter.h>
#include <arithmetic_node/arithmetic_reply.h>
#include <string>
#include <std_msgs/String.h>

using namespace std;

bool new_response = false;

bool process_arithmetic_string(const std::string & str){
  //////////////////////////////////////////////////////
  ///Take a message from GUI and check if it is/////////
  ////arithmetic. Process result and construct reply////
  ////if so, do nothing otherwise///////////////////////
  //////////////////////////////////////////////////////
  std::string::size_type i = 0;
  int operand1=0;
  int operand2=0;
  float result;
  char operater;

  if(!isdigit(str[0])){
    ///Enforce first character is numeric///////////////
    std::cout << "violation: non-integer start" << "\n";
    return false;
  }

  do{
    ///Parse first operand /////////////////////////////
    if(isdigit(str[i])){
      operand1 *= 10;
      operand1 += (int) (str[i] - '0'); 
    } else{
      break;
    }
    i++;
  } while(i < str.size());

  ///Parse operator //////////////////////////////////
  operater = str[i++];

  do{
    ///Parse second operand ////////////////////////////
    if(isdigit(str[i])){
      operand2 *= 10;
      operand2 += (int) (str[i] - '0'); 
    } else{
      std::cout << "violation: end of string nonnumeric" << "\n";
      return false;
    }
    i++;
  } while(i < str.size());

  ///Compute result based on operands and operator/////
  if     (operater == '+'){
    outgoing_reply_message.oper_type = "Add"; 
    result = operand1 + operand2;
    outgoing_reply_message.answer = operand1 + operand2;
  }
  else if(operater == '-'){
    outgoing_reply_message.oper_type = "Subtract"; 
    result = operand1 - operand2;
    outgoing_reply_message.answer = operand1 - operand2;
  }
  else if(operater == '/'){
    outgoing_reply_message.oper_type = "Divide"; 
    result = operand1 / (float) operand2;
    outgoing_reply_message.answer = operand1 / (float) operand2;
  }
  else if(operater == '*'){
    outgoing_reply_message.oper_type = "Multiply"; 
    result = operand1 * operand2;
    outgoing_reply_message.answer = operand1 * operand2;
  }else{
    std::cout << "violation: invalid operator" << "\n";
    return 0;
  }
  std::cout << "operand1:" << operand1 << " operand2:" << operand2 << " operator:" << operater << " result:" << result << "\n";
  return 1;
}
 

void message_callback(const message_ui::sent_msg data)
{
  outgoing_reply_message.time_received = ros::Time::now().toSec();
  if( process_arithmetic_string(data.message) ){
    new_response = true;
  }
}

int main(int argc, char **argv) {
  std::stringstream ss;
  ros::init(argc, argv, "arithmetic_node");
  ros::NodeHandle n;
  ros::Publisher  pub = n.advertise<arithmetic_node::arithmetic_reply>("arithmetic_reply",1000);
  ros::Subscriber sub = n.subscribe("sent_msg", 1000, message_callback);
  ros::Rate loop_rate(20);

  while(ros::ok()) {
    if(new_response){
      outgoing_reply_message.header.stamp = ros::Time::now();
      outgoing_reply_message.time_answered = ros::Time::now().toSec();
      outgoing_reply_message.process_time = outgoing_reply_message.time_answered - outgoing_reply_message.time_received;
      pub.publish(outgoing_reply_message);
      new_response = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}*/



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

void format_gps_message(std::vector<std::string> llh_vector){
  std::cout << "lat:    " << std::atof(llh_vector.at(2).c_str()) << '\n'; 
  std::cout << "long:   " << std::atof(llh_vector.at(3).c_str()) << '\n'; 
  std::cout << "height: " << std::atof(llh_vector.at(4).c_str()) << '\n'; 
}


/*
int main()
{
  //char buf[100] = "1.00 -2 3/3 -4.00 5";
  char buf[]  ="2017/11/11 17:35:58.400   40.443584380  -79.945276827   258.8221   5   4   6.5600  16.2426  30.9935   8.9197 -21.5177 -12.8924   0.00    0.0";
  char buf2[] ="2017/11/11 17:35:58.400   40.443584380  -79.945276827   258.8221   5   4   6.5600  16.2426  30.9935   8.9197 -21.5177 -12.8924   0.00  ";
  char buf3[] ="2017/11/11 17:35:58.400   50.4z584380  -79.945276827   258.8221   5   4   6.5600  16.2426  30.9935   8.9197 -21.5177 -12.8924   0.00    0.0";
  char buf4[] ="17:35:58.400   40.443584380  -79.945276827   258.8221   5   4   6.5600  16.2426  30.9935   8.9197 -21.5177 -12.8924   0.00    0.0";
  char buf5[] ="2017/11/11 40.443584380  -79.945276827   258.8221   5   4   6.5600  16.2426  30.9935   8.9197 -21.5177 -12.8924   0.00    0.0";
  char buf6[]  ="2017/11/11 17:35:58.400   60.443584380  -90.945276827   300.8221   5   4   6.5600  16.2426  30.9935   8.9197 -21.5177 -12.8924   0.00    0.0";
  std::vector<std::string> strings;

  std::cout << "buf\n";
  strings = split_string_on_spaces(buf,sizeof(buf));
  if (validate_llh_format(strings)) format_gps_message(strings);
  strings.clear();

  std::cout << "buf2\n";
  strings = split_string_on_spaces(buf2,sizeof(buf2));
  if (validate_llh_format(strings)) format_gps_message(strings);
  strings.clear();

  std::cout << "buf3\n";
  strings = split_string_on_spaces(buf3,sizeof(buf3));
  if (validate_llh_format(strings)) format_gps_message(strings);
  strings.clear();

  std::cout << "buf4\n";
  strings = split_string_on_spaces(buf4,sizeof(buf4));
  if (validate_llh_format(strings)) format_gps_message(strings);
  strings.clear();

  std::cout << "buf5\n";
  strings = split_string_on_spaces(buf5,sizeof(buf5));
  if (validate_llh_format(strings)) format_gps_message(strings);
  strings.clear();

  std::cout << "buf6\n";
  strings = split_string_on_spaces(buf6,sizeof(buf6));
  if (validate_llh_format(strings)) format_gps_message(strings);
  strings.clear();


}
*/
