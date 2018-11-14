/*!
  \example get_distance.c ãóó£ÉfÅ[É^ÇéÊìæÇ∑ÇÈ
  \author Satofumi KAMIMURA

  $Id$
  */

#include "urg_sensor.h"
#include "urg_utils.h"
#include "open_urg_sensor.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#define BAUDRATE B9600
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int rotation(urg_t *urg, long data[], int data_n);
void serial_arduinowrite(int, double);

double x, y, slope;

FILE *fp;


int rotation(urg_t *urg, long data[], int data_n)
{
  int first_index, second_index, third_index, l;
  int i, j;
  double x[2], y[2], l3;
  double radian;
  double f, t, theta, phi;

  (void)data_n;

  // Å12degÇÃÉfÅ[É^ÇÃÇ›Çï\é¶
  first_index = urg_step2index(urg, -20);
  second_index = urg_step2index(urg, +20);
  third_index = urg_step2index(urg, 0);

  for(i = 0; i < 2; i++){
    if(i == 0){
      j = first_index;
      l = data[first_index];
    }
    else{
      j = third_index;
      l = data[third_index];
    }
    radian = urg_index2rad(urg, j);
    x[i] = (double)(l * cos(radian)) / 1000.0;
    y[i] = (double)(l * sin(radian)) / 1000.0;
  }

  l3 = sqrt(pow(x[0] - x[1], 2.0) + pow(y[0] - y[1], 2.0));
  f = (double)data[first_index] / 1000.0;
  t = (double)data[third_index] / 1000.0;
  theta = acos((pow(t, 2.0) + pow(l3, 2.0) - pow(f, 2.0)) / (2.0 * t * l3)) * (180.0 / M_PI);
  if(data[first_index] > data[second_index]){
    phi = 180.0 - (90.0 + (180.0 - theta));
    //printf("rotation : -%f [deg]\n", phi);
    //fprintf(fp,"rotation : -%f [deg]\n", phi);
  }
  else{
    phi = 180.0 - (90.0 + theta);
    //printf("rotation : %f [deg]\n", phi);
    //fprintf(fp,"rotation : %f [deg]\n", phi);
  }
  printf("phi : %f  ", phi);
  fprintf(fp,"phi : %f  ", phi);

  return phi;
}


void print_data(urg_t *urg, long data[], int data_n, long time_stamp)
{
  int a = 0,i,j,k,l,m,n;	//Variable of if
  int flag;	//Whether a linear approximation
  int step[1081] = {0};
  int step_max,step_min,step_s;
  int Farther_value[10] = {0},Farther_flag;
  int counter;

  double X = 8.0,Y = 4.0,sl = 0;	//Tunnel size
  double x_t[1081] = {0},y_t[1081] = {0};	//Tunnel coordinates
  double deg,rad;	//Angle from corner to corner
  double stddeg,cmpdeg;	//Reference angle, Comparison angle
  double x_s,y_s;	//sum
  double x_a,y_a;	//average
  double part_1,part_2;	//Calculating element of approximate straight line
  double a_1[1081] = {0},b_1[1081] = {0};	//Slope and Intercept of wall line
  double a_2,b_2;	//Slope and Intercept of bottom line
  double x_c,y_c,x_cc,y_cc;	//Corner coordinates
  double cldis_1,cldis_2;	//Distance to corner
  double diff_1,diff_2,difference;	//Detection of corner step number
  int step_1,step_2;	//Number of steps in the corner
  double x_0,cldis_0;	//Coordinates and distance of 180th step
  double x_t_s,y_t_s;
  double angle,angle_jdg;
  (void)time_stamp;

  long min_distance;
  long max_distance;

  // ëSÇƒÇÃÉfÅ[É^ÇÃ X-Y ÇÃà íuÇï\é¶
  urg_distance_min_max(urg, &min_distance, &max_distance);
  for (i = 0; i < data_n; ++i) {
    long l = data[i];
    double radian;

    if ((l <= min_distance) || (l >= max_distance)) {
      continue;
    }
    radian = urg_index2rad(urg, i);
    x_t[i] = (double)(l * cos(radian)) / 1000.0;
    y_t[i] = (double)(l * sin(radian)) / 1000.0;
    //printf("%d (%lf, %lf) [mm]\n", i, x_t[i], y_t[i]);
  }
  //bottom line
  i = 410;
  k = 0;
  while(1){
    step[k] = i;
    k++;
    if(fabs(y_t[i] - y_t[i + 10]) > 0.1){
      if(fabs(y_t[i + 10] - y_t[i + 20]) > 0.1){
        if(fabs(y_t[i + 20] - y_t[i + 30]) > 0.1)
          break;
      }
    }
    if(i > 870)
      break;

    i += 10;
  }

  y_t_s = 0;
  for(i = 0;i < k;i++){
    y_t_s += y_t[step[i]];
  }
  y_t_s /= (float)k;

  j = 0;
  for(i = 0;i < k;i++){
    if(fabs(y_t[step[i]] - y_t_s) > 1.0){
      Farther_value[j] = step[i];
      j++;
    }
  }

  //sum
  x_s = 0;
  y_s = 0;
  m = 0;
  for(i = 0;i < k;i++){
    Farther_flag = 0;
    for(l = 0;l < j;l++){
      if(step[i] == Farther_value[l]){
        Farther_flag = 1;
        m++;
      }
    }
    if(Farther_flag != 1){
      x_s += x_t[ step[i] ];
      y_s += y_t[ step[i] ];
    }
  }

  //average
  x_a = x_s / (float)(k - m);
  y_a = y_s / (float)(k - m);

  //Linear equations
  part_1 = 0;
  part_2 = 0;
  for(i = 0;i < k;i++){
    Farther_flag = 0;
    for(l = 0;l < j;l++){
      if(step[i] == Farther_value[l])
        Farther_flag = 1;
    }
    if(Farther_flag != 1){
      part_1 += x_t[ step[i] ] * y_t[ step[i] ];
      part_2 += pow( x_t[ step[i] ] , 2.0 );
    }
  }
  a_2 = ( part_1 - ((float)(k - m) * x_a * y_a) ) / ( part_2 - ( (float)(k - m) * pow(x_a , 2.0) ) );
  b_2 = y_a - (a_2 * x_a);

  //side line
  for(i = 189;i < 400;i++){
    j = i;
    k = 0;
    while(1){
      step[k] = j;
      k++;
      if(fabs(x_t[j] - x_t[j + 10]) > 0.08){
        if(fabs(x_t[j + 10] - x_t[j + 20]) > 0.08){
          if(fabs(x_t[j + 20] - x_t[j + 30]) > 0.08){
            break;
          }
        }
      }
      if(j > 870)
        break;
      j += 10;
    }
    if(k > 3){
      x_t_s = 0;
      for(n = 0;n < k;n++)
        x_t_s += x_t[step[n]];
      x_t_s /= (float)k;
      j = 0;
      for(n = 0;n < k;n++){
        if(fabs(x_t[step[n]] - x_t_s) > 5.0){
          Farther_value[j] = step[n];
          j++;
        }
      }

      //sum
      x_s = 0;
      y_s = 0;
      m = 0;
      for(n = 0;n < k;n++){
        Farther_flag = 0;
        for(l = 0;l < j;l++){
          if(step[n] == Farther_value[l]){
            Farther_flag = 1;
            m++;
          }
        }
        if(Farther_flag != 1){
          x_s += x_t[ step[n] ];
          y_s += y_t[ step[n] ];
        }
      }

      //average
      x_a = x_s / (float)(k - m);
      y_a = y_s / (float)(k - m);

      //Linear equations
      part_1 = 0;
      part_2 = 0;
      for(n = 0;n < k;n++){
        Farther_flag = 0;
        for(l = 0;l < j;l++){
          if(step[n] == Farther_value[l])
            Farther_flag = 1;
        }
        if(Farther_flag != 1){
          part_1 += x_t[ step[n] ] * y_t[ step[n] ];
          part_2 += pow( x_t[ step[n] ] , 2.0 );
        }
      }

      a_1[i] = ( part_1 - ((float)(k - m) * x_a * y_a) ) / ( part_2 - ( (float)(k - m) * pow(x_a , 2.0) ) );
      b_1[i] = y_a - (a_1[i] * x_a);
    }
  }

  //next side line
  for(i = 900;i > 680;i--){
    j = i;
    k = 0;
    while(1){
      step[k] = j;
      k++;
      if(fabs(x_t[i] - x_t[i - 10]) > 0.08){
        if(fabs(x_t[i + 10] - x_t[i - 20]) > 0.08){
          if(fabs(x_t[i + 20] - x_t[i - 30]) > 0.08){
            break;
          }
        }
      }
      if(j < 219)
        break;
      j -= 10;
    }
    if(k > 3){
      x_t_s = 0;
      for(n = 0;n < k;n++)
        x_t_s += x_t[step[n]];
      x_t_s /= (float)k;
      j = 0;
      for(n = 0;n < k;n++){
        if(fabs(x_t[step[n]] - x_t_s) > 5.0){
          Farther_value[j] = step[n];
          j++;
        }
      }

      //sum
      x_s = 0;
      y_s = 0;
      m = 0;
      for(n = 0;n < k;n++){
        Farther_flag = 0;
        for(l = 0;l < j;l++){
          if(step[n] == Farther_value[l]){
            Farther_flag = 1;
            m++;
          }
        }
        if(Farther_flag != 1){
          x_s += x_t[ step[n] ];
          y_s += y_t[ step[n] ];
        }
      }

      //average
      x_a = x_s / (float)(k - m);
      y_a = y_s / (float)(k - m);

      //Linear equations
      part_1 = 0;
      part_2 = 0;
      for(n = 0;n < k;n++){
        Farther_flag = 0;
        for(l = 0;l < j;l++){
          if(step[n] == Farther_value[l])
            Farther_flag = 1;
        }
        if(Farther_flag != 1){
          part_1 += x_t[ step[n] ] * y_t[ step[n] ];
          part_2 += pow( x_t[ step[n] ] , 2.0 );
        }
      }

      a_1[i] = ( part_1 - ((float)(k - m) * x_a * y_a) ) / ( part_2 - ( (float)(k - m) * pow(x_a , 2.0) ) );
      b_1[i] = y_a - (a_1[i] * x_a);
    }
  }

  counter = 0;
  for(i = 189;i <= 900;i++){
    if(a_1[i] != 0){
      counter++;
      angle = acos( fabs(a_1[i]*a_2 + 1) / sqrt( pow(a_1[i] , 2) + 1 ) * sqrt( pow(a_2 , 2) + 1 ) );

      if(counter == 1){
        angle_jdg = angle * (180 / M_PI);
        step_s = i;
      }
      else if( fabs(90.0 - angle_jdg) > fabs(90.0 - angle * (180 / M_PI)) ){
        angle_jdg = angle * (180 / M_PI);
        step_s = i;
      }
    }
  }
  if(189 <= step_s && step_s < 400)
    flag = 1;
  else
    flag = 0;

  //corner
  x_c = (b_2 - b_1[step_s]) / (a_1[step_s] - a_2);
  y_c = (a_1[step_s] * x_c) + b_1[step_s];

  //reverse corner
  if(flag == 1){
    x_cc = -sqrt( pow(X , 2.0) / ( 1 + pow(a_2 , 2.0) ) ) + x_c;
    y_cc = a_2 * (x_cc - x_c) + y_c;
  }
  else{
    x_cc = sqrt( pow(X , 2.0) / ( 1 + pow(a_2 , 2.0) ) ) + x_c;
    y_cc = a_2 * (x_cc - x_c) + y_c;
  }

  //step of corner
  for(i = 180;i <= 900;i++){
    diff_1 = x_c - x_t[i];
    diff_2 = y_c - y_t[i];
    diff_1 = fabs(diff_1);
    diff_2 = fabs(diff_2);
    if(i == 180){
      difference = diff_1 + diff_2;
      step_1 = i;
    }
    else if(difference > diff_1 + diff_2){
      difference = diff_1 + diff_2;
      step_1 = i;
    }
  }

  for(i = 180;i <= 900;i++){
    diff_1 = x_cc - x_t[i];
    diff_2 = y_cc - y_t[i];
    diff_1 = fabs(diff_1);
    diff_2 = fabs(diff_2);
    if(i == 180){
      difference = diff_1 + diff_2;
      step_2 = i;
    }
    else if(difference > diff_1 + diff_2){
      difference = diff_1 + diff_2;
      step_2 = i;
    }
  }

  //distance to corner
  cldis_1 = sqrt( pow(x_t[step_1] , 2.0) + pow(y_t[step_1] , 2.0) );
  cldis_2 = sqrt( pow(x_t[step_2] , 2.0) + pow(y_t[step_2] , 2.0) );

  //coordinate
  if(flag == 1)
    deg = (step_2 - step_1) * 0.25;
  else
    deg = (step_1 - step_2) * 0.25;
  rad = deg * M_PI / 180.0;

  y = ( cldis_1 * cldis_2 * sin(rad) ) / X;
  if(flag == 1)
    x = (X / 2.0) - sqrt( pow(cldis_1 , 2.0) - pow(y , 2.0) );
  else
    x = (X / 2.0) - sqrt( pow(cldis_2 , 2.0) - pow(y , 2.0) );

  //slope
  x_0 = sqrt( pow((X / 2.0) , 2.0) - pow((y - sl) , 2.0) );
  cldis_0 = x_0 - x;
  if(flag == 1)
    stddeg = acos( ( (X / 2.0) - x) / cldis_1 ) * (180.0 / M_PI);
  else
    stddeg = acos( ( (X / 2.0) - x) / cldis_2 ) * (180.0 / M_PI);
  if(flag == 1)
    cmpdeg = (step_1 - 180) * 0.25;
  else
    cmpdeg = (step_2 - 180) * 0.25;

  slope = (cmpdeg - stddeg);

  x += 4.0;
  printf("x : %f  y : %f  deg : %f\n",x,y,slope);
}

void serial_arduinowrite(int fd, double phi)
{
  int data;
  char mark[1],temp;
  char buf[255];

  /*
     if(abs(deff) > 100){
     strcpy(buf, "");

  //ñ⁄àÛëóêM
  mark[0] = 126;
  if(write(fd, mark, 1) < 0){
  printf("ñ⁄àÛëóêMé∏îs\n");
  exit(1);
  }

  //deffëóêM
  temp = deff;
  buf[0] = deff>>8;
  if(write(fd, buf, 1) < 0){
  printf("deffëóêMé∏îs\n");
  exit(1);
  }

  buf[0] = temp;
  if(write(fd, buf, 1) < 0){
  printf("deffëóêMé∏îs\n");
  exit(1);
  }
  }
  else{*/
  data = x * 1000.0;
  fprintf(fp,"x : %f\n",x);
  strcpy(buf, "");

  //ñ⁄àÛëóêM
  mark[0] = 126;
  if(write(fd, mark, 1) < 0){
    printf("mark sending error\n");
    exit(1);
  }

  //è„à ÉrÉbÉgëóêM
  temp = data;
  buf[0] = data>>8;
  if(write(fd, buf, 1) < 0){
    printf("upper bits sending error\n");
    exit(1);
  }

  //â∫à ÉrÉbÉgëóêM
  buf[0] = temp;
  if(write(fd, buf, 1) < 0){
    printf("lower bit sending error\n");
    exit(1);
  }
  //}
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

int main(int argc, char *argv[])
{
  int fd;
  char name[255], devicename[] = "/dev/ttyACM0";
  struct termios oldtio, newtio;

  clock_t starttime, endtime;
  double difftime;

  enum {
    CAPTURE_TIMES = 1,
  };
  urg_t urg;
  long *data = NULL;
  long time_stamp;
  int n;

  int i;
  double phi;


  /////arduinoÇ∆ÇÃí êM/////
  fd = open(devicename,O_RDWR|O_NONBLOCK);
  if(fd<0)
  {
    printf("ERROR on device open.\n");
    exit(1);
  }
  ioctl(fd,TCGETS,&oldtio);
  newtio = oldtio;
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  ioctl(fd,TCSETS,&newtio);

  fp = fopen("20181114_1.txt","w");
  if(fp == NULL){
    printf("cant file open\n");
    exit(1);
  }

  if (open_urg_sensor(&urg, argc, argv, "172.16.0.10") < 0) {
    return 1;
  }

  while(!kbhit()){
    starttime = clock();
    /////ê˘âÒ/////
    /*
       if (open_urg_sensor(&urg, argc, argv, "192.168.0.10") < 0) {
       return 1;
       }

       data = (long *)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
       if (!data) {
       perror("urg_max_index()");
       return 1;
       }

    // ÉfÅ[É^éÊìæ
    urg_set_scanning_parameter(&urg,
    urg_deg2step(&urg, -90),
    urg_deg2step(&urg, +90), 0);

    urg_start_measurement(&urg, URG_DISTANCE, URG_SCAN_INFINITY, 0);
    for (i = 0; i < CAPTURE_TIMES; ++i) {
    n = urg_get_distance(&urg, data, &time_stamp);

    if (n <= 0) {
    printf("error\n");
    free(data);
    urg_close(&urg);
    return 1;
    }
    phi = rotation(&urg, data, n);
    }
    // êÿíf
    free(data);
    urg_close(&urg);
    */

    /////à íu/////
    /*
       if (open_urg_sensor(&urg, argc, argv, "172.16.0.10") < 0) {
       return 1;
       }
       */

    data = (long *)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
    if (!data) {
      perror("urg_max_index()");
      return 1;
    }

    // ÉfÅ[É^éÊìæ
    urg_set_scanning_parameter(&urg,
        urg_deg2step(&urg, -135),
        urg_deg2step(&urg, +135), 0);

    urg_start_measurement(&urg, URG_DISTANCE, URG_SCAN_INFINITY, 0);
    for (i = 0; i < CAPTURE_TIMES; ++i) {
      n = urg_get_distance(&urg, data, &time_stamp);

      if (n <= 0) {
        printf("error\n");
        free(data);
        urg_close(&urg);
        return 1;
      }
      print_data(&urg, data, n, time_stamp);
    }

    serial_arduinowrite(fd, phi);
    // êÿíf
    //free(data);
    //urg_close(&urg);
    endtime = clock();
    difftime = (double)(endtime - starttime) / CLOCKS_PER_SEC;
    while(difftime < 0.05){
      endtime = clock();
      difftime = (double)(endtime - starttime) / CLOCKS_PER_SEC;
    }
    printf("time : %f msec\n", difftime);

    //starttime = clock();
  }
  fclose(fp);

  /////arduinoÇ∆ÇÃí êMêÿíf/////
  ioctl(fd,TCSETS,&oldtio);
  close(fd);

#if defined(URG_MSC)
  getchar();
#endif
  return 0;
}
