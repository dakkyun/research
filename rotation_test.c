#include "urg_sensor.h"
#include "urg_utils.h"
#include "open_urg_sensor.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

FILE *fp;

int rotation(urg_t *urg, long data[], int data_n)
{
  int first_index, second_index, third_index, l;
  int i, j;
  double x[2], y[2], l3;
  double radian;
  double f, t, theta, phi;

  (void)data_n;

  first_index = urg_step2index(urg, -20);
  second_index = urg_step2index(urg, +20);
  third_index = urg_step2index(urg, 0);
  //printf("%ld %ld %ld\n", data[first_index], data[second_index], data[third_index]);

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
  //printf("%f %f\n", x[0], y[0]);
  //printf("%f %f\n", x[1], y[1]);

  l3 = sqrt(pow(x[0] - x[1], 2.0) + pow(y[0] - y[1], 2.0));
  //printf("%f\n", l3);
  f = (double)data[first_index] / 1000.0;
  t = (double)data[third_index] / 1000.0;
  //printf("%f %f\n", f, t);
  theta = acos((pow(t, 2.0) + pow(l3, 2.0) - pow(f, 2.0)) / (2.0 * t * l3)) * (180.0 / M_PI);
  //printf("%f\n", theta);
  if(data[first_index] > data[second_index]){
    phi = 180.0 - (90.0 + (180.0 - theta));
    printf("rotation : -%f [deg]\n", phi);
    //fprintf(fp,"rotation : -%f [deg]\n", phi);
  }
  else{
    phi = 180.0 - (90.0 + theta);
    printf("rotation : %f [deg]\n", phi);
    //fprintf(fp,"rotation : %f [deg]\n", phi);
  }

  return 0;
}

int main(int argc, char *argv[])
{
  char name[255], devicename[] = "/dev/ttyACM0";
  struct termios oldtio, newtio;

  enum {
    CAPTURE_TIMES = 1,
  };
  urg_t urg;
  long *data = NULL;
  long time_stamp;
  int n;

  int i;

  fp = fopen("0deg.txt","w");
  if(fp == NULL){
    printf("cant file open\n");
    exit(1);
  }

  while(1){
    if (open_urg_sensor(&urg, argc, argv, "172.16.0.10") < 0) {
      return 1;
    }

    data = (long *)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
    if (!data) {
      perror("urg_max_index()");
      return 1;
    }

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
      rotation(&urg, data, n);
    }
    free(data);
    urg_close(&urg);
  }
  fclose(fp);
}
