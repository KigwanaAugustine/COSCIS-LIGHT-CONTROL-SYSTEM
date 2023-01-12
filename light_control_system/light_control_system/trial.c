#include<stdio.h>
#include<string.h>
#include <stdlib.h>

int night_time();

int main()
{
    
  int i = 1;
  char str[100];
  itoa(i, str, 10);
  
  printf("%s\n", str);  // prints "123"
  return 0;

}

int night_time()
{
    unsigned char time[7] ;

    unsigned char digit1 = '2';
    unsigned char digit2 = '3';
    unsigned char digit3 = '0';
    unsigned char digit4 = '1';
    unsigned char digit5 = '0';
    unsigned char digit6 = '0';



    time[0] = digit1;
    time[1] = digit2;
    time[2] = digit3;
    time[3] = digit4;
    time[4] = digit5;
    time[5] = digit6;
    time[6] = '\0';

    unsigned char * night_time = "213000";
    unsigned char * midnight = "235959";

    if( (( (strcmp(time, night_time)) > 0) && (strcmp(time, midnight) < 0) ) || (((strcmp(time, "000000") > 0) && strcmp(time, "065959") < 0)) || (strcmp(time, "000000") == 0))
    {
        return 1;
    }
    else
    {
        return 0;
    }

}