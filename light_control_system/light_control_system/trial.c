#include<stdio.h>
#include<string.h>

int main()
{
    char  array_str[128] = "August";

    char *str = array_str;

    printf("%s", str);

    printf("%d", strlen(str));
    
    return 0;
}