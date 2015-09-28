#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>



float average_filter(float filter_input )
{
    int i;
    float filter_output;
    float Filter[10];
    Filter[9] = filter_input;
    
    filter_output = 0.66 * Filter[8] + 0.12 * Filter[6] + 0.1 * Filter[4] + 0.06 * Filter[2] + 0.04 * Filter[0];
    
    for(i=9;i>0;i--)
    {
        Filter[i-1]=Fileter[i];
    }
    
    return filter_output;
}