#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

float Filter[10];

float average_filter(float filter_input )
{
    int i;
    float filter_output;
    Filter[i] = filter_input;
    
    filter_output = 0.66 * filter_input + 0.12 * Filter[i-1] + 0.1 * Filter[i-2] + 0.06 * Filter[i-3] + 0.04 * Filter[i-4]
    + 0.02 * Filter[i-5];
    i++;
    return filter_output;
}