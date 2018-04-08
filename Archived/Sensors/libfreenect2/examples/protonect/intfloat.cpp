
#include<iostream>
#include<stdio.h>

int main() {
    int a = 0x40000000;
    int *b = &a;
    float *c = (float *)b;
    printf("%p, %p\n", b, c);
    
    printf("%d, %0.32f\n", *b, *c);
}

