#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <sys/mman.h>
#include <time.h>
#include <linux/ioctl.h>
#include <stdlib.h>
#include <linux/types.h>
#include <stdint.h>
#include "kyouko3_reg.h"

#define KYOUKO3_CONTROL_SIZE (65536)

struct fifoentry{
    uint32_t  command;
    uint32_t  value;
}fifoentry;

struct u_kyouko_device{
    unsigned int *u_control_base;
    unsigned int *u_card_ram_base;
}kyouko3;

float position[3][4] = {{-0.5, -0.5, 0.0, 1.0},
                         {0.5, 0.0, 0.0, 1.0},
                         {0.125, 0.5, 0.0, 1.0}};

float color[3][4] = {{1.0, 0.0, 0.0, 1.0},
                     {0.0, 1.0, 0.0, 1.0},
                     {0.0, 0.0, 1.0, 1.0}};

struct DMA_Header{
    uint32_t address: 14;
    uint32_t count: 10;
    uint32_t opcode: 8;
}header;


unsigned int U_READ_REG(unsigned int rgister){
    return (*(kyouko3.u_control_base+(rgister>>2)));
}

void U_WRITE_FB(unsigned int reg, unsigned int value){
    *(kyouko3.u_card_ram_base + reg) = value;
}

void U_WRITE_REG(unsigned int reg, unsigned int value){
    *(kyouko3.u_card_ram_base + (reg>>2)) = value;
}

int draw_Triangle(unsigned int *buffer){

    int count,i, j;

    /*Update the header*/
     header.address = 0x1045;
     header.count = 0x01;
     header.opcode = 0x14;
     *buffer = header;
     buffer++;
     count++;

  /*Load the vertices*/
   for(i = 0; i < 3; i++){
            /*set up color*/
        for(j = 0; j < 3; j++){
            *buffer++ = *(unsigned int *)&(-1+2*((float)rand())/RAND_MAX);
            count++;
        }
        /*set up position*/
        for(j = 0; j < 4; j++){
            *buffer++ = *(unsigned int *)&(-1+2*((float)rand())/RAND_MAX);
            count++;
        }
    }
    return count;
}

int main()
{
    int fd;
    int i;
    int Current_Buffer = 0;
    int size = 0;
    unsigned long *arg;
    unsigned int *current_Buffer;
    printf("in the main  : %d\n", result);
    fd = open("/dev/kyouko3", O_RDWR);


    /*Memory Map command region*/
    kyouko3.u_control_base = mmap(0, KYOUKO3_CONTROL_SIZE, PROT_READ|PROT_WRITE,
    MAP_SHARED, fd, 0);

    /*Memory map the card ram base*/
    kyouko3.u_card_ram_base = mmap(0, 1024*768*4, PROT_READ|PROT_WRITE,
    MAP_SHARED, fd, 0x80000000);

    /*Turn Graphics mode on*/
    ioctl(fd, VMODE, GRAPHICS_ON);


    /*Writing the DMA buffer*/

    while(i < Total_buffers){
        //size = 19;
        ioctl(fd, BIND_DMA, &arg);
        current_Buffer = (unsigned int *) arg;
        size = draw_Triangle(current_Buffer);

        /*send flush command before sending DMA buffer*/
        fifoentry.command = Flush;
        fifoentry.value = 0;
        ioctl(fd, FIFO_QUEUE, &fifoentry);

        int head = U_READ_REG(FifoHead);
        int tail = U_READ_REG(FifoTail);

        while(head != tail){
            ;
        }

        ioctl(fd, START_DMA, &size);

        /*send flush command after sending DMA buffer*/
        fifoentry.command = Flush;
        fifoentry.value = 0;
        ioctl(fd, FIFO_QUEUE, &fifoentry);

        //current_buffer = 1 - current_buffer;
        i++;
    }
    sleep(5);
    /*FIFO flush command*/
    ioctl(fd, FIFO_FLUSH);
    sleep(10);

    /*Graphics off command*/
    ioctl(fd, VMODE, GRAPHICS_OFF);
    close(fd);
    return 0;
}

