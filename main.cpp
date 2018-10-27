#include "stm32f103c8t6.h"
#include "mbed.h"

//------------------------------------
// Hyperterminal configuration
// 9600 bauds, 8-bit data, no parity
//------------------------------------

Serial pc(PB_6, PB_7, 115200);
Serial telit(PA_2, PA_3, 115200);

DigitalOut statusLed(LED1);
    

int main()
{
    pc.printf("Hello World !\n");
    char at[3] = {'A','T','\0'};
    char command[5] = {'+','G','M','I','\0'};
    char terminator[2] = {0xD, '\0'};
    char message[10];
    //char buffer[100];
    
    strcat(at,message);
    strcat(command,message);
    strcat(terminator,message);
    
    while(1) {
        pc.printf("%s\n", message);
        telit.printf("%s", message);
        while(telit.readable()){
            pc.putc(telit.getc());
        }
        wait(1);
    }
}
