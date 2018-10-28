#include "stm32f103c8t6.h"
#include "mbed.h"
#include <string>

//------------------------------------
// Hyperterminal configuration
// 9600 bauds, 8-bit data, no parity
//------------------------------------

// TX, RX, BAUD
RawSerial pc(PB_6, PB_7, 115200);
RawSerial telit(PA_2, PA_3, 115200);

DigitalOut statusLed(LED1);


void onReceive(){
    //pc.printf("datain\n");
    pc.putc(telit.getc());
}

int main(){

    pc.printf("Hello World !\n");
    /*char at[3] = {'A','T','\0'};
    char command[5] = {'+','G','M','I','\0'};
    char terminator[2] = {0xD, '\0'};
    char message[10];*/
    //char buffer[100];
    telit.attach(&onReceive);
    std::string at = "AT+";
    std::string at1 = "GMI\r";
    for (int i = 0; at[i]!='\0'; ++i){
        //pc.putc(at[i]);
        telit.putc(at[i]);
    }
    for (int i = 0; at1[i]!='\0'; ++i){
        //pc.putc(at[i]);
        telit.putc(at1[i]);
    }
    /*strcat(at,message);
    strcat(command,message);
    strcat(terminator,message);*/
    /*i=0;
    while (message[i]!='\0'){
        pc.printf("%s\n", message[i]);
        telit.printf("%s", message[i]);
        i++;
    }*/
    //telit.printf("%s", message);
    /*while(telit.readable()){
        pc.putc(telit.getc());
    }*/
    wait(1);
    
}
