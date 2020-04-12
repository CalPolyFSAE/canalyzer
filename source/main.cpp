#include "MKE18F16.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "can.h"
#include "uart.h"

using namespace BSP;

#define UARTNO 0
#define CANNO 0

uint16_t head = 0;
uint16_t tail = 0;
uint16_t remaining = 0;
#define framemax 100
can::CANlight::frame frames[framemax];

void pushframe(can::CANlight::frame f){
    frames[tail] = f;
    tail = (tail + 1) % framemax;
    remaining = remaining < framemax ? remaining + 1 : framemax;
}

can::CANlight::frame popframe(void){
    can::CANlight::frame f = frames[head];
    head = (head + 1) % framemax;
    remaining -= 1;
    return f;
}

void uartcb(uint8_t data){
    return;
}

void cancb(void){
    pushframe(can::CANlight::StaticClass().readrx(CANNO));
    return;
}

char buf[29]; // 1 type + 8 id + 1 dlc + 16 data + \r + \n + \0 = 29

uint8_t encapsulate(can::CANlight::frame f){
    uint8_t ptr = 0;
    char type = ' ';
    if(f.ext == 0){
        if(f.rtr == 0){
            type = 't';
        } else {
            type = 'r';
        }
    } else {
        if(f.rtr == 0){
            type = 'T';
        } else {
            type = 'R';
        }
    }
    ptr += sprintf(buf+ptr, "%c", type);

    if(f.ext == 1)
        ptr += sprintf(buf+ptr, "%08X", (unsigned int)f.id);
    else
        ptr += sprintf(buf+ptr, "%03X", (unsigned int)f.id);
    ptr += sprintf(buf+ptr, "%X", (unsigned int)f.dlc);
    for(uint8_t i = 0; i < f.dlc; i++)
        ptr += sprintf(buf+ptr, "%X", (unsigned int)f.data[i]);

    ptr += sprintf(buf+ptr, "\r");

    return ptr;
}

can::CANlight::frame f_out;
int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    uart::config uartcc;
    uart::UART::ConstructStatic(&uartcc);
    uart::UART& uart = uart::UART::StaticClass();

    uart::UART::uartconfig uartc;
    uartc.callback = uartcb;
    uartc.echo = 0;
    uartc.baudrate = 921600;

    can::can_config cancc;
    can::CANlight::ConstructStatic(&cancc);
    can::CANlight& can = can::CANlight::StaticClass();

    can::CANlight::canx_config canc;
    canc.callback = cancb;

    uart.init(UARTNO, &uartc);
    can.init(CANNO, &canc);

    canc.callback = NULL;
    can.init(1, &canc);

    SysTick_Config(60000);

    uint8_t sz;

    f_out.ext = 0;
    f_out.id = 0x1AA;
    f_out.dlc = 1;
    f_out.data[0] = 0x55;

    while(1){
        if(remaining){
            sz = encapsulate(popframe());
            uart.write(UARTNO, (uint8_t*)buf, sz);
        }
    }

    return 0;
}

#define period 1000

extern "C" {
    void SysTick_Handler(){
        static uint16_t i = 0;
        i = (i + 1) % period;
        if(!i){
            can::CANlight::StaticClass().tx(1, f_out);    
        }
    }
}

