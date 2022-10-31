#include "common.h"


#define START_COMMAND 0
#define ARRG_COMMAND 1
#define INVALID_COMMAND -1

namespace TXCommand{


    uint8_t command = INVALID_COMMAND;
    uint8_t line[2];
    uint8_t arrg[2];
    uint8_t lineiter;
    uint8_t arrgiter;
    

    void proccess(){

    }


    void encode (char c){
        switch (command){
            case START_COMMAND:
                line[lineiter] = c;
                lineiter++;
                if (lineiter > 1)
                    command = INVALID_COMMAND;
                break;
            case ARRG_COMMAND:
                
        }

        switch (c){
            case 'x':
                command = START_COMMAND;
                lineiter = 0;
                break;
            case ' ':
                command = ARRG_COMMAND;
                arrgiter = 0;
                break;
            case '\n':
                proccess();

        }
    }
}