#ifndef __CLI_H
#define __CLI_H

#define AVAILABLE_COMMANDS	"M: Manual Mode; A: Auto Mode; D: Debug Mode. \n\r"

extern uint32_t cli_input_value;

void CLI_decode(char* buffer);

#endif // __CLI_H
