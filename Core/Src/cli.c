/*
 * cli.c
 *
 *  Created on: Feb 18, 2022
 *      Author: Andre
 */
#include <stdio.h>

#include "usart.h"
#include "cli.h"
#include "global.h"

uint32_t cli_input_value = 0;

void CLI_decode(char* buffer)
{
	uint8_t i = 0;

	while (buffer[i] != '\0')
	{
		if (buffer[i] >= '0' && buffer[i] <= '9')
		{
			cli_input_value = 10*cli_input_value + (buffer[i] - '0');
		}
		else
		{
			switch (buffer[i])
			{
				case 'a':
				case 'A':
					op_mode = mode_auto;
					UART_TX_string("Auto mode selected. \n\r");
					UART_TX_string("Adjust pulse width via the trimpot on the DISCO BOARD. \n\r");
					break;

				case 'd':
				case 'D':
					op_mode = mode_debug;
					UART_TX_string("Debug mode selected. \n\r");
					UART_TX_string("Enter pulse width from 0 to 100 and terminate with 'p'. \n\r");
					break;

				case 'm':
				case 'M':
					op_mode = mode_manual;
					UART_TX_string("Manual mode selected. \n\r");
					UART_TX_string("Adjust pulse width via the trimpot on the DISCO BOARD. \n\r");
					break;

				case 'p':
				case 'P':
					flag_update_pulse = 1;
					break;

				case '\n':
				case '\r':
					break;

				default:
					flag_wrong_cmd = 1;
					break;
			}
		}
		i++;
	}
}
