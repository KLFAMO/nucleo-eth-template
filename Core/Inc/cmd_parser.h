/*
 * cmd_parser.h
 *
 *  Created on: Jul 25, 2024
 *      Author: pmorz
 */

#ifndef INC_CMD_PARSER_H_
#define INC_CMD_PARSER_H_

typedef struct{
	char name[20];
	double val;
} Param;

typedef struct{
	Param I;
	Param set_A;
} Params;

extern Params par;

int cmd_parse(char*, char*);
void init_params(void);
int ftostr(char*, double);

#endif /* INC_CMD_PARSER_H_ */
