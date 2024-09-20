/*
 * cmd_parser.c
 *
 *  Created on: Jul 25, 2024
 *      Author: Piotr Morzynski
 */

#include "cmd_parser.h"
#include <string.h>

Params par;

void init_params(void){
	par.I = (Param){"I", 1.435};
	par.set_A = (Param){"set_A", 0};
}


int cmd_parse(char* in_txt, char* out_txt){
	//strcpy(out_txt, in_txt);
	ftostr(out_txt, par.I.val);
	strcat(out_txt, "\n");
	return 0;
}

int ftostr(char* str, double val){
	int istr=0, i, j, isdot=0;
	double factor;
	int order;

	if (val==0){
	       	strcpy(str,"0");
		return 0;
	}
	/*check sign*/
	if (val<0){
		str[istr]='-';
		istr++;
		val = -1*val;
	}
	factor=100000000;
	while(factor>0.00000001 && factor>val) factor=factor/10;
	order = (int) log10(factor);
	if (order<0){
	       	order=-1*order;
		str[istr]='0';
		istr++;
		str[istr]='.';
		istr++;
		for(i=1; i<order; i++){
			str[istr]='0';
			istr++;
		}
	}
	for (j=0; j<10; j++){
		for(i=9; i>=0; i--){
			if((val-i*factor)>=0){
				str[istr]='0'+i;
				istr++;
				break;
			}
		}
		val=val-i*factor;
		factor=factor/10;
		if (factor<0.5 && factor>0.05){
			str[istr]='.';
			istr++;
			isdot=1;
		}
		if (val==0 && isdot){
			if(str[istr-1]=='.')istr--;
			str[istr]=0;
			break;
		}
	}
	str[istr]=0;
	return 0;
}
