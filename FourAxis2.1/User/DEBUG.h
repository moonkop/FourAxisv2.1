#define _DEBUG

#ifndef _DEBUG_H_
	#define _DEBUG_H_
	#ifdef _DEBUG
		#define SEND(x,y) Usart2_Send(x,y)
		#include "Usart.h"
	#endif
	#ifndef _DEBUG
		#define SEND(x,y)
	#endif
#endif
