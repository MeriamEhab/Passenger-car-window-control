#ifndef _MACROS__H_
#define _MACROS__H_

#define SET_BIT(Register,Bit) (Register|=(1u<<Bit))

#define CLEAR_BIT(Register,Bit) (Register&=(~(1u<<Bit)))

#define GET_BIT(Register,Bit) ((Register & (1u<<Bit)) >> Bit) 

#define TOGGLE_BIT(Regitser,Bit) (Register ^= (1u<<Bit))

#endif