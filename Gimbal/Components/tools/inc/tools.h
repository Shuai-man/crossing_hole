#ifndef _TOOLS_H
#define _TOOLS_H

#ifndef TRUE
#define TRUE 1
#endif // !
#ifndef FALSE
#define FALSE 0
#endif // 这个是有返回值的，必须加=才能生效
#define LIMIT_MAX_MIN(x, max, min) (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))
#ifndef PI
#define PI 3.14159265358979f
#endif
#ifndef PI_2
#define PI_2 6.2831853072f
#endif

/* 普通延时 */
void delay_ms(unsigned long t);
void delay_us(unsigned long t);
int float_rounding(float raw);

#endif // !_TOOLS_H
