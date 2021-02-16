// taken from https://github.com/achael/koral_lite/blob/master/mdefs.h

/*************************************************************************
                        Mathematica source file
        Copyright 1986 through 1999 by Wolfram Research Inc.
*************************************************************************/

/* C language definitions for use with Mathematica output */

#define Power(x, y)	(pow((double)(x), (double)(y)))
#define Sqrt(x)		(sqrt((double)(x)))
#define Sqrtl(x)        (sqrt((double)(x)))

#define Abs(x)		(fabs((double)(x)))

#define Exp(x)		(exp((double)(x)))
#define Log(x)		(log((double)(x)))

#define Sin(x)		(sin((double)(x)))
#define Cos(x)		(cos((double)(x)))
#define Tan(x)		(tan((double)(x)))

#define ArcSin(x)       (asin((double)(x)))
#define ArcCos(x)       (acos((double)(x)))
#define ArcTan(x)       (atan((double)(x)))

#define Sinh(x)          (sinh((double)(x)))
#define Cosh(x)          (cosh((double)(x)))
#define Tanh(x)          (tanh((double)(x)))

#define Cot(x)          (1./tan((double)(x)))
#define Csc(x)          (1./sin((double)(x)))
#define Sec(x)          (1./cos((double)(x)))

/** Could add definitions for Random(), SeedRandom(), etc. **/
