
#ifndef	_DFRSHIFTER_H_
#define _DFRSHIFTER_H_	(1)
//Upshift function
void upshift(void);

//Down shift function
void downshift(void);

//Off Shift Function
void offshift(void);


void test_shifter_algorithm (void);
int validShift(int target, int RPM);
#endif	// _DFRSHIFTER_H_
