#ifndef __PID
#define __PID

float Ang_PID_cal(float REF,float FDB);
float Pos_PID_cal(float REF,float FDB);
void Infantry_Pid_Contrl(float Ang_Ref,float Ang_Fdb,float Pos_Ref,float Pos_Fdb,float Speed_Fdb);

typedef struct
{
	double ref;
	double fdb;
	double kp;
	double ki;
	double kd;
	double dt;
	double Ek;
	double Sk;
	double Dk;
	double out;
	double outMax;
	double outMin;
	double Ek_1,Ek_2;
}PIDStrucTypeDef;

#endif
