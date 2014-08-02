#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <sys/time.h>
#include <fcntl.h>
#include <math.h>
#include <linux/i2c-dev.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include <sched.h>
#include <ctype.h>
#include <stdlib.h>
//#include <linux/watchdog.h>


#define I2C0 "/dev/i2c-0"
#define I2C1 "/dev/i2c-1"
#define SERIE "/dev/ttyS0"
#define FTELEMETRY "/tmp/teleIMU"

#define WATCHDOG "/dev/watchdog"

#define is_NaN(x) ((x) != (x))
#define DELTA_T 0.015 /*0.017*/

#define WINDOW_ROUND_1 (int)((1.2-DELTA_T)*5*1)
#define WINDOW_ROUND_2 (int)((1.2-DELTA_T)*5*2)
#define WINDOW_ROUND_3 (int)((1.2-DELTA_T)*5*3+0.3)
#define WINDOW_ROUND_4 (int)(WINDOW_ROUND_3+1)
#define COMPASS_ADDR   0x21
#define BARO_ADDR      0x77
#define MAGNETO_ADDR   0x1E
#define RADIO_ADDR	   0x02

#define WII_ADDR       0x52
#define WII_REQ_DATA   0x00


#define MAG_INIT_REG   0x02
#define MAG_INIT_DATA  0x00

#define ACCL_INIT_REC  0xF0
#define ACCL_INIT_DEC  0x55

#define ACCL_INIT_REG  0xFB
#define ACCL_INIT_DATA 0x00

#define GYRO_INIT_ADDR 0x53
#define GYRO_INIT_REG  0xFE
#define GYRO_INIT_DATA 0x04

#define GYRO_SLOW_FACTOR 20.0
#define GYRO_FAST_FACTOR 4.454545

#define MAG_SCALE	715
#define MAG_SCALE_X 1264.4 
#define MAG_SCALE_Y	1264.4
#define MAG_SCALE_Z	1177.2


#define TIMEOUT_COMMAND 20
#define TIMEOUT_GPS		100
#define WII_BUFFER_SIZE 6


#define CENTER_YAW 		6838
#define CENTER_PITCH	7922
#define CENTER_ROLL		8214

#define FIXED_ACCEL

#define CENTER_X 	521
#define CENTER_Y	519
#define CENTER_Z	512 //715*/

#define MAX_ANTIWINDUP 500

#define RANGE_X		1024//430 //pruebas //1015
#define RANGE_Y		1024//430            //998
#define RANGE_Z		1024//430           //1011 


#define MIN_MAG_X	-773
#define MIN_MAG_Y	-674
#define MIN_MAG_Z	-785

#define MAX_MAG_X	716
#define MAX_MAG_Y	799
#define MAX_MAG_Z	579


#define DEG_TO_RAD  0.017453  // PI/180
#define RAD_TO_DEG 57.295779
#define PI	    	3.141593
#define PI_DIV_2    1.570796
#define PI_MUL_2    6.283185



#define SERVOS 		5
#define PORTS		1
#define BUMPERS		2
#define ANALOGS		2

#define MIN_PWM 	1 
#define MED_PWM 	254
#define MAX_PWM 	511
#define RANGE_PWM 	510 
#define NOMINAL_PWM  220
#define MIN_NOMINAL_PWM 40


#define BUFFER_SERIE 12
#define STRT_CHAR    0xFF
#define I2C_NULL 	 0xFF
#define CALIBRATION_READS 100

#define MEM	        30
#define UDP_PORT    5555

#define TELEMETRY_MSG_QUEUE_SIZE 30
#define TELEMETRY_MSG_SIZE 80

#define MAX_GAS_START 110
#define FAILSAFE_GAS_TH 200000
struct gps{
	double longitud;
	double latitud;
	float altitud;
	float enlace;
	float max_angle;
	double ErrorLongitud;
	double ErrorLatitud;
	float ErrorAltitud;
};
struct comando{
	unsigned char orden;
	int  valor;
	int  gas;
	int  alabeo;
	int  cabeceo;
	int  ginnada;	
	double latitud;
	double longitud;
	double altitud;
	char texto[30];
};

struct imu{
	float alabeo;
	float cabeceo;
	float ginnada;
	float altitud;

	float filter_cabeceo;
	float filter_alabeo;
	float filter_ginnada;
	float offset_alabeo;
	float offset_cabeceo;
	float v_alabeo;
	float v_cabeceo;
	float v_ginnada;
	float v_altitud;
	float a_ginnada;
	float a_alabeo;
	float a_cabeceo;
	
	float last_alabeo;
	float last_cabeceo;
	float last_ginnada;
	float last_v_ginnada;
	float last_v_alabeo;
	float last_v_cabeceo;
	float last_suelo;
	
	float min_suelo;
	float suelo;

	
};
struct UDPConnection{
   struct sockaddr_in srv;
   struct sockaddr_in frm;
   int sock; 
   char  destination_ip[16];
   int port;   
};
struct intAxis{
	signed int x;
	signed int y;
	signed int z;
	float Telapsed;	
};

struct floatAxis{
	float x;
	float y;
	float z;
	float Telapsed;
};

struct pid{
	float output;
	double error;
	float d;
	float i;
    float p;
    float d1;
    float d2;
    float dlfp;
    float op;
    float oi;
    float od;
};
struct Kpid{
	float p;
	float i;
	float d;
    float imax;
};
struct puerto{
	int gyro;
	int accel;
	int mag;
	int alt;
	int comp;
	int serie;
	int radio;
};
short autopilot=0;
short yaw_control=0;
short target_yaw_control=1;
short magswitch=2;
short magmode=1;
short failsafe=0;
short tipoTelemetria=0; //0: web 1: udp //2: strout
short statusGPS=-1;
int pidGPS=-1;
int pidUDP=-1;

float declinacionMAG=0;
struct imu target;
struct gps targetGPS;
struct imu mando;
struct imu sensor;


struct puerto fdevice;

struct Kpid Klongitud;
struct Kpid Klatitud;
struct Kpid Kaltitud;

struct Kpid Kalabeo;
struct Kpid Kcabeceo;

struct Kpid Kalabeo_out;
struct Kpid Kcabeceo_out;
struct Kpid Kalabeo_in;
struct Kpid Kcabeceo_in;

struct Kpid Kginnada;
struct Kpid Kginnada_v;
struct Kpid Kaltitud;

struct Kpid DCM_KCabAla;
struct Kpid DCM_KGuinnada;

struct floatAxis offsetMag;

unsigned char timeout_deltaT=0;
unsigned int timeout_deltaT_overflows=0;
int f_i2c0=0;
int f_i2c1=0;
int f_serie=0;


float lowPassFactorGyro=1;
float lowPassFactorAccel=1;
float lowPassFactorMag=1;
float lowPassFactorDTerm=1;

int             fdwatchdog=0;
int             pre_encendido=0;
struct          UDPConnection  udpSendConn;
char            last_buffer_serie[BUFFER_SERIE];
signed int      escSpecialCom=-1;
unsigned int    motor[SERVOS];
unsigned char   motor_enabled[SERVOS];
signed int      motor_offset[SERVOS];
unsigned char   serial[10];
unsigned char   port[PORTS];

unsigned char   bumpers[BUMPERS];
unsigned char   analogs[ANALOGS];

extern int      errno;
unsigned char   roundCommand=0;
unsigned char   roundCommandTele=0;
double 		    lastGPSClock=0;
int 		    roundGPS=0;
char            *read_mem;
double          *gps_mem;
char            *write_mem;
int             shmid, shmidw;
struct          sched_param schedule;
int             calibration_reads;
int             pid_config_mode=0;
int             motor_config_mode=0;
int             current_window=0;
float           yaw_delay_timer=0;

char colaMensajes[TELEMETRY_MSG_QUEUE_SIZE][TELEMETRY_MSG_SIZE]={"","","","","","","","","","","","","","","","","","","",""};
int nextMsgToQueue=0;
int nextMsgToSend=-1;
unsigned char strt[2]={0xFF, 0xFF};

//comandos
unsigned char encendido=0x00; //motores encendidos
char lastCommand=0x00;
float lastValue=0;

int pid_output=-1;

float  DCM_Matriz[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
float  DCM_Matriz_Cambio[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
float  DCM_Temporal_Matriz[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
float  DCM_Omega_P[3] = {0,0,0};
float  DCM_Omega_I[3] = {0,0,0};

float DCM_errorXYZ [3] ={0,0,0};

float yawProportionLimit=0;

void reset_Matrices_globales(){

 	static const float dcm_init[3][3]= {{1,0,0},{0,1,0},{0,0,1}}; 
 	static const float dcm_zero[3][3]={{0,0,0},{0,0,0},{0,0,0}};
	static const float vector_zero[3]={0,0,0};

 	memcpy(DCM_Matriz, dcm_init, sizeof(dcm_init));
	memcpy(DCM_Matriz_Cambio, dcm_zero, sizeof(dcm_zero));
	memcpy(DCM_Temporal_Matriz, dcm_zero, sizeof(dcm_zero));
	memcpy(DCM_Omega_P, vector_zero, sizeof(vector_zero));
	memcpy(DCM_Omega_I, vector_zero, sizeof(vector_zero));
	memcpy(DCM_errorXYZ, vector_zero, sizeof(vector_zero));

}
inline  void multiplicaMatrices(float a[3][3], float b[3][3],float axb[3][3]){

	axb[0][0]=(a[0][0]*b[0][0])+(a[0][1]*b[1][0])+(a[0][2]*b[2][0]);
	axb[0][1]=(a[0][0]*b[0][1])+(a[0][1]*b[1][1])+(a[0][2]*b[2][1]);
	axb[0][2]=(a[0][0]*b[0][2])+(a[0][1]*b[1][2])+(a[0][2]*b[2][2]);
	
	axb[1][0]=(a[1][0]*b[0][0])+(a[1][1]*b[1][0])+(a[1][2]*b[2][0]);
	axb[1][1]=(a[1][0]*b[0][1])+(a[1][1]*b[1][1])+(a[1][2]*b[2][1]);
	axb[1][2]=(a[1][0]*b[0][2])+(a[1][1]*b[1][2])+(a[1][2]*b[2][2]);
	
	axb[2][0]=(a[2][0]*b[0][0])+(a[2][1]*b[1][0])+(a[2][2]*b[2][0]);
	axb[2][1]=(a[2][0]*b[0][1])+(a[2][1]*b[1][1])+(a[2][2]*b[2][1]);
	axb[2][2]=(a[2][0]*b[0][2])+(a[2][1]*b[1][2])+(a[2][2]*b[2][2]);	
}
 inline float productoEscalarVectores(float vector1[3], float vector2[3]){
	float error = 0.0;
	error += vector1[0]*vector2[0];
	error += vector1[1]*vector2[1];
	error += vector1[2]*vector2[2];
	return(error);
}
inline  void escalarVector (float vectorOut[3], float vectorIn[3], float escala){
	vectorOut[0]=vectorIn[0]*escala;
	vectorOut[1]=vectorIn[1]*escala;
	vectorOut[2]=vectorIn[2]*escala;
}
inline  void sumarVectores (float vectorOut[3], float vectorIn1[3], float vectorIn2[3]){
	vectorOut[0]=vectorIn1[0]+vectorIn2[0];
	vectorOut[1]=vectorIn1[1]+vectorIn2[1];
	vectorOut[2]=vectorIn1[2]+vectorIn2[2];
}
inline  void productoVectorial(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]){
	vectorOut[0] = (vectorIn1[1]*vectorIn2[2]) - (vectorIn1[2]*vectorIn2[1]);
	vectorOut[1] = (vectorIn1[2]*vectorIn2[0]) - (vectorIn1[0]*vectorIn2[2]);
	vectorOut[2] = (vectorIn1[0]*vectorIn2[1]) - (vectorIn1[1]*vectorIn2[0]);
}
 inline float moduloUnitario(float vector[3]){
      
	return  sqrtf((vector[0]*vector[0])+(vector[1]*vector[1])+(vector[2]*vector[2]));       

}
 inline float moduloSinRaiz(float vector[3]){
      
	return  ((vector[0]*vector[0])+(vector[1]*vector[1])+(vector[2]*vector[2]));
}
inline void renormalizar(float DCM_Temporal_MatrizX[3][3], float DCM_MatrizX[3][3]){
	float error = 0.0;
	error = (-productoEscalarVectores(DCM_MatrizX[0], DCM_MatrizX[1]))/2;

	escalarVector (DCM_Temporal_MatrizX[0], DCM_MatrizX[1], error);
	escalarVector (DCM_Temporal_MatrizX[1], DCM_MatrizX[0], error);       
	sumarVectores (DCM_Temporal_MatrizX[0], DCM_Temporal_MatrizX[0], DCM_MatrizX[0]);
	sumarVectores (DCM_Temporal_MatrizX[1], DCM_Temporal_MatrizX[1], DCM_MatrizX[1]);
	productoVectorial (DCM_Temporal_MatrizX[2], DCM_Temporal_MatrizX[0], DCM_Temporal_MatrizX[1]); 
	error = 0.5*(3 - productoEscalarVectores(DCM_Temporal_MatrizX[0], DCM_Temporal_MatrizX[0]));
	escalarVector (DCM_MatrizX[0], DCM_Temporal_MatrizX[0], error);   
	error = 0.5*(3 - productoEscalarVectores(DCM_Temporal_MatrizX[1], DCM_Temporal_MatrizX[1]));        
	escalarVector (DCM_MatrizX[1], DCM_Temporal_MatrizX[1], error); 
	error = 0.5*(3 - productoEscalarVectores(DCM_Temporal_MatrizX[2], DCM_Temporal_MatrizX[2]));
	escalarVector (DCM_MatrizX[2], DCM_Temporal_MatrizX[2], error);
}
inline void copiarMatriz(float copia[3][3], float original[3][3]){
	copia[0][0] = original[0][0];
	copia[0][1] = original[0][1];
	copia[0][2] = original[0][2];
	
	copia[1][0] = original[1][0];
	copia[1][1] = original[1][1];
	copia[1][2] = original[1][2];
	
	copia[2][0] = original[2][0];
	copia[2][1] = original[2][1];
	copia[2][2] = original[2][2];
}
inline void actualizarMatrizDCM(struct imu * gyro){
	  DCM_Matriz_Cambio[0][0]=1;
	  DCM_Matriz_Cambio[0][1] = - gyro->filter_ginnada;   		// -z  
	  DCM_Matriz_Cambio[0][2] =   gyro->filter_cabeceo;   		// y     
	  DCM_Matriz_Cambio[1][0] =   gyro->filter_ginnada;    		// z     
	  DCM_Matriz_Cambio[1][1] = 1;
	  DCM_Matriz_Cambio[1][2] = - gyro->filter_alabeo;   		// -x
	  DCM_Matriz_Cambio[2][0] = - gyro->filter_cabeceo;   		// -y
	  DCM_Matriz_Cambio[2][1] =   gyro->filter_alabeo;   		// x
	  DCM_Matriz_Cambio[2][2]=1;
	  multiplicaMatrices (DCM_Matriz , DCM_Matriz_Cambio , DCM_Temporal_Matriz); //a*b=c
	  copiarMatriz(DCM_Matriz, DCM_Temporal_Matriz);

}
inline  void actualizarMatrizDCMError(){
	
	  DCM_Matriz_Cambio[0][0] = 1;
	  DCM_Matriz_Cambio[0][1] = -DCM_errorXYZ[2];   	// -z
	  DCM_Matriz_Cambio[0][2] =  DCM_errorXYZ[1];   	// y
	  DCM_Matriz_Cambio[1][0] =  DCM_errorXYZ[2];      	// z 
	  DCM_Matriz_Cambio[1][1] = 1;
	  DCM_Matriz_Cambio[1][2] = -DCM_errorXYZ[0];  		// - x
	  DCM_Matriz_Cambio[2][0] = -DCM_errorXYZ[1];  		// - y
	  
	  DCM_Matriz_Cambio[2][1] =  DCM_errorXYZ[0];  		// x
	  DCM_Matriz_Cambio[2][2] = 1; 
	  multiplicaMatrices (DCM_Matriz , DCM_Matriz_Cambio , DCM_Temporal_Matriz); //a*b=c
	  copiarMatriz(DCM_Matriz, DCM_Temporal_Matriz);
}
inline void Drift_correction_ac(struct  floatAxis * accel){
	float  AC[3] = {accel->x,accel->y,accel->z};
	float Scaled_Omega [3] ={0,0,0};

	productoVectorial(DCM_errorXYZ, AC, DCM_Matriz[2]);
	escalarVector(DCM_Omega_P, DCM_errorXYZ, DCM_KCabAla.p);
	
	escalarVector(Scaled_Omega, DCM_errorXYZ, DCM_KCabAla.i);
	sumarVectores(DCM_Omega_I, DCM_Omega_I, Scaled_Omega);	
	sumarVectores(DCM_errorXYZ, DCM_Omega_P, DCM_Omega_I);

}

inline void Drift_correction_mag(float MAG_Rumbo)
    {
        float Scaled_Omega[3];
        float errorZ[3];
        float magX, magY, errorRumbo;

        magX = cosf(MAG_Rumbo);
        magY = sinf(MAG_Rumbo);

        errorRumbo = (DCM_Matriz[0][0]*magY) - (DCM_Matriz[1][0]*magX);
        escalarVector(errorZ, DCM_Matriz[2], errorRumbo);

        escalarVector(Scaled_Omega, errorZ, DCM_KGuinnada.p);
        sumarVectores(DCM_Omega_P, DCM_Omega_P, Scaled_Omega);

        escalarVector(Scaled_Omega, errorZ, DCM_KGuinnada.i);
        sumarVectores(DCM_Omega_I, DCM_Omega_I, Scaled_Omega);

        sumarVectores(DCM_errorXYZ, DCM_Omega_P, DCM_Omega_I);

    }

void queueMessage(char  listaMsg[TELEMETRY_MSG_QUEUE_SIZE][TELEMETRY_MSG_SIZE], int * nextMsgToQueue, char * msg, char print){
	strcpy(listaMsg[*nextMsgToQueue],msg);
	*nextMsgToQueue=*nextMsgToQueue+1;
	if (*nextMsgToQueue==TELEMETRY_MSG_QUEUE_SIZE){*nextMsgToQueue=0;}
	if (print ==1){
		printf("\r\n%s",msg);
		fflush(stdout);
	}
	
}

int nextMessage(char listaMsg[TELEMETRY_MSG_QUEUE_SIZE][TELEMETRY_MSG_SIZE],int * nextMsgToSend ){
	//char nextMSG[TELEMETRY_MSG_SIZE];
    int msgToErase=*nextMsgToSend-1;
    int msgToReturn=*nextMsgToSend;
    if(msgToErase<0){msgToErase=TELEMETRY_MSG_QUEUE_SIZE-1;}

	strcpy(listaMsg[msgToErase],"");
	*nextMsgToSend=*nextMsgToSend+1;
	if (*nextMsgToSend==TELEMETRY_MSG_QUEUE_SIZE){*nextMsgToSend=0;}
	return msgToReturn;
}

int shared_init(){
		
	if((shmid = shmget(9998, MEM, IPC_CREAT | 0666)) < 0) {queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[critical]Falló al establecer memoria compartida[1]",1);return -1;}	
	if((read_mem = (char *) shmat(shmid, NULL, 0)) < 0)   {queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[critical]Falló al establecer memoria compartida[2]",1);return -1;}
	if((shmid = shmget(9981,6*sizeof(double *) , IPC_CREAT | 0666)) < 0) { queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[1003]Falló al establecer memoria compartida[3]",1);return -1;}
	if((gps_mem = (double  *) shmat(shmid, NULL, 0)) < 0) {queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[critical]Falló al establecer memoria compartida[4]",1);  return -1;}
	if((shmidw = shmget(9997, 256, IPC_CREAT | 0666)) < 0){queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[critical]Falló al establecer memoria compartida[5]",1);return -1;}	
	if((write_mem = (char *) shmat(shmidw, NULL, 0)) < 0) {queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[critical]Falló al establecer memoria compartida[6]",1);return -1;}
	return 0;
}
int buffer_to_file(char * source_buffer,int buffer_size ,char *target_file){
        int fd;
        int result=-1;
        fd = open(target_file, O_WRONLY|O_CREAT|O_TRUNC);
        if (fd>0){
                result=write(fd,source_buffer,buffer_size);
                close(fd);
        }
        return(result);

}
float get_value(char * file, float defValue){
	int reads,fd;
	float result=defValue;
	char buffer[21];

	fd=open(file,O_RDONLY);
	if (fd>0){
		reads=read(fd,buffer,10);
		buffer[reads]='\0';
		close(fd);
		result=atoi(buffer);
		result=(float)result/1000000.0;
	}else{
		fd=open(file,O_WRONLY|O_CREAT|O_TRUNC);
		sprintf(buffer,"%i",(int)defValue*1000000);
		write(fd,buffer,strlen(buffer));
		close(fd);
	}
	
	
	
	return result;
}
inline char avoid_char(char input, char avoid){
	if(input==avoid){input--;}
	return input;
}
void apply_failsafe_behaviour(char * comm, int currGas){
    //int gas;
    //if (currGas>=FAILSAFE_GAS_TH){
    //    gas=FAILSAFE_GAS_TH;
    //}else{
    //    gas=currGas;
    //}
    sprintf(comm, "QQZ%iZ0Z0Z0", currGas);
}


struct comando get_strCommand(char * comm,unsigned char  defOrder, int defValue){
		struct comando com;
        
        char * ptok;
		char rawOrder[26]="";//21
		int ac=0;
		char maxlen, comlen;
		maxlen=25;
		comlen=strlen(comm);
		com.orden=defOrder;
		com.valor=defValue;	
		if(comlen>0){
			if(comm[0]=='W'){
                apply_failsafe_behaviour(comm, 99999999);
			}
			
			
			com.orden=comm[0];
			if(comlen<maxlen){maxlen=comlen;}
			for (ac=1;ac<maxlen;ac++){
				rawOrder[ac-1]=comm[ac];
			}
			
			rawOrder[ac]='\0';
		
			if(com.orden=='Q'){
				ptok = strtok (rawOrder,"Z"); 	if(ptok==NULL){com.orden=(int)NULL;return com;}
				ptok = strtok (NULL, "Z");	    if(ptok==NULL){com.orden=(int)NULL;return com;}
				com.gas=atoi(ptok);
				ptok = strtok (NULL, "Z");	    if(ptok==NULL){com.orden=(int)NULL;return com;}
				com.alabeo=atoi(ptok);
				ptok = strtok (NULL, "Z");  	if(ptok==NULL){com.orden=(int)NULL;return com;}
				com.cabeceo=atoi(ptok);
				ptok = strtok (NULL, "Z");	    if(ptok==NULL){com.orden=(int)NULL;return com;}
				com.ginnada=atoi(ptok);			
			}else{
				if(com.orden=='G'){
	                                ptok = strtok (rawOrder,"Z");	if(ptok==NULL){com.orden=(int)NULL;return com;}
        	                        ptok = strtok (NULL, "Z");	    if(ptok==NULL){com.orden=(int)NULL;return com;}
					com.latitud=atof(ptok);
					ptok = strtok (NULL, "Z");	if(ptok==NULL){com.orden=(int)NULL;return com;}
					com.longitud=atof(ptok);
					
				}
				if((com.orden=='I')||(com.orden=='T')){
					strcpy(com.texto,rawOrder);
				}else{
					com.valor=atoi(rawOrder);
				}
			}	
		}
		return com;
}
inline int send_serial( unsigned char * buffer_serie, int size){

	return write(fdevice.serie, buffer_serie, size);
	

}
int stop_kernel_watchdog(){
    int fd;
    system("/etc/init.d/watchdog disable 2>/dev/null");
    //sleep(1);
    //fd=open(WATCHDOG, O_WRONLY);
    //printf("\r\n WD FD %i",fd);fflush(stdout);
    //if (fd<=0){
    //    system("/etc/init.d/watchdog start");
    //}
    return fd;
}

void real_time_scheduling(int stage){
	struct sched_param schedule;
    system("killall httpd 2>/dev/null");
    system("killall crond 2>/dev/null");
    system("killall klogd 2>/dev/null");
    system("killall logger 2>/dev/null");
	schedule.sched_priority=sched_get_priority_min(SCHED_OTHER);
	sched_setscheduler(pthread_self(), SCHED_OTHER,&schedule);	

	//if(stage==1){
		
		//system("nice -n -20 httpd -p 80 -h /www -r TCCAtropos");	
        system("httpd -p 80 -h /www -r TCCAtropos");	
	//}
	schedule.sched_priority=sched_get_priority_min(SCHED_FIFO);
	sched_setscheduler(pthread_self(), SCHED_FIFO,&schedule);		
}
int get_order(char * command){
	struct comando com;//XYYYYYYYYYYYYYYYYYY (X:comando, Y:valor)
	char * ptok;
	int lectura_encendido=0;	
	char msg_telemetry[TELEMETRY_MSG_SIZE];
    float val;
	com=get_strCommand(command,'*','0');	
	if (com.orden==(int)NULL){return -1;}
		
	escSpecialCom=-1;
	lastCommand=com.orden;
	lastValue=(float)com.valor;
	switch(com.orden){
		case 'Q':
			
			//ANTES TARGET
			mando.altitud=  ((float)com.gas/1000);
			mando.cabeceo=  ((float)com.cabeceo/1000);
			mando.alabeo=  ((float)com.alabeo/1000);
			mando.v_ginnada=  ((float)com.ginnada/1000);
			
		break;	
		case 'G':
			//if(encendido==1){
				targetGPS.latitud=((double)com.latitud);
				targetGPS.longitud=((double)com.longitud);
				//printf(" Lat %f, lon %f , alt %f",  targetGPS.latitud, targetGPS.longitud,targetGPS.altitud);
			
			//}
		break;
		case 'Y': 
			lectura_encendido=(signed int)(com.valor/1000000);
			if (lectura_encendido==1){
				if (mando.altitud<=MAX_GAS_START){
					if(failsafe==1){
						encendido=0;
						
						queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[wrn] No hay recepcion de mando",1);
					}else{
						encendido=1;
					}	
				}else{
	
					queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[wrn] Bajar gas al minimo para encender",1);
					encendido=0;
				}
			}else{
				encendido=0;
			}
			if(encendido==1){
				queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[info]Encendido",1);
				send_serial(strt, 2);
				target.altitud=-1000;
			}else{
				queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[info]Apagado",1);
				
			}
			
				
			//}//acelerador al minimo
		break; //encendido-apagado motores
		
		case 'l': 
            target.ginnada=sensor.ginnada;
            target.v_ginnada=sensor.v_ginnada;
			if(com.valor>1){
                target_yaw_control=2;
                queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[info]Control de Guiñada establecido a *HEAD HOLD*",1);
			}else{
                target_yaw_control=1;
                queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[info]Control de Guiñada establecido a *ANGULAR RATE*",1);
            }
            
		break;
		/*
		case 'A': target.alabeo=  ((float)com.valor/1000000);break;
		case 'C': target.cabeceo= ((float)com.valor/1000000);break;
		case 'G': target.ginnada= ((float)com.valor/1000000);break;//target de velocidad angular, no de angulo
		
		
		
		case 'a': target.alabeo=  0;break;
		case 'c': target.cabeceo= 0;break;
		case 'g': target.ginnada= 0;break;//target de velocidad angular a 0
		
		
		case 'o': sensor.offset_alabeo =((float)com.valor/1000000);break;
		case 'O': sensor.offset_cabeceo= ((float)com.valor/1000000);break;
		case 'L': target.altitud= ((float)com.valor/1000000);break;
		case 'l': target.altitud= 0;break;//no se usa
		case 'U': autopilot=(short)com.valor;  break;
		*/
		//comandos especiales
		
		
		case 'P': port[0]=(char)((com.valor/1000000));break;
		case 'I': 
			if (encendido==0){
				ptok=strtok( com.texto,":");
				strcpy(udpSendConn.destination_ip,ptok);
				ptok=strtok(NULL,":");
				udpSendConn.port=atoi(ptok);
				sprintf(msg_telemetry,"#Comm:[info]Endpoint set: %s:%i ",udpSendConn.destination_ip, udpSendConn.port);
				queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetry,1);
				UDPtelemetrySender(&udpSendConn);
			}				
		break;
		case 'W':
			if(encendido==0){
				real_time_scheduling(com.valor);
			}
		break;
        
        case 'z': if(encendido==0){ escSpecialCom=MIN_PWM;}break;

		case 't': 
			tipoTelemetria=(short)com.valor;
			sprintf(msg_telemetry,"#Sys:[info] Tipo de Telemetria: %i", (int)tipoTelemetria);
			queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetry,1);
			
		break;
		
        case 'K': if (com.valor<SERVOS&&com.valor>=0){
                        sprintf(msg_telemetry,"#Sys:[wrn]Encendido motor %i", com.valor);
						queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetry,1);
                        
                        motor_enabled[com.valor]=1;
                  }
        break;
        case 'k': if (com.valor<SERVOS&&com.valor>=0&&encendido==0){
                        sprintf(msg_telemetry,"#Sys:[wrn]Apagado motor %i", com.valor);
						queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetry,1);	

                        motor_enabled[com.valor]=0;
                   }
		break;
        case 'x':
            pid_config_mode=(com.valor);                
        break;
        case 'X':
            
            val=(((float)com.valor)/1000000);
            switch (pid_config_mode){
                case 0: Kalabeo_in.p=val;sprintf(msg_telemetry,"#Sys:[wrn] Kalabeo_in.p %f", val);break;
                case 1: Kalabeo_in.i=val;sprintf(msg_telemetry,"#Sys:[wrn] Kalabeo_in.i %f", val);break;
                case 2: Kalabeo_in.d=val;sprintf(msg_telemetry,"#Sys:[wrn] Kalabeo_in.d %f", val);break;
                case 3: Kalabeo_in.imax=val*1000000;sprintf(msg_telemetry,"#Sys:[wrn] Kalabeo_in.imax %f", val*1000000);break;
                
                case 4: Kcabeceo_in.p=val;sprintf(msg_telemetry,"#Sys:[wrn] Kcabeceo_in.p %f", val);break;
                case 5: Kcabeceo_in.i=val;sprintf(msg_telemetry,"#Sys:[wrn] Kcabeceo_in.i %f", val);break;
                case 6: Kcabeceo_in.d=val;sprintf(msg_telemetry,"#Sys:[wrn] Kcabeceo_in.d %f", val);break;
                case 7: Kcabeceo_in.imax=val*1000000;sprintf(msg_telemetry,"#Sys:[wrn] Kcabeceo_in.imax %f", val*1000000);break;
                
                case 8: Kginnada_v.p=val;sprintf(msg_telemetry,"#Sys:[wrn] Kginnada_v.p %f", val);break;
                case 9: Kginnada_v.i=val;sprintf(msg_telemetry,"#Sys:[wrn] Kginnada_v.i %f", val);break;
                case 10: Kginnada_v.d=val;sprintf(msg_telemetry,"#Sys:[wrn] Kginnada_v.d %f", val);break;
                case 11: Kginnada_v.imax=val*1000000;sprintf(msg_telemetry,"#Sys:[wrn] Kginnada_v.imax %f", val*1000000);break;
                
                case 12: Kalabeo_out.p=val;sprintf(msg_telemetry,"#Sys:[wrn] Kalabeo_out.p %f", val);break;
                case 13: Kalabeo_out.i=val;sprintf(msg_telemetry,"#Sys:[wrn] Kalabeo_out.i %f", val);break;
                case 14: Kalabeo_out.d=val;sprintf(msg_telemetry,"#Sys:[wrn] Kalabeo_out.d %f", val);break;
                case 15: Kalabeo_out.imax=val*1000000;sprintf(msg_telemetry,"#Sys:[wrn] Kalabeo_out.imax %f", val*1000000);break;
                
                case 16: Kcabeceo_out.p=val;sprintf(msg_telemetry,"#Sys:[wrn] Kcabeceo_out.p %f", val);break;
                case 17: Kcabeceo_out.i=val;sprintf(msg_telemetry,"#Sys:[wrn] Kcabeceo_out.i %f", val);break;
                case 18: Kcabeceo_out.d=val;sprintf(msg_telemetry,"#Sys:[wrn] Kcabeceo_out.d %f", val);break;
                case 19: Kcabeceo_out.imax=val*1000000;sprintf(msg_telemetry,"#Sys:[wrn] Kcabeceo_out.imax %f", val*1000000);break;
                
                case 20: Kginnada.p=val;sprintf(msg_telemetry,"#Sys:[wrn] Kginnada.p %f", val);break;
                case 21: Kginnada.i=val;sprintf(msg_telemetry,"#Sys:[wrn] Kginnada.i %f", val);break;
                case 22: Kginnada.d=val;sprintf(msg_telemetry,"#Sys:[wrn] Kginnada.d %f", val);break;     
                case 23: Kginnada.imax=val*1000000;sprintf(msg_telemetry,"#Sys:[wrn] Kginnada.imax %f", val*1000000);break;
            }
            queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetry,1);
        break;
        case 'r':
            if((com.valor>=0)&&(com.valor<=3)){
                motor_config_mode=com.valor;
            }
        break;
        case 'R':
            
            motor_offset[motor_config_mode]=(int)com.valor;
            sprintf(msg_telemetry,"#Sys:[wrn] Motor %i offset %i",motor_config_mode, (int)com.valor);
            queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetry,1);
        break;    
	}
	return 0;
	
	
}


void get_config(int sig){
    char msg_telemetry[TELEMETRY_MSG_SIZE];
//http://wn.com/PID_loop
	Kalabeo.p=get_value("/tmp/configKalabeo.p",95);
	Kalabeo.i=get_value("/tmp/configKalabeo.i",0.039);
	Kalabeo.d=get_value("/tmp/configKalabeo.d",0.81);
    Kalabeo.imax=get_value("/tmp/configKalabeo.imax",MAX_ANTIWINDUP);

	Kcabeceo.p=get_value("/tmp/configKcabeceo.p",95);
	Kcabeceo.i=get_value("/tmp/configKcabeceo.i",0.039);
	Kcabeceo.d=get_value("/tmp/configKcabeceo.d",0.81);
    Kcabeceo.imax=get_value("/tmp/configKcabeceo.imax",MAX_ANTIWINDUP);
	
	/***NUEVOS VALORES PID ANIDADO******/
	Kalabeo_in.p=get_value("/tmp/configKalabeo_in.p",0);
	Kalabeo_in.i=get_value("/tmp/configKalabeo_in.i",0);
	Kalabeo_in.d=get_value("/tmp/configKalabeo_in.d",0);
    Kalabeo_in.imax=get_value("/tmp/configKalabeo_in.imax",MAX_ANTIWINDUP);

	Kcabeceo_in.p=get_value("/tmp/configKcabeceo_in.p",0);
	Kcabeceo_in.i=get_value("/tmp/configKcabeceo_in.i",0);
	Kcabeceo_in.d=get_value("/tmp/configKcabeceo_in.d",0);
    Kcabeceo_in.imax=get_value("/tmp/configKcabeceo_in.imax",MAX_ANTIWINDUP);    
	
	Kalabeo_out.p=get_value("/tmp/configKalabeo_out.p",0);
	Kalabeo_out.i=get_value("/tmp/configKalabeo_out.i",0);
	Kalabeo_out.d=get_value("/tmp/configKalabeo_out.d",0);
    Kalabeo_out.imax=get_value("/tmp/configKalabeo_out.imax",MAX_ANTIWINDUP);
    
	Kcabeceo_out.p=get_value("/tmp/configKcabeceo_out.p",0);
	Kcabeceo_out.i=get_value("/tmp/configKcabeceo_out.i",0);
	Kcabeceo_out.d=get_value("/tmp/configKcabeceo_out.d",0);
    Kcabeceo_out.imax=get_value("/tmp/configKcabeceo_out.imax",MAX_ANTIWINDUP);    
	/**************************************************/

	Kginnada.p=get_value("/tmp/configKginnada.p",30);
	Kginnada.i=get_value("/tmp/configKginnada.i",0);
	Kginnada.d=get_value("/tmp/configKginnada.d",0.2);
    Kginnada.imax=get_value("/tmp/configKginnada.imax",MAX_ANTIWINDUP);
    
	Kginnada_v.p=get_value("/tmp/configKginnada_v.p",30);
	Kginnada_v.i=get_value("/tmp/configKginnada_v.i",0);
	Kginnada_v.d=get_value("/tmp/configKginnada_v.d",0);	
	Kginnada_v.imax=get_value("/tmp/configKginnada_v.imax",MAX_ANTIWINDUP);
    
    Klatitud.p=get_value("/tmp/configKlatitud.p",60);
    Klatitud.i=get_value("/tmp/configKlatitud.i",0);
    Klatitud.d=get_value("/tmp/configKlatitud.d",0.5);
    Klatitud.imax=get_value("/tmp/configKlatitud.imax",MAX_ANTIWINDUP);
    
    Klongitud.p=get_value("/tmp/configKlongitud.p",60);
    Klongitud.i=get_value("/tmp/configKlongitud.i",0);
    Klongitud.d=get_value("/tmp/configKlongitud.d",0.5);
	Klongitud.imax=get_value("/tmp/configKlongitud.imax",MAX_ANTIWINDUP);
    
	Kaltitud.p=get_value("/tmp/configKaltitud.p",60);
    Kaltitud.i=get_value("/tmp/configKaltitud.i",0);
    Kaltitud.d=get_value("/tmp/configKaltitud.d",0.5);
    Kaltitud.imax=get_value("/tmp/configKaltitud.imax",MAX_ANTIWINDUP);
    
	DCM_KCabAla.p=get_value("/tmp/configDCM_KCabAla.p",0.02);
	DCM_KCabAla.i=get_value("/tmp/configDCM_KCabAla.i",0.00002);
	DCM_KGuinnada.p=get_value("/tmp/configDCM_KGuinnada.p",1.2);
	DCM_KGuinnada.i=get_value("/tmp/configDCM_KGuinnada.i",0.00002);

	DCM_KCabAla.p=0.02;
	DCM_KCabAla.i=0.00002;
	DCM_KGuinnada.p=1.2;
	DCM_KGuinnada.i=0.00002;


	lowPassFactorAccel=get_value("/tmp/configLowPassAccel",1);
	lowPassFactorMag=get_value("/tmp/configLowPassMag",1);
	lowPassFactorGyro=get_value("/tmp/configLowPassGyro",1);
	lowPassFactorDTerm=get_value("/tmp/configLowPassDTerm",1);
    
	offsetMag.x=get_value("/tmp/configOffsetMag.x",0);
	offsetMag.y=get_value("/tmp/configOffsetMag.y",0);
	offsetMag.z=get_value("/tmp/configOffsetMag.z",0);

	motor_offset[0]=get_value("/tmp/configMotorOffset.0",0);
    motor_offset[1]=get_value("/tmp/configMotorOffset.1",0);
    motor_offset[2]=get_value("/tmp/configMotorOffset.2",0);
    motor_offset[3]=get_value("/tmp/configMotorOffset.3",0);
    motor_offset[4]=get_value("/tmp/configMotorOffset.4",0);

	yawProportionLimit=get_value("/tmp/configYawProportionLimit",0.15);	

	declinacionMAG=get_value("/tmp/configDeclinacionMag",0)*DEG_TO_RAD;
	targetGPS.max_angle=get_value("/tmp/configMaxAutoPilotAngle",3)*DEG_TO_RAD;
    sprintf(msg_telemetry,"#Sys:[info] Lectura config");
    queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetry,1);
}
inline int i2c_write(int fileport,unsigned char device_addr,unsigned  char device_register,unsigned char device_data){
	unsigned char buff_out[2];	
	int data_size=2;

	buff_out[0]=device_register;
	
	if (device_data!=I2C_NULL){
		buff_out[1]=device_data;
	}else{
		data_size=1;
	}
	

	ioctl(fileport, I2C_SLAVE, device_addr);
	return write(fileport, buff_out, data_size);
}
inline int i2c_read(int fileport,unsigned char device_addr,unsigned char * buffer, int size){

	if(device_addr!=I2C_NULL)
	{
		ioctl(fileport, I2C_SLAVE, device_addr);
	}
	return read(fileport, buffer,size);
}
inline int get_wii(int fileport,unsigned char * buffer){	
	
	
	i2c_write(fileport,WII_ADDR,WII_REQ_DATA ,I2C_NULL);
    return  (i2c_read(fileport,I2C_NULL,buffer,WII_BUFFER_SIZE ));
	
}
void init_accel(int device){
	i2c_write(device,WII_ADDR,ACCL_INIT_REC,ACCL_INIT_DEC);
	i2c_write(device,WII_ADDR,ACCL_INIT_REG,ACCL_INIT_DATA);
}
void init_gyro(int device){
	i2c_write(device,GYRO_INIT_ADDR,GYRO_INIT_REG,GYRO_INIT_DATA);
}
void init_mag(int device){
	i2c_write(device,MAGNETO_ADDR,0x00,0x78);
	i2c_write(device,MAGNETO_ADDR,0x02,0x00);

}

inline int get_mag(int fileport, struct floatAxis * m,  struct floatAxis * bias){
	short rx,ry,rz;
	unsigned char tbuffer[6];

	i2c_write (fileport,MAGNETO_ADDR,0x03, I2C_NULL);
	i2c_read(fileport,MAGNETO_ADDR,tbuffer,6);

	rx=(((short)tbuffer[0]<<8) | tbuffer[1]);
	rz=(((short)tbuffer[2]<<8) | tbuffer[3]);
	ry=(((short)tbuffer[4]<<8) | tbuffer[5]);	
	
	if(rx==-4096||ry==-4096||rz==-4096){
		return -1;
	}else{
		m->x=-(float)rx*bias->x;    
		m->y=(float)ry*bias->y;
		m->z=(float)rz*bias->z;

	}
	
	return 0;

}

struct floatAxis get_magcalibration(int fileport){
	struct floatAxis rawRead;
	struct floatAxis outBias;
	struct floatAxis initBias;
	char msg_telemetria[TELEMETRY_MSG_SIZE];
    char biasMode=0x79; //0x11
	int intentos=0;
	int exitos=0;
	initBias.x=1;
	initBias.y=1;
	initBias.z=1;
	
	outBias.x=1;
	outBias.y=1;
	outBias.z=1;
	

	
	sprintf(msg_telemetria,"#Mag:[info] Comienzo calibrado");
	queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetria,1);
	
	while( exitos < 20 && intentos < 100 )
	{
      
      intentos++;
      i2c_write(fileport,MAGNETO_ADDR,0x00, biasMode);usleep (50); 
	  i2c_write(fileport,MAGNETO_ADDR,0x02, 0x01);usleep (50);             

	  usleep(6000);
	  get_mag(fileport, &rawRead,&initBias);
	  if( 	fabs(rawRead.x) > 900 && 
				fabs(rawRead.x) < 1400 && 
				fabs(rawRead.y) > 900 && 
				fabs(rawRead.y) < 1400 && 
				fabs(rawRead.z) > 900 && 
				fabs(rawRead.z) < 1400){
		  exitos ++;
	          outBias.x += fabs(rawRead.x);
	          outBias.y += fabs(rawRead.y);
	          outBias.z += fabs(rawRead.z);
	          
	  }
	 // printf("#Mag: %f\t%f\t%f",fabs(rawRead.x),fabs(rawRead.y),fabs(rawRead.z));
		
	}
	
	if (exitos>=1){
		sprintf(msg_telemetria,"#Mag:[info] Completada calibracion (%i)", exitos);
		queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetria,1);
		outBias.x=MAG_SCALE_X/(outBias.x/exitos);
		outBias.y=MAG_SCALE_Y/(outBias.y/exitos);
		outBias.z=MAG_SCALE_Z/(outBias.z/exitos);
	}else{
		sprintf(msg_telemetria,"#Mag:[wrn] Calibracion insuficiente. Ajuste por defecto (%i)", exitos);
		queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetria,1);
		outBias.x=1;
		outBias.y=1;
		outBias.z=1;
	}	
	sprintf(msg_telemetria,"#Mag:[info] Bias ajustadas a %f\t%f\t%f",outBias.x,outBias.y,outBias.z);
	queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetria,1);

	return outBias;
}

inline float get_course(struct floatAxis * mag,  float alabeo, float cabeceo, float decMAG){

  float MAG_X;
  float MAG_Y;
  float cos_alabeo;
  float sin_alabeo;
  float cos_cabeceo;
  float sin_cabeceo;

  cos_alabeo = cosf(alabeo);
  sin_alabeo = sinf(alabeo);
  cos_cabeceo = cosf(cabeceo);
  sin_cabeceo = sinf(cabeceo);
 
  //http://powet.eu/2011/03/19/tilt-compensation-azimuth-pitch-le-roll/?lang=en
  //http://www.ngdc.noaa.gov/geomagmodels/struts/calcDeclination*/
  MAG_X = mag->x * cos_cabeceo +  mag->z * sin_cabeceo;
  MAG_Y =  mag->x * sin_alabeo * sin_cabeceo +  mag->y * cos_alabeo -  mag->z * sin_alabeo * cos_cabeceo;

return(atan2(MAG_Y,MAG_X)+decMAG);
	
	
	
}

inline float angleInRadians(int range, int measured) {
  	return (float)(measured/(float)range) * PI;
}

inline unsigned int check_range_servo(int pwm){
	if (pwm>MAX_PWM){return MAX_PWM;}	
	if (pwm<MIN_NOMINAL_PWM){return MIN_NOMINAL_PWM;}
	return pwm;
}
inline unsigned char avoid_STRT_CHAR(unsigned char val){
	if (val==STRT_CHAR){
		val--;
	}
	return val;
}
inline unsigned char msb(unsigned int val){
	return (unsigned )val>>8;
}
inline unsigned char lsb(unsigned int val){
	unsigned int v;
	v=val<<8;
	v=v>>8;
	return (unsigned char)v;
}
inline unsigned int negar(int val){
	if(val==0){
		return 1;
	}else{
		return 0;
	}
}
inline float  lowPassFilter(float newResult ,float oldResult, float factor ){
	if (factor!=1){
		return (newResult*factor)+(oldResult*(1-factor));
	}else{
		return newResult;
	}
}
inline float ab(float v){
	if (v<0){
		return -v;
	}	
	return v;
	
}



inline struct pid  PID(struct Kpid K, struct pid p, float currAngle, float currAngVel, float targetAngle){//, int useAngVel, float dT, float lpfd){
	float error;	
	error=currAngle-targetAngle;
	if(K.i!=0){
		//p.i=p.i+(error*dT);
        p.i=p.i+error;
    	p.oi=K.i*p.i;
	}
	if(K.d!=0){
		//if (useAngVel==1){p.d=currAngVel;}else{p.d=(error-p.error)/dT;}
        p.d=(currAngVel+p.d1+p.d2)/2.8;
        //KdP=(K.d/dT)*p.d;
        p.od=K.d*p.d;
        p.d2=p.d1;
        p.d1=p.d;
	}	
    p.op=K.p*error;
	p.output=(p.op+p.oi+p.od);
	if(p.output<-K.imax){p.output=-K.imax;}
	if(p.output>K.imax){p.output=K.imax;}

	p.error=error;
	return p;	

}
inline struct pid  PIDd(struct Kpid K, struct pid p, double currAngle, double currAngVel, double targetAngle, int useAngVel, float dT){
        double error;
        double KiP=0;
        double KdP=0;

        error=currAngle-targetAngle;

        if(K.i!=0){
                p.i=p.i+(error*dT);

                if(p.i<-K.imax){p.i=-K.imax;}
                if(p.i>K.imax){p.i=K.imax;}

                //if(((p.i<0)&&(error>0))||((p.i>0)&&(error<0))){p.i=p.i/2;}

                KiP=K.i*p.i;
        }

        if(K.d!=0){
                if (useAngVel==1){p.d=currAngVel;}else{p.d=(error-p.error)/dT;}
                KdP=(K.d/dT)*p.d;
        }
        p.output=(K.p*error+KiP+KdP);
        p.error=error;
        return p;

}




void get_pidoutput(){
	int out=-1;
	out=(signed int)(get_value("/tmp/pidOUTPUT", -1)*1000000);
	if (out<0){out=-1;}

	
	pid_output=out;
}
struct intAxis zeroIAXISreg(){
	struct intAxis i;
	i.x=0;
	i.y=0;
	i.z=0;
	i.Telapsed=0.0;
	return i;
}
struct puerto zeroPort(){
	struct puerto i;
	i.gyro=-1;
	i.accel=-1;
	i.mag=-1;
	i.alt=-1;
	i.comp=-1;
	i.serie=-1;
	i.radio=-1;
	return i;
}
struct floatAxis zeroFAXISreg(){
	struct floatAxis i;
        i.x=0.0;
        i.y=0.0;
        i.z=0.0;
		i.Telapsed=0.0;		
	return i;
}
struct imu zeroIMUreg(){
	struct imu i;
	i.alabeo=0;
	i.cabeceo=0;
	i.ginnada=0;

	i.filter_cabeceo=0;
	i.filter_alabeo=0;
	i.filter_ginnada=0;
	i.altitud=0;
	i.v_alabeo=0;
	i.v_cabeceo=0;
	i.v_ginnada=0;
	i.last_v_ginnada=0;
	i.last_v_alabeo=0;
	i.last_v_cabeceo=0;
    i.a_ginnada=0;
	i.a_alabeo=0;
	i.a_cabeceo=0;
	
	i.offset_alabeo=0;
	i.offset_cabeceo=0;
	i.last_cabeceo=0;
	i.last_alabeo=0;
	i.last_ginnada=0;
	i.last_suelo=0;
	i.suelo=0;
	i.min_suelo=100;


	return i;
}
inline struct pid zeroPIDreg(){
	struct pid i;
	i.output=0;
	i.error=0;
	i.d=0;
	i.i=0;
    i.d1=0;
    i.d2=0;
    i.od=0;
    i.oi=0;
    i.op=0;

	return i;
}


inline void  getRawAccel( unsigned char * buffer, struct intAxis * rawAxis,unsigned char * an,unsigned char * bump ){

		if(buffer[2]!=255){
			rawAxis->x=(buffer[2]<<2)|((buffer[5]>>2)&0x03);
			rawAxis->y=(buffer[3]<<2)|((buffer[5]>>4)&0x03);		
			rawAxis->z=(buffer[4]<<2)|((buffer[5]>>6)&0x03);
			an[0]=buffer[0];
			an[1]=buffer[1];		
			bump[0]=~((buffer[5]>>0) & 0x01);
			bump[1]=~((buffer[5]>>1) & 0x01);

		}

}
inline int getRawGyro(unsigned char * buffer, struct floatAxis * rawAxis, struct floatAxis *baseAxis){
		int tX,tY, tZ;
        float YScale, XScale, ZScale;
		tY=(((buffer[4] >> 2) << 8))+buffer[1];
		tX=(((buffer[5] >> 2) << 8))+buffer[2];					
		tZ=(((buffer[3] >> 2) << 8))+buffer[0];

		//printf("SAMPLE %i %i %i ", tX, tY, tZ);
		if((tX!=0)&&(tY!=0)&&(tZ!=0)){
			if(baseAxis==NULL){
				rawAxis->x = (float)tX;
				rawAxis->y = (float)tY;
				rawAxis->z = (float)tZ;
				//printf(" %f \t %f \t %f",rawAxis->x,rawAxis->y,rawAxis->z);
				return 1;
			}		
			YScale=(buffer[3]&0x01) ? GYRO_SLOW_FACTOR : GYRO_FAST_FACTOR;
			XScale=(buffer[4]&0x02) ? GYRO_SLOW_FACTOR : GYRO_FAST_FACTOR;
			ZScale=(buffer[3]&0x02) ? GYRO_SLOW_FACTOR : GYRO_FAST_FACTOR;
			//printf(" %i   \t%i   \t %i\t %i",XScale, YScale,ZScale ,buffer[3]&1);
			rawAxis->x = ((float)tX-baseAxis->x)/(float)XScale; 
			rawAxis->y = ((float)tY-baseAxis->y)/(float)YScale;
			rawAxis->z = ((float)tZ-baseAxis->z)/(float)ZScale;

            rawAxis->x = ((float)tX-baseAxis->x)/(float)XScale; 
			rawAxis->y = ((float)tY-baseAxis->y)/(float)YScale;
			rawAxis->z = ((float)tZ-baseAxis->z)/(float)ZScale;            
				//printf(" %f \t %f \t %f",rawAxis->x,rawAxis->y,rawAxis->z);
			return 1;
		}
		
	    return 0;
}
void getScaledRawAccel(struct intAxis * rawAxis, struct intAxis * rawBaseAxis){

		
		#ifdef FIXED_ACCEL
			rawAxis->x-=CENTER_X;
			rawAxis->y-=CENTER_Y;
		#else
			rawAxis->x-=rawBaseAxis->x;
			rawAxis->y-=rawBaseAxis->y;
		#endif
		
		rawAxis->z-=CENTER_Z;
		rawAxis->y=-rawAxis->y;

		
}



inline void getAccelSample(int device , struct intAxis * rawAxis,struct intAxis * rawBaseAxis,unsigned char * an,unsigned char * bump ){
	//struct intAxis  newRawAxis;
	unsigned char buffer[6];

	get_wii(device,buffer);	
    if(buffer[2]!=255){
		getRawAccel(buffer,rawAxis, an, bump);		
		getScaledRawAccel(rawAxis, rawBaseAxis);  
        //printf("\r\n SAMP %i %i %i  ",rawAxis->x,rawAxis->y,rawAxis->z);     
        //fflush(stdout); 
    }
    
	/*if(buffer[2]!=255){
		getRawAccel(buffer,&newRawAxis, an, bump);		
		getScaledRawAccel(&newRawAxis, rawBaseAxis);
		printf(" SAMP %i %i %i B %i %i %i ",newRawAxis.x,newRawAxis.y,newRawAxis.z,rawBaseAxis->x,rawBaseAxis->y,rawBaseAxis->z);

		rawAxis->x+=(newRawAxis.x);
		rawAxis->y+=(newRawAxis.y);
		rawAxis->z+=(newRawAxis.z);		
	}else{
		rawAxis->x+=(rawAxis->x);
		rawAxis->y+=(rawAxis->y);
		rawAxis->z+=(rawAxis->z);		
	}*/	
}
inline void getMagSample(int device,struct floatAxis * rawAxis, struct floatAxis * biasAxis, struct floatAxis * offsetAxis ){
	struct floatAxis  lastRawAxis;	
	lastRawAxis.x=rawAxis->x;
	lastRawAxis.y=rawAxis->y;
	lastRawAxis.z=rawAxis->z;	
	get_mag(device, rawAxis,  biasAxis);	

	rawAxis->x+=offsetAxis->x;
	rawAxis->y+=offsetAxis->y;
	rawAxis->z+=offsetAxis->z;	
}

inline void getGyroSample(int device, struct floatAxis * rawAxis, struct floatAxis * rawBaseAxis){
	//struct floatAxis  lastRawAxis;
	struct floatAxis  newRawAxis;
	//int temp;
	unsigned char buffer[6];
	/*lastRawAxis.x=rawAxis->x;
	lastRawAxis.y=rawAxis->y;
	lastRawAxis.z=rawAxis->z;
	*/
	get_wii(device,buffer);	
    //getRawGyro(buffer,rawAxis,rawBaseAxis);
   // printf("\r\n %f \t %f \t %f ",rawAxis->x, rawAxis->y,rawAxis->z );
	if(getRawGyro(buffer,&newRawAxis,rawBaseAxis)==1){
        *rawAxis=newRawAxis;	
	}
    /*
		rawAxis->x=newRawAxis.x+lastRawAxis.x;
		rawAxis->y=newRawAxis.y+lastRawAxis.y;
		rawAxis->z=newRawAxis.z+lastRawAxis.z;		
	}
    else{
		rawAxis->x+=rawAxis->x;
		rawAxis->y+=rawAxis->y;
		rawAxis->z+=rawAxis->z;
	}
*/
}


inline void getAvgSample(struct intAxis * rawAxis, int samples){
	rawAxis->x=rawAxis->x/samples;
	rawAxis->y=rawAxis->y/samples;
	rawAxis->z=rawAxis->z/samples;
}
inline void getfAvgSample(struct floatAxis * rawAxis, int samples){
	rawAxis->x=rawAxis->x/samples;
	rawAxis->y=rawAxis->y/samples;
	rawAxis->z=rawAxis->z/samples;
}
void getGyroAccelCalibration(struct puerto *fdevice, struct floatAxis *baseGyro, struct intAxis *baseAccel, int reads){
	struct intAxis zeroI;
	struct floatAxis zeroF;
	struct floatAxis lastGyro;
	struct intAxis lastAccel;
	
	struct floatAxis generalAccel, generalGyro;
	char msg_telemetria[TELEMETRY_MSG_SIZE];
	int ac=0;
	
	zeroI.x=0;zeroF.x=0;
	zeroI.y=0;zeroF.y=0;
	zeroI.z=0;zeroF.z=0;
	baseGyro->x=0; lastGyro.x=0;
	baseGyro->y=0; lastGyro.y=0;
	baseGyro->z=0; lastGyro.z=0;
	baseAccel->x=0; lastAccel.x=0;
	baseAccel->y=0; lastAccel.y=0;
	baseAccel->z=0; lastAccel.z=0;
	
	getGyroSample(fdevice->gyro, baseGyro,NULL);
	getAccelSample(fdevice->accel,  baseAccel , &zeroI,analogs, bumpers );
	
	generalGyro.x=(float)baseGyro->x;
	generalGyro.y=(float)baseGyro->y;
	generalGyro.z=(float)baseGyro->z;	
	
	generalAccel.x=(float)baseAccel->x;
	generalAccel.y=(float)baseAccel->y;
	generalAccel.z=(float)baseAccel->z;

    
    int dummywd=0;

	for (ac=0; ac<reads;ac++){

		

		generalGyro=lastGyro;
		lastGyro.x=0;
		lastGyro.y=0;
		lastGyro.z=0;
		lastAccel.x=0;
		lastAccel.y=0;
		lastAccel.z=0;
		getGyroSample(fdevice->gyro, &lastGyro,NULL);
		getAccelSample(fdevice->accel,  &lastAccel , &zeroI,analogs, bumpers );
		
		/*generalGyro.x=(float)baseGyro->x;
		generalGyro.y=(float)baseGyro->y;
		generalGyro.z=(float)baseGyro->z;
		
		generalAccel.x=(float)baseAccel->x;
		generalAccel.y=(float)baseAccel->y;
		generalAccel.z=(float)baseAccel->z;*/
		
		generalGyro.x=lowPassFilter(lastGyro.x,generalGyro.x,0.001);
		generalGyro.y=lowPassFilter(lastGyro.y,generalGyro.y,0.001);
		generalGyro.z=lowPassFilter(lastGyro.z,generalGyro.z,0.001);
		//ioctl(fdwatchdog, WDIOC_KEEPALIVE, &dummywd);

		generalAccel.x=lowPassFilter(((float)lastAccel.x),generalAccel.x,0.001);
		generalAccel.y=lowPassFilter(((float)lastAccel.y),generalAccel.y,0.001);
		//generalAccel.z=lowPassFilter(abs(lastAccel.z),generalAccel.z,0.3);
		//baseAccel->z=lowPassFilter(abs(lastAccel.z),baseAccel->z,0.7);		
		//baseAccel->z=(baseAccel->x+baseAccel->y)/2;
		
		
	}
	
	baseGyro->x=(generalGyro.x);
	baseGyro->y=(generalGyro.y);
	baseGyro->z=(generalGyro.z);
	
	baseAccel->x=ceil(fabs(generalAccel.x));
	baseAccel->y=ceil(fabs(generalAccel.y));
	//baseAccel->z=ceil(generalAccel.z);
	
	
	
	sprintf(msg_telemetria,"#Gyro:[info]Calibracion   %.2f %.2f %.2f" ,baseGyro->x, baseGyro->y, baseGyro->z);
	queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetria,1);
	
	#ifndef FIXED_ACCEL
	sprintf(msg_telemetria,"#Acel:[info]Calibracion   %i %i %i" ,baseAccel->x, baseAccel->y, baseAccel->z);
	queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetria,1);
	#endif
	
}
void get_OnDemandRecalibration(struct puerto *fdevice, struct floatAxis *baseGyro, struct floatAxis *biasMag, int reads){
	struct intAxis dummyaccel;
	getGyroAccelCalibration(fdevice,baseGyro, &dummyaccel, reads);
	*biasMag=get_magcalibration(fdevice->mag);

}




void timeout_real(){
    if (timeout_deltaT>0){timeout_deltaT_overflows=current_window;}
	//timeout_deltaT_overflows+=timeout_deltaT;
	timeout_deltaT++;
}
int  UDPtelemetrySender(struct UDPConnection * endpoint ){
   int sock;
   
   struct hostent *hp;
   struct sockaddr_in server;
   if (endpoint->sock>0){
	close(endpoint->sock);
   }
   sock= socket(AF_INET, SOCK_DGRAM, 0);
   server.sin_family = AF_INET;

   server.sin_port = htons(endpoint->port);
   hp = gethostbyname(endpoint->destination_ip);
   if (hp==0) return 0;
   bcopy((char *)hp->h_addr, (char *)&server.sin_addr,hp->h_length);
   endpoint->srv=server;
   endpoint->sock=sock;
   return 1;
}
int  sendUDPMessage(struct UDPConnection * endpoint, char * msg){
	if (endpoint->sock<1){
		return -1;
	}
	return sendto(endpoint->sock,msg,strlen(msg),0,(const struct sockaddr *)&endpoint->srv,sizeof(struct sockaddr_in));
}
void systemOutput(char *command , char * output){
  FILE *fp;
  int status;
  char path[1035];
  fp = popen(command, "r");
  if (fp != NULL) {
	  while (fgets(path, sizeof(path)-1, fp) != NULL) {
		strcpy(output,path);
	  }  
  }
  pclose(fp);
}
void GPSreceiver(){
        int fd;
	char command[200];
	unsigned char * ptok;
	unsigned char  ents2[3]="";
	unsigned char  ents3[4]="";
	unsigned char  floats[8]="";
        int reads=0;
	unsigned int entero=0;
        unsigned char buffer[200]="";
	FILE *file = fopen ( SERIE, "r" );
	
	gps_mem[0]=0; gps_mem[1]=0; gps_mem[2]=0; gps_mem[3]=0; gps_mem[4]=0;  gps_mem[5]=0;
	while(1){
		if ( fgets ( buffer, 200, file ) != NULL ){
			ptok = strtok (buffer,",");
			 
			 if(strcmp(ptok,"$GPGGA")==0){
				ptok = strtok (NULL,",");
				gps_mem[0]=atof(ptok);
				ptok = strtok (NULL,",");

				strncpy(ents2,ptok,2);
				entero=atoi(ents2);
				strcpy(floats,ptok+2);
		                
				gps_mem[1]=(float)((signed int)entero+((double)atof(floats)/60));
				ptok = strtok (NULL,",");
				if(ptok[0]=='S'){
					gps_mem[1]=-gps_mem[1];
				}

				ptok = strtok (NULL,",");
				strncpy(ents3,ptok,3);
				entero=atoi(ents3);
				strcpy(floats,ptok+3);

				gps_mem[2]=(float)((signed int)entero+((double)atof(floats)/60));

				ptok = strtok (NULL,",");
				 if (ptok[0]=='W'){
					 gps_mem[2]=- gps_mem[2];
				 }
			
				ptok = strtok (NULL,",");
				gps_mem[4]=atoi(ptok);//level signal
				ptok = strtok (NULL,",");
				ptok = strtok (NULL,",");
				ptok = strtok (NULL,",");
				gps_mem[3]=atof(ptok);//altitud
				
			}else{
				gps_mem[4]=0;
			}
		}

	}

	
}






float shortestAngle(float target, float current){
    float res=target;


    res=target;
    if ((target-current)>PI){
            res=current-((PI-target)+(PI+current));
    }
    if ((target-current)<-PI){
            res=current+((PI+target)+(PI-current));
    }

    return res;
}


float currentAnguloContinuo(float current, float target){

	
	if((target<=-PI_DIV_2)&&(current>=PI_DIV_2)){
			return -PI_MUL_2+current;
	}
	if((target>=PI_DIV_2)&&(current<=-PI_DIV_2)){
			return PI_MUL_2+current;
	}

	
	return current;
}



float *scaleUpper(float *values,int size, float maxValue){
        float upper=0;
        float acAbs=0;
        float scale=0;
        int ac;
        for (ac=0;ac<size;ac++){
                acAbs=abs(values[ac]);
                if (acAbs>=upper){upper=acAbs;}
        }
        if(upper>maxValue){
                scale=maxValue/upper;
                for (ac=0;ac<size;ac++){
                        values[ac]=values[ac]*scale;
                }
        }
        return values;

}


int UDPcommandServer(){
   char  buffer[MEM]="";	
   int sock, length, n;
   socklen_t fromlen;
   struct sockaddr_in server;
   struct sockaddr_in from;

   sock=socket(AF_INET, SOCK_DGRAM, 0);
   length = sizeof(server);
   bzero(&server,length);
   server.sin_family=AF_INET;
   server.sin_addr.s_addr=INADDR_ANY;
   server.sin_port=htons(UDP_PORT);
   bind(sock,(struct sockaddr *)&server,length);
   fromlen = sizeof(struct sockaddr_in);

   printf("#Cmd:Esperando conexiones. Socket %i ",sock);
   return sock;

   
}
short is_magwindow(int mag_window){
	if ((mag_window==1)&&(yaw_control==2)){
		return 1;
	}
	return 0;
}
 



/**********************************************************************************************************/

int main(int argc, int argv[]){
	char msg_telemetria[TELEMETRY_MSG_SIZE]="";
    char msg_recv[TELEMETRY_MSG_SIZE]="";
	char shell_command[200]="";
	unsigned char  buffer_serie[BUFFER_SERIE]="";
	char save_gps=1;
	float *gpsScaled;
	float gpsToScale[3];
	float yawCompass=0;
	float lastYawCompass=0;
    
    int dummywd=0;
    int window_rounds[4];
    int fdUDPRecv=0;
    int limitYaw=0;
    int limitedYaw=0;


	struct imu error;

	struct itimerval i_int;
	struct timeval s_int;
	struct timeval s_tstart;
	struct timeval s_tend;
	long  ac=0;
	
	int   result;
	
	char  generic_buffer[250];
	char  udp_telemetry[550]="TINIT_TELEMETRY#";
	int fault=0;
	
	struct pid ctrlAlabeo;
	struct pid ctrlAlabeo_v;
	struct pid ctrlCabeceo;
	struct pid ctrlCabeceo_v;
	struct pid ctrlGinnada;
	struct pid ctrlGinnada_v;
	struct pid ctrlAltitud;
	struct pid ctrlLongitud;
	struct pid ctrlLatitud;

	struct floatAxis rawMag;
	struct floatAxis lastRawMag;
	struct floatAxis biasMag;
	
	
	struct intAxis rawAccel;
	struct intAxis lastRawAccel;
	struct intAxis rawBaseAccel;
	
	
	struct floatAxis rawGyro;
	struct floatAxis rawBaseGyro;
	struct floatAxis lastRawGyro;
	
	struct floatAxis radAccel;	

	
	
	unsigned char window_dT=0;
	unsigned char accel_window=0;
	unsigned char mag_window=0;
	unsigned char tele_window=0;
	unsigned char mag_sample=0;

	float radxdeltaTM=0;
    fdwatchdog=stop_kernel_watchdog();
    system("killall httpd 2>/dev/null");

	calibration_reads=CALIBRATION_READS;
	if (argc>1){calibration_reads=atoi((const char *)argv[1]);}
	if (calibration_reads<=0){
		
		calibration_reads=CALIBRATION_READS;
	}
	
	//lastOrder[0]='\0';
	sprintf(msg_telemetria,"#Sys:[info]Arranque IMU **PID ANG 1**(UDP,DT: %f Cals: %i )",DELTA_T,calibration_reads);
	queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetria,1);
	udpSendConn.sock=-1;

	system("rm -Rf /tmp/gps_start");

	system("cp -Rf /scripts/config/config* /tmp/");
	system("stty -F /dev/ttyS0 raw  >/dev/null");
	system("stty -F /dev/ttyS0 raw  >/dev/null");
	system("stty -F /dev/ttyS0 speed 115200  >/dev/null");
	
	system("killall udhcpc 2>/dev/null");
	system("killall syslogd 2>/dev/null");
	system("killall output 2>/dev/null");
	system("killall input 2>/dev/null");
	
	system("killall dnsmasq 2>/dev/null");
	system("killall hotplug2 2>/dev/null");
	system("killall wifi_survey.sh 2>/dev/null");
	system("killall nice 2>/dev/null");
	
	system("killall httpd 2>/dev/null");
	system("stty -F /dev/ttyS0 speed 115200  >/dev/null");

	//real_time_scheduling(1);
	//usleep(1000);
    
	 /*INIT************************************************************************************/


	fault=0;


	rawAccel=zeroIAXISreg();	rawBaseAccel=zeroIAXISreg();	rawGyro=zeroFAXISreg();			rawBaseGyro=zeroFAXISreg(); 	rawMag=zeroFAXISreg(); biasMag=zeroFAXISreg();
	radAccel=zeroFAXISreg();	offsetMag=zeroFAXISreg();
	ctrlAlabeo=zeroPIDreg();	ctrlCabeceo=zeroPIDreg();		ctrlGinnada=zeroPIDreg();	ctrlAltitud=zeroPIDreg();
	ctrlAlabeo_v=zeroPIDreg();  ctrlCabeceo_v=zeroPIDreg();     ctrlGinnada_v=zeroPIDreg();
	ctrlLongitud=zeroPIDreg();	ctrlLatitud=zeroPIDreg();	ctrlAltitud=zeroPIDreg();
	sensor=zeroIMUreg();		error=zeroIMUreg();				target=zeroIMUreg();	mando=zeroIMUreg();
	fdevice=zeroPort();
	
	biasMag.x=1; 	biasMag.y=1;	biasMag.z=1;
	offsetMag.x=0; 	offsetMag.y=0; 	offsetMag.z=0;
 	
	sprintf(generic_buffer,"%i",getpid());

	buffer_to_file(generic_buffer,5,"/tmp/pidIMU");


	get_config(SIGUSR2);
	

    
	motor[0]=0x00;		
    motor[1]=0x00;		
    motor[2]=0x00;		
    motor[3]=0x00;		
    motor[4]=MED_PWM;
    motor_enabled[0]=0x01;
    motor_enabled[1]=0x01;
    motor_enabled[2]=0x01;
    motor_enabled[3]=0x01;
    motor_enabled[4]=0x01;

	analogs[0]=0x00;	analogs[1]=0x00;	bumpers[0]=0x00;	bumpers[1]=0x00;	port[0]=0x00;	

	
	buffer_serie[0]=STRT_CHAR;
	last_buffer_serie[0]=0x00;
	shared_init();

	
	
	queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[info]Desplegando recepcion UDP no bloqueante",1);
	fdUDPRecv=UDPcommandServer();
	
	
	#ifdef GPS_TRACKING
	
	queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[info]Desplegando recepcion GPS paralela",1);
    if (pidGPS=fork()==0){GPSreceiver();}
	#endif

    real_time_scheduling(1);
	queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[info]Abriendo dispositivos",1);

        //gps_mem[0] UTC Time (3decimales)
        //gps_mem[1] Latitud grados (6 decimales)
        //gps_mem[2] Longitud grados (6 decimales)
        //gps_mem[3] Altitud metros (2 decimales)
        //gps_mem[4] Calidad recepcion (entero) (>=1-> ok)
        //sleep (1);
        //printf("GPS DATA: RECV: %f %f %f %f %f ",gps_mem[0],gps_mem[1],gps_mem[2],gps_mem[3],gps_mem[4]);

	f_serie=open(SERIE, O_WRONLY);
	f_i2c0=open(I2C0, O_RDWR);
	f_i2c1=open(I2C1, O_RDWR);
	
	
	
	if ((f_i2c0<=0)||(f_i2c1<=0)||(f_serie<=0)){
		sprintf(msg_telemetria,"#Sys:[critical]ERROR de E/S %i / %i / %i",f_i2c0,f_i2c1,f_serie);//exit(-1);
		queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetria,1);
	}
	
	fdevice.gyro=f_i2c1;
	fdevice.accel=f_i2c0;
	fdevice.serie=f_serie;
	fdevice.comp=f_i2c0;
	fdevice.radio=f_i2c0;
	fdevice.mag=f_i2c0;
	
	init_gyro(fdevice.gyro);	
	init_accel(fdevice.accel);
	
	
	

	(void) signal(SIGUSR2, get_config);
	(void) signal(SIGUSR1, get_pidoutput);

	
	buffer_serie[0]=STRT_CHAR;
	send_serial(buffer_serie, 1);

	/*****************************************************************************************/
	ac=0;
	biasMag=get_magcalibration(fdevice.mag);

	queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[info]Inicializando Magnetometro para medicion continua",1);
	init_mag(fdevice.mag);
	queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[info]Lanzando calibracion Gyro+Accel",1);
	getGyroAccelCalibration(&fdevice, &rawBaseGyro,&rawBaseAccel, calibration_reads);

	strcpy(write_mem,"EXIT");
	sleep(1);

	strcpy(read_mem,"");
	target.altitud=-1000;
	ctrlAltitud.output=target.altitud;
	
	serial[0]=0xFF;
	serial[5]=0x00;

	get_mag(fdevice.mag, &rawMag,  &biasMag);			
	yawCompass=get_course(&rawMag, 0, 0,declinacionMAG);
	lastYawCompass=yawCompass;
	sensor.ginnada=yawCompass;
	sensor.last_ginnada=yawCompass;

	lastRawGyro.x=lastRawGyro.x;
	lastRawGyro.y=lastRawGyro.y;
	lastRawGyro.z=lastRawGyro.z;
	
	lastRawAccel.x=rawAccel.x;
	lastRawAccel.y=rawAccel.y;
	lastRawAccel.z=rawAccel.z;
	
	lastRawMag.x=rawMag.x;
	lastRawMag.y=rawMag.y;
	lastRawMag.z=rawMag.z;
	
	



	
	s_int.tv_sec=0;
	s_int.tv_usec=1000000*DELTA_T;
	i_int.it_interval=s_int;
	i_int.it_value=s_int;
	
	
	int fromlen=sizeof(struct sockaddr_in);
	strcpy(udp_telemetry, "TINIT_ATROPOS#");



	radxdeltaTM=DEG_TO_RAD*DELTA_T;

    
    
	sprintf(msg_telemetria,"\r\n#Sys:[info]READY(Course %.2f)",yawCompass*RAD_TO_DEG);	
	queueMessage(colaMensajes, &nextMsgToQueue,msg_telemetria,1);

	setitimer(ITIMER_REAL,&i_int,NULL);
	(void)signal(SIGALRM ,timeout_real);	
	while(1 == 1){

		if(timeout_deltaT>0){
		    
            lastRawGyro=rawGyro;
            //rawAccel=zeroIAXISreg();	
         	//rawGyro=zeroFAXISreg();
			getGyroSample(fdevice.gyro, &rawGyro,&rawBaseGyro);
			

			mag_window=0;
			accel_window=0;
			switch(window_dT){
				case WINDOW_ROUND_1:
					mag_window=is_magwindow(1);
                    //mag_window=1;
                    current_window=1;
				break;				
				case WINDOW_ROUND_2:
                    //mag_window=1;
					mag_window=is_magwindow(1);
                    current_window=2;
				break;
				case WINDOW_ROUND_3:
					if (tele_window==0){tele_window=1;}
                    current_window=3;
                    window_dT=0;
                break;    
				//case WINDOW_ROUND_4:
                //    ioctl(fdwatchdog, WDIOC_KEEPALIVE, &dummywd);
                //    window_dT=0;
                //    current_window=4;
                //break;	
				default:
					accel_window=1;
                    current_window=99;
				break;
			}
			if ((mag_window==0)&&(tele_window==0)){accel_window=1;}

			if (mag_window==1){		
				getMagSample(fdevice.mag, &rawMag,  &biasMag , &offsetMag);
				if((abs(rawMag.x)>850)||(abs(rawMag.y)>850)||(abs(rawMag.z)>850)){
                    if(yaw_control==2){yaw_control=1;}
					queueMessage(colaMensajes, &nextMsgToQueue,"#Mag:[error]Overflow",0);

				}else{
					yawCompass=get_course(&rawMag, sensor.cabeceo, -sensor.alabeo,declinacionMAG);
				}
				if(!is_NaN(yawCompass)){					
					Drift_correction_mag(yawCompass);
				}else{
					if(yaw_control==2){yaw_control=1;}
					queueMessage(colaMensajes, &nextMsgToQueue,"#Mag:[error]NaN error",0);

				}
				
				
			}
			if(accel_window==1){
                getAccelSample(fdevice.accel,  &rawAccel , &rawBaseAccel,analogs, bumpers );
				accel_window=0;
            
				rawAccel.x=lowPassFilter(rawAccel.x,lastRawAccel.x,lowPassFactorAccel);
				rawAccel.y=lowPassFilter(rawAccel.y,lastRawAccel.y,lowPassFactorAccel);
				rawAccel.z=lowPassFilter(rawAccel.z,lastRawAccel.z,lowPassFactorAccel);
                
				/*lastRawAccel.x=rawAccel.x;
				lastRawAccel.y=rawAccel.y;
				lastRawAccel.z=rawAccel.z;	*/
                lastRawAccel=rawAccel;	
				radAccel.x = angleInRadians(RANGE_X, rawAccel.x);
				radAccel.y = angleInRadians(RANGE_Y, rawAccel.y);
				radAccel.z = angleInRadians(RANGE_Z, rawAccel.z);
				Drift_correction_ac(&radAccel);
                
			}  	


            if(recv(fdUDPRecv, msg_recv, TELEMETRY_MSG_SIZE, MSG_DONTWAIT)!=-1){

                strcpy(read_mem,msg_recv);
                roundCommand=0;
            }
			if(read_mem[0]!=0x00){
				get_order(read_mem);		
				read_mem[0]=0x00;
			}

			//getGyroSample(fdevice.gyro, &rawGyro,&rawBaseGyro);
			//getAccelSample(fdevice.accel,  &rawAccel , &rawBaseAccel,analogs, bumpers );			
			
			//getfAvgSample(&rawGyro,2);
			//getAvgSample(&rawAccel,2);

            rawGyro.x=lowPassFilter(rawGyro.x,lastRawGyro.x,lowPassFactorGyro);
            rawGyro.y=lowPassFilter(rawGyro.y,lastRawGyro.y,lowPassFactorGyro);
            rawGyro.z=lowPassFilter(rawGyro.z,lastRawGyro.z,lowPassFactorGyro);

            /**
            rawGyro.x=0;
            rawGyro.y=0;
            rawGyro.z=0;
            
            
            
            */
			// GYRO X[0]-> ROLL-> ALABEO
			// GYRO Y[1]-> PITCH-> CABECEO
			// GYRO Z[2]-> YAW-> GUIÑADA
			sensor.filter_alabeo=(((float)(rawGyro.x))*DEG_TO_RAD)*DELTA_T;///radxdeltaTM;
			sensor.filter_cabeceo=(((float)(rawGyro.y))*DEG_TO_RAD)*DELTA_T;//radxdeltaTM;
			sensor.filter_ginnada=(((float)(rawGyro.z))*DEG_TO_RAD)*DELTA_T;//radxdeltaTM;
	
			actualizarMatrizDCM(&sensor);							
			renormalizar(DCM_Temporal_Matriz, DCM_Matriz);	
			actualizarMatrizDCMError();						      
			renormalizar(DCM_Temporal_Matriz, DCM_Matriz);	

            
          
            
            
			sensor.alabeo=  atan2f(DCM_Matriz[2][1], DCM_Matriz[2][2])+sensor.offset_alabeo;
			sensor.cabeceo= asinf(-DCM_Matriz[2][0])+sensor.offset_cabeceo;
			sensor.ginnada= -atan2f(DCM_Matriz[1][0], DCM_Matriz[0][0]);

			sensor.v_alabeo=(sensor.alabeo-sensor.last_alabeo)/DELTA_T;
			sensor.v_cabeceo=(sensor.cabeceo-sensor.last_cabeceo)/DELTA_T;
			//sensor.v_ginnada=(sensor.ginnada-currentAnguloContinuo(sensor.last_ginnada, sensor.ginnada))/DELTA_T;	
            /*	
            sensor.v_alabeo=rawGyro.x*DEG_TO_RAD;
            sensor.v_cabeceo=rawGyro.y*DEG_TO_RAD;
            */
            sensor.v_ginnada=rawGyro.z*DEG_TO_RAD;			
            
			sensor.a_ginnada=(sensor.v_ginnada-sensor.last_v_ginnada)/DELTA_T;
			sensor.a_cabeceo=(sensor.v_cabeceo-sensor.last_v_cabeceo)/DELTA_T;
			sensor.a_alabeo=(sensor.v_alabeo-sensor.last_v_alabeo)/DELTA_T;

			sensor.last_alabeo=sensor.alabeo;
			sensor.last_cabeceo=sensor.cabeceo;
			sensor.last_ginnada=sensor.ginnada;
			sensor.last_v_ginnada=sensor.v_ginnada;
			sensor.last_v_alabeo=sensor.v_alabeo;
			sensor.last_v_cabeceo=sensor.v_cabeceo;
			
			if (is_NaN(sensor.alabeo)||is_NaN(sensor.cabeceo)||is_NaN(sensor.ginnada)){
				reset_Matrices_globales();
				ctrlAlabeo=zeroPIDreg();
				ctrlCabeceo=zeroPIDreg();
				ctrlGinnada=zeroPIDreg();
				ctrlAltitud=zeroPIDreg();
				ctrlAlabeo_v=zeroPIDreg();  ctrlCabeceo_v=zeroPIDreg();     ctrlGinnada_v=zeroPIDreg();
			
				ctrlLongitud=zeroPIDreg();
				ctrlLatitud=zeroPIDreg();
				ctrlAltitud=zeroPIDreg();
		
				sensor=zeroIMUreg();
				error=zeroIMUreg();
				target=zeroIMUreg();
				mando=zeroIMUreg();
				
				queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[error]ERROR NaN",1);
			}

            #ifdef GPS_TRACKING
			statusGPS=(short)gps_mem[4];
			if (lastGPSClock==gps_mem[0]){
				roundGPS++;				
			}else{
				lastGPSClock=gps_mem[0];
				roundGPS=0;
			}
			if (roundGPS>TIMEOUT_GPS){
				statusGPS=-1;
				roundGPS=TIMEOUT_GPS;
				if (gps_mem[0]>0){
					if (pidGPS>0){
						kill(pidGPS,SIGKILL);
					}
					
					if (pidGPS=fork()==0){GPSreceiver();}
					queueMessage(colaMensajes, &nextMsgToQueue,"#GPS:[info]System reload",1);

					gps_mem[0]=0;
					
				}
			}
			
			//autopilot
			if ((autopilot>0)&&(statusGPS>0)){

				//ojo! relativo a lat / long y no al morro 
				targetGPS.latitud=targetGPS.latitud-(mando.cabeceo*DELTA_T);
				targetGPS.longitud=targetGPS.longitud-(mando.alabeo*DELTA_T);
				
				
				//ojo! revisar velocidad
				ctrlLatitud=PID(Klatitud,ctrlLatitud,gps_mem[1],0, targetGPS.latitud,0,DELTA_T,1);
				ctrlLongitud=PID(Klongitud,ctrlLongitud,gps_mem[2],0, targetGPS.longitud,0,DELTA_T,1);
				
				gpsToScale[0]=ctrlLatitud.output;
				gpsToScale[1]=ctrlLongitud.output;	
				
				gpsScaled=scaleUpper(gpsToScale,2,targetGPS.max_angle);

				target.alabeo= cosf(sensor.ginnada)*(*(gpsScaled+1))+sinf(sensor.ginnada)*(*gpsScaled);
			    target.cabeceo=cosf(sensor.ginnada)*(*gpsScaled)+sinf(sensor.ginnada)*(*(gpsScaled+1));
				
				//printf(" GPS: LongT: %f LatT: %f ELongT: %f ELatT: %f Al: %f Cab: %f", targetGPS.longitud,targetGPS.latitud, *(gpsToScale+1), *(gpsToScale),target.alabeo,target.cabeceo );

				if (autopilot==1){target.altitud=mando.altitud;}
			}else{
            #endif
				if (encendido==1){
					target.alabeo=mando.alabeo;
					target.cabeceo=mando.cabeceo;
					target.altitud=mando.altitud;
				}
            #ifdef GPS_TRACKING
				if (statusGPS>0){
	                targetGPS.latitud=gps_mem[1];
                    targetGPS.longitud=gps_mem[2];
                    targetGPS.altitud=gps_mem[3];
				}

                				
			}
            #else
                statusGPS=0;
            #endif
			roundCommand++;
			if (roundCommand>TIMEOUT_COMMAND){
				roundCommand=TIMEOUT_COMMAND;
                if((autopilot==0)||(statusGPS==0)){
    				apply_failsafe_behaviour(read_mem, target.altitud*1000);
	    			//printf("#Sys: Failsafe FIRED!");
                        
       				failsafe=1;
                }    
			}else{
				failsafe=0;
			}
            roundCommandTele=roundCommand;
            
			if (encendido==1){
				target.v_ginnada=mando.v_ginnada;
				target.altitud=mando.altitud;//quitar cuando haga el autopilot>1
		
			    //fin autopilot}
                if(ctrlAltitud.output>100){
                    if(yaw_delay_timer>=0.5){
                        yaw_control=target_yaw_control;
                    }else{
                        yaw_delay_timer+=DELTA_T;
                        yaw_control=0;
                    }    
                    
                }else{
                    yaw_control=0;
                    yaw_delay_timer=0;
                }

				ctrlAlabeo=     PID(Kalabeo_out,    ctrlAlabeo,  sensor.alabeo,     sensor.v_alabeo,   target.alabeo);//, 1,DELTA_T);
                ctrlCabeceo=    PID(Kcabeceo_out,   ctrlCabeceo, sensor.cabeceo,    sensor.v_cabeceo,  target.cabeceo);//, 1,DELTA_T);
                
                switch(yaw_control){
                    case 0:
                        ctrlGinnada_v=zeroPIDreg();
                        target.v_ginnada=sensor.v_ginnada;
                        target.ginnada=sensor.ginnada;
                    break;
                    case 1:
                        ctrlGinnada_v=  PID(Kginnada_v,  ctrlGinnada_v,   sensor.v_ginnada, sensor.a_ginnada,target.v_ginnada);
                    break;
                    case 2:    
                        target.ginnada=shortestAngle(target.ginnada+(target.v_ginnada*(DELTA_T)/2),sensor.ginnada);
                        ctrlGinnada=    PID(Kginnada,    ctrlGinnada,     sensor.ginnada,   sensor.v_ginnada , target.ginnada);
                        ctrlGinnada_v=  PID(Kginnada_v,  ctrlGinnada_v,   sensor.v_ginnada, sensor.a_ginnada,  -ctrlGinnada.output);
                    break;    
                }                
				ctrlAlabeo_v=   PID(Kalabeo_in,  ctrlAlabeo_v,  sensor.v_alabeo,  sensor.a_alabeo,  -ctrlAlabeo.output);  //target.alabeo);//, 1,DELTA_T);
                ctrlCabeceo_v=  PID(Kcabeceo_in, ctrlCabeceo_v, sensor.v_cabeceo, sensor.a_cabeceo, -ctrlCabeceo.output); //target.cabeceo); //,1,DELTA_T);
 

		        
				ctrlAltitud.output=(target.altitud);

                if(ctrlAltitud.output<60){
                    ctrlAlabeo_v=zeroPIDreg();
                    ctrlCabeceo_v=zeroPIDreg();
                }
                //YAW_LIMIT
                //limitYaw=ctrlAltitud.output*yawProportionLimit;
                

                
                limitedYaw=ctrlGinnada_v.output;
                //if(limitedYaw<(-limitYaw)){
                //    limitedYaw=-limitYaw;
                //}
                //if(limitedYaw>limitYaw){
                //    limitedYaw=limitYaw;
                //}

                
				/*
					 3 (-g)
					 |
				1---------2
					 |
					 0 (-g)
				*/
			
				motor[1]=check_range_servo((long)(-ctrlAlabeo_v.output)+(limitedYaw)+(ctrlAltitud.output)+motor_offset[1]);
				motor[2]=check_range_servo((long)(ctrlAlabeo_v.output)+(limitedYaw)+(ctrlAltitud.output)+motor_offset[2]);
				
				motor[0]=check_range_servo((long)(ctrlCabeceo_v.output)+(-limitedYaw)+(ctrlAltitud.output+motor_offset[0]));
				motor[3]=check_range_servo((long)(-ctrlCabeceo_v.output)+(-limitedYaw)+(ctrlAltitud.output)+motor_offset[3]);

				save_gps=1;
				
			}else{	
			    failsafe=0;
				motor[0]=0x00;
				motor[1]=0x00;
				motor[2]=0x00;
				motor[3]=0x00;
				target.ginnada=sensor.ginnada;
				autopilot=0;
				targetGPS.latitud=gps_mem[1];
				targetGPS.longitud=gps_mem[2];
				targetGPS.altitud=gps_mem[3];

				ctrlAlabeo=zeroPIDreg();
				ctrlCabeceo=zeroPIDreg();
				ctrlGinnada=zeroPIDreg();
				ctrlAltitud=zeroPIDreg();
                
				ctrlAlabeo_v=zeroPIDreg();
				ctrlCabeceo_v=zeroPIDreg();
				ctrlGinnada_v=zeroPIDreg();                

				if ((save_gps==1)&&(statusGPS>0)){
					save_gps=0;
					sprintf(shell_command,"echo %f,%f,%f,%f > /tmp/gps_start", gps_mem[1],gps_mem[2],gps_mem[3],gps_mem[4]); 
					systemOutput(shell_command, NULL);
				}			
			}
            
            if(accel_window==0){
			    if (escSpecialCom>=0){
				    motor[0]=escSpecialCom;
				    motor[1]=escSpecialCom;
				    motor[2]=escSpecialCom;
				    motor[3]=escSpecialCom;
			    }

			    if((motor[0]>500)&&(motor[1]>500)&&(motor[2]>500)&&(motor[3]>500)){
				    motor[0]=0;motor[1]=0;motor[2]=0;motor[3]=0;
			
				    queueMessage(colaMensajes, &nextMsgToQueue,"#Sys:[critical]ERROR Motor overflow",1);
				
			    }			
                for (ac=0;ac<SERVOS;ac++){
                    if (motor_enabled[ac]==0){
                        motor[ac]=0;
                    }
                }	
            }


			if(tele_window>0){
				
				switch(tipoTelemetria){
					case 0:
						if((pid_output>0)&&(tele_window==1)){
											                     // 0   1   2    3    4   5   6 7  8  9 10 11 12 13 14 15 161718192021 22  23 24   25   26 27 28
							sprintf(write_mem,"<script>comet.input('%i;%i;%.3f;%.3f;%.3f;%.3f;0;%i;0;0;%i;%i;%i;%i;0;%i;0;0;0;0;0;%i;0;%.4f;%i;%.3f;%.3f;%s;%i;%.3f;%.3f;%.3f');</script>",
								analogs[0], //0
								analogs[1], //1
								sensor.alabeo, //2
								sensor.cabeceo, //3
								sensor.v_ginnada, //4
								sensor.suelo,  //5
								//gps_mem[3], //sensor.altitud, //6
								encendido, //7
								//bumpers[0],  //8
								//bumpers[1],  //9
								motor[0],    //10
								motor[1],    //11
								motor[2],    //12
								motor[3],    //13
								//motor[4],    //14
								timeout_deltaT_overflows, //15
								//statusGPS, //estado gps  //16 
								//gps_mem[1], //lat actual  //17
								//gps_mem[2], //long actual //18
								//targetGPS.latitud,  //19
								//targetGPS.longitud,  //20
								failsafe, //21
								//autopilot, //22
								target.v_ginnada, //23 volver a .ginnada
								(target_yaw_control<<1)+yaw_control, //24,
								target.alabeo, //25
								target.cabeceo,  //26
								colaMensajes[nextMessage(colaMensajes,&nextMsgToSend)],//27
								roundCommandTele, //28
                                (float)ctrlGinnada_v.op,
                                (float)ctrlGinnada_v.oi,
                                (float)ctrlGinnada_v.od
								);
								tele_window=2;
						}
						if (tele_window==2){
							kill(pid_output, SIGUSR2);
							tele_window=0;	
						}
					break;
					case 1:
						sprintf(udp_telemetry,"G%.0f;%.0f;%.0f;A%.0f;%.0f;%.0f;M%.2f;%.2f;%.2f;D%.2f;%.2f;%.2f;I%.2f;;;;", 
								(float)rawGyro.x,
								(float)rawGyro.y,
								(float)rawGyro.z, 
								(float)rawAccel.x,
								(float)rawAccel.y,
								(float)rawAccel.z, 
								(float)lastRawMag.x,
								(float)lastRawMag.y, 
								(float)lastRawMag.z, 
								sensor.cabeceo*RAD_TO_DEG, 
								sensor.alabeo*RAD_TO_DEG, 
								sensor.ginnada*RAD_TO_DEG, 
								yawCompass*RAD_TO_DEG
								);	
						
						sendUDPMessage(&udpSendConn, udp_telemetry);
						tele_window=0;
					break;
					case 2:
							printf("%i;%i;%.2f;%.2f;%.2f;%.2f;%.2f;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%f;%f;%f;%f;%i;%i",
								analogs[0], //0
								analogs[1], //1
								sensor.alabeo, //2
								sensor.cabeceo, //3
								sensor.ginnada, //4
								sensor.suelo,  //5
								gps_mem[2], //sensor.altitud, //6
								encendido, //7
								bumpers[0],  //8
								bumpers[1],  //9
								motor[0],    //10
								motor[1],    //11
								motor[2],    //12
								motor[3],    //13
								motor[4],    //14
								timeout_deltaT_overflows, //15
								statusGPS, //estado gps  //16 
								gps_mem[1], //lat actual  //17
								gps_mem[2], //long actual //18
								targetGPS.latitud,  //19
								targetGPS.longitud,  //20
								failsafe, //21
								autopilot //22
								
								);
							tele_window=0;	
					break;
				}
                timeout_deltaT_overflows=0;                
				
			}
		    


			serial[0]=0xFF;
			serial[1]=(unsigned char)(msb(motor[0])+(msb(motor[1])<<1)+(msb(motor[2])<<2)+(msb(motor[3])<<3));
			serial[2]=(unsigned char)avoid_char(lsb(motor[0]),0xFF);
			serial[3]=(unsigned char)avoid_char(lsb(motor[1]),0xFF);
			serial[4]=(unsigned char)avoid_char(lsb(motor[2]),0xFF);
			serial[5]=(unsigned char)avoid_char(lsb(motor[3]),0xFF);
			send_serial(serial, 6);
			
			window_dT++;
			timeout_deltaT=0;

		}

		sleep(1);
	}
	
}
