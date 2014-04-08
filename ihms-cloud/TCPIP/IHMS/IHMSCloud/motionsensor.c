#include "main.h"
#include <plib.h>
#include <math.h>

//#define GetSystemClock()	(40000000ul)                // 40MHz is set by Max Choi, but the system clock should be 80MHz according to MCU specification.
#define GetInstructionClock()	(GetSystemClock()/1)
#define GetPeripheralClock()	(GetInstructionClock()/1)	// Set your divider according to your Peripheral Bus Frequency configuration fuse setting
#define I2C_CLOCK_FREQ          400000

// EEPROM Constants
#define sensor_I2C_BUS              I2C5
#define sensor_ADDRESS              0x1C       // The MMA8451Q's standard slave address is a choice between the two sequential addresses 0011100 - 0x1C and 0011101 - 0X1D. The
                                                //selection is made by the high and low logic level of the SA0 (pin 7)

#define WINDOW_LENGTH 1000


BYTE sensortmpdata[7];  //for reading MMA845X Q  Data Registers from 0x00 to 0x06


#define PI 3.1415926535898
#define WINDOW_LENGTH 200
#define DATA_WINDOW_LENGTH 1250
#define COEFFICIENT (2*PI/WINDOW_LENGTH)
#define PEAK_WINDOW_LENGTH 12
#define SMOOTH_WINDOW_LENGTH 5
#define  RESULT_WINDOW_LENGTH 5

BYTE ResultSave[RESULT_WINDOW_LENGTH];
int PeakSave[RESULT_WINDOW_LENGTH];
float SmoothWindowX[SMOOTH_WINDOW_LENGTH];
float SmoothWindowY[SMOOTH_WINDOW_LENGTH];
float SmoothWindowZ[SMOOTH_WINDOW_LENGTH];
float SmoothSaveX[SMOOTH_WINDOW_LENGTH];
float SmoothSaveY[SMOOTH_WINDOW_LENGTH];
float SmoothSaveZ[SMOOTH_WINDOW_LENGTH];
float slip[50];
int PeakIndex;
float peak;
float TmpX[PEAK_WINDOW_LENGTH];
float TmpY[PEAK_WINDOW_LENGTH];
float TmpZ[PEAK_WINDOW_LENGTH];
float SlipWindowX[DATA_WINDOW_LENGTH];
float SlipWindowY[DATA_WINDOW_LENGTH];
float SlipWindowZ[DATA_WINDOW_LENGTH];
float DftXRE[30];
float DftXIM[30];
float DftYRE[30];
float DftYIM[30];
float DftZRE[30];
float DftZIM[30];


BOOL FALL_DETECTED = FALSE;
int LastSitDownTime;
int ResultIndex;
int JumpMax = -10000.0f;
int JumpMaxIndex = 0;

float MaxXmin = 0.2146, MaxXmax = 7.999, MeanXmin = -0.0171, MeanXmax = 0.0201, MaxYmin = 0.0059, MaxYmax = 7.9990, MaxZmin = 0.1934, MaxZmax = 7.9990, MeanZmin = -0.0370, MeanZmax = 0.0176, MeanYmin = -0.0553, MeanYmax = 0.0148, EnXmin = 0.0085, EnXmax= 1111.4, EnYmin = 0.0066, EnYmax = 1212.9, EnZmin = 0.0085, EnZmax = 1198.6, StdZmin = 0.0423, StdZmax = 0.8933, CovYZmin = -0.6177, CovYZmax = 0.7763, CovXZmax = 0.7429, CovXZmin = -0.5056;

BOOL StartTransfer( BOOL restart );
BOOL TransmitOneByte( UINT8 data );
void StopTransfer( void );
void init_LED(void);
int decision2(float maxY, float maxAbsY, float meanX, float meanZ, float covYZ, float stdY, float maxDftX, float maxDftY, float maxDftZ, float DftSumY);
int decision1(float maxindex, float maxX, float maxY, float maxZ, float maxDftX, float maxDftZ, float DftSumX, float DftSumY, float covXY, float covXZ);
int decision3(float maxindex, float maxX, float maxY, float meanX, float meanZ, float stdX, float covXY, float covYZ, float maxDftZ, float DftSumX, float DftSumY);
int decision4(float maxindex, float maxY, float maxZ, float stdX, float stdY, float covXY, float covYZ, float maxDftZ, float DftSumX);
int decision5(float maxindex, float maxX, float maxY, float meanY, float meanZ, float stdX, float stdY, float covXY, float covYZ, float maxDftZ, float DftSumX);
int decision6(float maxindex, float maxX, float maxY, float meanY, float meanZ, float stdX, float stdY, float stdZ, float covXY, float covYZ, float DftSumX);
int decision7(float maxindex, float maxX, float maxY, float meanY, float meanZ, float stdX, float stdY, float stdZ, float covXY, float covYZ, float maxDftX, float DftSumX);

void LED(char x);
void delay(int time);
BOOL i2c_write_reg(UINT reg_address, UINT data );
BOOL i2c_read_reg(int f_read);

BOOL motionsensor_init()
{
    UINT32     actualClock;
    BOOL Success = TRUE;
    LastWalkTime = 0;
    LastFallDetectedTime = 0;
    LastJumpTime = 0;
    ResultIndex = 0;

    int i;
    for (i = 0; i < PEAK_WINDOW_LENGTH; i++)
    {
        TmpZ[i] = TmpY[i] = TmpX[i] = 0;
    }
    for (i = 0; i < 5; i++)
    {
        ResultSave[i] = 5;
    }
    for (i = 0; i < 20; i++)
    {
        DftXRE[i] = 0;
        DftXIM[i] = 0;
        DftYRE[i] = 0;
        DftYIM[i] = 0;
        DftZRE[i] = 0;
        DftZIM[i] = 0;
    }
    actualClock = I2CSetFrequency(sensor_I2C_BUS, GetPeripheralClock(), I2C_CLOCK_FREQ);
    if ( abs(actualClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10 )
        {
//            TCPPutROMString(sktHTTP, "Clock freq have more than 10% error ! ");
            return FALSE;
        }
    TRISBbits.TRISB14 = 0;
    LATBbits.LATB14 = 0;

    TRISBbits.TRISB1 = 0;
    LATBbits.LATB1 = 0;

    // Enable the I2C bus
    I2CEnable(sensor_I2C_BUS, TRUE);
    //  set Normal Power Mode for 0x00, High resolution mode for 0x02 (the last two bit in control 2 register 0x2B
    if(!i2c_write_reg(0x2B,0x00)) Success = FALSE;

    // to set dynamic range to 8G for XYZ axis
    if(!i2c_write_reg(0x0E ,0x12)) Success = FALSE;

    // set sensor to be active mode, normal read mode, 0x11 ODR is 200Hz, 0x09 ODR is 400Hz
    // 0x01 ODR is 800Hz (Maximum ODR)
    if(!i2c_write_reg(0x2A ,0x11)) Success = FALSE;

    return Success;
}

/*float my_sqrt(float number)
{
	long tmp;
	float x, y;
	const float f = 1.5F;
	x = number * 0.5F;
	y = number;
	tmp = * ( long * ) &y;
	tmp = 0x5f3759df - ( tmp >> 1 );

	y = * ( float * ) &tmp;
	y = y * ( f - ( x * y * y ) );
	y = y * ( f - ( x * y * y ) );
	return number * y;
}*/

int collect_sensor_data(int f_read)
{
    int signX, signY, signZ;  //the sign of x and y
    float sum = 0.0;
    volatile char* p;
    int c = 56000;
    unsigned short sx, sy, sz;
    float fx, fy, fz;
    BOOL  Success = TRUE;
    register int i = 0;
    register int x = 0;
    register int f = 0;
    register int k = 0;
    int j = 0, h, m = 0, n = 0, count = 0, data_size, windowIndex = 0, windowStart = 0, peakXindex = -1, peakYindex = -1, peakZindex = -1, maxindex = -1;
    int halfXindex = -1, halfYindex = -1, halfZindex = -1;
    float sumPeakX = 0, sumPeakY = 0, sumPeakZ = 0, halfPeakX = 0, halfPeakY = 0, halfPeakZ = 0, tmpx, tmpy, tmpz, maxX = -10000, maxY = -10000, maxZ = -10000;
    float PeakX = -10000, PeakY = -10000, PeakZ = -10000, sumX = 0, sumY = 0, sumZ = 0;
    float sumXZ = 0, sumYZ = 0, sumXY = 0, energyX = 0, energyY = 0, energyZ = 0, DftSumX = 0, DftSumY = 0, DftSumZ = 0;
    float DftX = 0, DftY = 0, DftZ = 0, maxDftX = -10000, maxDftY = -10000, maxDftZ = -10000;
    float meanX = 0, meanY = 0, meanZ = 0, stdX = 0, stdY = 0, stdZ = 0, covXY = 0, covXZ = 0, covYZ = 0;
    float smoothSumX = 0, smoothSumY = 0, smoothSumZ = 0, sumXSqrt, sumYSqrt, sumZSqrt;
    float maxAbsY = 0.0f;
    register float slipWindowXData1;
    register float slipWindowXData2;
    register float slipWindowYData1;
    register float slipWindowYData2;
    register float slipWindowZData1;
    register float slipWindowZData2;
    char* tmpcharptr;
    int tmpindex = 0, tmpindex1;
    int preIndex = -1;
    ResultIndex = 0;
    JumpMax = -10000.0f;
    JumpMaxIndex = 0;

    for (j = 0; j < 1000; j++){
        motionsensor[56000 + j] = 0;
    }

    for(j = 0; j < 5; j++){
        ResultSave[j] = 5;
        PeakSave[j] = 0;
    }
    j = 0;

    if(f_read>0) data_size = 4;
            else    data_size = 7;


    LED2_ON();

    while(f<14000)//MAXSENSORDATASIZE)
    {
        Success = i2c_read_reg(f_read);
        if(Success)
        {
            if(count > 2){
                for(h=0; h<data_size; h++)
                {
                    /*
                    if(j==0)
                    {
                        if(sensortmpdata[j]!=0x0f) break;
                    }
                     */
                  motionsensor[f]=sensortmpdata[h];
                  f++;

                  /*
                  if(i>= MAXSENSORDATASIZE)
                  {
                        LED2_OFF();
                    return i;
                  }
                  */
                }


                //convert data of x,y,z to float type
                //if(sensortmpdata[0] != 0x0f)LED1_INV();
                signX = (sensortmpdata[1] & 0x80) ? -1 : 1;
                signY = (sensortmpdata[3] & 0x80) ? -1 : 1;
                signZ = (sensortmpdata[5] & 0x80) ? -1 : 1;;
                sx = ((sensortmpdata[1]<<6) + ((sensortmpdata[2]>>2)&0x3F)) & 0x1FFF;
                if(signX < 0){
                    sx = 0x2000 - sx;
                }
                sy = ((sensortmpdata[3]<<6) + ((sensortmpdata[4]>>2)&0x3F)) & 0x1FFF;
                if(signY < 0){
                    sy = 0x2000 - sy;
                }
                sz = ((sensortmpdata[5]<<6) + ((sensortmpdata[6]>>2)&0x3F)) & 0x1FFF;
                if(signZ < 0){
                    sz = 0x2000 - sz;
                }
                fx = signX * (float)sx / 1024.0;
                fy = signY * (float)sy / 1024.0;
                fz = signZ * (float)sz / 1024.0;
                //smooth the waveform use smooth(15), the number in front of fifteen
                if(x < SMOOTH_WINDOW_LENGTH){
                    //save the original data, use in the smoothing of data
		    SmoothWindowX[x % SMOOTH_WINDOW_LENGTH] = fx;
		    SmoothWindowY[x % SMOOTH_WINDOW_LENGTH] = fy;
		    SmoothWindowZ[x % SMOOTH_WINDOW_LENGTH] = fz;
		    smoothSumX += fx;
		    smoothSumY += fy;
		    smoothSumZ += fz;
		    if(x % 2 == 0){
		    	SmoothSaveX[x / 2] = smoothSumX/(x + 1);
		    	SmoothSaveY[x / 2] = smoothSumY/(x + 1);
		    	SmoothSaveZ[x / 2] = smoothSumZ/(x + 1);
		    }
		}else{
                    SlipWindowX[i % DATA_WINDOW_LENGTH] = SmoothSaveX[i % SMOOTH_WINDOW_LENGTH];
                    SlipWindowY[i % DATA_WINDOW_LENGTH] = SmoothSaveY[i % SMOOTH_WINDOW_LENGTH];
                    SlipWindowZ[i % DATA_WINDOW_LENGTH] = SmoothSaveZ[i % SMOOTH_WINDOW_LENGTH];
                    /*if(i < 1000){
                        int tmpindex2 = i;
                        float tmpX = SlipWindowX[i % DATA_WINDOW_LENGTH];
                        float tmpY = SlipWindowY[i % DATA_WINDOW_LENGTH];
                        float tmpZ = SlipWindowZ[i % DATA_WINDOW_LENGTH];
                        char* tmpcharptr1 = &tmpindex2;
                        char* tmpcharptr2 = &tmpX;
                        char* tmpcharptr3 = &tmpY;
                        char* tmpcharptr4 = &tmpZ;
                        for(tmpindex1 = 0; tmpindex1 < 4; tmpindex1++)
                        {
                            motionsensor[56000 + tmpindex * 4 + tmpindex1] = *(tmpcharptr1 + tmpindex1);
                            motionsensor[56000 + (tmpindex + 1) * 4 + tmpindex1] = *(tmpcharptr2 + tmpindex1);
                            motionsensor[56000 + (tmpindex + 2) * 4 + tmpindex1] = *(tmpcharptr3 + tmpindex1);
                            motionsensor[56000 + (tmpindex + 3) * 4 + tmpindex1] = *(tmpcharptr4 + tmpindex1);
                        }
                        tmpindex += 4;
                    }*/
                    if(fabsf(SmoothSaveX[i % SMOOTH_WINDOW_LENGTH]) < 0.05)
                    {
                            tmpx = 0;
                    }
                    else{
                            tmpx = SmoothSaveX[i % SMOOTH_WINDOW_LENGTH] + 1;
                    }
                    if(fabsf(SmoothSaveY[i % SMOOTH_WINDOW_LENGTH]) < 0.05)
                    {
                            tmpy = 0;
                    }
                    else{
                            tmpy = SmoothSaveY[i % SMOOTH_WINDOW_LENGTH] + 1;
                    }
                    if(fabsf(SmoothSaveZ[i % SMOOTH_WINDOW_LENGTH]) < 0.05)
                    {
                        tmpz = 0;
                    }
                    else{
                            tmpz = SmoothSaveZ[i % SMOOTH_WINDOW_LENGTH] + 1;
                    }
                    if(m  < WINDOW_LENGTH / 2){
                        if((i + 1) < PEAK_WINDOW_LENGTH){
                            sumPeakX += tmpx;
                            sumPeakY += tmpy;
                            sumPeakZ += tmpz;
                            TmpX[j] = tmpx;
                            TmpY[j] = tmpy;
                            TmpZ[j] = tmpz;
                        }else{
                            sumPeakX = sumPeakX + tmpx - TmpX[j];
                            sumPeakY = sumPeakY + tmpy - TmpY[j];
                            sumPeakZ = sumPeakZ + tmpz - TmpZ[j];
                            TmpX[j] = tmpx;
                            TmpY[j] = tmpy;
                            TmpZ[j] = tmpz;
                            if(sumPeakX > PeakX){
                                PeakX = sumPeakX;
                                peakXindex = i;
                            }
                            if(sumPeakY > PeakY){
                                PeakY = sumPeakY;
                                peakYindex = i;
                            }
                            if(sumPeakZ > PeakZ){
                                PeakZ = sumPeakZ;
                                peakZindex = i;
                            }
                            m++;
                        }
                    }else{
                        sumPeakX = sumPeakX + tmpx - TmpX[j];
                        sumPeakY = sumPeakY + tmpy - TmpY[j];
                        sumPeakZ = sumPeakZ + tmpz - TmpZ[j];
                        TmpX[j] = tmpx;
                        TmpY[j] = tmpy;
                        TmpZ[j] = tmpz;

                        if(sumPeakX > halfPeakX){
                            halfPeakX = sumPeakX;
                            halfXindex = i;
                            if(sumPeakX > PeakX){
                                PeakX = sumPeakX;
                                peakXindex = i;
                            }
                        }

                        if(sumPeakY > halfPeakY){
                            halfPeakY = sumPeakY;
                            halfYindex = i;
                            if(sumPeakY > PeakY){
                                PeakY = sumPeakY;
                                peakYindex = i;
                            }
                        }

                        if(sumPeakZ > halfPeakZ){
                            halfPeakZ = sumPeakZ;
                            halfZindex = i;
                            if(sumPeakZ > PeakZ){
                                PeakZ = sumPeakZ;
                                peakZindex = i;
                            }
                        }
                        if(maxindex >= 0 && windowIndex - windowStart < WINDOW_LENGTH && maxindex != preIndex){
                            int index1 = windowIndex % DATA_WINDOW_LENGTH;
                            int index2 = (windowIndex + 1) % DATA_WINDOW_LENGTH;
                            slipWindowXData1 = SlipWindowX[index1];
                            slipWindowXData2 = SlipWindowX[index2];
                            slipWindowYData1 = SlipWindowY[index1];
                            slipWindowYData2 = SlipWindowY[index2];
                            slipWindowZData1 = SlipWindowZ[index1];
                            slipWindowZData2 = SlipWindowZ[index2];
                            sumX = sumX + slipWindowXData1 + slipWindowXData2;
                            sumY = sumY + slipWindowYData1 + slipWindowYData2;
                            sumZ = sumZ + slipWindowZData1 + slipWindowZData2;
                            /*if(i < 1000){
                                int tmpindex2 = windowIndex;
                                float tmpX1 = slipWindowYData1;
                                float tmpX2 = slipWindowYData2;
                                char* tmpcharptr1 = &tmpindex2;
                                char* tmpcharptr2 = &tmpX1;
                                char* tmpcharptr3 = &tmpX2;
                                for(tmpindex1 = 0; tmpindex1 < 4; tmpindex1++)
                                {
                                    motionsensor[56000 + tmpindex * 4 + tmpindex1] = *(tmpcharptr1 + tmpindex1);
                                    motionsensor[56000 + (tmpindex + 1) * 4 + tmpindex1] = *(tmpcharptr2 + tmpindex1);
                                    motionsensor[56000 + (tmpindex + 2) * 4 + tmpindex1] = *(tmpcharptr3 + tmpindex1);
                                }
                                tmpindex += 3;
                            }*/
                            if(slipWindowXData1 > maxX)
                            {
                                maxX = slipWindowXData1;
                            }
                            if(slipWindowXData2 > maxX)
                            {
                                maxX = slipWindowXData2;
                            }
                            if(slipWindowYData1 > maxY)
                            {
                                maxY = slipWindowYData1;
                            }
                            if(slipWindowYData2 > maxY)
                            {
                                maxY = slipWindowYData2;
                            }
                            if(slipWindowZData1 > maxZ)
                            {
                                maxZ = slipWindowZData1;
                            }
                            if(slipWindowZData2 > maxZ)
                            {
                                maxZ = slipWindowZData2;
                            }

                            /*if(fabsf(slipWindowYData1) > maxAbsY){
                                maxAbsY = fabsf(slipWindowYData1);
                            }

                            if(fabsf(slipWindowYData2) > maxAbsY){
                                maxAbsY = fabsf(slipWindowYData2);
                            }*/

                            sumXY = sumXY + slipWindowXData1 * slipWindowYData1 + slipWindowXData2 * slipWindowYData2;
                            sumYZ = sumYZ + slipWindowYData1 * slipWindowZData1 + slipWindowYData2 * slipWindowZData2;
                            sumXZ = sumXZ + slipWindowXData1 * slipWindowZData1 + slipWindowXData2 * slipWindowZData2;

                            energyX = energyX + slipWindowXData1 * slipWindowXData1 + slipWindowXData2 * slipWindowXData2;
                            energyY = energyY + slipWindowYData1 * slipWindowYData1 + slipWindowYData2 * slipWindowYData2;
                            energyZ = energyZ + slipWindowZData1 * slipWindowZData1 + slipWindowZData2 * slipWindowZData2;
                            float a;
                            float rad1;
                            float rad2;
                            float sinResult1, sinResult2, cosResult1, cosResult2;
                            for (k = 0; k < 20; k++)
                            {
                                //sin cos function fast compute
                                a = COEFFICIENT * k;
                                rad1 = a * n ;
                                rad2 = rad1 + a;
                                sinResult1 = sin(rad1);
                                sinResult2 = sin(rad2);
                                cosResult1 = cos(rad1);
                                cosResult2 = cos(rad2);

                                DftXRE[k] += slipWindowXData1 * cosResult1 + slipWindowXData2 * cosResult2;
                                DftXIM[k] -= slipWindowXData1 * sinResult1 + slipWindowXData2 * sinResult2;
                                DftYRE[k] += slipWindowYData1 * cosResult1 + slipWindowYData2 * cosResult2;
                                DftYIM[k] -= slipWindowYData1 * sinResult1 + slipWindowYData2 * sinResult2;
                                DftZRE[k] += slipWindowZData1 * cosResult1 + slipWindowZData2 * cosResult2;
                                DftZIM[k] -= slipWindowZData1 * sinResult1 + slipWindowZData2 * sinResult2;
                                //DftYRE[k] += SlipWindowY[index1] * cosResult1 + SlipWindowY[index2] * cosResult2;
                                //DftYIM[k] -= SlipWindowY[index1] * sinResult1 + SlipWindowY[index2] * sinResult2;
                                //DftZRE[k] += SlipWindowZ[index1] * cosResult1 + SlipWindowZ[index2] * cosResult2;
                                //DftZIM[k] -= SlipWindowZ[index1] * sinResult1 + SlipWindowZ[index2] * sinResult2;
                                //DftXRE[k] += SlipWindowX[index1] * cos(2 * PI * k * n / WINDOW_LENGTH) + SlipWindowX[index2] * cos(2 * PI * k * (n + 1) / WINDOW_LENGTH);
                                //DftXIM[k] -= SlipWindowX[index1] * sin(2 * PI * k * n / WINDOW_LENGTH) + SlipWindowX[index2] * sin(2 * PI * k * (n + 1) / WINDOW_LENGTH);

                                //DftYRE[k] += SlipWindowY[index1] * cos(2 * PI * k * n / WINDOW_LENGTH) + SlipWindowY[index2] * cos(2 * PI * k * (n + 1) / WINDOW_LENGTH);
                                //DftYIM[k] -= SlipWindowY[index1] * sin(2 * PI * k * n / WINDOW_LENGTH) + SlipWindowY[index2] * sin(2 * PI * k * (n + 1) / WINDOW_LENGTH);

                                //DftZRE[k] += SlipWindowZ[index1] * cos(2 * PI * k * n / WINDOW_LENGTH) + SlipWindowZ[index2] * cos(2 * PI * k * (n + 1) / WINDOW_LENGTH);
                                //DftZIM[k] -= SlipWindowZ[index1] * sin(2 * PI * k * n / WINDOW_LENGTH) + SlipWindowZ[index2] * sin(2 * PI * k * (n + 1) / WINDOW_LENGTH);
                            }

                            windowIndex += 2;
                            n += 2;
                        }
                        //if(i < 2800)
                        //printf("i:%d,  m:%d,  sumPeakX:%f,  sumPeakY:%f,  sumPeakZ:%f,  peakXindex:%d,  peakYindex:%d,  peakZindex:%d\n", i, m, sumPeakX, sumPeakY, sumPeakZ, peakXindex, peakYindex, peakZindex);

                        if((m + 1) % WINDOW_LENGTH == 0){
                                //TODO:
                            if(maxindex >= 0 && maxindex != preIndex){
                                preIndex = maxindex;
                                for (k = 0; k < 20; k++)
                                {
                                    DftX = sqrt(DftXRE[k] * DftXRE[k] + DftXIM[k] * DftXIM[k]);
                                    DftY = sqrt(DftYRE[k] * DftYRE[k] + DftYIM[k] * DftYIM[k]);
                                    DftZ = sqrt(DftZRE[k] * DftZRE[k] + DftZIM[k] * DftZIM[k]);
                                    DftXRE[k] = 0;
                                    DftXIM[k] = 0;
                                    DftYRE[k] = 0;
                                    DftYIM[k] = 0;
                                    DftZRE[k] = 0;
                                    DftZIM[k] = 0;
                                    if(maxDftX < DftX)
                                    {
                                        maxDftX = DftX;
                                    }
                                    if(maxDftY < DftY)
                                    {
                                        maxDftY = DftY;
                                    }
                                    if(maxDftZ < DftZ)
                                    {
                                        maxDftZ = DftZ;
                                    }
                                    DftSumX += DftX;
                                    DftSumY += DftY;
                                    DftSumZ += DftZ;
                                }

                                meanX = sumX / WINDOW_LENGTH;
                                meanY = sumY / WINDOW_LENGTH;
                                meanZ = sumZ / WINDOW_LENGTH;
                                stdX = sqrt((energyX - sumX * meanX) / WINDOW_LENGTH);
                                stdY = sqrt((energyY - sumY * meanY) / WINDOW_LENGTH);
                                stdZ = sqrt((energyZ - sumZ * meanZ) / WINDOW_LENGTH);
                                //stdY = sqrt((energyY - WINDOW_LENGTH * meanY * meanY) / WINDOW_LENGTH);
                                //stdZ = sqrt((energyZ - WINDOW_LENGTH * meanZ * meanZ) / WINDOW_LENGTH);
                                sumXSqrt = sqrt(WINDOW_LENGTH * energyX - sumX * sumX);
                                sumYSqrt = sqrt(WINDOW_LENGTH * energyY - sumY * sumY);
                                sumZSqrt = sqrt(WINDOW_LENGTH * energyZ - sumZ * sumZ);
                                covXY = (WINDOW_LENGTH * sumXY - sumX * sumY) / (sumXSqrt * sumYSqrt);
                                covXZ = (WINDOW_LENGTH * sumXZ - sumX * sumZ) / (sumXSqrt * sumZSqrt);
                                covYZ = (WINDOW_LENGTH * sumYZ - sumY * sumZ) / (sumYSqrt * sumZSqrt);
                                //decision();

                                /*if(maxindex >= 0){
                                    int tmpindex2 = maxindex - PEAK_WINDOW_LENGTH;
                                    char* tmpcharptr1 = &tmpindex2;
                                    char* tmpcharptr2 = &windowStart;
                                    for(tmpindex1 = 0; tmpindex1 < 4; tmpindex1++)
                                    {
                                        motionsensor[56000 + tmpindex * 4 + tmpindex1] = *(tmpcharptr1 + tmpindex1);
                                        motionsensor[56000 + (tmpindex + 2) * 4 + tmpindex1] = *(tmpcharptr2 + tmpindex1);
                                    }
                                    tmpindex += 3;
                                }*/
                                //int result = decision2(maxY, maxAbsY, meanX, meanZ, covYZ, stdY, maxDftX, maxDftY, maxDftZ, DftSumY);
                                //decision(stdX, stdY, stdZ, meanX, meanZ, energyX, DftSumY, DftSumZ, covXY);
                                //int result =  decision1(maxindex, maxX, maxY, maxZ, maxDftX, maxDftZ, DftSumX, DftSumY, covXY, covXZ);
                                //int result = decision3(maxindex, maxX, maxY, meanX, meanZ, stdX, covXY, covYZ, maxDftZ, DftSumX, DftSumY);
                                //int result = decision4(maxindex, maxY, maxZ, stdX, stdY, covXY, covYZ, maxDftZ, DftSumX);
                                //int result = decision5(maxindex, maxX, maxY, meanY, meanZ, stdX, stdY, covXY, covYZ, maxDftZ, DftSumX);
                                //int result = decision6(maxindex, maxX, maxY, meanY, meanZ, stdX, stdY, stdZ, covXY, covYZ, DftSumX);
                                int result = decision7(maxindex, maxX, maxY, meanY, meanZ, stdX, stdY, stdZ, covXY, covYZ, maxDftX, DftSumX);

                                /*if(ResultSave[(ResultIndex + 2) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 2) % RESULT_WINDOW_LENGTH] < 150 && result == 3){
                                    ResultSave[(ResultIndex + 2) % RESULT_WINDOW_LENGTH] = 5;
                                    motionsensor[56000 + tmpindex * 4 - 13] = 5;
                                }
                                
                                if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 150 && result == 3){
                                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                                    motionsensor[56000 + tmpindex * 4 - 1] = 5;
                                }*/
                                int tmpindex2 = maxindex - PEAK_WINDOW_LENGTH;
                                int result1 = ResultSave[(ResultIndex + 2) % RESULT_WINDOW_LENGTH];
                                int result2 = ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH];
                                int maxindex1 = PeakSave[(ResultIndex + 2) % RESULT_WINDOW_LENGTH];
                                int maxindex2 = PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH];
                                char* tmpcharptr1 = &tmpindex2;
                                char* tmpcharptr2 = &windowStart;
                                char* tmpcharptr3 = &result;
                                char* tmpcharptr4 = &result1;
                                char* tmpcharptr5 = &result2;
                                char* tmpcharptr6 = &maxindex1;
                                char* tmpcharptr7 = &maxindex2;
                                char* tmpcharptr8 = &maxZ;
                                for(tmpindex1 = 0; tmpindex1 < 4; tmpindex1++)
                                {
                                    motionsensor[56000 + tmpindex * 4 + tmpindex1] = *(tmpcharptr1 + tmpindex1);
                                    motionsensor[56000 + (tmpindex + 1) * 4 + tmpindex1] = *(tmpcharptr2 + tmpindex1);
                                    motionsensor[56000 + (tmpindex + 2) * 4 + tmpindex1] = *(tmpcharptr3 + tmpindex1);
                                    motionsensor[56000 + (tmpindex + 3) * 4 + tmpindex1] = *(tmpcharptr4 + tmpindex1);
                                    motionsensor[56000 + (tmpindex + 4) * 4 + tmpindex1] = *(tmpcharptr5 + tmpindex1);
                                    motionsensor[56000 + (tmpindex + 5) * 4 + tmpindex1] = *(tmpcharptr6 + tmpindex1);
                                    motionsensor[56000 + (tmpindex + 6) * 4 + tmpindex1] = *(tmpcharptr7 + tmpindex1);
                                    motionsensor[56000 + (tmpindex + 7) * 4 + tmpindex1] = *(tmpcharptr8 + tmpindex1);
                                }
                                tmpindex += 8;
                                maxDftX = -10000.0f;
                                maxDftY = -10000.0f;
                                maxDftZ = -10000.0f;
                                DftSumX = 0.0f;
                                DftSumY = 0.0f;
                                DftSumZ = 0.0f;
                            }


                            sumX = 0.0f;
                            sumY = 0.0f;
                            sumZ = 0.0f;
                            sumXY = 0.0f;
                            sumXZ = 0.0f;
                            sumYZ = 0.0f;
                            maxX = -10000.0f;
                            maxY = -10000.0f;
                            maxZ = -10000.0f;
                            maxAbsY = 0.0f;
                            energyX = 0.0f;
                            energyY = 0.0f;
                            energyZ = 0.0f;
                            n = 0;

                            if(PeakY > 1.7){ //PeakX > 1.25 || PeakY > 1.25 || PeakZ > 1.25
                                /*if(abs(peakXindex-peakYindex) <= 12 && abs(peakXindex - peakZindex) <= 12 && abs(peakYindex - peakZindex) <= 12){
                                    maxindex = (peakXindex + peakYindex + peakZindex) / 3;
                                }else if(abs(peakXindex-peakYindex) <= 12 && abs(peakXindex - peakZindex) <= 12){
                                    maxindex = (peakYindex + peakZindex) / 2;
                                }else if(abs(peakXindex - peakZindex) <= 12 && abs(peakYindex - peakZindex) <= 12){
                                    maxindex = (peakXindex + peakYindex) / 2;
                                }else if(abs(peakXindex-peakYindex) <= 12 && abs(peakYindex - peakZindex) <= 12){
                                    maxindex = (peakXindex + peakZindex) / 2;
                                }else if(abs(peakXindex-peakYindex) <= 12){
                                    maxindex = (peakXindex + peakYindex) / 2;
                                }else if(abs(peakXindex-peakZindex) <= 12){
                                    maxindex = (peakXindex + peakZindex) / 2;
                                }else if(abs(peakYindex-peakZindex) <= 12){
                                    maxindex = (peakYindex + peakZindex) / 2;
                                }else{
                                    maxindex = (peakXindex + peakYindex + peakZindex) / 3;
                                }*/
                                maxindex = peakYindex;
                                if(maxindex >= WINDOW_LENGTH / 2 + PEAK_WINDOW_LENGTH - 2){
                                    windowStart = windowIndex = maxindex - (WINDOW_LENGTH / 2 + PEAK_WINDOW_LENGTH - 2);
                                }else{
                                    windowStart = windowIndex = 0;
                                }
                            }else{
                                maxindex = -1;
                            }
                            //if(i < 2800)
                            //if(maxindex >= 0)
                            //printf("maxindex:%d\n", maxindex);
                            PeakX = halfPeakX;
                            PeakY = halfPeakY;
                            PeakZ = halfPeakZ;
                            peakXindex = halfXindex;
                            peakYindex = halfYindex;
                            peakZindex = halfZindex;
                            halfXindex = -1;
                            halfYindex = -1;
                            halfZindex = -1;
                            halfPeakX = -10000.0f;
                            halfPeakY = -10000.0f;
                            halfPeakZ = -10000.0f;
                            m = WINDOW_LENGTH / 2;
                        }else{
                            m++;
                        }
                    }
                    j = (j + 1) % PEAK_WINDOW_LENGTH;
                    i++;

                    smoothSumX = smoothSumX + fx - SmoothWindowX[x % SMOOTH_WINDOW_LENGTH];
                    smoothSumY = smoothSumY + fy - SmoothWindowY[x % SMOOTH_WINDOW_LENGTH];
                    smoothSumZ = smoothSumZ + fz - SmoothWindowZ[x % SMOOTH_WINDOW_LENGTH];
		    SmoothWindowX[x % SMOOTH_WINDOW_LENGTH] = fx;
		    SmoothWindowY[x % SMOOTH_WINDOW_LENGTH] = fy;
		    SmoothWindowZ[x % SMOOTH_WINDOW_LENGTH] = fz;
                    SmoothSaveX[(x - SMOOTH_WINDOW_LENGTH / 2) % SMOOTH_WINDOW_LENGTH] = smoothSumX / SMOOTH_WINDOW_LENGTH;
                    SmoothSaveY[(x - SMOOTH_WINDOW_LENGTH / 2) % SMOOTH_WINDOW_LENGTH] = smoothSumY / SMOOTH_WINDOW_LENGTH;
                    SmoothSaveZ[(x - SMOOTH_WINDOW_LENGTH / 2) % SMOOTH_WINDOW_LENGTH] = smoothSumZ / SMOOTH_WINDOW_LENGTH;
		}
		x++;
            }
            count++;
        }
    }

    LED2_OFF();
 
    return f;

}

int decision7(float maxindex, float maxX, float maxY, float meanY, float meanZ, float stdX, float stdY, float stdZ, float covXY, float covYZ, float maxDftX, float DftSumX)
{
    int result = 5;
    if(maxY <= 1.0)
    {
        if(stdX <= 0.0577865)
        {
            if(meanZ > -0.00078418){
                //printf("5");
                result = 51;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else if(stdX <= 0.0509739){
                //printf("5");
                result = 52;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                //printf("1");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 53;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else if(maxindex - JumpMaxIndex > 250){
                    result = 11;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }else if(covYZ <= -0.0646763){
            if(covXY <= 0.712989){
                //printf("1");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 54;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else if(maxindex - JumpMaxIndex > 250){
                    result = 12;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }else{
                //printf("2");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 55;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                    {
                        result = 56;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }else{
                        result = 21;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }
                }
            }
        }else if(maxY > 0.400156){
            //printf("2");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 57;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                {
                    result = 58;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    result = 22;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }else if(maxX > 0.392383){
            //printf("5");
            result = 59;
            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
            ResultIndex++;
        }else{
            //printf("1");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 510;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else if(maxindex - JumpMaxIndex > 250){
                result = 13;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }
    }else if(maxY <= 2.02773){
        if(maxDftX <= 12.4145){
            if(meanY > -0.0109277){
                if(meanZ > -0.0045){
                    //printf(2);
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                    {
                        result = 511;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }else{
                        if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                        {
                            result = 512;
                            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                            ResultIndex++;
                        }else{
                            result = 23;
                            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                            ResultIndex++;
                        }
                    }
                }else{
                    //printf("3");
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                    {
                        result = 513;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }else{
                        if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                            ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                        }
                        if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                            ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                        }
                        result = 31;
                        if(maxY > JumpMax)
                        {
                            JumpMax = maxY;
                            JumpMaxIndex = maxindex;
                        }
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }
                }
            }else{
                //printf("3");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 514;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 32;
                    if(maxY > JumpMax)
                    {
                        JumpMax = maxY;
                        JumpMaxIndex = maxindex;
                    }
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }else{
            //printf("3");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 515;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 33;
                if(maxY > JumpMax)
                {
                    JumpMax = maxY;
                    JumpMaxIndex = maxindex;
                }
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }
    }else if(maxY > 2.02773 && stdY > 0.249478){
        if(stdZ <= 0.355768){
            if(DftSumX <= 176.943){
                if(maxDftX > 8.36){
                    //printf("3");
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                    {
                        result = 516;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }else{
                        if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                            ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                        }
                        if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                            ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                        }
                        result = 34;
                        if(maxY > JumpMax)
                        {
                            JumpMax = maxY;
                            JumpMaxIndex = maxindex;
                        }
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }
                }else{
                    //printf("4");
                    if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 41;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }else if(stdZ <= 0.254229){
                //printf("3");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 517;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 35;
                    if(maxY > JumpMax)
                    {
                        JumpMax = maxY;
                        JumpMaxIndex = maxindex;
                    }
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }else{
                //printf("4");
                if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 42;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else{
            //printf("4");
            if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
            }
            if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
            }
            result = 43;
            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
            ResultIndex++;
        }
    }
    if(ResultIndex == 1000)ResultIndex = 0;
    return result;
}

int decision6(float maxindex, float maxX, float maxY, float meanY, float meanZ, float stdX, float stdY, float stdZ, float covXY, float covYZ, float DftSumX)
{
    int result = 5;
    if(maxY <= 1.0)
    {
        if(stdX <= 0.0577865)
        {
            if(meanZ > -0.00078418){
                //printf("5");
                result = 51;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else if(stdX <= 0.0509739){
                //printf("5");
                result = 52;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                //printf("1");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 53;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else if(maxindex - JumpMaxIndex > 250){
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 11;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }else if(covYZ <= -0.0646763){
            if(covXY <= 0.712989){
                //printf("1");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 54;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else if(maxindex - JumpMaxIndex > 250){
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 12;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }else{
                //printf("2");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 55;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                    {
                        result = 56;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }else{
                        result = 21;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }
                }
            }
        }else if(maxY > 0.470156){
            //printf("2");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 57;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                {
                    result = 58;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    result = 22;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }else if(maxX > 0.392383){
            //printf("5");
            result = 59;
            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
            ResultIndex++;
        }else{
            //printf("1");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 510;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else if(maxindex - JumpMaxIndex > 250){
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                {
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                {
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 13;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }
    }else if(stdX <= 0.158234){
        //printf("2");
        if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
        {
            result = 511;
            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
            ResultIndex++;
        }else{
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
            {
                result = 512;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                result = 23;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }
    }else if(stdZ <= 0.379868){
        if(meanY <= -0.015708){
            //printf("3");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 513;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 31;
                if(maxY > JumpMax)
                {
                    JumpMax = maxY;
                    JumpMaxIndex = maxindex;
                }
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else if(covYZ > 0.308817){
            if(DftSumX <= 108.623){
                //printf("2");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 514;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                    {
                        result = 515;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }else{
                        result = 24;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }
                }
            }else{
                //printf("3");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 516;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 32;
                    if(maxY > JumpMax)
                    {
                        JumpMax = maxY;
                        JumpMaxIndex = maxindex;
                    }
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }else if(meanY > -0.00263574){
            //printf("3");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 517;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 33;
                if(maxY > JumpMax)
                {
                    JumpMax = maxY;
                    JumpMaxIndex = maxindex;
                }
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else if(maxY <= 2.31074 && maxY >= 1.117){
            //printf("3");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 518;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 34;
                if(maxY > JumpMax)
                {
                    JumpMax = maxY;
                    JumpMaxIndex = maxindex;
                }
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else if(stdY > 0.249478 && maxY > 1.3){
            //printf("4");
            if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
            }
            if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
            }
            result = 41;
            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
            ResultIndex++;
        }
    }else if(stdY > 0.249478 && maxY > 1.3){
        //printf("4");
        if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
            ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
        }
        if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
            ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
        }
        result = 42;
        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
        ResultIndex++;
    }
    if(ResultIndex == 1000)ResultIndex = 0;
    return result;
}

int decision5(float maxindex, float maxX, float maxY, float meanY, float meanZ, float stdX, float stdY, float covXY, float covYZ, float maxDftZ, float DftSumX)
{
    int result = 5;
    if(maxY <= 0.720313)
    {
        if(stdX <= 0.0577865)
        {
            if(meanZ > -0.00078418){
                //printf("5");
                result = 51;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else if(stdX <= 0.0509739){
                //printf("5");
                result = 52;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                //printf("1");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 53;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else if(maxindex - JumpMaxIndex > 250){
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 11;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }else if(covYZ <= -0.0646763){
            if(covXY <= 0.712989){
                //printf("1");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 54;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else if(maxindex - JumpMaxIndex > 250){
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 12;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }else{
                //printf("2");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 55;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                    {
                        result = 56;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }else{
                        result = 21;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }
                }
            }
        }else if(maxY > 0.390156){
            //printf("2");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 57;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                {
                    result = 58;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    result = 22;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }else if(maxX > 0.392383){
            //printf("5");
            result = 59;
            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
            ResultIndex++;
        }else{
            //printf("1");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 510;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else if(maxindex - JumpMaxIndex > 250){
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                {
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                {
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 13;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }
    }else if(maxDftZ <= 15.6523){
        if(meanY > -0.009155){
            if(DftSumX <= 111.218){
                if(meanZ > -0.00893294){
                    //printf("2");
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                    {
                        result = 511;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }else{
                        if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                        {
                            result = 512;
                            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                            ResultIndex++;
                        }else{
                            result = 23;
                            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                            ResultIndex++;
                        }
                    }
                }else{
                    //printf("3");
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                    {
                        result = 513;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }else{
                        if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                            ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                        }
                        if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                            ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                        }
                        result = 31;
                        if(maxY > JumpMax)
                        {
                            JumpMax = maxY;
                            JumpMaxIndex = maxindex;
                        }
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }
                }
            }else{
                //printf("3");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 514;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 32;
                    if(maxY > JumpMax)
                    {
                        JumpMax = maxY;
                        JumpMaxIndex = maxindex;
                    }
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }else if(maxY <= 2.48691 && maxY > 1.0){
            //printf("3");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 515;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 33;
                if(maxY > JumpMax)
                {
                    JumpMax = maxY;
                    JumpMaxIndex = maxindex;
                }
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else if(maxDftZ <= 15.5277 && maxY > 1.0){
            //printf("3");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 516;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 34;
                if(maxY > JumpMax)
                {
                    JumpMax = maxY;
                    JumpMaxIndex = maxindex;
                }
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else if(maxY > 1.2){
            //printf("4");
            if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
            }
            if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
            }
            result = 41;
            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
            ResultIndex++;
        }
    }else if(stdY > 0.249478 && maxY > 1.3){
        //printf("4");
        if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
            ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
        }
        if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
            ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
        }
        result = 42;
        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
        ResultIndex++;
    }
    if(ResultIndex == 1000)ResultIndex = 0;
    return result;
}

int decision4(float maxindex, float maxY, float maxZ, float stdX, float stdY, float covXY, float covYZ, float maxDftZ, float DftSumX)
{
    int result = 5;
    if(maxY <= 0.75)
    {
        if(stdX <= 0.0577865){
            //printf("5");
            result = 51;
            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
            ResultIndex++;
        }else if(covYZ <= -0.0593937){
            if(covXY <= 0.712989){
                //printf("1");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 52;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else if(maxindex - JumpMaxIndex > 250){
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 11;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }else{
                //printf("2");
                 if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 53;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                    {
                        result = 54;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }else{
                        result = 21;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }
                }
            }
        }else if(maxY > 0.44668){
            //printf("2");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 55;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                {
                    result = 56;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    result = 22;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }else if(maxZ <= 0.118555){
            //printf("5");
            result = 57;
            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
            ResultIndex++;
        }else{
            //printf("1");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 58;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else if(maxindex - JumpMaxIndex > 250){
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                {
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                {
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 12;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }
    }else if(stdY > 0.249478){
        if(DftSumX <= 67.0907){
            //printf("2");
             if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 59;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                {
                    result = 510;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    result = 23;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }else if(maxDftZ <= 15.5277){
            //printf("3");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 511;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 31;
                if(maxY > JumpMax)
                {
                    JumpMax = maxY;
                    JumpMaxIndex = maxindex;
                }
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else{
            //printf("4");
            if(maxY > 1.3){
                if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 41;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }
            /*if(maxDftZ <= 15.5277 && maxDftZ > 10.7131){
            if(covYZ <= -0.33749){
                //printf("3");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 512;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 32;
                    if(maxY > JumpMax)
                    {
                        JumpMax = maxY;
                        JumpMaxIndex = maxindex;
                    }
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }else{
                //printf("4");
                if(maxY > 1.3){
                    if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 41;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }else if(maxDftZ > 15.5277 || (maxDftZ <= 9.9999 && maxDftZ > 8.78012)){
            //printf(""4);
            if(maxY > 1.3){
                if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 42;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }*/
    }
    if(ResultIndex == 1000)ResultIndex = 0;
    return result;
}


int decision3(float maxindex, float maxX, float maxY, float meanX, float meanZ, float stdX, float covXY, float covYZ, float maxDftZ, float DftSumX, float DftSumY)
{
    int result = 5;
    if(DftSumY <= 103.566){
        if(stdX <= 0.0577865){
            if(stdX <= 0.052968){
                //printf("5");
                result = 51;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else if(meanZ <= -0.00078418){
                //printf("1");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 52;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else if(maxindex - JumpMaxIndex > 250){
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 11;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }else{
                //printf("5");
                result = 52;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else if(covYZ > 0.202592){
            //printf("2");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 53;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                {
                    result = 54;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    result = 21;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }else if(maxX <= 0.154297){
            if(maxX <= 0.130078){
                //printf("5");
                result = 55;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                //printf("2");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 56;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                    {
                        result = 57;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }else{
                        result = 22;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }
                }
            }
        }else if(covXY > 0.712989){
            //printf("2");
            if(maxY > 0.5)
            {
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 58;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                    {
                        result = 59;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }else{
                        result = 23;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }
                }
            }
        }else if(DftSumX > 34.5972){
            //printf("1");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 510;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else if(maxindex - JumpMaxIndex > 250){
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                {
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                {
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 12;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else if(maxY <= 0.216602){
            //printf("1");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 511;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else if(maxindex - JumpMaxIndex > 250){
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 13;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
        }else{
            //printf("5");
            result = 512;
            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
            ResultIndex++;
        }
    }else if(DftSumX <= 69.2956){
        //printf("2");
        if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
        {
            result = 513;
            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
            ResultIndex++;
        }else{
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
            {
                result = 514;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                result = 24;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }
    }else if(maxDftZ <= 8.55995){
        if(DftSumY > 113.39){
            //printf("3");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 515;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 31;
                if(maxY > JumpMax)
                {
                    JumpMax = maxY;
                    JumpMaxIndex = maxindex;
                }
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else if(maxX <= 0.587695){
            //printf("3");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 516;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 32;
                if(maxY > JumpMax)
                {
                    JumpMax = maxY;
                    JumpMaxIndex = maxindex;
                }
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else{
            //printf("1");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 517;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else if(maxindex - JumpMaxIndex > 250){
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                {
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                {
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 14;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }
    }else if(meanX <= -0.00680735){
        if(maxDftZ <= 17.108){
            //printf("3");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 518;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 33;
                if(maxY > JumpMax)
                {
                    JumpMax = maxY;
                    JumpMaxIndex = maxindex;
                }
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else if(maxY > 1.3){
            //printf("4");
            if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
            }
            if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
                ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
            }
            result = 41;
            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
            ResultIndex++;
        }
    }else if(maxY > 1.3){
        //printf("4");
        if((ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
            ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
        }
        if((ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 || ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 1) && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 250){
            ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
        }
        result = 42;
        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
        ResultIndex++;
    }
    if(ResultIndex == 1000)ResultIndex = 0;
    return result;
}

/*
 * 5 classes:1-walk, 2-sit, 3-jump, 4-fall, 5-still
 */
int decision2(float maxY, float maxAbsY, float meanX, float meanZ, float covYZ, float stdY, float maxDftX, float maxDftY, float maxDftZ, float DftSumY)
{
    int result = 5;
    if(maxY <= 0.720313)
    {
        if(maxDftX <= 3.33053){
            if(stdY <= 0.0141886){
                //printf("5");
                result = 51;
            }else{
                result = 21;
            }/*else if(meanZ <= -0.000445312){
                LED2_OFF();
                //printf("1");
                result =  11;
            }else if(maxDftY <= 4.25285){
                LED1_ON();
                //printf("2");
                result = 21;
            }else{
                LED2_OFF();
                //printf("1");
                result = 12;
            }*/
        }else if(DftSumY <= 82.2049){
            LED2_OFF();
            //printf("1");
            result = 11;
        }else if(covYZ <= -0.31292){
            LED2_OFF();
            //printf("1");
            result = 12;
        }else{
            //LED1_ON();
            //printf("3");
            result = 31;
        }
    }else if(maxDftZ <= 7.92188){
        //LED1_ON();
        //printf("3");
        result = 32;
    }else if(meanX <= -0.0300247){
        //LED1_ON();
        //printf("3");
        result = 33;
    }else{
        //LED2_OFF();
        //printf("4");
        result = 41;
    }
    return result;
}


int decision1(float maxindex, float maxX, float maxY, float maxZ, float maxDftX, float maxDftZ, float DftSumX, float DftSumY, float covXY, float covXZ)
{
    int result = 5;
    if(DftSumY <= 103.566){
        if(maxX <= 0.144727){
            //printf("2");
            if(covXZ <= -0.0344956){
                if(covXY <= 0.1890){
                    //printf("5");
                    result = 51;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    //printf("2");
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                    {
                        result = 4;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }else{
                        if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                        {
                            result = 5;
                        }else{
                            result = 21;
                            ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                            PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                            ResultIndex++;
                        }
                    }
                }
            }else if(covXY <= -0.298488){
                //printf("5");
                result = 52;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else if(covXY > -0.25618){
                //printf("2");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 41;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                    {
                        result = 5;
                    }else{
                        result = 22;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }
                }
            }
        }else if(maxDftX <= 3.02855){
            if(DftSumY <= 16.2501){
                //printf("5");
                result = 53;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 5;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                //printf("2");
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 42;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else{
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                    {
                        result = 5;
                    }else{
                        result = 23;
                        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                        ResultIndex++;
                    }
                }
            }
        }else if(DftSumX > 34.5972){
            //printf("1");
            if(maxY > 0.13){
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
                {
                    result = 43;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }else if(maxindex - JumpMaxIndex > 250){
                    if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                    {
                        ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                    }
                    result = 11;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }else if(maxX <= 0.230273){
            //printf("1");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 44;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else if(maxindex - JumpMaxIndex > 250){
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                {
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                {
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 12;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else{
            //printf("2");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 45;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 3 && (maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300) )
                {
                    result = 5;
                }else{
                    result = 24;
                    ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 2;
                    PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                    ResultIndex++;
                }
            }
        }
    }else if(maxZ > 1.16523 && maxY > 1.20){
        //printf("4");
        result = 46;
        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
        ResultIndex++;
    }else if(maxDftZ <= 8.55995){
        if(DftSumY > 123.961){
            //printf("3");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 47;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 31;
                if(maxY > JumpMax)
                {
                    JumpMax = maxY;
                    JumpMaxIndex = maxindex;
                }
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else if(maxX <= 0.465){ //0.506594
            //printf("3");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 48;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 32;
                if(maxY > JumpMax)
                {
                    JumpMax = maxY;
                    JumpMaxIndex = maxindex;
                }
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }else{
            //printf("1");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 49;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else if(maxindex - JumpMaxIndex > 250){
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 200)
                {
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                if(ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] < 200)
                {
                    ResultSave[(ResultIndex + 3) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 13;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 1;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
        }
    }else if(maxY <= 1.39844){
        //printf("3");
            if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 4 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 500)
            {
                result = 410;
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }else{
                if(ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] == 2 && maxindex - PeakSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] < 300){
                    ResultSave[(ResultIndex + 4) % RESULT_WINDOW_LENGTH] = 5;
                }
                result = 33;
                if(maxY > JumpMax)
                {
                    JumpMax = maxY;
                    JumpMaxIndex = maxindex;
                }
                ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 3;
                PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
                ResultIndex++;
            }
    }else{
        //printf("4");
        result = 411;
        ResultSave[ResultIndex % RESULT_WINDOW_LENGTH] = 4;
        PeakSave[ResultIndex % RESULT_WINDOW_LENGTH] = maxindex;
        ResultIndex++;
    }
    if(ResultIndex == 1000)ResultIndex = 0;
    return result;
}


/*
 * 5 classes:1-walk, 2-sit, 3-jump, 4-fall, 5-still
 */
void decision(float stdX, float stdY, float stdZ, float meanX, float meanZ, float energyX, float DftSumY, float DftSumZ, float covXY)
{
    if(stdY <= 0.00861045){
        //printf("5");
    }else if(stdZ <= 0.139965){
        if(stdX <= 0.0464632)
        {
            if(meanX > 0.00257726){
                //printf("2");
            }else if(meanZ <= -0.000696181){
                if(meanX <= -0.0000638021)
                {
                    //printf("1");
                    LED1_INV();
                }else{
                    //printf("2");
                }
            }
        }else if(DftSumY <= 85.9771){
            //printf("1");
            LED1_INV();
        }else if(energyX <= 1.99312){
            //printf("2");
        }else if(covXY <= 0.299073){
            //printf("3");
        }else{
            //printf("1");
            LED1_INV();
        }
    }else if(DftSumZ > 105.048){
        //printf("4");
    }else if(meanX > 0.00257726){
        //printf("4");
    }else if(covXY <= -0.868539){
        //printf("2");
    }else{
        //printf("3");
    }
}

BOOL i2c_read_reg(int f_read)
{
    BOOL                Success = FALSE;
    int                 DataSz;
    UINT8               i2cData[10];
    I2C_7_BIT_ADDRESS   SlaveAddress;
    int                 Index;
    UINT8               i2cbyte=0x00;
    int                 delay_time = 100, i=0;

    UINT8        reg_address = 0x00;

    for (i=0; i<7; i++) sensortmpdata[i]=0;

        // Initialize the data buffer
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, sensor_ADDRESS, I2C_WRITE);
        i2cData[0] = SlaveAddress.byte;
        i2cData[1] = reg_address;              // EEPROM location to program (high address byte)
        DataSz = 2;
        // Start the transfer to write data to the EEPROM
     //   delay(delay_time);

        if( !StartTransfer(FALSE) )
        {
            return FALSE;
        }
        // Transmit all data
        Index = 0;

        while( Success && (Index < DataSz) )
        {
            // Transmit a byte

         //   delay(delay_time);

            if (TransmitOneByte(i2cData[Index]))
            {
                // Advance to the next byte
                Index++;
         //       delay(delay_time);
                // Verify that the byte was acknowledged
                if(!I2CByteWasAcknowledged(sensor_I2C_BUS))
                {
                    Success = FALSE;
                }
            }
            else
            {
                Success = FALSE;
            }
        }
    // End the transfer (hang here if an error occured)
    //StopTransfer();
    StopTransfer();
        // Send a Repeated Started condition
    //    delay(delay_time);

    // now restart i2c transfer again, so we can use repeatstart to speed the i2c bus configuration

     if( !StartTransfer(TRUE) )
        {
            Success = FALSE;
        }


        // Transmit the address with the READ bit set
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, sensor_ADDRESS, I2C_READ);
     //   delay(delay_time);

        if (TransmitOneByte(SlaveAddress.byte))
        {
            // Verify that the byte was acknowledged
       //     delay(delay_time);
            if(!I2CByteWasAcknowledged(sensor_I2C_BUS))
            {
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }

    
    // Read the data from the desired address
    if(f_read>0){
        DataSz = 4;
    }
    else
        DataSz = 7;

    LED0_ON();

    Success = TRUE;
    i=0;
    while(Success&&i<DataSz)
    {
             if(I2CReceiverEnable(sensor_I2C_BUS, TRUE) == I2C_RECEIVE_OVERFLOW)
                 Success = FALSE;
             else
             {
                while(!I2CReceivedDataIsAvailable(sensor_I2C_BUS))  ;

                sensortmpdata[i] = I2CGetByte(sensor_I2C_BUS);
                if(i == 0 &&(sensortmpdata[i]==0x00||sensortmpdata[i]==0x09||sensortmpdata[i]==0x0b))
                {
                     I2CAcknowledgeByte(sensor_I2C_BUS,FALSE);
                     while(!I2CAcknowledgeHasCompleted(sensor_I2C_BUS));
                     StopTransfer();
                     return FALSE;
                }

        //     if(i==0 && sensortmpdata[i]!=0x0F )  Success = FALSE;

                if(i<(DataSz-1)) I2CAcknowledgeByte(sensor_I2C_BUS,TRUE);
                else
                    I2CAcknowledgeByte(sensor_I2C_BUS,FALSE);
                i++;

                while(!I2CAcknowledgeHasCompleted(sensor_I2C_BUS)); //  delay(delay_time);
             }
    }

    StopTransfer();

    if(!Success) LEDS_ON();

    return Success;

}

/*******************************************************************************
  Function:
    BOOL StartTransfer( BOOL restart )

  Summary:
    Starts (or restarts) a transfer to/from the EEPROM.

  Description:
    This routine starts (or restarts) a transfer to/from the EEPROM, waiting (in
    a blocking loop) until the start (or re-start) condition has completed.

  Precondition:
    The I2C module must have been initialized.

  Parameters:
    restart - If FALSE, send a "Start" condition
            - If TRUE, send a "Restart" condition

  Returns:
    TRUE    - If successful
    FALSE   - If a collision occured during Start signaling

  Example:
    <code>
    StartTransfer(FALSE);
    </code>

  Remarks:
    This is a blocking routine that waits for the bus to be idle and the Start
    (or Restart) signal to complete.
  *****************************************************************************/
BOOL StartTransfer( BOOL restart )
{
    I2C_STATUS  status;

    // Send the Start (or Restart) signal
    if(restart)
    {
        I2CRepeatStart(sensor_I2C_BUS);
    }
    else
    {
        // Wait for the bus to be idle, then start the transfer
        while( !I2CBusIsIdle(sensor_I2C_BUS) );

        if(I2CStart(sensor_I2C_BUS) != I2C_SUCCESS)
        {
        //    DBPRINTF("Error: Bus collision during transfer Start\n");
            return FALSE;
        }
    }

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(sensor_I2C_BUS);

    } while ( !(status & I2C_START) );

    return TRUE;
}


/*******************************************************************************
  Function:
    BOOL TransmitOneByte( UINT8 data )

  Summary:
    This transmits one byte to the EEPROM.

  Description:
    This transmits one byte to the EEPROM, and reports errors for any bus
    collisions.

  Precondition:
    The transfer must have been previously started.

  Parameters:
    data    - Data byte to transmit

  Returns:
    TRUE    - Data was sent successfully
    FALSE   - A bus collision occured

  Example:
    <code>
    TransmitOneByte(0xAA);
    </code>

  Remarks:
    This is a blocking routine that waits for the transmission to complete.
  *****************************************************************************/

BOOL TransmitOneByte( UINT8 data )
{
    // Wait for the transmitter to be ready
    while(!I2CTransmitterIsReady(sensor_I2C_BUS))Nop();

    // Transmit the byte
    if(I2CSendByte(sensor_I2C_BUS, data) == I2C_MASTER_BUS_COLLISION)
    {
        DBPRINTF("Error: I2C Master Bus Collision\n");
        return FALSE;
    }

    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(sensor_I2C_BUS));

    return TRUE;
}


/*******************************************************************************
  Function:
    void StopTransfer( void )

  Summary:
    Stops a transfer to/from the EEPROM.

  Description:
    This routine Stops a transfer to/from the EEPROM, waiting (in a
    blocking loop) until the Stop condition has completed.

  Precondition:
    The I2C module must have been initialized & a transfer started.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    StopTransfer();
    </code>

  Remarks:
    This is a blocking routine that waits for the Stop signal to complete.
  *****************************************************************************/

void StopTransfer( void )
{
    I2C_STATUS  status;
    int i;
    // Send the Stop signal
    //I2CStop(sensor_I2C_BUS);

    // Wait for the signal to complete
    do
    {   I2CStop(sensor_I2C_BUS);
    for(i = 0; i < 15; i++);
        status = I2CGetStatus(sensor_I2C_BUS);

    } while ( !(status & I2C_STOP) );
}


/*******************************************************************************
 init the hardware LED light

 ******************************************************************************/

 void init_LED(void){

    //LEDS_OFF();
    LED0_TRIS = 0;
    LED1_TRIS = 0;
    LED2_TRIS = 0;
 }
 void LED(char x){
    if((x%2)==1){
        LED0_ON();
    }
    else{
        LED0_OFF();
    }
    if(((x/2)%2)==1){
        LED1_ON();
    }
    else{
        LED1_OFF();
    }
    if(((x/4)%2)==1){
        LED2_ON();
    }
    else{
        LED2_OFF();
    }
    }
/*******************************************************************************
 delay time

 ******************************************************************************/
void delay(int time){
    int i = 0;
    BOOL a = TRUE;

    while(a){
        i++;

        if(i>10*time)
        {
            a = FALSE;
        }

    }
}

BOOL i2c_write_reg(UINT reg_address, UINT data )
{
    BOOL                Success = FALSE;
    int                 DataSz;
    UINT8               i2cData[10];
    I2C_7_BIT_ADDRESS   SlaveAddress;
    int                 Index;
    int                 delay_time = 100;

        Success = TRUE;
        // Initialize the data buffer
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, sensor_ADDRESS, I2C_WRITE);
        i2cData[0] = SlaveAddress.byte;
        i2cData[1] = reg_address;              // EEPROM location to program (high address byte)
        i2cData[2] = data;
        DataSz = 3;
        
        // Start the transfer to write data

        if( !StartTransfer(FALSE) )
        {
    //        TCPPutROMString(sktHTTP, "Cannot start i2c");
            return FALSE;
        }

        // Transmit all data
        Index = 0;

        while( Success && (Index < DataSz) )
        {
            // Transmit a byte

          //  delay(delay_time);

            if (TransmitOneByte(i2cData[Index]))
            {
                // Advance to the next byte
                Index++;
            //    delay(delay_time);
                // Verify that the byte was acknowledged
                if(!I2CByteWasAcknowledged(sensor_I2C_BUS))
                {
                    Success = FALSE;
                }
            }
            else
            {
                Success = FALSE;
            }
        }
    // End the transfer (hang here if an error occured)

    StopTransfer();
  
    return Success;
}



//int testing_counter = 0 ;
//			Get_i2c_data(0x01);     // get x_ data  high 8bit
//			Get_i2c_data(0x02);     // get x_ data  low 8bit

//			Get_i2c_data(0x03);     // get y_ data high 8bit
//			Get_i2c_data(0x04);     // get y_ data  low 8bit

//			Get_i2c_data(0x05);     // get z_ data


void Get_i2c_data(char num)
{

    UINT8               i2cbyte=0x00;
    BYTE data_send_out[] = "0000";
    INT16               output_data_in_number;


    LEDS_ON();                                              //status LED
    i2cbyte = i2c_read_reg(num);

    output_data_in_number = i2cbyte*4;

    if(output_data_in_number<512)           // convert counting to value
    {
        data_send_out[0]= '-';
    }
    else
    {
        output_data_in_number = 1024 - output_data_in_number ;
        data_send_out[0]= '+';
    }


    data_send_out[1]= output_data_in_number/100%10+0x30;    //convert value into ascii code
    data_send_out[2]= output_data_in_number/10%10+0x30;
    data_send_out[3]= output_data_in_number%10+0x30;

    LEDS_OFF();                                             //status LED
//    TCPPutROMString(sktHTTP, data_send_out);
    return;

}

