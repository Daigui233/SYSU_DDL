#include "Trace.h"
#include "entry.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "ti_msp_dl_config.h"


#define ChANNEL_NUMS 6
uint16_t gAdcResult[ChANNEL_NUMS];
int16_t normalized[ChANNEL_NUMS];

uint16_t inf_Min[ChANNEL_NUMS]={340,1000,600,300,300,500};
uint16_t inf_Max[ChANNEL_NUMS]={1750,2050,1700,1500,1600,1900};

const int16_t weight[ChANNEL_NUMS]={1,100,200,300,400,500};
int32_t sum_Weighted;
int32_t sum_Row;
float inf_std;
uint32_t digitResult[2];

uint16_t KalmanFilter(uint16_t inData)
{
	static float prevData = 0;
	static float p = 1, q = 0.001, r = 0.15, kGain = 0;

	p = p + q;
	kGain = p / ( p + r );
	inData = prevData + ( kGain * ( inData - prevData ) );
	p = ( 1 - kGain ) * p;
	prevData = inData;
	return inData;
}

void Scan(void){
        gAdcResult[0]=DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_0);
        gAdcResult[1]=DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_1);
        gAdcResult[2]=DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_2);
        gAdcResult[3]=DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_3);
        gAdcResult[4]=DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_4);
        gAdcResult[5]=DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_5);
        sprintf(sendmsg,"%d,%d,%d,%d,%d,%d,%.2f\r\n",gAdcResult[0],gAdcResult[1],gAdcResult[2],gAdcResult[3],gAdcResult[4],gAdcResult[5],inf_std);
}
void inf_Read(void){
        static float Lastinf_std;
        gAdcResult[0]=DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_0);
        gAdcResult[1]=DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_1);
        gAdcResult[2]=DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_2);
        gAdcResult[3]=DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_3);
        gAdcResult[4]=DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_4);
        gAdcResult[5]=DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_5);
        
        for(int16_t i=0;i < 6;i++){
            if(gAdcResult[i] < inf_Min[i])
                gAdcResult[i] = inf_Min[i];
        }
        Normalization();
        Standardization();

        inf_std = 0.6 * Lastinf_std + (1 - 0.6) * inf_std;
        //sprintf(sendmsg,"%d,%d,%d,%d,%d,%d,%.2f\r\n",gAdcResult[0],gAdcResult[1],gAdcResult[2],gAdcResult[3],gAdcResult[4],gAdcResult[5],inf_std);
        Lastinf_std=inf_std;
}


void Normalization(void) {
    for(int i = 0;i < ChANNEL_NUMS;i++) {
        normalized[i] = ((gAdcResult[i] - inf_Min[i])* 1000/(inf_Max[i] - inf_Min[i]));
        if(normalized[i] < 5)
            normalized[i] = 1;
    }
}

void Standardization() {
    static int16_t sum=0;
    static int16_t Last_sun=0;
    sum_Weighted = 0;
    sum_Row= 0;
    for(int i = 0; i < ChANNEL_NUMS; i++) {
        sum_Weighted += weight[i] * normalized[i];
        sum_Row += normalized[i];
    }
    if(sum_Row != 0)
        inf_std = (float)sum_Weighted / (float)sum_Row;
    else
        inf_std = (float)sum_Weighted / 1;

    //sprintf(sendmsg,"Standardization:%d,%d,%.2f\r\n",digitResult[0],digitResult[1],inf_std);
}