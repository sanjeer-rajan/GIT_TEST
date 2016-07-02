/*
 * cl_dprep_controller.c
 *
 * Created: 12/27/2013 12:24:08 PM
 *  Author: user
 */ 
#include "cl_app/inc/cl_types.h"
#include "inc/cl_dprep_controller.h"
#include "cl_app/cl_mac/inc/cl_mac_controller.h"
#include "cl_app/cl_console/inc/cl_consolecontroller.h"
#include "sv_stubs/inc/sv_types.h"
#include "Platform/Service/sv_interface.h"
#include "cl_app/cl_alarms/inc/cl_alarmdetector.h"
#include "cl_app/cl_heatcntrl/inc/cl_heatercontroller.h"
#include "cl_app/cl_bc_cntrl/inc/Cl_bc_controller.h"
#include "cl_app/cl_dprep/inc/cl_dprep_primecontroller.h"
#include "cl_app/cl_status/inc/cl_status.h"
#include "asf.h"


Cl_ReturnCodes Cl_dprep_init(void);
Cl_ReturnCodes Cl_dprep_controller(MAC_EVENTS);
Cl_ReturnCodes  cl_dprep_translatemacevent(MAC_EVENTS ,Cl_Dprep_Events* );
Cl_ReturnCodes Cl_Dprep_UpdateTimeInfo(void);
Cl_ReturnCodes cl_dprep_notifydacandgotodpreptandby(void);
Cl_ReturnCodes cl_dprep_notifydacandgotodprep_postprimetandby(void);
Cl_ReturnCodes Cl_Dprep_ProcessAlarms(void );
Cl_ReturnCodes Cl_Dprep_UpdateAlarmTable( void);
Cl_ReturnCodes  CL_DrepAlarmActon(Cl_NewAlarmIdType cl_dprepalarmid);
Cl_ReturnCodes Cl_Dprep_CheckforfillingCompletion(void);
Cl_ReturnCodes Cl_DprepFillingFlowOff(void);
Cl_ReturnCodes Cl_DprepFillingFlowOn(void);
Cl_ReturnCodes Cl_dprep_filling_FSM(Cl_Dprep_Events cl_dprepevent);
Cl_ReturnCodes  Cl_DprepSelectDialysateInlet( void);
Cl_ReturnCodes  Cl_DprepFlowOn(void );
Cl_ReturnCodes  Cl_DprepFlowOff(void );
Cl_ReturnCodes UpdateDprepMinuteTick(void);
Cl_ReturnCodes Cl_Dprep_UpdateFillingTimeInfo(void);
Cl_ReturnCodes UpdateDprepFillingMinuteTick(void);
Cl_ReturnCodes Cl_Dprep_UpdatePrimeTimeInfo(void);
Cl_ReturnCodes UpdateDprepPrimeMinuteTick(void);
Cl_ReturnCodes UpdateDprepDialyserPrimeMinuteTick(void);
Cl_ReturnCodes Cl_Dprep_UpdateDialyserPrimeTimeInfo(void);
Cl_ReturnCodes cl_prep_checkforprimecompletion(void);
Cl_ReturnCodes Cl_dprep_filling_FSM_Init(Cl_Dprep_Events cl_dprepevent);
Cl_ReturnCodes Cl_Dprep_SendPrepStateData(Cl_Console_bulkdatatype);
Cl_ReturnCodes Cl_Dprep_Stoppreparation(void);
Cl_ReturnCodes Cl_Dprep_ResetAlertsforReassertion(void );
Cl_ReturnCodes Cl_Dprep_SendtreatementData(void);
Cl_ReturnCodes Cl_Dprep_setdata(Cl_ConsoleRxDataType DataId,cl_PrepDatatype cl_PrepData , uint8_t size);
Cl_ReturnCodes Cl_Dprep_Get_treatmentdata(Cl_ConsoleRxDataType DataId,cl_PrepDatatype cl_PrepData , uint8_t size);
Cl_ReturnCodes Cl_Dprep_Get_data(Cl_ConsoleRxDataType DataId,   uint8_t size);

Cl_ReturnCodes Cl_drepUpdateHeaterControls(void);
Cl_ReturnCodes	Cl_Dprep_StartPreparation(void);
Cl_ReturnCodes	Cl_Dprep_StartDialyserPrime(void);
Cl_ReturnCodes Cl_dprep_StopMixing(void);

Cl_ReturnCodes cl_dprep_activate_prime_related_alarms(void);


extern Cl_ReturnCodes  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , uint8_t* ,uint8_t );
extern Cl_ReturnCodes Cl_mac_apprequesthandler(MAC_EVENTS );
extern Cl_ReturnCodes  sv_nvmgetdata(uint8_t,uint8_t*);
extern Cl_ReturnCodes  sv_nvmsetdata(uint8_t ,uint8_t* ,uint8_t datasize);
extern Cl_ReturnCodes cl_wait(uint32_t ul_dly_ticks);
extern uint8_t sv_cntrl_setpumpspeed(sv_pumptype sv_pump_id,uint32_t speed);
extern uint8_t  sv_cntrl_deactivatepump(sv_pumptype);
extern uint8_t  sv_cntrl_activatepump(sv_pumptype);
extern Cl_ReturnCodes Cl_Alarm_GetAlarmStatus(Cl_NewAlarmIdType  , bool* );
extern Cl_ReturnCodes Cl_SysStat_GetSensor_Status_Query(Cl_SensorDeviceIdType, uint16_t*);
extern uint8_t sv_cntrl_setflowpath(sv_flowpathtype sv_flowpath);
extern Cl_ReturnCodes Cl_AlarmActivateAlarms(Cl_NewAlarmIdType,bool );
extern uint8_t sv_cntrl_activate_valve(sv_valvetype );
extern uint8_t sv_cntrl_deactivate_valve(sv_valvetype );
extern Cl_ReturnCodes Cl_AlarmConfigureAlarmType(Cl_NewAlarmIdType,Cl_AlarmTriggerType,uint16_t,uint16_t,uint8_t);
extern Cl_ReturnCodes Cl_AlarmResetAlarm(Cl_NewAlarmIdType  );
extern uint8_t sv_cntrl_poweroffheater(void);
extern uint8_t sv_cntrl_poweroffheater(void);
extern uint8_t sv_cntrl_incheater(int32_t dty_val);
extern uint8_t sv_cntrl_poweronheater(void);
extern Cl_ReturnCodes UpdateHeaterControls(void);
extern Cl_ReturnCodes SetHeaterState(HeaterStateType Param_HeaterState);
extern Cl_ReturnCodes  Cl_bc_controller(Cl_BC_EventType cl_bc_event);
extern Cl_ReturnCodes cl_dprep_primecontroller(Cl_Dprep_PrimeEvents prime_event , int16_t data);
extern uint8_t sv_cntrl_activatevenousclamp(void);
extern uint8_t sv_cntrl_deactivatevenousclamp(void);
extern void sv_prop_startmixing();
extern void sv_prop_stopmixing();
extern uint8_t sv_cntrl_enable_loopback(void);
extern uint8_t sv_cntrl_disable_loopback(void);
extern Cl_Dlsis_SenddlsisData(void);
extern uint8_t sv_cntrl_enable_bypass(void);
extern uint8_t sv_cntrl_disable_bypass(void);
extern Cl_ReturnCodes cl_dprep_activate_prime_related_alarms(void);
extern void calibration_apt(uint16_t sensordata);


extern Cl_ConsoleMsgType Cl_ConsoleRxMsg;
extern Cl_Dprep_PrimeStates cl_dprep_prime_state;
extern bool BC_window; //test
extern bool g_testbcfreeze; //test
extern bool Current_sense_trigger; // test
extern float dummy3 ;
extern Cl_AlarmThresholdType  Cl_alarmThresholdTable;
extern volatile int16_t pressure_final_apt,pressure_final_vpt,pressure_final_ps1,pressure_final_ps2,pressure_final_ps3;

	int 	Cl_Dprepsecondscounter = 0;
	int 	Cl_DprepMinutescounter= 0;
	int 	Cl_Dprephourscounter= 0;
	int 	Cl_DprepTotalMinutescounter= 0;
	int 	Cl_DprepTotalhourscounter=0;
	int16_t		Cl_Dprep_filling_secondscounter = 0;
	int16_t		Cl_Dprep_filling_Minutescounter = 0;
	int16_t		Cl_Dprep_filling_TotalMinutescounter = 0;
	
	int16_t		Cl_Dprep_Prime_secondscounter = 0;
	int16_t		Cl_Dprep_Prime_Minutescounter = 0;
	int16_t		Cl_Dprep_Prime_TotalMinutescounter = 0;
	
	int16_t		Cl_Dprep_DialyserPrime_secondscounter = 0;
	int16_t		Cl_Dprep_DialyserPrime_Minutescounter = 0;
	int16_t		Cl_Dprep_DialyserPrime_TotalMinutescounter = 0;
	
float temp3_cel_backup=0;
Bool cl_temp3_stable_confirm = false;
Bool cl_temp3_stable= false,cl_temp3_range_stable= false, cl_temp3_37stable = false;
Bool Cl_Prime_state ;
static uint16_t cl_50ms_timer = 0;
Cl_PatientStateType Cl_PatientState = CL_DPREP_PATIENT_STATE_NOT_CONNECTED;
Cl_Dprep_States cl_dprepstate = CL_DPREP_STATE_IDLE;
Cl_Dprep_filling_States cl_dprepfillingState = CL_DPREP_FILLING_IDLE;
DprepAlarmsType Cl_DprepAlarmTable[CL_DPREP_ALRM_MAX] =
{
	
	{BLOODDOOR_STATUS_OPEN,CL_ALARM_ALARM,false,false,false},
	{HOLDER1STATUS_CLOSED,CL_ALARM_ALARM,false,false,false},
	{HOLDER2STATUS_CLOSED,CL_ALARM_ALARM,false,false,false},
	{COND_STATUS_LOW,CL_ALARM_ALARM,false,false,false},
	{COND_STATUS_HIGH,CL_ALARM_ALARM,false,false,false},
	{COND_DAC_OPEN,CL_ALARM_ALARM,false,false,false},
	{COND_DAC_RO,CL_ALARM_ALARM,false,false,false},
	{COND_DAC_HIGH,CL_ALARM_ALARM,false,false,false},
	{FLOW_NO_FLOW,CL_ALARM_ALARM,false,false,false},
	{FLOW_LOW_FLOWRATE,CL_ALARM_ALARM,false,false,false},
	{ABD_EVENT,CL_ALARM_ALARM,false,false,false},
	{BD_EVENT,CL_ALARM_ALARM,false,false,false},
	{BLD_EVENT,CL_ALARM_ALARM,false,false,false},
	{APTSTATUS_HIGH,CL_ALARM_ALARM,false,false,false},
	{VPTSTATUS_HIGH,CL_ALARM_ALARM,false,false,false},
	{PS1_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS2_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP1_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP2_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP3_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{FPCURRENTSTATUS,CL_ALARM_ALARM,false,false,false},

};

 int Cl_dprepsecondscounter =0 ,Cl_dprepMinutescounter=0, Cl_dprephourscounter=0;
 int Cl_dprepTotalMinutescounter=0, Cl_dprepTotalhourscounter=0;

 Cl_ReturnCodes Cl_dprep_init(void)
 {
	 
	 return CL_OK;
 }
 

 
Cl_ReturnCodes Cl_dprep_controller(MAC_EVENTS Cl_MacDprepEvent)
{
	
	Cl_ReturnCodes  Cl_dprepretcode = CL_OK;
	Cl_Dprep_Events cl_dprepevent = EVENT_DPREP_EVENT_NULL;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	uint8_t data = 0;
	uint8_t datasize = 0;
	uint8_t dataarray[4] =  {0,0,0,0};
	uint8_t systemdataarray[40] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	cl_PrepDatatype tempdata;
	Bool cl_status;
	cl_PrepDatatype cl_PrepData;															
	bool alarmstatus1 = false,alarmstatus2 = false,alarmstatus3 = false,flowstatus = false;
	uint16_t static cl_temp3;
	uint16_t temp_temp3;
	float temp3_cel=0;
	uint16_t temp = 0;
	
	cl_dprep_translatemacevent( Cl_MacDprepEvent, &cl_dprepevent);
	
	switch(cl_dprepevent)
	{
		case EVENT_DPREP_TICK_SECOND:
		//Cl_dprepretcode = Cl_bc_controller(BC_EVENT_SECOND);
		//Cl_dprepretcode = cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_TICK_SEC,0);
		break;
		case EVENT_DPREP_TICK_50MS:
		//Cl_dprepretcode = Cl_bc_controller(BC_EVENT_50MS);
		break;
		default:break;
		
	}
	if(cl_dprepevent == EVENT_DPREP_TICK_SECOND)
	{

		Cl_dprepretcode =  Cl_AlarmResetAlarm( SENSOR_TEMP3STATUS );
		Cl_dprepretcode =  Cl_AlarmResetAlarm( SENSOR_TEMP2STATUS );
		Cl_dprepretcode =  Cl_AlarmResetAlarm( FLOW_NO_FLOW );
		//	Cl_rinseretcode =  Cl_AlarmResetAlarm( FLOWSTATUS_FLOWOFF );
	}


	switch(cl_dprepstate)
	{

		case CL_DPREP_STATE_INIT:
		break;
		case CL_DPREP_STATE_IDLE:
		case CL_DPREP_STATE_STOPPED:
		switch (cl_dprepevent)
		{
			case EVENT_DPREP_COMMAND_GET_DATA:

				if(Cl_ConsoleRxMsg.msgready == true)
				{
					switch(Cl_ConsoleRxMsg.data.byte[0])
					{
						case	CON_RX_PARAM_DATA_PRIME_STATUS:
						command = CON_TX_COMMAND_SYSDATA;
						dataarray[0] = CON_TX_PARAM_DATA_PRIME_STATUS;
						Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_NVM_PRIME_STATUS, &dataarray[1]);
						Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
						break;
						case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
						command = CON_TX_COMMAND_SYSDATA;
						dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
						Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
						Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
						break;
						default:
						break;
					}
				}
			break;
			case EVENT_DPREP_COMMAND_SET_DATA:
					if(Cl_ConsoleRxMsg.msgready == true)
							{
										cl_Datastreamtype cl_temp  ;	
										cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[1];
										cl_temp.bytearray[1] = Cl_ConsoleRxMsg.data.byte[2];
										cl_temp.bytearray[2] = Cl_ConsoleRxMsg.data.byte[3];
										cl_temp.bytearray[3] = Cl_ConsoleRxMsg.data.byte[4];
											
								switch(Cl_ConsoleRxMsg.data.byte[0])
								{
										case ID_dflow:
										break;
										case ID_settemp:
										break;
										case ID_heprate:
										break;
										case ID_setcond:
										break;
										case ID_ufrate:
										break;
										case ID_ufgoal:
										break;
										case ID_bolusvol:
										break;
										case ID_bloodrate:
										break;
										case ID_hepstoptime:
										break;
										case ID_syringetype:
										break;
										case ID_heparincheck:
										break;
										case ID_minufrate:
										break;
										case ID_treattime:
										break;
										case ID_bloodratereturn:
										break;
										case ID_bloodratetreat:
										break;
										case ID_tempulimit:
											Cl_alarmThresholdTable.temp3_high_threshold =  (cl_temp.word)/10;
										break;
										case ID_templlimit:
											Cl_alarmThresholdTable.temp3_low_threshold =  (cl_temp.word)/10;
										break;
										case ID_tmpllimit:
										Cl_alarmThresholdTable.tmp_low_threshold =  cl_temp.word;
										break;
										case ID_tmpulimit:
										Cl_alarmThresholdTable.tmp_high_threshold =  cl_temp.word;
										break;
										case ID_vptllimit:
											Cl_alarmThresholdTable.vpt_low_threshold =  cl_temp.word;
										break;
										case ID_vptulimit:
											Cl_alarmThresholdTable.vpt_high_threshold =  cl_temp.word;
										break;
										case ID_ufllimit:
										break;
										case ID_ufulimit:
										break;
										case ID_dflowllimit:
										break;
										case ID_dflowulimit:
										break;
										case ID_condllimit:
										Cl_alarmThresholdTable.cond_low_threshold =  cl_temp.word;
										break;
										case ID_condulimit:
										Cl_alarmThresholdTable.cond_high_threshold =  cl_temp.word;
										break;
										case ID_aptllimit:
											Cl_alarmThresholdTable.apt_low_threshold =  cl_temp.word;
										break;
										case ID_aptulimit:
										Cl_alarmThresholdTable.apt_high_threshold =  cl_temp.word;
										break;

									default:break;
								}
							}
						
	
			break;
			case EVENT_DPREP_DIALYSIS_PREP:
			
			Cl_Dprep_StartPreparation();
			break;
			case EVENT_DPREP_MIXING_PREP_START:
			Cl_Dprep_StartPreparation();
			break;
			case EVENT_DPREP_ALARM:
			Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
			break;
			case EVENT_DPREP_START_PRIME:
			if(cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING)
			{
				Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PRIME1",6);
				cl_dprep_activate_prime_related_alarms();
				cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_START,0);
			}
			
			break;
			default:
			break;
		}
		break;
		case CL_DPREP_STATE_DPREP_FILLING:
		switch (cl_dprepevent)
		{
			case EVENT_DPREP_START_RECIRC:
						if((cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING) || (cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIME_RCIRC_STARTED) || (cl_dprep_prime_state !=  CL_DPREP_PRIME_STATE_DIALYSER_PRIMING))
						{
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RCIRC",6);
							cl_dprep_primecontroller(CL_DPREP_PRIME_RCIRC_START,0);
						}
			break;
			case EVENT_DPREP_COMMAND_GET_DATA:

				if(Cl_ConsoleRxMsg.msgready == true)
				{
					switch(Cl_ConsoleRxMsg.data.byte[0])
					{
						case	CON_RX_PARAM_DATA_PRIME_STATUS:
						command = CON_TX_COMMAND_SYSDATA;
						dataarray[0] = CON_TX_PARAM_DATA_PRIME_STATUS;
						Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_NVM_PRIME_STATUS, &dataarray[1]);
						Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
						break;
						case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
						command = CON_TX_COMMAND_SYSDATA;
						dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
						Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
						Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
						break;
						default:
						break;
					}
				}
			break;
			case EVENT_DPREP_COMMAND_SET_DATA:
					 if(Cl_ConsoleRxMsg.msgready == true)
					 {
						 
						 
						 Cl_ConsoleRxDataType dataId;
						 uint8_t count;
						 dataId = Cl_ConsoleRxMsg.data.byte[0];
						 for (count =0; count < Cl_ConsoleRxMsg.datasize;count++)
						 {
							 cl_PrepData.bytearray[count] = Cl_ConsoleRxMsg.data.byte[count+1];
							 
						 }
						 Cl_Dprep_setdata(dataId, cl_PrepData,count);
						 
						 
						 
					 }
	
			break;
			case EVENT_DPREP_TICK_50MS:
			
			cl_50ms_timer++;
			if(cl_50ms_timer == 2)
			{
				cl_50ms_timer =0;
			//	sv_cntrl_deactivate_valve(VALVE_ID19);
			}
			
			break;
			case EVENT_DPREP_TICK_500MS:
				//UpdateHeaterControls();
	
			break;
			case EVENT_DPREP_TICK_MINUTE:
				Cl_Dprep_ResetAlertsforReassertion();
				Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&temp_temp3);
					temp_temp3 = temp_temp3 * 0.805;
					temp3_cel = 0.0000116 * temp_temp3 *temp_temp3 + 0.0035 *temp_temp3 + 11.157;
				if((temp3_cel) > 36.8 && (temp3_cel < 37.2))
				{
					if( cl_temp3_37stable == true) 
					{
					 cl_temp3_stable_confirm = true;	
					}
					else
					{
					 cl_temp3_37stable = true;
					 cl_temp3_stable_confirm = false;
					}
				}
				else
				{
					cl_temp3_37stable = false;
					cl_temp3_stable_confirm = false;
					
				}
			


			break;
			case EVENT_DPREP_TICK_SECOND:
			#if 0
						Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&temp_temp3);
					temp_temp3 = temp_temp3 * 0.805;
					temp3_cel = 0.0000116 * temp_temp3 *temp_temp3 + 0.0035 *temp_temp3 + 11.157;
					
					
					if(!(Cl_Dprep_filling_secondscounter%20))
					{
										if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 0.3)) || ((temp3_cel_backup < temp3_cel + 0.3)&&(temp3_cel_backup > temp3_cel)))
										{
											cl_temp3_stable = true;
										}
										else
										{
											cl_temp3_stable =false;
										}
										if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 1)) || ((temp3_cel_backup < temp3_cel + 1)&&(temp3_cel_backup > temp3_cel)))
										{
											cl_temp3_range_stable = true;
										}
										else
										{
											cl_temp3_range_stable =false;
										}
										
										temp3_cel_backup = temp3_cel;
					}
					#endif
					Cl_Dprep_filling_secondscounter++;
					if(Cl_Dprep_filling_secondscounter == 60)
					{
						UpdateDprepFillingMinuteTick();
					}
					Cl_Dprep_SendPrepStateData(DIALYSIS_PREP_DATA);
				//	Cl_Dprep_ResetAlertsforReassertion();
					Cl_dprepretcode = Cl_Dprep_ProcessAlarms();

					if(Cl_Dprep_CheckforfillingCompletion() == CL_OK )
					{
						//if(cl_dprep_prime_state == CL_DPREP_PRIME_STATE_PRIME_COMPLETED)
						if(cl_dprep_prime_state == CL_DPREP_PRIME_STATE_PRIME_RCIRC_COMPLETED)
						{
						//	SetHeaterState(CL_HEATER_STATE_CLOSED_HEATING);
						//	sv_cntrl_enable_loopback();
							 //sv_prop_stopmixing();
							 sv_cntrl_enable_bypass();
							cl_dprepstate = CL_DPREP_STATE_POST_PRIME_STANDBY;
									 
						}
						else
						{
						//	sv_cntrl_enable_loopback();
						//	 sv_prop_stopmixing();
							sv_cntrl_enable_bypass();
							cl_dprepstate = CL_DPREP_STATE_DPREP_FILLING_DONE;
							
						}
						
						//	Cl_rinseretcode = (Cl_ReturnCodes)sv_setflowpath(Default path ?);//set appropirate flow path configuration
					//	Cl_dprepretcode = sv_cntrl_poweroffheater();							
					//	Cl_dprepretcode =  sv_cntrl_deactivatepump(DCMOTOR1);
					//	Cl_dprepretcode =  sv_cntrl_deactivatepump(DCMOTOR2);
						command = CON_TX_COMMAND_DIALYSATE_FILLING_COMPLETED;
						data = (uint8_t)COMMAND_RESULT_SUCCESS;
						Cl_Dprep_filling_secondscounter = 0;
						Cl_Dprep_filling_Minutescounter = 0;
						Cl_Dprep_filling_TotalMinutescounter = 0;
										
						Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,1);
					}
					
			break;
			case EVENT_DPREP_STOP_DIALYSATE_FILL:
					Cl_dprep_StopMixing();
			break;
			break;
			case EVENT_DPREP_ALARM:
					Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
			break;
			case EVENT_DPREP_ALERT:
					Cl_Alarm_GetAlarmStatus(FPCURRENTSTATUS,&alarmstatus1);
					if(alarmstatus1)
					{
						//Cl_dprepretcode = Cl_bc_controller(BC_EVENT_CS);
					//	sv_cntrl_activate_valve(VALVE_ID19);
						cl_50ms_timer  = 1;
					
					}
					Cl_Alarm_GetAlarmStatus(TEMP2_HIGH_THRESHOLD,&alarmstatus3);
					if(alarmstatus3)
					{
						//	UpdateHeaterControls();
					}
			
					Cl_Alarm_GetAlarmStatus(FLOW_NO_FLOW,&flowstatus);
					//	if(flowstatus)
					//	{
					//		Cl_rinseretcode = sv_cntrl_poweroffheater();
					//		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"FLOW_OFF",8);
					//	}
			break;
			case EVENT_DPREP_START_PRIME:
			if(cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING)
			{
				Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PRIME1",6);
				cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_START,0);
			}
			
			break;

		}
		break;

		case CL_DPREP_STATE_CRITICAL_ALARM:
					switch (cl_dprepevent)
					{
						case EVENT_DPREP_DIALYSIS_PREP:
						Cl_Dprep_StartPreparation();
						break;
						case EVENT_DPREP_MIXING_PREP_START:
						Cl_Dprep_StartPreparation();
						break;
									default:break;
					}
		break;
		case CL_DPREP_STATE_DPREP_FILLING_DONE:
					switch (cl_dprepevent)
					{
						case EVENT_DPREP_START_RECIRC:
									if((cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING) || (cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIME_RCIRC_STARTED) || (cl_dprep_prime_state !=  CL_DPREP_PRIME_STATE_DIALYSER_PRIMING))
									{
										Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RCIRC",6);
										cl_dprep_primecontroller(CL_DPREP_PRIME_RCIRC_START,0);
									}
						break;
						case EVENT_DPREP_START_PRIME:
						if(cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING)
						{
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PRIME1",6);
							cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_START,0);
						}
						
						break;
						case EVENT_DPREP_MAC_PRIME_COMPLETED:
					//	cl_dprepstate = CL_DPREP_STATE_POST_PRIME_STANDBY;
						break;

						case EVENT_DPREP_TICK_MINUTE:
							Cl_Dprep_ResetAlertsforReassertion();
											
											#if 0
											if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 0.3)) || ((temp3_cel_backup < temp3_cel + 0.3)&&(temp3_cel_backup > temp3_cel)))
											{
												cl_temp3_stable = true;
											}
											else
											{
												cl_temp3_stable =false;
											}
											if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 1)) || ((temp3_cel_backup < temp3_cel + 1)&&(temp3_cel_backup > temp3_cel)))
											{
												cl_temp3_range_stable = true;
											}
											else
											{
												cl_temp3_range_stable =false;
											}
											
											temp3_cel_backup = temp3_cel
											
											#endif
										
						break;
						
						case EVENT_DPREP_TICK_SECOND:
					//	if(cl_dprep_prime_state == CL_DPREP_PRIME_STATE_PRIME_COMPLETED)
						if(cl_dprep_prime_state == CL_DPREP_PRIME_STATE_PRIME_RCIRC_COMPLETED)
					
						{
							cl_dprepstate = CL_DPREP_STATE_POST_PRIME_STANDBY;
									 
						}
						break;


						case EVENT_DPREP_ALARM:
						Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
						break;
						case EVENT_DPREP_DILYSER_CONNECTED:
		
						// check for micro switches
						Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_CONNECTED_CONFIRMED,&data,0);
		
						break;
						case EVENT_DPREP_COMMAND_GET_DATA:

							if(Cl_ConsoleRxMsg.msgready == true)
							{
								switch(Cl_ConsoleRxMsg.data.byte[0])
								{
									case	CON_RX_PARAM_DATA_PRIME_STATUS:
									command = CON_TX_COMMAND_SYSDATA;
									dataarray[0] = CON_TX_PARAM_DATA_PRIME_STATUS;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_NVM_PRIME_STATUS, &dataarray[1]);
									Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
									break;
									case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
									command = CON_TX_COMMAND_SYSDATA;
									dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
									Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
									break;
									default:
									break;
								}
							}
						break;
						default:break;
					}

		break;
		case CL_DPREP_STATE_POST_PRIME_STANDBY:
		switch(cl_dprepevent)
					{
						case EVENT_DPREP_COMMAND_GET_DATA:
							if(Cl_ConsoleRxMsg.msgready == true)
							{
								switch(Cl_ConsoleRxMsg.data.byte[0])
								{
									case CON_RX_PARAM_DATA_DIALYSER_PRIME_STATUS:
									command = CON_TX_COMMAND_SYSDATA;
									dataarray[0] = CON_TX_PARAM_DATA_DIALYSER_PRIME_STATUS;
									dataarray[1] = 0;									
									Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
									break;
									case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
									command = CON_TX_COMMAND_SYSDATA;
									dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
									Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
									break;
									default:break;
								}
							}
						break;
						case EVENT_DPREP_COMMAND_SET_DATA:
								 if(Cl_ConsoleRxMsg.msgready == true)
								 {
						 
						 
									 Cl_ConsoleRxDataType dataId;
									 uint8_t count;
									 dataId = Cl_ConsoleRxMsg.data.byte[0];
									 for (count =0; count < Cl_ConsoleRxMsg.datasize;count++)
									 {
										 cl_PrepData.bytearray[count] = Cl_ConsoleRxMsg.data.byte[count+1];
							 
									 }
									 Cl_Dprep_setdata(dataId, cl_PrepData,count);
						 
						 
						 
								 }
	
						break;
						case EVENT_DPREP_DILYSER_CONNECTED:
		
						// check for micro switches
						Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_CONNECTED_CONFIRMED,&data,0);
		
						break;
						case EVENT_DPREP_START_DIALISER_PRIME:
								Cl_Dprep_StartDialyserPrime();
								sv_cntrl_disable_loopback();
								 sv_prop_startmixing();
						break;
						case EVENT_DPREP_ALERT:
						Cl_Alarm_GetAlarmStatus(FPCURRENTSTATUS,&alarmstatus1);
						if(alarmstatus1)
						{
							//Cl_dprepretcode = Cl_bc_controller(BC_EVENT_CS);		
				
						}

						break;
						case EVENT_DPREP_ALARM:
						Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
						break;
						case EVENT_DPREP_COMMAND_SET_BLDPUMPRATE:
											if(Cl_ConsoleRxMsg.msgready == true)
											{
												
												cl_Datastreamtype cl_temp  ;
												cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[0];
												cl_temp.bytearray[1] = Cl_ConsoleRxMsg.data.byte[0];
												cl_temp.bytearray[2] = Cl_ConsoleRxMsg.data.byte[0];
												cl_temp.bytearray[3] = Cl_ConsoleRxMsg.data.byte[0];
												cl_dprep_primecontroller(CL_DPREP_PRIME_BLOODPUMP_SETRATE,cl_temp.Twobyte);
											}
						break;
						case EVENT_DPREP_TICK_500MS:
						//UpdateHeaterControls();
						break;
						case  EVENT_DPREP_TICK_MINUTE:
								Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&temp_temp3);
								temp_temp3 = temp_temp3 * 0.805;
								temp3_cel = 0.0000116 * temp_temp3 *temp_temp3 + 0.0035 *temp_temp3 + 11.157;
								Cl_Dprep_filling_secondscounter++;
					#if 0
							//	if(!(Cl_Dprep_filling_secondscounter%20))
								{
													if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 0.6)) || ((temp3_cel_backup < temp3_cel + 0.6)&&(temp3_cel_backup > temp3_cel)))
													{
														cl_temp3_stable = true;
													}
													else
													{
														cl_temp3_stable =false;
													}
													if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 1)) || ((temp3_cel_backup < temp3_cel + 1)&&(temp3_cel_backup > temp3_cel)))
													{
														cl_temp3_range_stable = true;
													}
													else
													{
														cl_temp3_range_stable =false;
													}
										
													temp3_cel_backup = temp3_cel;
								}
					#endif
						break;
						default:
						break;
					}
		break;

		case CL_DPREP_STATE_DIALISER_PRIME:
					switch(cl_dprepevent)
					{
						
						case EVENT_DPREP_TICK_50MS:
									
						break;
						case EVENT_DPREP_TICK_500MS:
					//	UpdateHeaterControls();
						break;
						case EVENT_DPREP_TICK_MINUTE:
							Cl_Dprep_ResetAlertsforReassertion();
						break;
						case EVENT_DPREP_TICK_SECOND:
						//	UpdateHeaterControls();
												Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&temp_temp3);
					temp_temp3 = temp_temp3 * 0.805;
					temp3_cel = 0.0000116 * temp_temp3 *temp_temp3 + 0.0035 *temp_temp3 + 11.157;
					Cl_Dprep_filling_secondscounter++;
					#if 0
					if(!(Cl_Dprep_DialyserPrime_secondscounter%20))
					{
										if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 0.3)) || ((temp3_cel_backup < temp3_cel + 0.3)&&(temp3_cel_backup > temp3_cel)))
										{
											cl_temp3_stable = true;
										}
										else
										{
											cl_temp3_stable =false;
										}
										if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 1)) || ((temp3_cel_backup < temp3_cel + 1)&&(temp3_cel_backup > temp3_cel)))
										{
											cl_temp3_range_stable = true;
										}
										else
										{
											cl_temp3_range_stable =false;
										}
										
										temp3_cel_backup = temp3_cel;
					}
					
					#endif

							Cl_Dprep_DialyserPrime_secondscounter++;
							if(Cl_Dprep_DialyserPrime_secondscounter == 60)
							{
								UpdateDprepDialyserPrimeMinuteTick();
							
							}
							Cl_Dprep_SendPrepStateData(DIALYSER_PRIME_DATA);
						//	Cl_Dprep_ResetAlertsforReassertion();
							Cl_dprepretcode = Cl_Dprep_ProcessAlarms();

					// check for sub state time out and transition
					

							if( Cl_Dprep_DialyserPrime_Minutescounter >= CL_DPREP_DIALISER_PRIME_TIMEOUT_MIN)
							{
								Cl_Dprep_DialyserPrime_Minutescounter = 0;
							//	sv_cntrl_deactivatepump(BLOODPUMP);
							//	cl_dprep_primecontroller(CL_DPREP_PRIME_BLOODPUMP_STOP,0);
								cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING, 1); //1 == STOPPED BECAUSE COMPLTED

								//	Cl_rinseretcode = (Cl_ReturnCodes)sv_setflowpath(Default path ?);//set appropirate flow path configuration
							//	Cl_dprepretcode = sv_cntrl_poweroffheater();
							//	Cl_dprepretcode =  sv_cntrl_deactivatepump(DCMOTOR1);
							//	Cl_dprepretcode =  sv_cntrl_deactivatepump(DCMOTOR2);
								//sv_cntrl_enable_loopback();
								// sv_prop_stopmixing();
								sv_cntrl_enable_bypass();
								Cl_dprepretcode =  	cl_dprep_notifydacandgotodpreptandby();	
								cl_dprepstate = CL_DPREP_STATE_POST_DPREP_STANDBY;	
							}
						break;
						case EVENT_DPREP_STOP_DIALYSER_PRIME:
						//	Cl_Rinse_StopRinse();
						break;
						break;
						case EVENT_DPREP_ALARM:
						Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
						break;
						case EVENT_DPREP_ALERT:
						Cl_Alarm_GetAlarmStatus(FPCURRENTSTATUS,&alarmstatus1);
						if(alarmstatus1)
						{
							//Cl_dprepretcode = Cl_bc_controller(BC_EVENT_CS);
						}

						break;
					}
		break;
		
		case CL_DPREP_STATE_POST_DPREP_STANDBY:
		
			switch(cl_dprepevent)
			{
				case EVENT_DPREP_ALERT:
				Cl_Alarm_GetAlarmStatus(FPCURRENTSTATUS,&alarmstatus1);
				if(alarmstatus1)
				{
				//	Cl_dprepretcode = Cl_bc_controller(BC_EVENT_CS);
				}

				break;
				case EVENT_DPREP_ALARM:
				Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
				break;
				case EVENT_DPREP_PATIENT_READY:
				Cl_PatientState = CL_DPREP_PATIENT_STATE_WAITING_FOR_BD;
				break;
				case EVENT_DPREP_PATIENT_CONNECTED:
				if(Cl_PatientState == CL_DPREP_PATIENT_STATE_BLOOD_DETECTED )
				{
					cl_dprepstate = CL_DPREP_STATE_READY_FOR_DALYSIS;
				}
				
				break;

				case EVENT_DPREP_TICK_MINUTE:
					Cl_Dprep_ResetAlertsforReassertion();
				break;
				case EVENT_DPREP_TICK_SECOND:
				
				//uint16_t temp = 0;
				
					Cl_SysStat_GetSensor_Status_Query(BD_EVENT , &temp);
					if( temp == 0)
					{
						
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"BLOOD",5);
					}
				//Cl_Dprep_SendPrepStateData();
			//	Cl_Dprep_ResetAlertsforReassertion();
			if(Cl_PatientState == CL_DPREP_PATIENT_STATE_WAITING_FOR_BD )
			{
				Cl_Alarm_GetAlarmStatus(BD_EVENT  , &cl_status);
				
				//	if(cl_status == true)
				{
					//	Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_READY_FOR_DIALYSIS,NULL,0);
					//	cl_dprepstate = CL_DPREP_STATE_READY_FOR_DALYSIS;
									
									Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_BLOOD_DETECTED,NULL,0);
									Cl_PatientState = CL_DPREP_PATIENT_STATE_BLOOD_DETECTED;
				}
			}
				
				break;
			}
		
		break;
		case CL_DPREP_STATE_READY_FOR_DALYSIS:
				switch(cl_dprepevent)
				{
					
					case EVENT_DPREP_ALERT:
					Cl_Alarm_GetAlarmStatus(FPCURRENTSTATUS,&alarmstatus1);
					if(alarmstatus1)
					{
					//	Cl_dprepretcode = Cl_bc_controller(BC_EVENT_CS);
					}

					break;
					case EVENT_DPREP_TICK_MINUTE:
						Cl_Dprep_ResetAlertsforReassertion();
					break;
					case EVENT_DPREP_TICK_SECOND:
									
				
					Cl_SysStat_GetSensor_Status_Query(BD_EVENT , &temp);
					if( temp == 0)
					{
						
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"BLOOD",5);
					}
					break;
					case 	EVENT_DPREP_COMMAND_GET_DATA:
							if(Cl_ConsoleRxMsg.msgready == true)
							{
								switch(Cl_ConsoleRxMsg.data.byte[0])
								{
									case CON_RX_PARAM_DATA_DIALYSER_PRIME_STATUS:
									command = CON_TX_COMMAND_SYSDATA;
									dataarray[0] = CON_TX_PARAM_DATA_DIALYSER_PRIME_STATUS;
									dataarray[1] = 0;
									Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
									break;
									case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
									command = CON_TX_COMMAND_SYSDATA;
									dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
									Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
									break;
									default:break;
								}
							}
				
						break;
						case EVENT_DPREP_COMMAND_SET_DATA:
								 if(Cl_ConsoleRxMsg.msgready == true)
								 {
						 
						 
									 Cl_ConsoleRxDataType dataId;
									 uint8_t count;
									 dataId = Cl_ConsoleRxMsg.data.byte[0];
									 for (count =0; count < Cl_ConsoleRxMsg.datasize;count++)
									 {
										 cl_PrepData.bytearray[count] = Cl_ConsoleRxMsg.data.byte[count+1];
							 
									 }
									 Cl_Dprep_setdata(dataId, cl_PrepData,count);
						 
						 
						 
								 }
	
						break;
			
						case EVENT_DPREP_ALARM:
						Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
						break;
					default:
					break;
				}
		break;

		}

	return Cl_dprepretcode;
}
 
 
Cl_ReturnCodes  cl_dprep_translatemacevent(MAC_EVENTS Cl_MacDprepEvt,Cl_Dprep_Events* cl_dprepevent)
{
	switch(Cl_MacDprepEvt)
	{
	
		case EVT_CONSOLE_COMMAND_DIALYSIS_PREP:
		*cl_dprepevent = EVENT_DPREP_DIALYSIS_PREP;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
		*cl_dprepevent = EVENT_DPREP_COMMAND_SET_BLDPMP_ON;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
		*cl_dprepevent = EVENT_DPREP_COMMAND_SET_BLDPMP_OFF;
		break;
		case 	EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
		*cl_dprepevent = EVENT_DPREP_COMMAND_SET_BLDPUMPRATE;
		break;
		case EVT_CONSOLE_COMMAND_SET_DATA:
		*cl_dprepevent =  EVENT_DPREP_COMMAND_SET_DATA;
		break;
		case EVT_CONSOLE_COMMAND_GET_DATA:
		*cl_dprepevent = EVENT_DPREP_COMMAND_GET_DATA;
		break;
		
		case EVT_CONSOLE_COMMAND_START_DIALYSATE_FILLING:
				*cl_dprepevent = EVENT_DPREP_START_DIALYSATE_FILL;
				break;
		case EVT_CONSOLE_COMMAND_START_PRIME:
		*cl_dprepevent = EVENT_DPREP_START_PRIME;
		break;
		
		case EVT_CONSOLE_COMMAND_STOP_PRIME:
		*cl_dprepevent = EVENT_DPREP_STOP_PRIME;
		break;
		case EVT_CONSOLE_COMMAND_DILYSER_CONNECTED:
		*cl_dprepevent = EVENT_DPREP_DILYSER_CONNECTED;
		break;
		case EVT_CONSOLE_COMMAND_START_DIALISER_PRIME:
		*cl_dprepevent = EVENT_DPREP_START_DIALISER_PRIME;
		break;
		case EVT_CONSOLE_COMMAND_STOP_DIALISER_PRIME:
		*cl_dprepevent = EVENT_DPREP_STOP_DIALYSER_PRIME;
		break;	
		
		case EVT_CONSOLE_COMMAND_HEP_PMP_SET:
		*cl_dprepevent = EVENT_DPREP_HEP_PMP_SET;
		break;
		case EVT_CONSOLE_COMMAND_MAN_PREP_COMPLETED:
		*cl_dprepevent =   EVENT_DPREP_MAN_PREP_COMPLETED;
		break;
		
		case EVT_TICK_50M:
		*cl_dprepevent =   EVENT_DPREP_TICK_50MS;
		break;
		case  EVT_TICK_100M:
		*cl_dprepevent =   EVENT_DPREP_TICK_100MS;
		break;
		
		case EVT_TICK_500M:
		*cl_dprepevent =   EVENT_DPREP_TICK_500MS;
		break;
		case EVT_TICK_SEC:
		*cl_dprepevent =   EVENT_DPREP_TICK_SECOND;
		break;
		case EVT_TICK_MIN:
		*cl_dprepevent =   EVENT_DPREP_TICK_MINUTE;
		break;			 
		case EVT_TICK_HOUR:
		*cl_dprepevent =   EVENT_DPREP_TICK_HOUR;
		break;
		
		case MACREQ_PRIME_COMPLETED:
		*cl_dprepevent = EVENT_DPREP_MAC_PRIME_COMPLETED;
		break;
		case MACREQ_DIALYSER_PRIME_COMPLETED:
			*cl_dprepevent = EVENT_DPREP_MAC_DIALYSER_PRIME_COMPLETED;
		break;
		case EVT_ALARM_TRIGGERED:
		*cl_dprepevent =  EVENT_DPREP_ALARM;
		break;
		case EVT_ALERT_TRIGGERED:
		*cl_dprepevent =  EVENT_DPREP_ALERT;
		break;
		case EVT_CONSOLE_COMMAND_PATIENT_CONNECTED:
		*cl_dprepevent =  EVENT_DPREP_PATIENT_CONNECTED;
		break;
		case EVT_CONSOLE_COMMAND_PATIENT_READY:
		*cl_dprepevent = EVENT_DPREP_PATIENT_READY;
		break;
		case EVT_SEND_MIXING_PREP_START:
		*cl_dprepevent =  EVENT_DPREP_MIXING_PREP_START;
		break;
		case EVT_CONSOLE_COMMAND_RCIRC_START:
		*cl_dprepevent = EVENT_DPREP_START_RECIRC;
		break;
		default:
		break;
	}
		 return CL_OK;
}
Cl_ReturnCodes Cl_Dprep_UpdateTimeInfo(void)
{
	
	Cl_ReturnCodes  Cl_dprepretcode = CL_ERROR;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	uint8_t data[7] = {0,0,0,0,0,0,0};
	
	command = CON_TX_COMMAND_REM_TIME;

	
	data[1]= (uint8_t)Cl_dprepTotalMinutescounter;
	data[2]= (uint8_t)Cl_dprepTotalhourscounter;
	data[3]= (uint8_t)Cl_dprepsecondscounter;
	data[4]= (uint8_t) (CL_DPREP_PRIME_TIMEOUT_MIN - Cl_dprepTotalMinutescounter );
	data[5]= (uint8_t) (CL_DPREP_PRIME_TIMEOUT_HRS - Cl_dprepTotalhourscounter );
	data[6]= (uint8_t) (60 - Cl_dprepsecondscounter );
	
	Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,7);
	
	return CL_OK;
	}
	
	Cl_ReturnCodes Cl_Dprep_UpdateFillingTimeInfo(void)
{
	
	Cl_ReturnCodes  Cl_dprepretcode = CL_ERROR;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	uint8_t data[7] = {0,0,0,0,0,0,0};
	#if 0
	command = CON_TX_COMMAND_REM_TIME;

	data[0]= (uint8_t) DIALYSATE_FILLING_DATA;
	data[1]= (uint8_t)Cl_Dprep_filling_Minutescounter;
	data[2]= (uint8_t)0;
	data[3]= (uint8_t)Cl_Dprep_filling_secondscounter;
	data[4]= (uint8_t) (CL_DPREP_PRIME_TIMEOUT_MIN - Cl_Dprep_filling_TotalMinutescounter );
	data[5]= (uint8_t) 0;
	data[6]= (uint8_t) 0;
	
	Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,7);
	#endif
	return CL_OK;
	}
	
#if 0
		Cl_ReturnCodes Cl_Dprep_UpdatePrimeTimeInfo(void)	
			{
	
				Cl_ReturnCodes  Cl_dprepretcode = CL_ERROR;
				Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
				uint8_t data[7] = {0,0,0,0,0,0,0};
	
				command = CON_TX_COMMAND_REM_TIME;

				data[0] = (uint8_t) PRIMING_DATA;

				data[1]= (uint8_t)Cl_Dprep_Prime_Minutescounter;
				data[2]= (uint8_t)0;
				data[3]= (uint8_t)Cl_Dprep_Prime_secondscounter;
				data[4]= (uint8_t) (CL_DPREP_PRIME_TIMEOUT_MIN - Cl_Dprep_Prime_TotalMinutescounter );
				data[5]= (uint8_t) 0;
				data[6]= (uint8_t) 0;
	
				Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,7);
	
				return CL_OK;
			}
		Cl_ReturnCodes Cl_Dprep_UpdateDialyserPrimeTimeInfo(void)	
			{
				
				Cl_ReturnCodes  Cl_dprepretcode = CL_ERROR;
				Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
				uint8_t data[7] = {0,0,0,0,0,0,0};
				
				command = CON_TX_COMMAND_REM_TIME;
				data[0] = (uint8_t)DIALYSER_PRIME_DATA;

				data[1]= (uint8_t)Cl_Dprep_DialyserPrime_Minutescounter;
				data[2]= (uint8_t)0;
				data[3]= (uint8_t)Cl_Dprep_DialyserPrime_secondscounter;
				data[4]= (uint8_t) (CL_DPREP_PRIME_TIMEOUT_MIN - Cl_Dprep_DialyserPrime_TotalMinutescounter );
				data[5]= (uint8_t) 0;
				data[6]= (uint8_t) 0;
				
				Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,7);
				
				return CL_OK;
			}	
		
	#endif
	
	Cl_ReturnCodes cl_dprep_notifydacandgotodprep_postprimetandby(void)
	{
		Cl_ReturnCodes 	Cl_dprepretcode = CL_OK;
		
		uint8_t data =0;
		//inform DAC about rinse completed state.
		Cl_dprepsecondscounter = 0;
		Cl_dprepMinutescounter= 0;
		Cl_dprephourscounter= 0;
		Cl_dprepTotalMinutescounter= 0;
		Cl_dprepTotalhourscounter=0;
		data = 1;
		Cl_dprepretcode = sv_nvmsetdata(NV_NVM_PRIME_STATUS,&data,1);
		
		Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_COMPLETED,&data,0);
	//	Cl_dprepretcode = Cl_mac_apprequesthandler(MACREQ_PRIME_COMPLETED);
		
		return Cl_dprepretcode;
		
	}
	Cl_ReturnCodes cl_dprep_notifydacandgotodpreptandby(void)
	{
		Cl_ReturnCodes 	Cl_dprepretcode = CL_OK;
		
		uint8_t data =0;
		//inform DAC about rinse completed state.
		Cl_dprepsecondscounter = 0;
		Cl_dprepMinutescounter= 0;
		Cl_dprephourscounter= 0;
		Cl_dprepTotalMinutescounter= 0;
		Cl_dprepTotalhourscounter=0;
		data = 1;
		Cl_dprepretcode = sv_nvmsetdata(NV_NVM_PRIME_STATUS,&data,1);
		
		Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALISYS_PRIME_COMPLETED,&data,0);
		Cl_dprepretcode = Cl_mac_apprequesthandler(MACREQ_DIALYSER_PRIME_COMPLETED);
		
		
		return Cl_dprepretcode;
	}
	
	Cl_ReturnCodes Cl_dprep_prime_FSM(Cl_Dprep_Events cl_dprepevent)
	{
	}
	
	Cl_ReturnCodes Cl_dprep_filling_FSM_Init(Cl_Dprep_Events cl_dprepevent)
	{
		cl_dprepfillingState = CL_DPREP_FILLING_IDLE;
			Cl_Dprep_filling_Minutescounter = 0;
			Cl_Dprep_filling_TotalMinutescounter = 0;
										
	}
	
	
	Cl_ReturnCodes Cl_Dprep_ProcessAlarms(void )
	{
		Cl_ReturnCodes 	Cl_dprepretcode = CL_OK;
		ClDprepAlarmIdType CldprepAlarmId;
		uint8_t data;
		Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
		Cl_NewAlarmIdType cl_dprepalarmid;
		//	cl_wait(200);
		Cl_dprepretcode = Cl_Dprep_UpdateAlarmTable();
		//   	Cl_rinseretcode =	Cl_Alarm_GetLastAlarm(&cl_rinsealarmid);
		//   	data = (uint8_t)cl_rinsealarmid;
		//   	command = CON_TX_COMMAND_ALARM;
		//   	Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
		
		
		return (Cl_dprepretcode);
	}
	
	
	Cl_ReturnCodes Cl_Dprep_UpdateAlarmTable( void)
	{
		Cl_ReturnCodes 	Cl_dprepretcode = CL_OK;
		uint8_t tempcount = 0;
		uint8_t data[2] ={0, 0} ,*data1 = NULL;
		uint8_t data3;
		Cl_NewAlarmIdType cl_alarmId;
		
		Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
		
		
		for (tempcount = 0 ; tempcount < CL_DPREP_ALRM_MAX ; tempcount++)
		{
			
			Cl_Alarm_GetAlarmStatus(Cl_DprepAlarmTable[tempcount].Cl_DprepAlarmId,&Cl_DprepAlarmTable[tempcount].IsActive);
			//	if(Cl_RinseAlarmTable[tempcount].Cl_RinseAlarmType == CL_ALARM_ALARM)
			//	{
			
			
			if (Cl_DprepAlarmTable[tempcount].IsActive)
			{
				if(!Cl_DprepAlarmTable[tempcount].IsRaised)
				{
				//	cl_alarmId = Cl_DprepAlarmTable[tempcount].Cl_DprepAlarmId;

				//	data[0] = (uint8_t)(Cl_DprepAlarmTable[tempcount].Cl_DprepAlarmId);
				//	data[1] = (uint8_t) CL_ALARM_TRIGGERED;
				//	data[1] = (uint8_t) CRITICAL;
					//	data1 = (uint8_t) cl_alarmId;

				//	command = CON_TX_COMMAND_ALARM;

				//	Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,3);
					Cl_DprepAlarmTable[tempcount].IsRaised = true;
					
					Cl_dprepretcode = CL_DrepAlarmActon(Cl_DprepAlarmTable[tempcount].Cl_DprepAlarmId);
					
					
				}
				
			}
			else
			{
				// alarm was present before , but not active now.
				if(Cl_DprepAlarmTable[tempcount].IsRaised == true)
				{
					Cl_DprepAlarmTable[tempcount].IsRaised = false;
					command = CON_TX_COMMAND_ALARM;
					data[0] = (uint8_t)(Cl_DprepAlarmTable[tempcount].Cl_DprepAlarmId);
					data[1] = (uint8_t) 0;
				//	Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,2);
				}
				Cl_DprepAlarmTable[tempcount].IsActive = true;

			}
			//}

		}

		return (Cl_dprepretcode );
	}

Cl_ReturnCodes  CL_DrepAlarmActon(Cl_NewAlarmIdType cl_dprepalarmid)
{
	Cl_ReturnCodes 	Cl_dprepretcode = CL_OK;
	uint16_t levelswitchstatus = 0;
	uint16_t wait_cnt = 0,TmpVal=0;
	static uint8_t fillseccounter=0;
	uint8_t data[3] ={0, 0};
	Cl_NewAlarmIdType NewAlarmId = _NO_ALARM;
				uint8_t data3;
				Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	
	switch(cl_dprepalarmid)
	{
			case BLOODDOOR_STATUS_OPEN:
			Cl_Dprep_Stoppreparation();
			//NewAlarmId = BLOODDOOR_STATUS_OPEN;
			 Cl_Dprep_Stoppreparation();
			cl_dprepstate = CL_DPREP_STATE_CRITICAL_ALARM;
			//Cl_SysStat_GetSensor_Status_Query(HOLDER1STATUS_OPEN,&levelswitchstatus);
			
			break;

			case HOLDER1STATUS_OPEN:
			// stop rinsing
			NewAlarmId = HOLDER1STATUS_OPEN;
			 Cl_Dprep_Stoppreparation();
			cl_dprepstate = CL_DPREP_STATE_CRITICAL_ALARM;
			//Cl_SysStat_GetSensor_Status_Query(HOLDER1STATUS_OPEN,&levelswitchstatus);
			//enterl_saferinse_state();
			break;
			case HOLDER2STATUS_OPEN:
			// stop rinsing
			//NewAlarmId = HOLDER2STATUS_OPEN;
			 Cl_Dprep_Stoppreparation();
			cl_dprepstate = CL_DPREP_STATE_CRITICAL_ALARM;
			//enterl_saferinse_state();
			break;
			case LEVELSWITCH_OFF_TO_ON:
			// TURN OFF  WATER INLET
		//	Cl_SysStat_GetSensor_Status_Query(LEVELSWITCH_OFF_TO_ON,&levelswitchstatus);
			{
				if(levelswitchstatus == 1)
				{
	
					//fillseccounter++;

			//		Cl_DprepFlowOff();
					//cl_gfillinprogress = false;		
					
				}
			}
			break;
			case LEVELSWITCH_ON_TO_OFF:
			// TURN ON WATER INLET
			//Cl_SysStat_GetSensor_Status_Query(LEVELSWITCH_ON_TO_OFF,&levelswitchstatus);
			{
			//	if(levelswitchstatus == 0)
				{
					
					//fillseccounter++;

			//		Cl_DprepFlowOn();
					//cl_gfillinprogress = true;
					
				}
			}
			break;
			case TEMP3_HIGH_THRESHOLD:
	
		//	Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&TmpVal);
		//	int16_t temp3,temp4;
		//	temp3 = (0.805 * TmpVal) - 1004 ;
		//	temp4 = 3000 + (temp3 * 1000)/382;
		//	if(temp4 > 3680)
			{
			//		NewAlarmId = _TEMP3_HIGH_THRESHOLD;
					Cl_Dprep_Stoppreparation();
	
					cl_dprepstate = CL_DPREP_STATE_CRITICAL_ALARM;
			}
		//	if(temp4 < 3500)
			{
			//	NewAlarmId = _TEMP3_LOW_THRESHOLD;
			//	Cl_Dprep_Stoppreparation();

			//	cl_dprepstate = CL_DPREP_STATE_CRITICAL_ALARM;
			}
			//Cl_Rinse_StopRinse();
			//enterl_saferinse_state();

			break;
			case 	PS1_HIGH_THRESHOLD:
			//	NewAlarmId = _PS1_HIGH_THRESHOLD;
			 Cl_Dprep_Stoppreparation();
			cl_dprepstate = CL_DPREP_STATE_CRITICAL_ALARM;
			//enterl_saferinse_state();
			case	PS2_HIGH_THRESHOLD:
			//	NewAlarmId = _PS2_HIGH_THRESHOLD;
			 Cl_Dprep_Stoppreparation();
			cl_dprepstate = CL_DPREP_STATE_CRITICAL_ALARM;
			//enterl_saferinse_state();
			case	PS3_HIGH_THRESHOLD:
			//	NewAlarmId = _PS3_HIGH_THRESHOLD;
			 Cl_Dprep_Stoppreparation();
			cl_dprepstate = CL_DPREP_STATE_CRITICAL_ALARM;
			//enterl_saferinse_state();
			break;
			
			case APTSTATUS_HIGH:
						// Cl_Dprep_Stoppreparation();
			break;
			case VPTSTATUS_HIGH:
						// Cl_Dprep_Stoppreparation();
			break;
			default:
			break;
	}
		 if(NewAlarmId != _NO_ALARM)
		 {
			data[0] = (uint8_t)cl_dprepalarmid;
			data[1] = (uint8_t) CL_ALARM_TRIGGERED;
			data[2] = (uint8_t)CRITICAL;
			command = CON_TX_COMMAND_ALARM;

			Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,3);
			
					
			command = CON_TX_COMMAND_SYS_STATE;
			data[0] = (uint8_t)POST_CLEAN_STANDBY;
			data[1] = (uint8_t)PRIME_IDLE;
			Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,2);
		 }
			
	
}
	
	
	Cl_ReturnCodes Cl_Dprep_CheckforfillingCompletion(void)
	{
		Cl_ReturnCodes Cl_dprepretcode = CL_ERROR;
		uint16_t cl_cond,cl_temp3,temp;
		float cl_cond_ms,cl_temp3_cel;
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&Cl_Dprep_filling_TotalMinutescounter,2);
		if(Cl_Dprep_filling_TotalMinutescounter > CL_DPREP_FILLING_TIMEOUT_MIN )
		{
			
		//	Cl_dprepretcode = CL_REJECTED;
			Cl_dprepretcode = CL_OK;
		}
		else
		{
			Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&cl_temp3);
			Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS,&cl_cond);
			temp = cl_temp3 * 0.805;
			cl_temp3_cel = 0.0000116 * temp *temp + 0.0035 *temp + 11.157;
			  
			if(cl_cond > 13000000)
			{
				if ( cl_temp3_cel > 36.8 && cl_temp3_cel < 37.2)
				{
					if(cl_temp3_stable_confirm == true)
							Cl_dprepretcode = CL_OK;
				}
				
			}

			
		}
		
		return Cl_dprepretcode;
		
	}
	
	
	Cl_ReturnCodes cl_prep_checkforprimecompletion(void)
	{
		Cl_ReturnCodes Cl_dprepretcode = CL_ERROR;

		
		if(Cl_Dprep_Prime_TotalMinutescounter > CL_DPREP_PRIME_TIMEOUT_MIN )
		{
			
			Cl_dprepretcode = CL_OK;
		}

		
		return Cl_dprepretcode;
		
	}
	
	
	Cl_ReturnCodes  Cl_DprepSelectDialysateInlet(void)
	{
		Cl_ReturnCodes Cl_RetVal = CL_OK;
		//sv_cntrl_deactivate_valve(VALVE_ID18);
		//sv_cntrl_activate_valve(VALVE_ID19);
		sv_cntrl_activate_valve(VALVE_ID18);
		
	}
	Cl_ReturnCodes  Cl_DprepFlowOn(void )
	{
		Cl_ReturnCodes Cl_RetVal = CL_OK;
		sv_cntrl_activate_valve(VALVE_ID1);
		
		return Cl_RetVal;
	
	}

	Cl_ReturnCodes  Cl_DprepFlowOff(void )
	{
		Cl_ReturnCodes Cl_RetVal = CL_OK;
		sv_cntrl_deactivate_valve(VALVE_ID1);
		
		return Cl_RetVal;
	
	}
		//	#endif
		
		
Cl_ReturnCodes UpdateDprepMinuteTick(void)
{
					Cl_ReturnCodes Cl_dpreptcode = CL_OK;
					Cl_Dprepsecondscounter = 0;
					Cl_DprepMinutescounter++;
					Cl_DprepTotalMinutescounter++;
					Cl_dpreptcode = Cl_Dprep_UpdateTimeInfo();
					return Cl_dpreptcode;
}

Cl_ReturnCodes UpdateDprepFillingMinuteTick(void)
{
					Cl_ReturnCodes Cl_dpreptcode = CL_OK;

					Cl_Dprep_filling_secondscounter = 0;
					Cl_Dprep_filling_Minutescounter++;
					Cl_Dprep_filling_TotalMinutescounter++;

			//		Cl_dpreptcode = Cl_Dprep_UpdateFillingTimeInfo();
					return Cl_dpreptcode;
					

}

Cl_ReturnCodes UpdateDprepPrimeMinuteTick(void)
{
	Cl_ReturnCodes Cl_dpreptcode = CL_OK;

	Cl_Dprep_Prime_secondscounter = 0;
	Cl_Dprep_Prime_Minutescounter++;
	Cl_Dprep_Prime_TotalMinutescounter++;

//	Cl_dpreptcode = Cl_Dprep_UpdatePrimeTimeInfo();
	return Cl_dpreptcode;
	

}

Cl_ReturnCodes UpdateDprepDialyserPrimeMinuteTick(void)
{
	Cl_ReturnCodes Cl_dpreptcode = CL_OK;

	Cl_Dprep_DialyserPrime_secondscounter = 0;
	Cl_Dprep_DialyserPrime_Minutescounter++;
	Cl_Dprep_DialyserPrime_TotalMinutescounter++;

	Cl_dpreptcode = Cl_Dprep_UpdateDialyserPrimeTimeInfo();
	return Cl_dpreptcode;
	

}


Cl_ReturnCodes Cl_Dprep_SendtreatementData(void)
{

	uint8_t systemdataarray[40] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	cl_PrepDatatype tempdata;
	static timecount = 0;
	Cl_ReturnCodes Cl_dprepretcode = CL_OK;
	uint8_t count = 0;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	
	//Cl_Console_bulkdatatype
	
	
						
	command = CON_TX_COMMAND_SYS_STATE_DATA ;
	systemdataarray[0] = TREATMENT_DATA;
	count++;


/*
	Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSATE_FLOW, &tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_ARTERIAL_BLOODFLOW_RATE, &tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_HEPARIN_FLOW_RATE, &tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_REMOVAL_RATE, &tempdata);

	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_HEPARIN_BOLUS, &tempdata);
						
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_GOAL, &tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_CONDUCTIVITY, &tempdata);
	Cl_SysStat_GetSensor_Status_Query(COND_STATUS_HIGH,&tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	//Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSATE_TEMP, &tempdata);
	Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
						
	//Cl_dprepretcode = (uint8_t)sv_nvmgetdata(APTSTATUS_HIGH, &tempdata);
	Cl_SysStat_GetSensor_Status_Query(APTSTATUS_HIGH,&tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	//Cl_dprepretcode = (uint8_t)sv_nvmgetdata(VPTSTATUS_HIGH, &tempdata);
	Cl_SysStat_GetSensor_Status_Query(VPTSTATUS_HIGH,&tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
						
	systemdataarray[count++] = 0xAA;
	systemdataarray[count++] = 0xAA;
*/

//	Cl_dprepretcode = Cl_SendDatatoconsole(command,&systemdataarray,count);
	
	
}
Cl_ReturnCodes Cl_Dprep_SendPrepStateData(Cl_Console_bulkdatatype datatype)
{

	uint8_t systemdataarray[40] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	cl_PrepDatatype tempdata;
	int16_t temp, temp1;
	static timecount = 0;
	static float avgtmp3=0;
	Cl_ReturnCodes Cl_dprepretcode = CL_OK;
	uint8_t count = 0;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	uint16_t sensordatamillivolts;
	//Cl_Console_bulkdatatype
	
	
						
					command = CON_TX_COMMAND_SYS_STATE_DATA ;
					systemdataarray[0] = datatype;
					count++;


					Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS,&temp);
					{
						tempdata.word = temp;

						if( temp < 0)
						{
								temp = 0;
							//	avgcond = 0;
						}
						if( temp > 2400)
						{
							tempdata.word = temp/20 + 9;
						}
						else
						{
							tempdata.word = 100;
						}
						
						tempdata.word = 139;
						systemdataarray[count++] = tempdata.bytearray[0];
						systemdataarray[count++] = tempdata.bytearray[1];
						systemdataarray[count++] = tempdata.bytearray[2];
						systemdataarray[count++] = tempdata.bytearray[3];
					}
					

		
		Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&temp);
		{
			tempdata.word = temp;
			float ftemp,ftemp1;
			ftemp = tempdata.word * 0.805;
			ftemp1 = 0.0000116 * ftemp *ftemp + 0.0035 *ftemp + 11.157 + 0.2;
			avgtmp3 =	(avgtmp3*5 + ftemp1)/6;
			//avgtmp3 = dummy3 ;
			tempdata.word = (uint16_t)(avgtmp3 * 10);
			systemdataarray[count++] = tempdata.bytearray[0];
			systemdataarray[count++] = tempdata.bytearray[1];
			systemdataarray[count++] = tempdata.bytearray[2];
			systemdataarray[count++] = tempdata.bytearray[3];
		}
		
		Cl_SysStat_GetSensor_Status_Query(SENSOR_APTSTATUS,&tempdata.Twobyte);
		{
						
		
		//	float ftemp,ftemp1;
		//	ftemp = data.twobytedata * 0.805;
		//	ftemp1 = 0.0000116 * ftemp *ftemp + 0.0035 *ftemp + 11.157 + 0.6;
		//	avgtmp3 =	(avgtmp3*5 + ftemp1)/6;
		//	data.twobytedata = (uint16_t)(avgtmp3 * 100);
							sensordatamillivolts = (tempdata.Twobyte * 0.793) ;
							calibration_apt(sensordatamillivolts);
							tempdata.word	 = pressure_final_apt;
							systemdataarray[count++] = tempdata.bytearray[0] ;
							systemdataarray[count++] = tempdata.bytearray[1] ;
							systemdataarray[count++] = tempdata.bytearray[2] ;
							systemdataarray[count++] = tempdata.bytearray[3] ;
		}
		Cl_SysStat_GetSensor_Status_Query(SENSOR_VPTSTATUS,&tempdata.Twobyte);
		{
					
			//	float ftemp,ftemp1;
			//	ftemp = data.twobytedata * 0.805;
			//	ftemp1 = 0.0000116 * ftemp *ftemp + 0.0035 *ftemp + 11.157 + 0.6;
						//	avgtmp3 =	(avgtmp3*5 + ftemp1)/6;
						//	data.twobytedata = (uint16_t)(avgtmp3 * 100);
					sensordatamillivolts = (tempdata.Twobyte * 0.793) ;
					calibration_apt(sensordatamillivolts);
					tempdata.word	 = pressure_final_vpt;
					tempdata.word	 = 30 * 100;
					systemdataarray[count++] = tempdata.bytearray[0] ;
					systemdataarray[count++] = tempdata.bytearray[1] ;
					systemdataarray[count++] = tempdata.bytearray[2] ;
					systemdataarray[count++] = tempdata.bytearray[3] ;
		}
		Cl_SysStat_GetSensor_Status_Query(SENSOR_PS1STATUS,&tempdata.Twobyte);
							sensordatamillivolts = (tempdata.Twobyte * 0.793) ;
							calibration_apt(sensordatamillivolts);
							
		Cl_SysStat_GetSensor_Status_Query(SENSOR_PS2STATUS,&tempdata.Twobyte);
							sensordatamillivolts = (tempdata.Twobyte * 0.793) ;
							calibration_apt(sensordatamillivolts);
							tempdata.word	 = ((pressure_final_apt + pressure_final_vpt ) - (pressure_final_ps1+pressure_final_ps2))/2;

		if( datatype == DIALYSIS_PREP_DATA)
		{
			
			systemdataarray[count++] = tempdata.bytearray[0] ;
			systemdataarray[count++] = tempdata.bytearray[1] ;
			systemdataarray[count++] = tempdata.bytearray[2] ;
			systemdataarray[count++] = tempdata.bytearray[3] ;
		}
		
					Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_SYS_STATE_DATA,&systemdataarray,count);
					//Cl_Dlsis_SenddlsisData();
		
	
	
}


Cl_ReturnCodes Cl_Dprep_Stoppreparation(void)
{
	
	Cl_ReturnCodes Cl_dprepretcode = CL_OK;
	
				if(!((cl_dprepstate == CL_DPREP_STATE_IDLE ) || (cl_dprepstate == CL_DPREP_STATE_STOPPED ) ||(cl_dprepstate == CL_DPREP_STATE_CRITICAL_ALARM )  ))
				{
					
					
					 	Cl_Dprepsecondscounter = 0;
					 	Cl_DprepMinutescounter= 0;
					 	Cl_Dprephourscounter= 0;
					 	Cl_DprepTotalMinutescounter= 0;
					 	Cl_DprepTotalhourscounter=0;
							Cl_Dprep_filling_secondscounter = 0;
							Cl_Dprep_filling_Minutescounter = 0;
							Cl_Dprep_filling_TotalMinutescounter = 0;
					
							Cl_Dprep_Prime_secondscounter = 0;
							Cl_Dprep_Prime_Minutescounter = 0;
							Cl_Dprep_Prime_TotalMinutescounter = 0;
					
							Cl_Dprep_DialyserPrime_secondscounter = 0;
							Cl_Dprep_DialyserPrime_Minutescounter = 0;
							Cl_Dprep_DialyserPrime_TotalMinutescounter = 0;
				Cl_dprepretcode =  sv_cntrl_deactivatevenousclamp();
				Cl_dprepretcode = sv_cntrl_setflowpath(FLOW_PATH_IDLE_RINSE);
				Cl_dprepretcode = sv_cntrl_poweroffheater();
				Cl_dprepretcode = SetHeaterState(CL_HEATER_STATE_OFF);
				cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP,0);

				
			//	cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
				
			
				cl_dprepstate = CL_DPREP_STATE_IDLE;
				//Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_RINSE_STOPPED,&data,0);
				
					
				}
}


Cl_ReturnCodes Cl_Dprep_ResetAlertsforReassertion(void )
{
	Cl_ReturnCodes 	Cl_dprepretcode = CL_OK;
	ClDprepAlarmIdType CldprepAlarmId;
	uint8_t data;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_NewAlarmIdType cl_dprepalarmid;
	
	Cl_DprepAlarmTable[CL_DPREP_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_HOLDER1STATUS_CLOSED].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_HOLDER2STATUS_CLOSED].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_COND_STATUS_LOW].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_COND_STATUS_HIGH].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_COND_DAC_OPEN].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_COND_DAC_RO].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_COND_DAC_HIGH].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_FLOW_NO_FLOW].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_FLOW_LOW_FLOWRATE].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_ABD_EVENT].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_BD_EVENT].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_BLD_EVENT].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_APTSTATUS_HIGH].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_VPTSTATUS_HIGH].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_PS1_HIGH_THRESHOLD].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_PS2_HIGH_THRESHOLD].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_FLOW_LOW_FLOWRATE].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_TEMP1_HIGH_THRESHOLD].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_TEMP2_HIGH_THRESHOLD].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_TEMP3_HIGH_THRESHOLD].IsRaised = false;

	return (Cl_dprepretcode);
	
}


Cl_ReturnCodes Cl_Dprep_Get_data(Cl_ConsoleRxDataType DataId, uint8_t size)
{
	Cl_ReturnCodes Cl_dprepretcode = CL_OK;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	uint8_t dataarray[8] =  {0,0,0,0};
	uint8_t  datasize = 0;
	
	if(DataId == CON_RX_PARAM_DATA_TREATMENT_DATA )
	{
		Cl_Dprep_SendtreatementData();
	}
	else
	{
			switch (DataId)
	{

						case	CON_RX_PARAM_DATA_RINSE_STATUS:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_RINSE_STATUS;
							Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
							datasize = 2;
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
							break;
							case	CON_RX_PARAM_DATA_PRIME_STATUS:
						//	cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_PRIME_STATUS;
							Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_NVM_PRIME_STATUS, &dataarray[1]);
							datasize = 2;
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
						break;
									
						case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
						//cl_wait(10);
			
						command = CON_TX_COMMAND_SYSDATA;

						dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
						Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
						datasize = 2;

						break;
						case	CON_RX_PARAM_DATA_DIALYSIS_FLOW:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_FLOW;
							Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSATE_FLOW, &dataarray[1]);
							datasize = 2;
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,3);
							//cl_wait(10);
							break;
							case	CON_RX_PARAM_DATA_DIALYSIS_TEMP:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_TEMP;
							Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSATE_TEMP, &dataarray[1]);
							datasize = 2;
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
						break;
						case	CON_RX_PARAM_DATA_HEPARIN_RATE:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_HEPARIN_RATE;
							//				Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_HEPARIN_RATE, &dataarray[1]);
						
						datasize = 2;
						break;
						case	CON_RX_PARAM_DATA_APT_VALUE:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_APT_VALUE;
							//			Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_APT_VALUE, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
							datasize = 2;
						break;
						case	CON_RX_PARAM_DATA_VPT_VALUE:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_VPT_VALUE;
							//	Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_VPT_VALUE, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
							datasize = 2;
						break;
						case	CON_RX_PARAM_DATA_TMP_VALUE:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_TMP_VALUE;
							//			Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
							datasize = 2;
						break;
						case	CON_RX_PARAM_DATA_COND_VALUE:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_COND_VALUE;
							//				Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
							datasize = 2;
						break;
						case	CON_RX_PARAM_DATA_UF_RATE:
						//	cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_UF_RATE;
							Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_REMOVAL_RATE, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
						//	cl_wait(10);
						datasize = 2;
						break;
						case	CON_RX_PARAM_DATA_ABF_RATE:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_ABF_RATE;
							Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_ARTERIAL_BLOODFLOW_RATE, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
							datasize = 2;
						break;
						#if 0
						case	CON_RX_PARAM_DATA_ABD_THRESHOLD:
						//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_ABD_THRESHOLD;
							Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_ABD_THRESHOLD, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
						//	cl_wait(10);
						break;
						case	CON_RX_PARAM_DATA_BLD_THRESHOLD:
						//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (uint8_t)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_BLD_THRESHOLD;
							Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_BLD_THRESHOLD, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
							datasize = 2;
						break;
						#endif
						
						
						case CON_RX_PARAM_DATA_DIALYSIS_FLOW_CONFIG: //0x13
						
						
			
						
						dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_FLOW_CONFIG;
						Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSATE_FLOW, &dataarray[1]);	
						Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSIS_FLOW_CONFIG_UPPER, &dataarray[3]);
						Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSIS_FLOW_CONFIG_LOWER, &dataarray[5]);	
						
						datasize = 	7;
							
					break;
					case CON_RX_PARAM_DATA_DIALYSIS_TEMP_CONFIG://0x14
											

											dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_TEMP_CONFIG;
											Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSATE_TEMP, &dataarray[1]);
											Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSIS_TEMP_CONFIG_UPPER, &dataarray[3]);
											Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSIS_TEMP_CONFIG_LOWER, &dataarray[5]);
											datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_HEPARIN_RATE_CONFIG://0x15

									dataarray[0] = CON_TX_PARAM_DATA_HEPARIN_RATE_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_HEPARIN_FLOW_RATE, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_HEPARIN_RATE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_HEPARIN_RATE_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_APT_VALUE_CONFIG://0x16
							
									dataarray[0] = CON_TX_PARAM_DATA_APT_VALUE_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_APT_VALUE, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_APT_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_APT_VALUE_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
									
					break;
					case CON_RX_PARAM_DATA_VPT_VALUE_CONFIG://0x17

									dataarray[0] = CON_TX_PARAM_DATA_VPT_VALUE_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_VPT_VALUE, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_VPT_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_VPT_VALUE_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_TMP_VALUE_CONFIG://0x18

									dataarray[0] = CON_TX_PARAM_DATA_TMP_VALUE_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_TMP, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_TMP_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_TMP_VALUE_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_COND_VALUE_CONFIG://0x19

									dataarray[0] = CON_TX_PARAM_DATA_COND_VALUE_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_CONDUCTIVITY, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_COND_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_COND_VALUE_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_UF_RATE_CONFIG://0x1A

									dataarray[0] = CON_TX_PARAM_DATA_UF_RATE_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_REMOVAL_RATE, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_RATE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_RATE_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_UF_GOAL_CONFIG://0x1B,

									dataarray[0] = CON_TX_PARAM_DATA_UF_GOAL_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_GOAL, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_GOAL_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_GOAL_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_BOLUS_VOLUME_CONFIG://1C

									dataarray[0] = CON_TX_PARAM_DATA_BOLUS_VOLUME_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_HEPARIN_BOLUS, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_BOLUS_VOLUME_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_BOLUS_VOLUME_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_ABF_RATE_CONFIG://0X1D

									dataarray[0] = CON_TX_PARAM_DATA_ABF_RATE_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_HEPARIN_BOLUS, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_ABF_RATE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_ABF_RATE_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
						default:
						break;
	}
					command = CON_TX_COMMAND_SYSDATA;
					Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,datasize);
		
	}

}
Cl_ReturnCodes Cl_Dprep_setdata(Cl_ConsoleRxDataType DataId,cl_PrepDatatype cl_PrepData , uint8_t size)
{
	Cl_ReturnCodes Cl_dprepretcode = CL_OK;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	uint8_t dataarray[8] =  {0,0,0,0};
	uint8_t  databytecnt = 0;

				switch(DataId)
				{
	 
					 case	CON_RX_PARAM_DATA_DIALYSIS_FLOW:
					 Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSATE_FLOW,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);					 
					 dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_FLOW;
					 Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSATE_FLOW, &dataarray[1]);
					
					 break;
					 case	CON_RX_PARAM_DATA_DIALYSIS_TEMP:
					 Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSATE_TEMP,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_TEMP; 
					 Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSATE_TEMP, &dataarray[1]);

					 break;
	 
					 case	CON_RX_PARAM_DATA_HEPARIN_RATE:
					 Cl_dprepretcode =sv_nvmsetdata(NV_HEPARIN_FLOW_RATE,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_HEPARIN_RATE;
					 Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_HEPARIN_FLOW_RATE, &dataarray[1]);
					 break;
					 
					case CON_RX_PARAM_DATA_BOLUS_VOLUME:

					 Cl_dprepretcode =sv_nvmsetdata(NV_HEPARIN_BOLUS,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_BOLUS_VOLUME;
					 Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_HEPARIN_BOLUS, &dataarray[1]);

					break;
					 case	CON_RX_PARAM_DATA_APT_VALUE:
					 Cl_dprepretcode =sv_nvmsetdata(NV_APT_VALUE,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_APT_VALUE;	 
					 Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_APT_VALUE, &dataarray[1]);

					 break;
					 case	CON_RX_PARAM_DATA_VPT_VALUE:
					 Cl_dprepretcode =sv_nvmsetdata(NV_VPT_VALUE,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_VPT_VALUE;	 
					 Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_VPT_VALUE, &dataarray[1]);

					 break;
					 case	CON_RX_PARAM_DATA_TMP_VALUE:

					 break;
					 case	CON_RX_PARAM_DATA_COND_VALUE:
					 Cl_dprepretcode =sv_nvmsetdata(NV_CONDUCTIVITY,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_COND_VALUE;
					 Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_CONDUCTIVITY, &dataarray[1]);

					 break;

					 case	CON_RX_PARAM_DATA_UF_RATE:
					 Cl_dprepretcode =sv_nvmsetdata(NV_UF_REMOVAL_RATE,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_UF_RATE;
					 Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_REMOVAL_RATE, &dataarray[1]);

					 break;
					 case CON_RX_PARAM_DATA_UF_GOAL:
					 Cl_dprepretcode =sv_nvmsetdata(NV_UF_GOAL,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_UF_GOAL;
					 Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_GOAL, &dataarray[1]);

					 break;
					 case	CON_RX_PARAM_DATA_ABF_RATE:
					 Cl_dprepretcode =sv_nvmsetdata(NV_HEPARIN_FLOW_RATE,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);					 
					 command = CON_TX_COMMAND_SYSDATA;					 
					 dataarray[0] = CON_TX_PARAM_DATA_ABF_RATE;
					 Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_ARTERIAL_BLOODFLOW_RATE, &dataarray[1]);					 
					 break;
					
					#if 0
					 case CON_RX_PARAM_DATA_ABD_THRESHOLD://0x0C
					 Cl_dprepretcode =sv_nvmsetdata(NV_ABD_THRESHOLD,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 command = CON_TX_COMMAND_SYSDATA;
					 dataarray[0] = CON_TX_PARAM_DATA_ABD_THRESHOLD;
					 Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_ABD_THRESHOLD, &dataarray[1]);		
					 
					 break;
					 case CON_RX_PARAM_DATA_BLD_THRESHOLD://0x0D
					 Cl_dprepretcode =sv_nvmsetdata(NV_BLD_THRESHOLD,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 command = CON_TX_COMMAND_SYSDATA;
					 dataarray[0] = CON_TX_PARAM_DATA_ABF_RATE;
					 Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_BLD_THRESHOLD, &dataarray[1]);
					 break;
					 
					#endif
					case CON_RX_PARAM_DATA_DIALYSIS_FLOW_CONFIG: //0x13
						
						
						
						Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSATE_FLOW,&cl_PrepData.bytearray[0],2);
						Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSIS_FLOW_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
						Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSIS_FLOW_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
						
						dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_FLOW_CONFIG;
						Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSATE_FLOW, &dataarray[1]);	
						Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSIS_FLOW_CONFIG_UPPER, &dataarray[3]);
						Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSIS_FLOW_CONFIG_LOWER, &dataarray[5]);		
							
					break;
					case CON_RX_PARAM_DATA_DIALYSIS_TEMP_CONFIG://0x14
											
											Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSATE_TEMP,&cl_PrepData.bytearray[0],2);
											Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSIS_TEMP_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
											Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSIS_TEMP_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
											
											dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_TEMP_CONFIG;
											Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSATE_TEMP, &dataarray[1]);
											Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSIS_TEMP_CONFIG_UPPER, &dataarray[3]);
											Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_DIALYSIS_TEMP_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_HEPARIN_RATE_CONFIG://0x15
									Cl_dprepretcode =sv_nvmsetdata(NV_HEPARIN_FLOW_RATE,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_HEPARIN_RATE_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_HEPARIN_RATE_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_HEPARIN_RATE_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_HEPARIN_FLOW_RATE, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_HEPARIN_RATE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_HEPARIN_RATE_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_APT_VALUE_CONFIG://0x16
									Cl_dprepretcode =sv_nvmsetdata(NV_APT_VALUE,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_APT_VALUE_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_APT_VALUE_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
																
									dataarray[0] = CON_TX_PARAM_DATA_APT_VALUE_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_APT_VALUE, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_APT_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_APT_VALUE_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_VPT_VALUE_CONFIG://0x17
									Cl_dprepretcode =sv_nvmsetdata(NV_VPT_VALUE,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_VPT_VALUE_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_VPT_VALUE_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_VPT_VALUE_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_VPT_VALUE, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_VPT_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_VPT_VALUE_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_TMP_VALUE_CONFIG://0x18
									Cl_dprepretcode =sv_nvmsetdata(NV_TMP,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_TMP_VALUE_CONFIG_UPPER,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_TMP_VALUE_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_TMP_VALUE_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_TMP, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_TMP_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_TMP_VALUE_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_COND_VALUE_CONFIG://0x19
									Cl_dprepretcode =sv_nvmsetdata(NV_CONDUCTIVITY,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_COND_VALUE_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_COND_VALUE_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_COND_VALUE_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_CONDUCTIVITY, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_COND_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_COND_VALUE_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_UF_RATE_CONFIG://0x1A
									Cl_dprepretcode =sv_nvmsetdata(NV_UF_REMOVAL_RATE,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_UF_RATE_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_UF_RATE_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_UF_RATE_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_REMOVAL_RATE, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_RATE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_RATE_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_UF_GOAL_CONFIG://0x1B,
									Cl_dprepretcode =sv_nvmsetdata(NV_UF_GOAL,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_UF_GOAL_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_UF_GOAL_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_UF_GOAL_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_GOAL, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_GOAL_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_UF_GOAL_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_BOLUS_VOLUME_CONFIG://1C
									Cl_dprepretcode =sv_nvmsetdata(NV_HEPARIN_BOLUS,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_BOLUS_VOLUME_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_BOLUS_VOLUME_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_BOLUS_VOLUME_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_HEPARIN_BOLUS, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_BOLUS_VOLUME_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_BOLUS_VOLUME_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_ABF_RATE_CONFIG://0X1D
									Cl_dprepretcode =sv_nvmsetdata(NV_ARTERIAL_BLOODFLOW_RATE,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_ABF_RATE_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_ABF_RATE_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_ABF_RATE_CONFIG;
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_ARTERIAL_BLOODFLOW_RATE, &dataarray[1]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_ABF_RATE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (uint8_t)sv_nvmgetdata(NV_ABF_RATE_CONFIG_LOWER, &dataarray[5]);
					break;
					
					 default:
					 break;
				}
					command = CON_TX_COMMAND_SYSDATA;
					Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,size);
}

Cl_ReturnCodes	Cl_Dprep_StartPreparation(void)
	{
		
			Cl_ReturnCodes Cl_dprepretcode = CL_OK;
			uint8_t data =0;
				
												
							Cl_dprepretcode = Cl_DprepSelectDialysateInlet();
							//Cl_DprepFlowOn();

							data = 1;
							Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DLSIS_PREP_CNFRM,&data,0);	
							Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSATE_FILLING_STARTED,&data,0);																			

						
					//		Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(HOLDER1STATUS_OPEN,LOGIC_LOW,0,0,0);
					//		Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(HOLDER2STATUS_OPEN,LOGIC_LOW,0,0,0);
							
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true );
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON,true );
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_ON_TO_OFF,true );
							Cl_dprepretcode =  sv_cntrl_activatevenousclamp();
							

							Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,true );
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,true );
							
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true );
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,true );					
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,true );
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true );
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,true );
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,true );
							
							
						//	Cl_dprepretcode =  Cl_AlarmActivateAlarms(SENSOR_TEMP3STATUS,true );
						//	Cl_dprepretcode =  Cl_AlarmActivateAlarms(SENSOR_TEMP2STATUS,true );
							
							
						//	Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS1STATUS_HIGH,true );
						//	Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS2STATUS_HIGH,true );
							
						//	Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS3STATUS_HIGH,true );
						//	Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,true );
							
						//	Cl_dprepretcode =  Cl_AlarmActivateAlarms(APTSTATUS_HIGH,true );
						//	Cl_dprepretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_HIGH,true );
							if(Current_sense_trigger)
							{
								Cl_dprepretcode =  Cl_AlarmActivateAlarms( FPCURRENTSTATUS,true );
							}
							else
							{
								//			Cl_rinseretcode =  Cl_AlarmActivateAlarms( PS3STATUS_HIGH,true );
							}
							Cl_dprepretcode =  sv_cntrl_activatepump(DCMOTOR1);
							Cl_dprepretcode =  sv_cntrl_activatepump(DCMOTOR2);
							Cl_dprepretcode = Cl_bc_controller(BC_EVENT_RESUME);
							//sv_cntrl_enable_bypass();
							sv_prop_startmixing();
							Cl_dprepretcode = SetHeaterState(CL_HEATER_STATE_ON);
							cl_dprepstate = CL_DPREP_STATE_DPREP_FILLING;
							Cl_Dprep_ResetAlertsforReassertion();
							return 0;
				
	}
	
	
	Cl_ReturnCodes	Cl_Dprep_StartDialyserPrime(void)
	{
		
		
			Cl_ReturnCodes Cl_dprepretcode = CL_OK;
			uint8_t data =0;
				
			Cl_dprepMinutescounter = 0;
			Cl_dprepsecondscounter = 0;
			//cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME;
			//Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_DIALISER_PRIME_CNFRM,&data,0);
			
			//check bypass switches
		//	cl_dprep_primecontroller(CL_DPREP_PRIME_BLOODPUMP_START,0);
			Cl_dprepretcode = Cl_DprepSelectDialysateInlet();
			Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(BLOODDOOR_STATUS_OPEN,LOGIC_LOW,0,0,0);
			Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(HOLDER1STATUS_OPEN,LOGIC_LOW,0,0,0);
			Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(HOLDER2STATUS_OPEN,LOGIC_LOW,0,0,0);
			

			Cl_dprepretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON,true );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_ON_TO_OFF,true );
			

	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,true );
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,true );
			
			
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(SENSOR_TEMP3STATUS,true );
		//	Cl_dprepretcode =  Cl_AlarmActivateAlarms(SENSOR_TEMP2STATUS,true );
			
			
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS1STATUS_HIGH,true );
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS2STATUS_HIGH,true );
			
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS3STATUS_HIGH,true );
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,true );
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,true );
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_DAC_OPEN,true );
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_DAC_RO,true );
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_DAC_HIGH,true );
			
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(APTSTATUS_HIGH,true );
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_HIGH,true );
			if(Current_sense_trigger)
			{
				Cl_dprepretcode =  Cl_AlarmActivateAlarms( FPCURRENTSTATUS,true );
			}
			else
			{
				//			Cl_rinseretcode =  Cl_AlarmActivateAlarms( PS3STATUS_HIGH,true );
			}
			
			Cl_dprepretcode =  sv_cntrl_activatepump(DCMOTOR1);
			Cl_dprepretcode =  sv_cntrl_activatepump(DCMOTOR2);
			Cl_dprepretcode =  sv_cntrl_disable_bypass();
			if(cl_dprep_prime_state != CL_DPREP_PRIME_STATE_DIALYSER_PRIMING)
			{
				cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_START_DIALYSER_PRIMING,0);
			}
			Cl_dprepretcode = sv_cntrl_disable_loopback(); 
			sv_prop_startmixing();
			Cl_dprepretcode = Cl_bc_controller(BC_EVENT_RESUME);
			Cl_dprepretcode = SetHeaterState(CL_HEATER_STATE_ON);
			
			UpdateDprepFillingMinuteTick();
			
			cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME;
							
			return 0;
	}
	
Cl_ReturnCodes Cl_dprep_StopMixing(void)
{
				Cl_ReturnCodes Cl_dprepretcode = CL_OK;
				uint8_t data;
				if(cl_dprepstate == CL_DPREP_STATE_DPREP_FILLING)
				{
					
				Cl_dprepMinutescounter = 0;
				Cl_dprepsecondscounter = 0;
;
				Cl_dprepretcode = sv_cntrl_poweroffheater();
				Cl_bc_controller(BC_EVENT_STOP);
				//Cl_rinseretcode = sv_cntrl_setflowpath(FLOW_PATH_IDLE_RINSE);
				cl_dprepstate = CL_DPREP_STATE_STOPPED;
				Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSATE_FILLING_STOPPED,&data,0);
				
					
				}

				return Cl_dprepretcode;
}


	
