/*
 * cl_dprep_primecontroller.c
 *
 * Created: 11/3/2014 1:03:14 PM
 *  Author: user
 */ 
#include "inc/cl_dprep_primecontroller.h"
#include "cl_app/cl_console/inc/cl_consolecontroller.h"
#include "cl_app/inc/cl_types.h"
#include "inc/cl_dprep_controller.h"
#include "sv_stubs/inc/sv_types.h"
#include "Platform/Service/sv_interface.h"
#include "../comp/bloodpumpcntrl/inc/cl_bloodpumpcontroller.h"

Cl_Dprep_PrimeStates cl_dprep_prime_state = CL_DPREP_PRIME_STATE_IDLE;
Cl_Dprep_PrimeEvents cl_dprep_prime_event = CL_DPREP_PRIME_PRIME_NULL;
static int8_t cl_prime_sec = 0;
static int8_t cl_prime_min = 0;
static int8_t cl_dialyser_prime_sec = 0;
static int8_t cl_dialyser_prime_min = 0;
Cl_ReturnCodes cl_dprep_primecontroller(Cl_Dprep_PrimeEvents,int16_t);
Cl_ReturnCodes Cl_Dprep_primeUpdatePrimeTimeInfo(void);
Cl_ReturnCodes Cl_Dprep_UpdateDialyserPrimeTimeInfo(void);
Cl_ReturnCodes cl_dprep_activate_prime_related_alarms(void);


extern Cl_ReturnCodes        Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , uint8_t* ,uint8_t );
extern uint8_t  sv_cntrl_activatepump(sv_pumptype);
extern uint8_t  sv_cntrl_deactivatepump(sv_pumptype);
extern uint8_t sv_cntrl_setpumpspeed(sv_pumptype sv_pump_id,uint32_t speed);
extern Cl_ReturnCodes cl_bp_controller(cl_bp_events bp_event , int16_t data);
extern Cl_ReturnCodes Cl_Dprep_SendPrepStateData(Cl_Console_bulkdatatype datatype);
extern Cl_ReturnCodes Cl_AlarmConfigureAlarmType(Cl_NewAlarmIdType,Cl_AlarmTriggerType,uint16_t,uint16_t,uint8_t);
extern Cl_ReturnCodes Cl_AlarmActivateAlarms(Cl_NewAlarmIdType,bool );


Cl_ReturnCodes cl_dprep_primecontroller(Cl_Dprep_PrimeEvents prime_event , int16_t data)
{
	Cl_ReturnCodes 	 Cl_dprep_primeretcode = CL_ERROR;
		switch(cl_dprep_prime_state)
		{
			case CL_DPREP_PRIME_STATE_IDLE:
				switch(prime_event)
				{
					case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
					cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
					break;
					case CL_DPREP_PRIME_PRIME_START:
					Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PRIME2",6);
						cl_bp_controller(CL_BP_EVENT_START,0);
						Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
						Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STARTED,&data,0);
						Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
						cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIMING;
					break;
					default:break;
				}
			break;
			case CL_DPREP_PRIME_STATE_PRIMING:
				switch(prime_event)
				{
					case CL_DPREP_PRIME_PRIME_TICK_SEC:
					Cl_dprep_primeretcode = Cl_Dprep_SendPrepStateData(PRIMING_DATA);
					if(cl_prime_sec++ >= 60)
					{
						cl_prime_sec = 0;
						cl_prime_min++;
						Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
						
					}
					if(cl_prime_min >= CL_DPREP_PRIME_TIMEOUT_MIN )
					{
						cl_bp_controller(CL_BP_EVENT_STOP,0);
						
						Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_COMPLETED,&data,0);
						cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_COMPLETED;
					}

					break;
					case CL_DPREP_PRIME_PRIME_SET_RATE:
					break;
					case CL_DPREP_PRIME_PRIME_STOP:
					cl_bp_controller(CL_BP_EVENT_STOP,0);
					Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STOPPED,&data,0);
					cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_STOPPED;
					break;
					case CL_DPREP_PRIME_PRIME_PAUSE:
					cl_bp_controller(CL_BP_EVENT_STOP,0);
					cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_PAUSED;
					break;
					case 	CL_DPREP_PRIME_BLOODPUMP_START:
					 cl_bp_controller(CL_BP_EVENT_START,0);
					break;
					case CL_DPREP_PRIME_BLOODPUMP_STOP:
					 cl_bp_controller(CL_BP_EVENT_STOP,0);
					break;
					case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
					cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
					break;
					default:break;
				}

			
			break;
			case CL_DPREP_PRIME_STATE_PRIME_STOPPED:
						switch(prime_event)
						{
							case CL_DPREP_PRIME_PRIME_START:
								//cl_bp_controller(CL_BP_EVENT_START,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STARTED,&data,0);
								Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
								cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIMING;
							break;
							case 	CL_DPREP_PRIME_BLOODPUMP_START:
							cl_bp_controller(CL_BP_EVENT_START,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
							cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
							break;
						}
			break;
			case CL_DPREP_PRIME_STATE_PRIME_PAUSED:
						switch(prime_event)
						{
							case CL_DPREP_PRIME_PRIME_START:
								cl_bp_controller(CL_BP_EVENT_START,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STARTED,&data,0);
								Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
								cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIMING;
							break;
							case CL_DPREP_PRIME_PRIME_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STOPPED,&data,0);
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_STOPPED;
							break;
							case 	CL_DPREP_PRIME_BLOODPUMP_START:
							cl_bp_controller(CL_BP_EVENT_START,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
							cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
							break;
						}
			break;
			case CL_DPREP_PRIME_STATE_PRIME_COMPLETED:
						switch(prime_event)
						{
							case CL_DPREP_PRIME_RCIRC_START:
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RCIRC2",6);
							cl_prime_min =  0;
							cl_prime_sec = 0;
					
								cl_bp_controller(CL_BP_EVENT_START,0);
								//Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CONT_TX_COMMAND_SYS_RCIRC_STARTED,&data,0);
								Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
								cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_RCIRC_STARTED;
							break;
							case CL_DPREP_PRIME_RCIRC_STOP:
							break;
							case CL_DPREP_PRIME_PRIME_START:
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PRIME2",6);
							cl_prime_min =  0;
							cl_prime_sec = 0;
					
								cl_bp_controller(CL_BP_EVENT_START,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STARTED,&data,0);
								Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
								cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIMING;
							break;

							case CL_DPREP_PRIME_PRIME_START_DIALYSER_PRIMING:
							cl_bp_controller(CL_BP_EVENT_START,0);
						//	Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_DIALISER_PRIME_CNFRM,&data,0);
							Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_PRIME_STARTED,&data,0);
							Cl_dprep_primeretcode = Cl_Dprep_UpdateDialyserPrimeTimeInfo();
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING;
							break;
							case 	CL_DPREP_PRIME_BLOODPUMP_START:
							cl_bp_controller(CL_BP_EVENT_START,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
							cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
							break;
							
						}
						
			break;
			case CL_DPREP_PRIME_STATE_PRIME_RCIRC_STARTED:
		
					switch(prime_event)
					{
						case CL_DPREP_PRIME_PRIME_TICK_SEC:
						if(cl_prime_sec++ >= 60)
						{
							cl_prime_sec = 0;
							cl_prime_min++;
							Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
						}
						if(cl_prime_min >= CL_DPREP_PRIME_TIMEOUT_MIN )
						{
							cl_bp_controller(CL_BP_EVENT_STOP,0);
						
							Cl_dprep_primeretcode = Cl_SendDatatoconsole(CONT_TX_COMMAND_SYS_RCIRC_COMPLETED,&data,0);
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_RCIRC_COMPLETED;
						}

						break;
						case CL_DPREP_PRIME_PRIME_SET_RATE:
						break;
						case CL_DPREP_PRIME_PRIME_STOP:
						cl_bp_controller(CL_BP_EVENT_STOP,0);
						Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STOPPED,&data,0);
						cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_STOPPED;
						break;
						case CL_DPREP_PRIME_PRIME_PAUSE:
						cl_bp_controller(CL_BP_EVENT_STOP,0);
						cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_PAUSED;
						break;
						case 	CL_DPREP_PRIME_BLOODPUMP_START:
						 cl_bp_controller(CL_BP_EVENT_START,0);
						break;
						case CL_DPREP_PRIME_BLOODPUMP_STOP:
						 cl_bp_controller(CL_BP_EVENT_STOP,0);
						break;
						case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
						cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
						break;
						default:break;
					}

			
			break;
			case CL_DPREP_PRIME_STATE_PRIME_RCIRC_COMPLETED:
						switch(prime_event)
						{
							case CL_DPREP_PRIME_RCIRC_START:
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RCIRC2",6);
							cl_prime_min =  0;
							cl_prime_sec = 0;
					
								cl_bp_controller(CL_BP_EVENT_START,0);
								//Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CONT_TX_COMMAND_SYS_RCIRC_STARTED,&data,0);
								Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
								cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_RCIRC_STARTED;
							break;
							case CL_DPREP_PRIME_RCIRC_STOP:
							break;
							case CL_DPREP_PRIME_PRIME_START:
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PRIME2",6);
							cl_prime_min =  0;
							cl_prime_sec = 0;
					
								cl_bp_controller(CL_BP_EVENT_START,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STARTED,&data,0);
								Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
								cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIMING;
							break;

							case CL_DPREP_PRIME_PRIME_START_DIALYSER_PRIMING:
							cl_bp_controller(CL_BP_EVENT_START,0);
						//	Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_DIALISER_PRIME_CNFRM,&data,0);
							Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_PRIME_STARTED,&data,0);
							Cl_dprep_primeretcode = Cl_Dprep_UpdateDialyserPrimeTimeInfo();
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING;
							break;
							case 	CL_DPREP_PRIME_BLOODPUMP_START:
							cl_bp_controller(CL_BP_EVENT_START,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
							cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
							break;
							
						}
			break;
			
			case CL_DPREP_PRIME_STATE_DIALYSER_PRIMING:
						switch(prime_event)
						{
							case CL_DPREP_PRIME_PRIME_TICK_SEC:
							if(cl_dialyser_prime_sec++ >= 60)
							{
								cl_dialyser_prime_sec = 0;
								cl_dialyser_prime_min++;
							}
							if(cl_dialyser_prime_min >= CL_DPREP_DIALISER_PRIME_TIMEOUT_MIN )
							{
								cl_bp_controller(CL_BP_EVENT_STOP,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALISYS_PRIME_COMPLETED,&data,0);
								cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_COMPLETED;
						
							}
							break;
							case CL_DPREP_PRIME_PRIME_SET_RATE:
							break;
							case CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING:
							case CL_DPREP_PRIME_PRIME_STOP:
							
								cl_bp_controller(CL_BP_EVENT_STOP,0);
								if(data == 1)
								{
									Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALISYS_PRIME_COMPLETED,&data,0);
									cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_COMPLETED;
								}else
								{
									cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_STOPPED;
								}

								
								
							break;
							case CL_DPREP_PRIME_PRIME_PAUSE_DIALYSER_PRIMING:
							
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_PAUSED;
							break;
							case 	CL_DPREP_PRIME_BLOODPUMP_START:
							cl_bp_controller(CL_BP_EVENT_START,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
							cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
							break;
							default:break;
						}
			break;
			case CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_STOPPED:
						switch(prime_event)
						{
							case CL_DPREP_PRIME_PRIME_START_DIALYSER_PRIMING:
							cl_bp_controller(CL_BP_EVENT_START,0);
					//		Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_DIALISER_PRIME_CNFRM,&data,0);
							Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_PRIME_STARTED,&data,0);
							Cl_dprep_primeretcode = Cl_Dprep_UpdateDialyserPrimeTimeInfo();
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING;
							break;
							case 	CL_DPREP_PRIME_BLOODPUMP_START:
							cl_bp_controller(CL_BP_EVENT_START,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
							cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
							break;
						}
			break;
			case CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_PAUSED:
						switch(prime_event)
						{
							case CL_DPREP_PRIME_PRIME_START_DIALYSER_PRIMING:
							cl_bp_controller(CL_BP_EVENT_START,0);
						//	Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_DIALISER_PRIME_CNFRM,&data,0);
							Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_PRIME_STARTED,&data,0);
							Cl_dprep_primeretcode = Cl_Dprep_UpdateDialyserPrimeTimeInfo();
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING;
							break;
							case CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING:
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_STOPPED;
							break;
							case 	CL_DPREP_PRIME_BLOODPUMP_START:
							cl_bp_controller(CL_BP_EVENT_START,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
							cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
							break;

						}
			break;
			case CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_COMPLETED:
			break;


		}
	

	
	return 0;
}


Cl_ReturnCodes Cl_Dprep_primeUpdatePrimeTimeInfo(void)
{
	
	Cl_ReturnCodes  Cl_dprep_primeretcode = CL_ERROR;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	uint8_t data[7] = {0,0,0,0,0,0,0};
	
	command = CON_TX_COMMAND_REM_TIME;

	data[0] = (uint8_t) PRIMING_DATA;

	data[1]= (uint8_t)cl_prime_min;
	data[2]= (uint8_t)0;
	data[3]= (uint8_t)cl_prime_sec;
	data[4]= (uint8_t) (CL_DPREP_PRIME_TIMEOUT_MIN - cl_prime_min );
	data[5]= (uint8_t) 0;
	data[6]= (uint8_t) 0;
	
	Cl_dprep_primeretcode = Cl_SendDatatoconsole(command,&data,7);
	
	return CL_OK;
}

Cl_ReturnCodes Cl_Dprep_UpdateDialyserPrimeTimeInfo(void)
{
	
	Cl_ReturnCodes  Cl_dprep_primeretcode = CL_ERROR;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	uint8_t data[7] = {0,0,0,0,0,0,0};
	
	command = CON_TX_COMMAND_REM_TIME;

	data[0] = (uint8_t) DIALYSER_PRIME_DATA;

	data[1]= (uint8_t)cl_dialyser_prime_min;
	data[2]= (uint8_t)0;
	data[3]= (uint8_t)cl_dialyser_prime_sec;
	data[4]= (uint8_t) (CL_DPREP_DIALISER_PRIME_TIMEOUT_MIN - cl_prime_min );
	data[5]= (uint8_t) 0;
	data[6]= (uint8_t) 0;
	
	Cl_dprep_primeretcode = Cl_SendDatatoconsole(command,&data,7);
	
	return CL_OK;

}
		
Cl_ReturnCodes cl_dprep_activate_prime_related_alarms(void)
{
			Cl_ReturnCodes  Cl_dprep_primeretcode = CL_ERROR;
			//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(APTSTATUS_HIGH,true );
			//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_HIGH,true );
			Cl_dprep_primeretcode =  Cl_AlarmConfigureAlarmType(BLOODDOOR_STATUS_OPEN,LOGIC_LOW,0,0,0);
			Cl_dprep_primeretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true );
	
}