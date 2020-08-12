/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Periodic Advertisement Scanner Program
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "app.h"
#include "retargetserial.h"

/* Timer and GPIO libraries*/
#include "em_timer.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_msc.h"
#include "em_prs.h"

#define RX_OBS_PRS_CHANNEL 0

/* Print boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);

/* Flag for indicating DFU Reset must be performed */
static uint8_t boot_to_dfu = 0;

struct Advertising_Parameters {
	bd_addr     local_addr;
	bd_addr	    connection_addr;
	uint8_t     ad_handle;
	uint16_t 	min_interval;
	uint16_t 	max_interval;
	uint8_t		ch_map[5];
};

struct CTE_TX_params {
	uint8_t		cte_length;
	uint8_t		cte_type;
	uint8_t		cte_count;
	uint8_t		s_len;
	uint8_t		sa[1];
};
struct CTE_RX_params {
	uint8_t		sync_handle;
	uint8_t		slot_dur;
	uint8_t		cte_count;
	uint8_t		s_len;
	uint8_t		sa[1];
};

static struct Advertising_Parameters ad_params;
static struct CTE_RX_params RX_params;
static struct CTE_TX_params TX_params;

bool compare_bd_addr(bd_addr *address1, bd_addr *address2){
	for(int i = 0;i<5;i++){
		if(address1->addr[5-i] > address2->addr[i]){
			return false;
		}
	}
	return true;
}
//
const uint8 periodicSyncService[16] = {0x81,0xc2,0x00,0x2d,0x31,0xf4,0xb0,0xbf,0x2b,0x42,0x49,0x68,0xc7,0x25,0x71,0x41};

// Parse advertisements looking for advertised periodicSync Service.
static uint8_t findServiceInAdvertisement(uint8_t *data, uint8_t len)
{
  uint8_t adFieldLength;
  uint8_t adFieldType;
  uint8_t i = 0;
  printLog("packet length %d\r\n", len);
  // Parse advertisement packet
  while (i < len) {
    adFieldLength = data[i];
    adFieldType = data[i + 1];
    // Partial ($02) or complete ($03) list of 128-bit UUIDs
	printLog("adField type %d \r\n", adFieldType);
    if (adFieldType == 0x06 || adFieldType == 0x07) {
      // compare UUID to service UUID
      if (memcmp(&data[i + 2], periodicSyncService, 16) == 0) {
        return 1;
      }
    }
    // advance to the next AD struct
    i = i + adFieldLength + 1;
  }
  return 0;
}

static volatile uint32_t overflow = 5;
static volatile uint32_t* ad_data_ptr[4];
uint8_t temp[16];

void initGpio(void){
	// Turn on the GPIO clocks to access their registers.
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_PRS, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
}

void initPrs(void){
	// Configure PRS Channel 0 to output RAC_RX.
	PRS_SourceAsyncSignalSet(0,PRS_ASYNC_CH_CTRL_SOURCESEL_RACL,_PRS_ASYNC_CH_CTRL_SIGSEL_RACLRX);

	//PRS_PinOutput(RX_OBS_PRS_CHANNEL, prsTypeAsync, OBS_PRS_PORT, RX_OBS_PRS_PIN);
	PRS_Combine (0, 0, prsLogic_A);
	PRS_ConnectConsumer(0, prsTypeAsync, prsConsumerTIMER0_CC0);
	//PRS_ConnectConsumer(1, prsTypeAsync, prsConsumerTIMER1_CC1);
}

void initTimer(void){
	// Configure CC TIMER0
	TIMER_InitCC_TypeDef timerCCInit0 = TIMER_INITCC_DEFAULT;
	timerCCInit0.eventCtrl = timerEdgeFalling;
	timerCCInit0.edge = timerEdgeFalling;
	timerCCInit0.mode = timerCCModeCapture;
	timerCCInit0.prsSel = RX_OBS_PRS_CHANNEL;
	timerCCInit0.prsInput = true;
	timerCCInit0.prsInputType = timerPrsInputAsyncLevel;
	TIMER_InitCC(TIMER0, 0, &timerCCInit0);
	// Configure TIMER0
	TIMER_Init_TypeDef timerInit0 = TIMER_INIT_DEFAULT;
	timerInit0.clkSel = cmuClock_TIMER0;
	timerInit0.debugRun = false;
	timerInit0.prescale = timerPrescale1;
	timerInit0.enable = true;
	timerInit0.mode = timerModeUp;
	//timerInit0.riseAction = timerInputActionReloadStart;
	TIMER_TopSet(TIMER0, TIMER_MaxCount(TIMER0));
	TIMER_Init(TIMER0, &timerInit0);

	ad_data_ptr[0] = &TIMER0->CNT;
/*
	// Configure CC TIMER1
	TIMER_InitCC_TypeDef timerCCInit1 = TIMER_INITCC_DEFAULT;


	TIMER_InitCC(TIMER1, 0, &timerCCInit1);
	// Configure TIMER1
	TIMER_Init_TypeDef timerInit1 = TIMER_INIT_DEFAULT;
	timerInit1.prescale = timerPrescale4;
	timerInit1.enable = false;
	timerInit1.mode = timerModeUp;
	TIMER_TopSet(TIMER1, 35625);
	TIMER_Init(TIMER1, &timerInit1);
*/

	// Enable overflow and CC0 interrupt
	TIMER_IntEnable(TIMER0, TIMER_IF_OF | TIMER_IF_CC0);
	// Enable overflow
	//TIMER_IntEnable(TIMER1, TIMER_IF_OF);
	// Enable TIMER0 interrupt vector in NVIC
	NVIC_EnableIRQ(TIMER0_IRQn);
	//NVIC_EnableIRQ(TIMER1_IRQn);
}

void setTemp(){
	memcpy(temp, *ad_data_ptr, sizeof(uint32_t));
	memcpy(temp+4, *(ad_data_ptr+1), sizeof(uint32_t));
	memcpy(temp+8, *(ad_data_ptr+2), sizeof(uint32_t));
	memcpy(temp+12, *(ad_data_ptr+3), sizeof(uint32_t));
}

void TIMER0_IRQHandler(void){
	uint32_t flags = TIMER_IntGet(TIMER0);
	TIMER_IntClear(TIMER0, flags);
	// Check if the timer overflowed
	//overflow = 4;
	if (flags & TIMER_IF_OF) {
		overflow++;
	}
	//update timer
	if (flags & TIMER_IF_CC0){
		//setTemp();
		//uint16_t result = gecko_cmd_le_gap_bt5_set_adv_data(0, 8,  sizeof(uint32_t)*4, temp)->result;
		//printLog("RES: 0x%x\r\n", result);
	}
}


void initializeTimers()
{
	initGpio();
	initPrs();
	initTimer();
}
/*
void updateTime(uint32_t* delta, uint32_t* old, uint8_t ad_data[12]){
	if(overflow>0){
		*(delta) += ((TIMER_TopGet(TIMER0)*overflow +1)+*timer_count);
		overflow = 0;
	}
	else
		*(delta) += *timer_count;
	TIMER_CounterSet(TIMER0, 0);
	memcpy(ad_data,delta, sizeof(uint32_t));
	gecko_cmd_le_gap_bt5_set_adv_data(0, 8, sizeof(uint8_t)*12, ad_data);
}
*/




/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif

	/* Set the maximum number of periodic sync allowed*/
	pconfig->bluetooth.max_periodic_sync = 1;

	/* Set maximum number of periodic advertisers */
	pconfig->bluetooth.max_advertisers = 1;

	/* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
	initLog();

	/* Initialize stack */
	gecko_init(pconfig);

	gecko_init_periodic_advertising();

	uint8 sync_handle;
	gecko_bgapi_class_sync_init();

	// Initialize advertising parameters
	ad_params.min_interval 	= 120;
	ad_params.max_interval 	= 120;
	ad_params.ad_handle 	= 0;
	ad_params.ch_map[0]   	= 3;
	for(int i = 1; i<5; i++)
		ad_params.ch_map[i] = 0;

	// Initialize CTE TX parameters
	TX_params.cte_length 	= 0x14;
	TX_params.cte_type		= 0x0;
	TX_params.cte_count 	= 1;
	TX_params.s_len 		= 1;
	TX_params.sa[0] 		= 0;

	// Initialize CTE RX parameters
	RX_params.slot_dur		= 1;
	RX_params.cte_count 	= 0;
	RX_params.s_len 		= 1;
	RX_params.sa[0] 		= 0;


	//Initialize CTE Reciever & Transmitter
	gecko_bgapi_class_cte_receiver_init();
	gecko_bgapi_class_cte_transmitter_init();

	uint16 result;
	uint32_t offset;
	uint32_t* data = malloc(16);
	uint8_t itr = 0;
	uint8_t* ram = malloc(184*10);

	initializeTimers();
	uint32_t* other_overflow = malloc(4);
	uint32_t* other_time = malloc(4);
	uint32_t* num = malloc(4);
	*(other_overflow) = 0;
	*(other_time) = 0;
	*(num) = 1;

	ad_data_ptr[1] = other_time;
	ad_data_ptr[2] = num;
	ad_data_ptr[3] = &overflow;
	printLog("%lu\r\n", *ad_data_ptr[0]);
	printLog("%lu\r\n", *ad_data_ptr[1]);
	printLog("%lu\r\n", *ad_data_ptr[2]);
	printLog("%lu\r\n", *ad_data_ptr[3]);


	while (1) {
		/* Event pointer for handling events */
		struct gecko_cmd_packet* evt;

		/* if there are no events pending then the next call to gecko_wait_event() may cause
		* device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
		if (!gecko_event_pending()) {
		flushLog();
		}

		/* Check for stack event. This is a blocking event listener. If you want non-blocking please see UG136. */

		//printLog("%lu\r\n", overflowCount);
		//printLog("%lu\r\n", delta);
		//printLog("%lu\r\n", temp[0]);
		//printLog("t%lu\r\n", TIMER_CounterGet(TIMER0));
		setTemp();
		result = gecko_cmd_le_gap_bt5_set_adv_data(0, 8, sizeof(uint32_t)*4, temp)->result;
		evt = gecko_wait_event();

		/* Handle events */
		switch (BGLIB_MSG_ID(evt->header)) {
			case gecko_evt_system_boot_id:
				bootMessage(&(evt->data.evt_system_boot));
				printLog("Periodic Sync scanner system boot\r\n");

				gecko_cmd_system_set_tx_power(100);
				gecko_cmd_le_gap_set_advertise_tx_power(0,30);
				gecko_cmd_le_gap_set_advertise_timing(0, ad_params.max_interval, 160, 0, 0);
				gecko_cmd_le_gap_clear_advertise_configuration(0,1);
				result = gecko_cmd_le_gap_set_data_channel_classification(5, ad_params.ch_map)->result;
				printLog("set data channel classification: %d\r\n", result);

				result = gecko_cmd_le_gap_start_advertising(0,le_gap_general_discoverable, le_gap_non_connectable)->result;
				printLog("le_gap_start_advertising() returns 0x%X\r\n", result);

				/* adv set #1 , 100 ms min/max interval, include tx power in PDU*/
				//TIMER_Enable(TIMER1,true);
				result = gecko_cmd_le_gap_start_periodic_advertising(0,ad_params.min_interval,ad_params.max_interval,1)->result;
				printLog("start_periodic_advertising returns 0x%X\r\n",result);

				//Set BT5 advertisement data

				//result = gecko_cmd_le_gap_bt5_set_adv_data(0,8,sizeof(ad_data),ad_data)->result;
				//printLog("set_adv_data for periodic advertising data returns 0x%X\r\n",result);

				result = gecko_cmd_cte_transmitter_enable_connectionless_cte(ad_params.ad_handle,
													TX_params.cte_length, TX_params.cte_type, TX_params.cte_count, TX_params.s_len,
													TX_params.sa)->result;
				printLog("gecko_cmd_cte_transmitter_enable_connectionless_cte: 0x%x\r\n", result);
				/* It is recommended to enable event le_gap_extended_scan_response
				which contains useful information for establishing a synchronization. */
				gecko_cmd_le_gap_set_discovery_extended_scan_response(true);
				gecko_cmd_le_gap_set_discovery_timing(le_gap_phy_1m,200,200);
				gecko_cmd_le_gap_set_discovery_type(le_gap_phy_1m,0);
				gecko_cmd_le_gap_start_discovery(le_gap_phy_1m,le_gap_discover_observation);
			break;

			case gecko_evt_le_connection_closed_id:
				/* Check if need to boot to dfu mode */
				if (boot_to_dfu) {
				/* Enter to DFU OTA mode */
				gecko_cmd_system_reset(2);
				} else {
				/* Restart advertising after client has disconnected */
				gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
				}
			break;

			case gecko_evt_le_gap_extended_scan_response_id: {
				if(compare_bd_addr(&ad_params.local_addr, &evt->data.evt_le_gap_extended_scan_response.address)){
					break;
				}
				/* only look at extended advertisements */
				if(evt->data.evt_le_gap_extended_scan_response.packet_type & 0x80){
					printLog("got ext adv indication with tx_power = %d\r\n",
					  evt->data.evt_le_gap_extended_scan_response.tx_power );
					if (findServiceInAdvertisement(&(evt->data.evt_le_gap_extended_scan_response.data.data[0]), evt->data.evt_le_gap_extended_scan_response.data.len) != 0) {

						printLog("found periodic sync service, attempting to open sync\r\n");

						uint16 skip = 1, timeout = 20;
						sync_handle = gecko_cmd_sync_open(evt->data.evt_le_gap_extended_scan_response.adv_sid,
							 skip,
							 timeout,
							 evt->data.evt_le_gap_extended_scan_response.address,
							 evt->data.evt_le_gap_extended_scan_response.address_type)->sync;
						printLog("cmd_sync_open() sync = 0x%2X\r\n", sync_handle);
					}
				}
			}
			break;

			case gecko_evt_sync_opened_id:
				/* now that sync is open, we can stop scanning*/
				printLog("evt_sync_opened\r\n");
				gecko_cmd_le_gap_end_procedure();
				result = gecko_cmd_cte_receiver_enable_connectionless_cte(sync_handle,
								RX_params.slot_dur, RX_params.cte_count, RX_params.s_len, RX_params.sa)->result;
				printLog("Result of gecko_cmd_cte_receiver_enable_connectionless_cte: 0x%x\r\n",result);
				TIMER_Enable(TIMER0,true);
			break;

			case gecko_evt_sync_closed_id:
				printLog("periodic sync closed. reason 0x%2X, sync handle %d",
				evt->data.evt_sync_closed.reason,
				evt->data.evt_sync_closed.sync);
				gecko_cmd_le_gap_start_discovery(le_gap_phy_1m,le_gap_discover_observation);
			break;

			case gecko_evt_sync_data_id:
				printLog("PERIODIC PACKET RECIEVED\r\n");
				printLog("%d\r\n", evt->data.evt_sync_data.data.len);
				printLog("DATA: %lu %lu %lu %lu \r\n", data[0], data[1], data[2], data[3]);
				data = (uint32_t*) &(evt->data.evt_sync_data.data.data);
				memcpy(num, data+2,sizeof(uint32_t));
				(*num)++;
				memcpy(other_time, data, sizeof(uint32_t));
				memcpy(other_overflow, data+3, sizeof(uint32_t));
				TIMER_CounterSet(TIMER0, 0);
				overflow = 0;
			break;
			/* Events related to OTA upgrading
			----------------------------------------------------------------------------- */

			/* Check if the user-type OTA Control Characteristic was written.
			* If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
			case gecko_evt_gatt_server_user_write_request_id:

				if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
					/* Set flag to enter to OTA mode */
					boot_to_dfu = 1;
					/* Send response to Write Request */
					gecko_cmd_gatt_server_send_user_write_response(
					evt->data.evt_gatt_server_user_write_request.connection,
					gattdb_ota_control,
					bg_err_success);

					/* Close connection to enter to DFU OTA mode */
					gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
				}
			break;

			case gecko_evt_cte_receiver_connectionless_iq_report_id: {
				printLog("CTE REPORT\r\n");

				struct gecko_msg_cte_receiver_connectionless_iq_report_evt_t *report =
				&(evt->data.evt_cte_receiver_connectionless_iq_report);

				printLog(
				"status: %d, ch: %d, rssi: %d, ant:%d, cte:%d, duration:%d, sync:%d, event: %d, len:%d\r\n",
				report->status, report->channel, report->rssi, report->rssi_antenna_id,
				report->cte_type, report->slot_durations, report->sync, report->event_counter,
				report->samples.len);

				/*
				for(int i=0; i<4;i++)
					RETARGET_WriteChar(0xFF);
				RETARGET_WriteChar(report->channel);
				RETARGET_WriteChar(report->samples.len);
				RETARGET_WriteChar(num);
				uint8_t* temp = (uint8_t*)&report->event_counter;
				RETARGET_WriteChar(temp[0]);
				RETARGET_WriteChar(temp[1]);
				for (int i=0; i<report->samples.len; i++) {
					RETARGET_WriteChar(report->samples.data[i]);
				}
				RETARGET_WriteChar('\r');
				RETARGET_WriteChar('\n');
				*/

				uint8_t* userDataPage = ram;
				offset = (16+report->samples.len);
				if(itr < 9){
					//updateTime(&delta, &old, ad_data);
					uint8_t* temp = (uint8_t*)&report->event_counter;
					uint8_t* temp2 = (uint8_t*)other_time;
					uint8_t* temp3 = (uint8_t*)num;
					uint8_t* temp4 = (uint8_t*)other_overflow;
					uint8_t preData[] = {
						report->channel,
						report->samples.len,
						temp[0],
						temp[1],
						temp2[0],
						temp2[1],
						temp2[2],
						temp2[3],
						temp3[0],
						temp3[1],
						temp3[2],
						temp3[3],
						temp4[0],
						temp4[1],
						temp4[2],
						temp4[3]

					};
					memcpy(userDataPage+offset*itr, preData, sizeof(preData));
					memcpy(userDataPage+offset*itr+sizeof(preData), &report->samples.data, report->samples.len);

				}
				itr++;

				if( itr == 8 ){
					uint8_t* ch = malloc(1);
					uint8_t* len = malloc(1);
					uint8_t* event_num = malloc(2);
					uint8_t* num = malloc(4);
					uint8_t* time = malloc(4);
					uint8_t* overflow_n = malloc(4);
					uint8_t* data = malloc(report->samples.len);
					for(int k=0; k<itr; k++){
						//setTemp();
						//result = gecko_cmd_le_gap_bt5_set_adv_data(0, 8, sizeof(uint32_t)*4, temp)->result;
						for(int i=0; i< 4; i++)
							RETARGET_WriteChar(0xFF);

						memcpy(ch, userDataPage + 0*sizeof(uint8_t) + offset*k,sizeof(uint8_t));
						RETARGET_WriteChar(*ch);

						memcpy(len, userDataPage + 1*sizeof(uint8_t) + offset*k,sizeof(uint8_t));
						RETARGET_WriteChar(*len);

						memcpy(event_num, userDataPage + 2*sizeof(uint8_t) + offset*k,sizeof(uint16_t));
						RETARGET_WriteChar(event_num[0]);
						RETARGET_WriteChar(event_num[1]);

						memcpy(num, userDataPage + 4*sizeof(uint8_t) + offset*k,sizeof(uint32_t));
						for(int i=0; i<4; i++)
							RETARGET_WriteChar(num[i]);

						memcpy(time, userDataPage + 8*sizeof(uint8_t) + offset*k,sizeof(uint32_t));
						for(int i=0; i<4; i++)
							RETARGET_WriteChar(time[i]);

						memcpy(overflow_n, userDataPage + 12*sizeof(uint8_t) + offset*k,sizeof(uint32_t));
						for(int i=0; i<4; i++)
							RETARGET_WriteChar(overflow_n[i]);


						memcpy(data, userDataPage + 16*sizeof(uint8_t) + offset*k, report->samples.len);
						for (int i=0; i<report->samples.len; i++) {
							RETARGET_WriteChar(data[i]);
						}

						RETARGET_WriteChar('\r');
						RETARGET_WriteChar('\n');
					}
					free(ch);
					free(len);
					free(data);
					free(event_num);
					free(overflow_n);
					free(time);
					itr = 0;
				}

			}
			break;

			default:
			break;
		}
	}
}

/* Print stack version and local Bluetooth address as boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
{
	ad_params.local_addr = gecko_cmd_system_get_bt_address()->address;
#if DEBUG_LEVEL
  bd_addr local_addr;
  int i;

  printLog("stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor, bootevt->patch);
  local_addr = gecko_cmd_system_get_bt_address()->address;

  printLog("local BT device address: ");
  for (i = 0; i < 5; i++) {
    printLog("%2.2x:", local_addr.addr[5 - i]);
  }
  printLog("%2.2x\r\n", local_addr.addr[0]);
#endif
}
