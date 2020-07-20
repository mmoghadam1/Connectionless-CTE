/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Periodic Advertisement Program
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

/* Print boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);

/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;

uint8 sync_handle;

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


/* Main application */
void appMain(gecko_configuration_t *pconfig) {
#if DISABLE_SLEEP > 0
	pconfig->sleep.flags = 0;
#endif

	/* Set maximum number of periodic advertisers */
	pconfig->bluetooth.max_advertisers = 1;
	/* Set the maximum number of periodic sync allowed*/
	pconfig->bluetooth.max_periodic_sync = 1;
	/* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
	initLog();

	/* Initialize stack */
	gecko_init(pconfig);

	/* Initialize Periodic advertisement */
	gecko_init_periodic_advertising();

	//Initialize CTE
	gecko_bgapi_class_cte_transmitter_init();

	//Initialize CTE Reciever
	gecko_bgapi_class_cte_receiver_init();

	gecko_bgapi_class_sync_init();

	while (1) {
		/* Event pointer for handling events */
		struct gecko_cmd_packet* evt;

		/* Check for stack event. */
		evt = gecko_wait_event();

		/* Handle events */
		switch (BGLIB_MSG_ID(evt->header)) {

		/* Array for advertisement data */
		static uint8 periodic_adv_data[11];
		uint16 result;

		/* This boot event is generated when the system boots up after reset.
		 * Do not call any stack commands before receiving the boot event.
		 * Here the system is set to start advertising immediately after boot procedure. */
	case gecko_evt_system_boot_id:

		bootMessage(&(evt->data.evt_system_boot));
		gecko_cmd_system_set_tx_power(0);
		gecko_cmd_le_gap_set_advertise_tx_power(0, 0);
		/* Set advertising parameters. 100ms advertisement interval.
		 * The first parameter is advertising set handle
		 * The next two parameters are minimum and maximum advertising interval, both in
		 * units of (milliseconds * 1.6).
		 * The last two parameters are duration and maxevents left as default. */
		gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);

		/* turn off legacy PDU flag*/
		gecko_cmd_le_gap_clear_advertise_configuration(0, 1);

		uint8_t channel_map_data[5] = {3, 0, 0, 0, 0};
		result = gecko_cmd_le_gap_set_data_channel_classification(5, channel_map_data)->result;
		printLog("set data channel classification: %d\r\n", result);

		result = gecko_cmd_le_gap_start_advertising(0,
				le_gap_general_discoverable, le_gap_non_connectable)->result;
		printf("le_gap_start_advertising() returns 0x%X\r\n", result);

		/* adv set #1 , 100 ms min/max interval, include tx power in PDU*/
		result =
				gecko_cmd_le_gap_start_periodic_advertising(0, 16, 16, 1)->result;
		printf("start_periodic_advertising returns 0x%X\r\n", result);

		uint8 handle = 0;
		uint8 cte_length = 0x14;
		uint8 cte_type = 0;
		uint8 cte_count = 1;
		uint8 s_len = 0;
		uint8 sa[1] = { 0 };

		uint16 response = gecko_cmd_cte_transmitter_enable_connectionless_cte(
				handle, cte_length, cte_type, cte_count, s_len, sa)->result;
		printf("Response: 0x%x", response);
		//Set BT5 advertisement data
		result = gecko_cmd_le_gap_bt5_set_adv_data(0, 8,
				sizeof(periodic_adv_data), periodic_adv_data)->result;
		printf("set_adv_data for periodic advertising data returns 0x%X\r\n",
				result);
		gecko_cmd_hardware_set_soft_timer(32768, 0, 0);

		/* It is recommended to enable event le_gap_extended_scan_response
		 which contains useful information for establishing a synchronization. */
		gecko_cmd_le_gap_set_discovery_extended_scan_response(true);
		gecko_cmd_le_gap_set_discovery_timing(le_gap_phy_1m, 200, 200);
		gecko_cmd_le_gap_set_discovery_type(le_gap_phy_1m, 0);
		gecko_cmd_le_gap_start_discovery(le_gap_phy_1m,
				le_gap_discover_observation);

		break;

	case gecko_evt_le_connection_closed_id:

		/* Check if need to boot to dfu mode */
		if (boot_to_dfu) {
			/* Enter to DFU OTA mode */
			gecko_cmd_system_reset(2);
		} else {
			/* Restart advertising after client has disconnected */
			gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable,
					le_gap_connectable_scannable);
		}
		break;

	case gecko_evt_hardware_soft_timer_id:
		//periodic_adv_data = gecko_cmd_system_get_random_data(16)->data;
		for (int i = 0; i < 10; i++) {
			periodic_adv_data[i] = rand() % 9;
		}
		printf("\r\n");
		for (int i = 0; i < 10; i++) {
			printf(" %X", periodic_adv_data[i]);
		}
		printf("\r\n");

		//Reset the advertising data pointers to reflect the change in data
		result = gecko_cmd_le_gap_bt5_set_adv_data(0, 8,
				sizeof(periodic_adv_data), periodic_adv_data)->result;
		printf("set_adv_data for periodic advertising data returns 0x%X \r\n",
				result);
		break;

	case gecko_evt_le_gap_extended_scan_response_id: {

		/* only look at extended advertisements */
		if (evt->data.evt_le_gap_extended_scan_response.packet_type & 0x80) {
			printLog("got ext adv indication with tx_power = %d\r\n",
					evt->data.evt_le_gap_extended_scan_response.tx_power);
			if (findServiceInAdvertisement(
					&(evt->data.evt_le_gap_extended_scan_response.data.data[0]),
					evt->data.evt_le_gap_extended_scan_response.data.len)
					!= 0) {

				printLog(
						"found periodic sync service, attempting to open sync\r\n");

				uint16 skip = 1, timeout = 20; /* similar to connection params? */
				sync_handle =
						gecko_cmd_sync_open(
								evt->data.evt_le_gap_extended_scan_response.adv_sid,
								skip, timeout,
								evt->data.evt_le_gap_extended_scan_response.address,
								evt->data.evt_le_gap_extended_scan_response.address_type)->sync;
				printLog("cmd_sync_open() sync = 0x%2X\r\n", sync_handle);

			}
		}
	}
		break;

	case gecko_evt_sync_opened_id: {
		//configure & enable cte after periodic sync established
		uint8 handle = sync_handle;
		uint8 slot_dur = 1;
		uint8 cte_count = 0;
		uint8 s_len = 1;
		uint8 sa[1] = { 0 };

		//uint16 config_res = gecko_cmd_cte_receiver_configure(1)->result;
		uint16 res = gecko_cmd_cte_receiver_enable_connectionless_cte(handle,
				slot_dur, cte_count, s_len, sa)->result;
		//printf("Config Response: 0x%x\r\n",config_res);
		printf("Response: 0x%x\r\n", res);
		/* now that sync is open, we can stop scanning*/
		printLog("evt_sync_opened\r\n");
		gecko_cmd_hardware_set_soft_timer(0, 1, 0);
		// gecko_cmd_hardware_set_soft_timer(32768,1,1);
		gecko_cmd_le_gap_end_procedure();

		break;
	}

	case gecko_evt_sync_closed_id:
		printLog("periodic sync closed. reason 0x%2X, sync handle %d\r\n",
				evt->data.evt_sync_closed.reason,
				evt->data.evt_sync_closed.sync);
		/* restart discovery */
		gecko_cmd_le_gap_start_discovery(le_gap_phy_1m,
				le_gap_discover_observation);
		break;

	case gecko_evt_sync_data_id:

		// printLog("periodic sync handle %d\r\n", evt->data.evt_sync_data.sync);

		printLog("got following sync data:");
		for (int i = 0; i < evt->data.evt_sync_data.data.len; i++) {
			printLog(" %X", evt->data.evt_sync_data.data.data[i]);

		}

		printLog("\r\n");
		/*
		 printLog("periodic sync RSSI %d and Tx power %d\r\n",
		 evt->data.evt_sync_data.rssi,
		 evt->data.evt_sync_data.tx_power);
		 */
		// printLog("periodic data status %d\r\n", evt->data.evt_sync_data.data_status);
		break;

		/* Events related to OTA upgrading
		 ----------------------------------------------------------------------------- */

		/* Check if the user-type OTA Control Characteristic was written.
		 * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
	case gecko_evt_gatt_server_user_write_request_id:

		if (evt->data.evt_gatt_server_user_write_request.characteristic
				== gattdb_ota_control) {
			/* Set flag to enter to OTA mode */
			boot_to_dfu = 1;
			/* Send response to Write Request */
			gecko_cmd_gatt_server_send_user_write_response(
					evt->data.evt_gatt_server_user_write_request.connection,
					gattdb_ota_control, bg_err_success);

			/* Close connection to enter to DFU OTA mode */
			gecko_cmd_le_connection_close(
					evt->data.evt_gatt_server_user_write_request.connection);
		}
		break;

	default:
		break;
		}
	}
}

/* Print stack version and local Bluetooth address as boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt) {
#if DEBUG_LEVEL
	bd_addr local_addr;
	int i;

	printLog("stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor,
			bootevt->patch);
	local_addr = gecko_cmd_system_get_bt_address()->address;

	printLog("local BT device address: ");
	for (i = 0; i < 5; i++) {
		printLog("%2.2x:", local_addr.addr[5 - i]);
	}
	printLog("%2.2x\r\n", local_addr.addr[0]);
#endif
}
