/*
 * b4_i2c_probe.c
 * Test-build-only I2C probe for identifying a likely LSM303DLHC.
 */

#include "b4_i2c_probe.h"
#include "b4_lsm303dlhc.h"
#include "b4_processor.h"
#include "subsystem4.h"

#if APP_TEST

#include <stdint.h>
#include <tk/device.h>
#include <sys/sysdef.h>
#include "../../device/include/dev_i2c.h"

/* LSM303DLHC accelerometer WHO_AM_I_A register and expected value. */
#define B4_LSM303DLHC_ACCEL_ID_REG 0x0FU
#define B4_LSM303DLHC_ACCEL_ID     0x33U
#define B4_LSM303DLHC_ACCEL_ADDR_0 0x18U
#define B4_LSM303DLHC_ACCEL_ADDR_1 0x19U

/* LSM303DLHC magnetometer address and three-byte identification string. */
#define B4_LSM303DLHC_MAG_ADDR     0x1EU
#define B4_LSM303DLHC_MAG_ID_REG   0x0AU
#define B4_LSM303DLHC_MAG_ID_0     0x48U
#define B4_LSM303DLHC_MAG_ID_1     0x34U
#define B4_LSM303DLHC_MAG_ID_2     0x33U
#define B4_I2C_PROBE_PERIOD_MS     3000
#define B4_TEST_SAMPLE_PERIOD_MS   TIMER_PERIOD
#define B4_TEST_SAMPLE_PERIOD_US   (B4_TEST_SAMPLE_PERIOD_MS * 1000U)
#define B4_TEST_LOG_EVERY_SAMPLES  10U

typedef struct {
	int32_t turn_rate_mdps;
	int32_t peak_height_mm;
	int16_t tilt_cdeg;
	int16_t slope_cdeg;
	uint16_t dynamic_accel_mg;
	uint8_t calibration_ready;
	uint8_t calibration_status;
	uint8_t tilt_active;
	uint8_t terrain_state;
	uint8_t height_valid;
	uint8_t motion_class;
	uint8_t turn_rate_valid;
	uint8_t event_count;
	uint8_t events_dropped;
	int32_t event_encode_status;
	uint8_t events[B4_PROCESSOR_MAX_EVENTS][B4_EVENT_ENCODED_SIZE];
} B4_TEST_PROCESS_OUTPUT;

typedef struct {
	B4_IMU_SAMPLE sample;
	B4_TEST_PROCESS_OUTPUT processed;
	int32_t read_status;
	int32_t io_error;
	uint32_t sequence;
	uint32_t dropped_reports;
	uint32_t poll_interval_ms;
	uint32_t poll_interval_max_ms;
	uint32_t poll_interval_us;
	uint32_t poll_interval_max_us;
	uint32_t loop_period_us;
	uint32_t loop_period_max_us;
	uint32_t tick_wait_us;
	uint32_t accel_burst_us;
	uint32_t accel_status_after_us;
	uint32_t mag_status_us;
	uint32_t mag_data_us;
	uint32_t sensor_read_us;
	uint32_t logger_print_us;
	uint32_t overrun_events;
	uint8_t ctrl1;
	uint8_t ctrl4;
	uint8_t accel_status;
	uint8_t status_after_read;
	uint8_t raw_xyz[6];
} B4_TEST_LOG_RECORD;

static ID b4_test_log_mbf = -1;
static volatile uint32_t b4_test_log_drops;
static ID b4_sampler_task_id = -1;

static uint32_t b4_probe_timer_us(void)
{
	return *(volatile uint32_t *)TIMER_TIMERAWL;
}

static void b4_probe_store_processed(B4_TEST_PROCESS_OUTPUT *destination,
				     const B4_PROCESSOR_OUTPUT *source)
{
	uint8_t event_index;

	destination->turn_rate_mdps = source->turn_rate_mdps;
	destination->peak_height_mm = source->peak_height_mm;
	destination->tilt_cdeg = source->tilt_cdeg;
	destination->slope_cdeg = source->slope_cdeg;
	destination->dynamic_accel_mg = source->dynamic_accel_mg;
	destination->calibration_ready = source->calibration_ready;
	destination->calibration_status = source->calibration_status;
	destination->tilt_active = source->tilt_active;
	destination->terrain_state = source->terrain_state;
	destination->height_valid = source->height_valid;
	destination->motion_class = source->motion_class;
	destination->turn_rate_valid = source->turn_rate_valid;
	destination->event_count = source->event_count;
	destination->events_dropped = source->events_dropped;
	destination->event_encode_status = B4_EVENT_STATUS_OK;

	for (event_index = 0U; event_index < source->event_count; event_index++) {
		B4_EVENT_STATUS status = b4_event_encode(
			&source->events[event_index],
			destination->events[event_index],
			B4_EVENT_ENCODED_SIZE);

		if (status != B4_EVENT_STATUS_OK) {
			destination->event_encode_status = (int32_t)status;
			destination->event_count = event_index;
			return;
		}
	}
}

static void b4_probe_make_log_record(B4_TEST_LOG_RECORD *record,
				     const B4_LSM303DLHC *sensor)
{
	(void)b4_lsm303dlhc_get_accel_diagnostics(
		sensor, &record->ctrl1, &record->ctrl4,
		&record->accel_status, &record->status_after_read,
		&record->overrun_events, &record->poll_interval_ms,
		&record->poll_interval_max_ms, &record->poll_interval_us,
		&record->poll_interval_max_us, &record->accel_burst_us,
		&record->accel_status_after_us, &record->mag_status_us,
		&record->mag_data_us, &record->sensor_read_us,
		record->raw_xyz);
}

/*
 * The USB console has a finite output ring. Wait until TinyUSB is mounted
 * before printing diagnostics so startup output is not lost before a host
 * enumerates the CDC interface.
 */
static void b4_probe_wait_for_console(void)
{
#if TM_CONSOLE_USB_CDC
	IMPORT UW tm_usb_state(void);

	while (tm_usb_state() < 2U) {
		tk_dly_tsk(100);
	}
#endif
}

static void b4_probe_accel(ID device, uint8_t address)
{
	UB id = 0U;
	ER status;

	status = i2c_read_reg(device, address, B4_LSM303DLHC_ACCEL_ID_REG, &id);
	if (status != E_OK) {
		tm_printf((UB *)"Buddy4 probe: ACCEL addr=0x%02x WHO_AM_I_A "
			  "read failed, I2C error=%d\n", address, status);
		return;
	}

	tm_printf((UB *)"Buddy4 probe: ACCEL addr=0x%02x ACK, "
		  "WHO_AM_I_A[0x0f]=0x%02x (%s)\n",
		  address, id,
		  (id == B4_LSM303DLHC_ACCEL_ID)
		  ? "LSM303DLHC signature match" : "unexpected ID");
}

static void b4_probe_mag(ID device)
{
	UB id[3] = {0U, 0U, 0U};
	INT index;
	ER status;

	status = i2c_read_reg(device, B4_LSM303DLHC_MAG_ADDR,
			      B4_LSM303DLHC_MAG_ID_REG, &id[0]);
	if (status != E_OK) {
		tm_printf((UB *)"Buddy4 probe: MAG addr=0x%02x ID read "
			  "failed, I2C error=%d\n",
			  B4_LSM303DLHC_MAG_ADDR, status);
		return;
	}

	for (index = 1; index < 3; index++) {
		status = i2c_read_reg(device, B4_LSM303DLHC_MAG_ADDR,
				      (UW)(B4_LSM303DLHC_MAG_ID_REG + index),
				      &id[index]);
		if (status != E_OK) {
			tm_printf((UB *)"Buddy4 probe: MAG addr=0x%02x "
				  "ID register 0x%02x read failed, I2C error=%d\n",
				  B4_LSM303DLHC_MAG_ADDR,
				  B4_LSM303DLHC_MAG_ID_REG + index, status);
			return;
		}
	}

	tm_printf((UB *)"Buddy4 probe: MAG addr=0x%02x ACK, "
		  "ID[0x0a..0x0c]=0x%02x 0x%02x 0x%02x%s\n",
		  B4_LSM303DLHC_MAG_ADDR, id[0], id[1], id[2],
		  ((id[0] == B4_LSM303DLHC_MAG_ID_0) &&
		   (id[1] == B4_LSM303DLHC_MAG_ID_1) &&
		   (id[2] == B4_LSM303DLHC_MAG_ID_2))
		  ? "LSM303DLHC signature match" : "unexpected ID sequence");
}

EXPORT ER b4_i2c_probe_run(void)
{
	static const UB device_name[] = "iica";
	ID device;
	ER status;

	device = tk_opn_dev(device_name, TD_READ | TD_WRITE);
	if (device < E_OK) {
		tm_printf((UB *)"Buddy4 probe: I2C0 initialization/device open "
			  "failed for iica, error=%d\n", device);
		return (ER)device;
	}

	b4_probe_accel(device, B4_LSM303DLHC_ACCEL_ADDR_0);
	b4_probe_accel(device, B4_LSM303DLHC_ACCEL_ADDR_1);
	b4_probe_mag(device);

	status = tk_cls_dev(device, 0U);
	if (status != E_OK) {
		tm_printf((UB *)"Buddy4 probe: close iica failed (%d)\n", status);
		return status;
	}

	return E_OK;
}

EXPORT void b4_i2c_log_task(INT stacd, void *exinf)
{
	B4_TEST_LOG_RECORD record;
	uint32_t previous_print_us = 0U;

	(void)stacd;
	(void)exinf;

	b4_probe_wait_for_console();
	tm_printf((UB *)"\n=== Buddy 4 LSM303DLHC timing test (TEST=1) ===\n");
	tm_printf((UB *)"Acquisition target=100 Hz; USB reports every 100 ms\n");
	tm_printf((UB *)"Algorithms: defaults use sensor X as forward, Z as up; "
		  "verify mount axes/signs before interpreting terrain output\n");

	while (1) {
		INT length = tk_rcv_mbf(b4_test_log_mbf, &record, TMO_FEVR);

		if (length < E_OK) {
			tm_printf((UB *)"Buddy4 logger: message receive failed=%d\n",
				  length);
			continue;
		}

		{
			uint32_t print_start_us = b4_probe_timer_us();

			tm_printf((UB *)"Buddy4 sample #%u t=%u ms status=%d "
				  "flags=0x%02x",
				  (unsigned int)record.sequence,
				  (unsigned int)record.sample.timestamp_ms,
				  (int)record.read_status,
				  (unsigned int)record.sample.flags);
			if ((record.sample.flags &
			     B4_IMU_SAMPLE_FLAG_ACCEL_VALID) != 0U) {
				tm_printf((UB *)" accel_mg=(%d,%d,%d)",
					  record.sample.accel_mg[0],
					  record.sample.accel_mg[1],
					  record.sample.accel_mg[2]);
			} else {
				tm_printf((UB *)" accel=not-fresh");
			}
			if ((record.sample.flags &
			     B4_IMU_SAMPLE_FLAG_ACCEL_OVERRUN) != 0U) {
				tm_printf((UB *)" accel_history=overrun");
			}
			if ((record.sample.flags &
			     B4_IMU_SAMPLE_FLAG_MAG_VALID) != 0U) {
				tm_printf((UB *)" mag_dut=(%d,%d,%d)",
					  record.sample.mag_dut[0],
					  record.sample.mag_dut[1],
					  record.sample.mag_dut[2]);
			} else {
				tm_printf((UB *)" mag=not-fresh");
			}
			tm_printf((UB *)" overruns=%u drops=%u\n",
				  (unsigned int)record.overrun_events,
				  (unsigned int)record.dropped_reports);
			tm_printf((UB *)"Buddy4 timing us: period=%u max_period=%u "
				  "poll=%u max_poll=%u "
				  "tick_wait=%u "
				  "accel_burst=%u accel_status_after=%u "
				  "mag_status=%u mag_data=%u sensor_read=%u "
				  "logger_prev=%u\n",
				  (unsigned int)record.loop_period_us,
				  (unsigned int)record.loop_period_max_us,
				  (unsigned int)record.poll_interval_us,
				  (unsigned int)record.poll_interval_max_us,
				  (unsigned int)record.tick_wait_us,
				  (unsigned int)record.accel_burst_us,
				  (unsigned int)record.accel_status_after_us,
				  (unsigned int)record.mag_status_us,
				  (unsigned int)record.mag_data_us,
				  (unsigned int)record.sensor_read_us,
				  (unsigned int)previous_print_us);
			tm_printf((UB *)"Buddy4 accel diag: CTRL1_A=0x%02x "
				  "CTRL4_A=0x%02x STATUS_A=0x%02x "
				  "STATUS_A_after=0x%02x XYZ bytes=%02x %02x "
				  "%02x %02x %02x %02x ZYXDA=%u ZYXOR=%u\n",
				  (unsigned int)record.ctrl1,
				  (unsigned int)record.ctrl4,
				  (unsigned int)record.accel_status,
				  (unsigned int)record.status_after_read,
				  (unsigned int)record.raw_xyz[0],
				  (unsigned int)record.raw_xyz[1],
				  (unsigned int)record.raw_xyz[2],
				  (unsigned int)record.raw_xyz[3],
				  (unsigned int)record.raw_xyz[4],
				  (unsigned int)record.raw_xyz[5],
				  (unsigned int)((record.accel_status >> 3) & 1U),
				  (unsigned int)((record.accel_status >> 7) & 1U));
			if (record.processed.calibration_ready != 0U) {
				tm_printf((UB *)"Buddy4 motion: tilt=%d cdeg "
					  "slope=%d cdeg dynamic=%u mg "
					  "class=%u terrain=%u",
					  (int)record.processed.tilt_cdeg,
					  (int)record.processed.slope_cdeg,
					  (unsigned int)record.processed.dynamic_accel_mg,
					  (unsigned int)record.processed.motion_class,
					  (unsigned int)record.processed.terrain_state);
				if (record.processed.height_valid != 0U) {
					tm_printf((UB *)" peak_height_est=%d mm",
						  (int)record.processed.peak_height_mm);
				} else {
					tm_printf((UB *)" peak_height_est=unavailable");
				}
				if (record.processed.turn_rate_valid != 0U) {
					tm_printf((UB *)" turn_rate_est=%d mdps\n",
						  (int)record.processed.turn_rate_mdps);
				} else {
					tm_printf((UB *)" turn_rate_est=unavailable\n");
				}
			} else {
				tm_printf((UB *)"Buddy4 calibration: state=%u "
					  "(0=collecting, 2=movement reset), "
					  "class=%u\n",
					  (unsigned int)record.processed.calibration_status,
					  (unsigned int)record.processed.motion_class);
			}
			{
				uint8_t event_index;

				for (event_index = 0U;
				     event_index < record.processed.event_count;
				     event_index++) {
					B4_EVENT event;
					B4_EVENT_STATUS decode_status =
						b4_event_decode(
							record.processed.events[event_index],
							B4_EVENT_ENCODED_SIZE,
							&event);

					if (decode_status != B4_EVENT_STATUS_OK) {
						tm_printf((UB *)"Buddy4 ERROR: "
							  "event decode failed=%d\n",
							  (int)decode_status);
					} else {
						tm_printf((UB *)"Buddy4 event: "
							  "type=%u t=%u ms "
							  "value=%s%d\n",
							  (unsigned int)event.type,
							  (unsigned int)event.timestamp_ms,
							  ((event.flags &
							    B4_EVENT_FLAG_VALUE_VALID)
							   != 0U)
							  ? "" : "unavailable/",
							  (int)event.value);
					}
				}
			}
			if (record.processed.event_encode_status !=
			    B4_EVENT_STATUS_OK) {
				tm_printf((UB *)"Buddy4 ERROR: event encode failed=%d\n",
					  (int)record.processed.event_encode_status);
			}
			if (record.processed.events_dropped != 0U) {
				tm_printf((UB *)"Buddy4 ERROR: %u event(s) "
					  "omitted from bounded sample output\n",
					  (unsigned int)
					  record.processed.events_dropped);
			}
			if (record.io_error < E_OK) {
				tm_printf((UB *)"Buddy4 ERROR: I2C operation error=%d\n",
					  (int)record.io_error);
			}

			previous_print_us = b4_probe_timer_us() - print_start_us;
		}
	}
}

/*
 * Cyclic handler runs at the kernel tick rate and only wakes the acquisition
 * task. Sensor I/O and logging stay in task context.
 */
EXPORT void b4_i2c_sample_tick(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(b4_sampler_task_id);
}

EXPORT void b4_i2c_probe_task(INT stacd, void *exinf)
{
	B4_LSM303DLHC sensor = {0};
	B4_TEST_LOG_RECORD record = {0};
	B4_PROCESSOR processor = {0};
	B4_PROCESSOR_CONFIG processor_config;
	B4_CALIBRATION_STATUS processor_status;
	uint32_t sample_sequence = 0U;
	uint32_t loop_start_us;
	uint32_t loop_start_previous_us = 0U;
	uint32_t max_loop_period_us = 0U;
	uint32_t tick_wait_us = 0U;

	(void)stacd;
	(void)exinf;

	/* Identify/configure once; this task exclusively owns I2C0 afterward. */
	(void)b4_i2c_probe_run();
	b4_processor_default_config(&processor_config);
	processor_status = b4_processor_init(&processor, &processor_config);
	if (processor_status != B4_CALIBRATION_COLLECTING) {
		tm_printf((UB *)"Buddy4 processor: configuration invalid=%d\n",
			  (int)processor_status);
		return;
	}

	while (1) {
		ER send_status;

		if (sensor.initialized == 0U) {
			B4_LSM303DLHC_STATUS init_status =
				b4_lsm303dlhc_init(&sensor);

			if (init_status != B4_LSM303DLHC_STATUS_OK) {
				record.read_status = (int32_t)init_status;
				record.io_error = b4_lsm303dlhc_last_error();
				record.sequence = sample_sequence++;
				send_status = tk_snd_mbf(b4_test_log_mbf,
							 &record,
							 (INT)sizeof(record),
							 TMO_POL);
				if (send_status < E_OK) {
					b4_test_log_drops++;
				}
				goto wait_for_next_tick;
			}
		}

		loop_start_us = b4_probe_timer_us();
		if (loop_start_previous_us != 0U) {
			record.loop_period_us =
				loop_start_us - loop_start_previous_us;
			if (record.loop_period_us > max_loop_period_us) {
				max_loop_period_us = record.loop_period_us;
			}
		}
		loop_start_previous_us = loop_start_us;

		record.sample = (B4_IMU_SAMPLE){0};
		record.read_status = b4_lsm303dlhc_read(&sensor, &record.sample);
		record.io_error = b4_lsm303dlhc_last_error();
		record.sensor_read_us = sensor.read_total_us;
		record.dropped_reports = b4_test_log_drops;
		record.processed = (B4_TEST_PROCESS_OUTPUT){0};
		if ((record.read_status == B4_LSM303DLHC_STATUS_OK) &&
		    (record.sample.flags != 0U)) {
			B4_PROCESSOR_OUTPUT processed = {0};

			(void)b4_processor_update(&processor, &record.sample,
						  0U, 0,
						  &processed);
			b4_probe_store_processed(&record.processed, &processed);
		}

		if (((sample_sequence % B4_TEST_LOG_EVERY_SAMPLES) == 0U) ||
		    (record.processed.event_count != 0U) ||
		    ((record.read_status != B4_LSM303DLHC_STATUS_OK) &&
		     (record.read_status !=
		      B4_LSM303DLHC_STATUS_NO_FRESH_DATA))) {
			record.sequence = sample_sequence;
			record.loop_period_max_us = max_loop_period_us;
			record.tick_wait_us = tick_wait_us;
			b4_probe_make_log_record(&record, &sensor);
			send_status = tk_snd_mbf(b4_test_log_mbf, &record,
							 (INT)sizeof(record), TMO_POL);
			if (send_status < E_OK) {
				b4_test_log_drops++;
			}
		}
		sample_sequence++;

wait_for_next_tick:
		{
			uint32_t wait_start_us = b4_probe_timer_us();
			ER wait_status = tk_slp_tsk(TMO_FEVR);

			tick_wait_us = b4_probe_timer_us() - wait_start_us;
			if (wait_status < E_OK) {
				record.read_status = wait_status;
				record.io_error = wait_status;
				(void)tk_snd_mbf(b4_test_log_mbf, &record,
						 (INT)sizeof(record), TMO_POL);
			}
		}
	}
}

EXPORT ER b4_i2c_probe_start(void)
{
	T_CMBF cmbf = {0};
	T_CCYC ccyc = {0};
	ID task_id;
	ID cycle_id;

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz = 4 * (INT)sizeof(B4_TEST_LOG_RECORD);
	cmbf.maxmsz = (INT)sizeof(B4_TEST_LOG_RECORD);
	b4_test_log_mbf = tk_cre_mbf(&cmbf);
	if (b4_test_log_mbf < E_OK) {
		tm_printf((UB *)"Buddy4 logger: message buffer create failed=%d\n",
			  b4_test_log_mbf);
		return (ER)b4_test_log_mbf;
	}

	task_id = app_start_task(b4_i2c_log_task, 21, TP_PRC1,
				 "b4_i2c_log");
	if (task_id < E_OK) {
		(void)tk_del_mbf(b4_test_log_mbf);
		return (ER)task_id;
	}

	/* High-priority core-1 task avoids the slower monitor/USB work on core 0. */
	task_id = app_start_task(b4_i2c_probe_task, 10, TP_PRC2,
				 "b4_i2c_sample");
	if (task_id < E_OK) {
		tm_printf((UB *)"Buddy4 sampler: task start failed, error=%d\n",
			  task_id);
		(void)tk_del_mbf(b4_test_log_mbf);
		return (ER)task_id;
	}
	b4_sampler_task_id = task_id;

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)b4_i2c_sample_tick;
	ccyc.cyctim = B4_TEST_SAMPLE_PERIOD_MS;
	ccyc.cycphs = B4_TEST_SAMPLE_PERIOD_MS;
	cycle_id = tk_cre_cyc(&ccyc);
	if (cycle_id < E_OK) {
		tm_printf((UB *)"Buddy4 sampler: cyclic handler create failed=%d\n",
			  cycle_id);
		return (ER)cycle_id;
	}

	task_id = tk_sta_cyc(cycle_id);
	if (task_id < E_OK) {
		tm_printf((UB *)"Buddy4 sampler: cyclic handler start failed=%d\n",
			  task_id);
		(void)tk_del_cyc(cycle_id);
		return (ER)task_id;
	}

	return E_OK;
}

#endif /* APP_TEST */
