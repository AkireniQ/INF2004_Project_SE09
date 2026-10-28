/*
 * b4_lsm303dlhc.c
 * LSM303DLHC I2C access, configuration, and physical-unit conversion.
 */

#include "../app.h"
#include "../../device/include/dev_i2c.h"
#include "b4_lsm303dlhc.h"

#include <sys/sysdef.h>
#include <tk/device.h>

#define B4_ACCEL_ID_REG       0x0fU
#define B4_ACCEL_ID_VALUE     0x33U
#define B4_ACCEL_STATUS_REG   0x27U
#define B4_ACCEL_DATA_REG     0x28U
#define B4_ACCEL_DATA_READY   0x08U
#define B4_ACCEL_ANY_OVERRUN  0xf0U
#define B4_ACCEL_CTRL1_REG    0x20U
#define B4_ACCEL_CTRL1_100HZ  0x57U
#define B4_ACCEL_CTRL4_REG    0x23U
#define B4_ACCEL_CTRL4_HR_2G  0x08U

#define B4_MAG_ID_REG         0x0aU
#define B4_MAG_ID_0           0x48U
#define B4_MAG_ID_1           0x34U
#define B4_MAG_ID_2           0x33U
#define B4_MAG_STATUS_REG     0x09U
#define B4_MAG_DATA_READY     0x01U
#define B4_MAG_DATA_LOCK      0x02U
#define B4_MAG_DATA_REG       0x03U
#define B4_MAG_CRA_REG        0x00U
#define B4_MAG_CRA_75HZ       0x18U
#define B4_MAG_CRB_REG        0x01U
#define B4_MAG_CRB_1P3GAUSS   0x20U
#define B4_MAG_MODE_REG       0x02U
#define B4_MAG_CONTINUOUS     0x00U

static int32_t b4_last_error;

static uint32_t b4_timer_us(void)
{
	return *(volatile uint32_t *)TIMER_TIMERAWL;
}

static ER b4_read_registers(ID device, UW address, UB reg,
			    UB *data, SZ length)
{
	T_I2C_EXEC exec;
	UB subaddress;
	SZ actual_size = 0;
	ER status;

	if ((data == 0) || (length <= 0)) {
		return E_PAR;
	}

	subaddress = (UB)(reg | 0x80U);
	exec.sadr = address;
	exec.snd_size = 1;
	exec.snd_data = &subaddress;
	exec.rcv_size = length;
	exec.rcv_data = data;

	status = tk_swri_dev(device, TDN_I2C_EXEC, &exec,
			    sizeof(exec), &actual_size);
	if (status < E_OK) {
		b4_last_error = status;
	} else if (actual_size != sizeof(exec)) {
		b4_last_error = E_IO;
		return E_IO;
	}

	return status;
}

static ER b4_read_register(ID device, UW address, UB reg, UB *value)
{
	ER status;

	status = i2c_read_reg(device, address, reg, value);
	if (status < E_OK) {
		b4_last_error = status;
	}

	return status;
}

static ER b4_write_register(ID device, UW address, UW reg, UB value)
{
	ER status;

	status = i2c_write_reg(device, address, reg, value);
	if (status < E_OK) {
		b4_last_error = status;
	}

	return status;
}

static B4_LSM303DLHC_STATUS b4_check_identity(ID device)
{
	UB accel_id = 0U;
	UB mag_id[3] = {0U, 0U, 0U};
	INT index;
	ER status;

	status = b4_read_register(device, B4_LSM303DLHC_ACCEL_ADDRESS,
				  B4_ACCEL_ID_REG, &accel_id);
	if (status < E_OK) {
		return B4_LSM303DLHC_STATUS_IO_ERROR;
	}

	if (accel_id != B4_ACCEL_ID_VALUE) {
		return B4_LSM303DLHC_STATUS_ID_MISMATCH;
	}

	for (index = 0; index < 3; index++) {
		status = b4_read_register(device, B4_LSM303DLHC_MAG_ADDRESS,
					  (UB)(B4_MAG_ID_REG + index),
					  &mag_id[index]);
		if (status < E_OK) {
			return B4_LSM303DLHC_STATUS_IO_ERROR;
		}
	}

	if ((mag_id[0] != B4_MAG_ID_0) ||
	    (mag_id[1] != B4_MAG_ID_1) ||
	    (mag_id[2] != B4_MAG_ID_2)) {
		return B4_LSM303DLHC_STATUS_ID_MISMATCH;
	}

	return B4_LSM303DLHC_STATUS_OK;
}

B4_LSM303DLHC_STATUS b4_lsm303dlhc_init(B4_LSM303DLHC *sensor)
{
	static const UB device_name[] = "iica";
	ID device;
	ER status;
	B4_LSM303DLHC_STATUS sensor_status;

	if (sensor == 0) {
		return B4_LSM303DLHC_STATUS_INVALID_ARGUMENT;
	}

	sensor->initialized = 0U;
	sensor->device_id = -1;
	b4_last_error = 0;

	device = tk_opn_dev(device_name, TD_READ | TD_WRITE);
	if (device < E_OK) {
		b4_last_error = device;
		return B4_LSM303DLHC_STATUS_IO_ERROR;
	}

	sensor_status = b4_check_identity(device);
	if (sensor_status != B4_LSM303DLHC_STATUS_OK) {
		status = tk_cls_dev(device, 0U);
		if ((status < E_OK) && (b4_last_error == 0)) {
			b4_last_error = status;
		}
		sensor->device_id = -1;
		return sensor_status;
	}

	status = b4_write_register(device, B4_LSM303DLHC_ACCEL_ADDRESS,
				   B4_ACCEL_CTRL4_REG,
				   (UB)(B4_ACCEL_CTRL4_HR_2G | 0x80U));
	if (status >= E_OK) {
		status = b4_write_register(device, B4_LSM303DLHC_ACCEL_ADDRESS,
					   B4_ACCEL_CTRL1_REG,
					   B4_ACCEL_CTRL1_100HZ);
	}
	if (status >= E_OK) {
		status = b4_write_register(device, B4_LSM303DLHC_MAG_ADDRESS,
					   B4_MAG_CRA_REG,
					   B4_MAG_CRA_75HZ);
	}
	if (status >= E_OK) {
		status = b4_write_register(device, B4_LSM303DLHC_MAG_ADDRESS,
					   B4_MAG_CRB_REG,
					   B4_MAG_CRB_1P3GAUSS);
	}
	if (status >= E_OK) {
		status = b4_write_register(device, B4_LSM303DLHC_MAG_ADDRESS,
					   B4_MAG_MODE_REG,
					   B4_MAG_CONTINUOUS);
	}
	if (status >= E_OK) {
		status = b4_read_register(device, B4_LSM303DLHC_ACCEL_ADDRESS,
					  B4_ACCEL_CTRL1_REG,
					  &sensor->accel_ctrl1);
	}
	if (status >= E_OK) {
		status = b4_read_register(device, B4_LSM303DLHC_ACCEL_ADDRESS,
					  B4_ACCEL_CTRL4_REG,
					  &sensor->accel_ctrl4);
	}
	if (status < E_OK) {
		ER close_status = tk_cls_dev(device, 0U);

		if ((close_status < E_OK) && (b4_last_error == 0)) {
			b4_last_error = close_status;
		}
		sensor->device_id = -1;
		return B4_LSM303DLHC_STATUS_IO_ERROR;
	}

	sensor->accel_status = 0U;
	sensor->accel_status_after_read = 0U;
	sensor->accel_raw_available = 0U;
	sensor->accel_overrun_active = 0U;
	sensor->accel_overrun_count = 0U;
	sensor->poll_interval_ms = 0U;
	sensor->poll_interval_max_ms = 0U;
	sensor->poll_interval_us = 0U;
	sensor->poll_interval_max_us = 0U;
	sensor->last_poll_ms = 0U;
	sensor->last_poll_us = 0U;
	sensor->poll_time_valid = 0U;
	sensor->accel_burst_us = 0U;
	sensor->accel_status_after_us = 0U;
	sensor->mag_status_us = 0U;
	sensor->mag_data_us = 0U;
	sensor->read_total_us = 0U;
	sensor->device_id = device;
	sensor->initialized = 1U;
	return B4_LSM303DLHC_STATUS_OK;
}

B4_LSM303DLHC_STATUS b4_lsm303dlhc_read(B4_LSM303DLHC *sensor,
					B4_IMU_SAMPLE *sample_out)
{
	B4_IMU_SAMPLE sample = {0};
	UB accel_status = 0U;
	UB mag_status = 0U;
	UB accel_status_and_data[7];
	UB mag_bytes[6];
	uint8_t axis;
	ER status;
	SYSTIM now;
	uint32_t now_ms;
	uint32_t operation_start_us;
	uint32_t poll_start_us;

	if ((sensor == 0) || (sample_out == 0)) {
		return B4_LSM303DLHC_STATUS_INVALID_ARGUMENT;
	}

	if (sensor->initialized == 0U) {
		return B4_LSM303DLHC_STATUS_NOT_INITIALIZED;
	}

	b4_last_error = 0;
	tk_get_tim(&now);
	now_ms = now.lo;
	poll_start_us = b4_timer_us();
	if (sensor->poll_time_valid != 0U) {
		sensor->poll_interval_us = poll_start_us - sensor->last_poll_us;
		sensor->poll_interval_ms = sensor->poll_interval_us / 1000U;
		if (sensor->poll_interval_ms > sensor->poll_interval_max_ms) {
			sensor->poll_interval_max_ms = sensor->poll_interval_ms;
		}
		if (sensor->poll_interval_us > sensor->poll_interval_max_us) {
			sensor->poll_interval_max_us = sensor->poll_interval_us;
		}
	}
	sensor->last_poll_ms = now_ms;
	sensor->last_poll_us = poll_start_us;
	sensor->poll_time_valid = 1U;
	operation_start_us = b4_timer_us();

	/*
	 * One auto-increment burst captures STATUS_REG_A and XYZ together.
	 * This avoids a separate status transaction and ensures the status used
	 * for validity corresponds to the block of output registers just read.
	 */
	{
		uint32_t transaction_start_us = b4_timer_us();

		status = b4_read_registers(sensor->device_id,
				   B4_LSM303DLHC_ACCEL_ADDRESS,
				   B4_ACCEL_STATUS_REG,
				   accel_status_and_data,
				   sizeof(accel_status_and_data));
		sensor->accel_burst_us = b4_timer_us() - transaction_start_us;
	}
	if (status < E_OK) {
		return B4_LSM303DLHC_STATUS_IO_ERROR;
	}
	accel_status = accel_status_and_data[0];
	sensor->accel_status = accel_status;

	{
		uint32_t transaction_start_us = b4_timer_us();

		status = b4_read_register(sensor->device_id, B4_LSM303DLHC_MAG_ADDRESS,
				  B4_MAG_STATUS_REG, &mag_status);
		sensor->mag_status_us = b4_timer_us() - transaction_start_us;
	}
	if (status < E_OK) {
		return B4_LSM303DLHC_STATUS_IO_ERROR;
	}

	for (axis = 0U; axis < 6U; axis++) {
		sensor->accel_raw[axis] = accel_status_and_data[axis + 1U];
	}
	sensor->accel_raw_available = 1U;

	/*
	 * ZYXOR (and per-axis overrun bits) report conversion history lost
	 * before this read. With BDU enabled, the six-byte burst returns the
	 * currently latched coherent XYZ set; consume it rather than discarding
	 * every future sample until an overrun bit happens to clear.
	 */
	if ((accel_status & B4_ACCEL_ANY_OVERRUN) != 0U) {
		if ((sensor->accel_overrun_active == 0U) &&
		    (sensor->accel_overrun_count < UINT32_MAX)) {
			sensor->accel_overrun_count++;
		}
		sensor->accel_overrun_active = 1U;
	} else {
		sensor->accel_overrun_active = 0U;
	}
	if ((accel_status & B4_ACCEL_DATA_READY) != 0U) {
		(void)b4_lsm303dlhc_decode_accel(
			&accel_status_and_data[1], sample.accel_mg);
		sample.flags |= B4_IMU_SAMPLE_FLAG_ACCEL_VALID;
		if ((accel_status & B4_ACCEL_ANY_OVERRUN) != 0U) {
			sample.flags |= B4_IMU_SAMPLE_FLAG_ACCEL_OVERRUN;
		}
	}

	{
		uint32_t transaction_start_us = b4_timer_us();

		status = b4_read_register(sensor->device_id, B4_LSM303DLHC_ACCEL_ADDRESS,
				  B4_ACCEL_STATUS_REG,
				  &sensor->accel_status_after_read);
		sensor->accel_status_after_us =
			b4_timer_us() - transaction_start_us;
	}
	if (status < E_OK) {
		return B4_LSM303DLHC_STATUS_IO_ERROR;
	}

	if ((mag_status & (B4_MAG_DATA_READY | B4_MAG_DATA_LOCK)) != 0U) {
		uint32_t transaction_start_us = b4_timer_us();

		status = b4_read_registers(sensor->device_id,
					   B4_LSM303DLHC_MAG_ADDRESS,
					   B4_MAG_DATA_REG, mag_bytes,
					   sizeof(mag_bytes));
		sensor->mag_data_us = b4_timer_us() - transaction_start_us;
		if (status < E_OK) {
			return B4_LSM303DLHC_STATUS_IO_ERROR;
		}

		if (((mag_status & B4_MAG_DATA_READY) != 0U) &&
		    ((mag_status & B4_MAG_DATA_LOCK) == 0U)) {
			(void)b4_lsm303dlhc_decode_mag(mag_bytes, sample.mag_dut);
			sample.flags |= B4_IMU_SAMPLE_FLAG_MAG_VALID;
		}
	} else {
		sensor->mag_data_us = 0U;
	}

	if (sample.flags == 0U) {
		sensor->read_total_us = b4_timer_us() - operation_start_us;
		return B4_LSM303DLHC_STATUS_NO_FRESH_DATA;
	}

	tk_get_tim(&now);
	sample.timestamp_ms = now.lo;
	*sample_out = sample;
	sensor->read_total_us = b4_timer_us() - operation_start_us;

	return B4_LSM303DLHC_STATUS_OK;
}

B4_IMU_STATUS b4_lsm303dlhc_source_read(void *context,
					B4_IMU_SAMPLE *sample_out)
{
	B4_LSM303DLHC_STATUS status;

	if ((context == 0) || (sample_out == 0)) {
		return B4_IMU_STATUS_INVALID_ARGUMENT;
	}

	status = b4_lsm303dlhc_read((B4_LSM303DLHC *)context, sample_out);
	switch (status) {
	case B4_LSM303DLHC_STATUS_OK:
		return B4_IMU_STATUS_OK;
	case B4_LSM303DLHC_STATUS_INVALID_ARGUMENT:
		return B4_IMU_STATUS_INVALID_ARGUMENT;
	case B4_LSM303DLHC_STATUS_NO_FRESH_DATA:
		return B4_IMU_STATUS_NO_DATA;
	case B4_LSM303DLHC_STATUS_IO_ERROR:
	case B4_LSM303DLHC_STATUS_NOT_INITIALIZED:
	case B4_LSM303DLHC_STATUS_ID_MISMATCH:
	default:
		return B4_IMU_STATUS_IO_ERROR;
	}
}

B4_LSM303DLHC_STATUS b4_lsm303dlhc_close(B4_LSM303DLHC *sensor)
{
	ER status;

	if (sensor == 0) {
		return B4_LSM303DLHC_STATUS_INVALID_ARGUMENT;
	}

	if (sensor->initialized == 0U) {
		return B4_LSM303DLHC_STATUS_OK;
	}

	status = tk_cls_dev((ID)sensor->device_id, 0U);
	sensor->initialized = 0U;
	sensor->device_id = -1;
	if (status < E_OK) {
		b4_last_error = status;
		return B4_LSM303DLHC_STATUS_IO_ERROR;
	}

	return B4_LSM303DLHC_STATUS_OK;
}

int32_t b4_lsm303dlhc_last_error(void)
{
	return b4_last_error;
}

uint8_t b4_lsm303dlhc_get_accel_diagnostics(
	const B4_LSM303DLHC *sensor,
	uint8_t *ctrl1,
	uint8_t *ctrl4,
	uint8_t *status,
	uint8_t *status_after_read,
	uint32_t *overrun_events,
	uint32_t *poll_interval_ms,
	uint32_t *poll_interval_max_ms,
	uint32_t *poll_interval_us,
	uint32_t *poll_interval_max_us,
	uint32_t *accel_burst_us,
	uint32_t *accel_status_after_us,
	uint32_t *mag_status_us,
	uint32_t *mag_data_us,
	uint32_t *read_total_us,
	uint8_t raw_xyz[6])
{
	uint8_t index;

	if ((sensor == 0) || (ctrl1 == 0) || (ctrl4 == 0) ||
	    (status == 0) || (status_after_read == 0) ||
	    (overrun_events == 0) || (poll_interval_ms == 0) ||
	    (poll_interval_max_ms == 0) || (poll_interval_us == 0) ||
	    (poll_interval_max_us == 0) || (accel_burst_us == 0) ||
	    (accel_status_after_us == 0) || (mag_status_us == 0) ||
	    (mag_data_us == 0) || (read_total_us == 0) ||
	    (raw_xyz == 0) ||
	    (sensor->initialized == 0U) ||
	    (sensor->accel_raw_available == 0U)) {
		return 0U;
	}

	*ctrl1 = sensor->accel_ctrl1;
	*ctrl4 = sensor->accel_ctrl4;
	*status = sensor->accel_status;
	*status_after_read = sensor->accel_status_after_read;
	*overrun_events = sensor->accel_overrun_count;
	*poll_interval_ms = sensor->poll_interval_ms;
	*poll_interval_max_ms = sensor->poll_interval_max_ms;
	*poll_interval_us = sensor->poll_interval_us;
	*poll_interval_max_us = sensor->poll_interval_max_us;
	*accel_burst_us = sensor->accel_burst_us;
	*accel_status_after_us = sensor->accel_status_after_us;
	*mag_status_us = sensor->mag_status_us;
	*mag_data_us = sensor->mag_data_us;
	*read_total_us = sensor->read_total_us;
	for (index = 0U; index < 6U; index++) {
		raw_xyz[index] = sensor->accel_raw[index];
	}

	return 1U;
}
