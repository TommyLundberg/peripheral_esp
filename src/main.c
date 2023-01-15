/* main.c - Application main entry point */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <pm/pm.h>  
#include <device.h> 

#include <stdbool.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>

#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/drivers/sensor/sht4x.h>

#include <zephyr/kernel.h>
#include <drivers/adc.h>
#include <hal/nrf_saadc.h>
#include <zephyr/drivers/gpio.h>

#define SENSOR_1_NAME				"Temperature Sensor 1"
#define SENSOR_3_NAME				"Humidity Sensor 1"
#define SENSOR_4_NAME				"Soil Moisture Sensor"

/* Sensor Internal Update Interval [seconds] */
#define SENSOR_1_UPDATE_IVAL				60
#define SENSOR_3_UPDATE_IVAL				60
#define SENSOR_4_UPDATE_IVAL				60

/* ESS error definitions */
#define ESS_ERR_WRITE_REJECT				0x80
#define ESS_ERR_COND_NOT_SUPP				0x81

/* ESS Trigger Setting conditions */
#define ESS_TRIGGER_INACTIVE				0x00
#define ESS_FIXED_TIME_INTERVAL				0x01
#define ESS_NO_LESS_THAN_SPECIFIED_TIME		0x02
#define ESS_VALUE_CHANGED					0x03
#define ESS_LESS_THAN_REF_VALUE				0x04
#define ESS_LESS_OR_EQUAL_TO_REF_VALUE		0x05
#define ESS_GREATER_THAN_REF_VALUE			0x06
#define ESS_GREATER_OR_EQUAL_TO_REF_VALUE	0x07
#define ESS_EQUAL_TO_REF_VALUE				0x08
#define ESS_NOT_EQUAL_TO_REF_VALUE			0x09

#if !DT_HAS_COMPAT_STATUS_OKAY(sensirion_sht4x)
#error "No sensirion,sht4x compatible node found in the device tree"
#endif

#define ADC_DEVICE_NAME DT_ADC_0_NAME
#define ADC_RESOLUTION 12
#define ADC_GAIN ADC_GAIN_1_5
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_1ST_CHANNEL_ID 0
#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_AIN0

#define BUFFER_SIZE 8
static volatile uint16_t adc_sample_buffer[BUFFER_SIZE];
static uint32_t adc_sum = 0;
static uint16_t adc_val_bt;
const struct device *adc_dev;
static const int adc_gain_inv = 5;


static const struct adc_channel_cfg adc_channel_cfg = 
{
	.gain 			  = ADC_GAIN,
	.reference 		  = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id 	  = ADC_1ST_CHANNEL_ID,

	#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive = 	ADC_1ST_CHANNEL_INPUT,
	#endif
};

const struct adc_sequence_options sequence_opts = 
{
	.interval_us 	 = 0,
	.callback 		 = NULL,
	.user_data 		 = NULL,
	.extra_samplings = 7,
};

static int adc_sample(int adc_gain_inv, uint16_t *adc_val_out)
{
	const struct adc_sequence sequence = 
	{
		.options     = &sequence_opts,
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	if (!adc_dev) 
	{
		return -1;
	}

	int ret;
	ret = adc_read(adc_dev, &sequence);
	//printk("ADC read err: %d\n", ret);
	adc_sum = 0;
	
	//printk("ADC raw value: ");
	for (int i = 0; i < BUFFER_SIZE; i ++) 
	{
		printk("%d ", adc_sample_buffer[i]);
		adc_sum = adc_sum + adc_sample_buffer[i];
	}
	
	//printf("\n Measured voltage: ");
	int32_t adc_voltage = 0;
	adc_voltage = ((adc_sum * 75 * adc_gain_inv) >> 12);
	
	//printk("\n");
	*adc_val_out = (uint16_t)adc_voltage;
	return ret;
}

static ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			            void *buf, uint16_t len, uint16_t offset)
{
	const uint16_t *u16 = attr->user_data;
	uint16_t value = sys_cpu_to_le16(*u16);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, 
							 sizeof(value));
}

/* Environmental Sensing Service Declaration */
struct es_measurement 
{
	uint16_t flags; // Reserved for Future Use
	uint8_t  sampling_func;
	uint32_t meas_period;
	uint32_t update_interval;
	uint8_t  application;
	uint8_t  meas_uncertainty;
};

struct temperature_sensor 
{
	int16_t temp_value;
	int16_t lower_limit; // Valid range, min
	int16_t upper_limit; // Valid range, max
	uint8_t condition;   // ES trigger setting, Value Notification condition

	union 
	{
		uint32_t seconds;
		int16_t  ref_val; // Reference temperature
	};

	struct es_measurement meas;
};

struct humidity_sensor 
{
	int16_t humid_value;
	struct es_measurement meas;
};

static bool simulate_temp;

static struct temperature_sensor sensor_1 = 
{
		.temp_value 		   = 1200,
		.lower_limit 		   = -10000,
		.upper_limit           = 10000,
		.condition             = ESS_VALUE_CHANGED,
		.meas.sampling_func    = 0x00,
		.meas.meas_period      = 0x01,
		.meas.update_interval  = SENSOR_1_UPDATE_IVAL,
		.meas.application      = 0x1c,
		.meas.meas_uncertainty = 0x04,
};

static struct humidity_sensor sensor_3 = 
{
		.humid_value           = 6233,
		.meas.sampling_func    = 0x02,
		.meas.meas_period      = 0x0e10,
		.meas.update_interval  = SENSOR_3_UPDATE_IVAL,
		.meas.application      = 0x1c,
		.meas.meas_uncertainty = 0x01,
};


// Soil moisture sensor, read from ADC
static struct humidity_sensor sensor_4 = 
{
		.humid_value 		   = 6233,
		.meas.sampling_func    = 0x02,
		.meas.meas_period      = 0x0e10,
		.meas.update_interval  = SENSOR_4_UPDATE_IVAL,
		.meas.application      = 0x1c,
		.meas.meas_uncertainty = 0x01,
};

static void temp_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				   				 uint16_t value)
{
	simulate_temp = value == BT_GATT_CCC_NOTIFY;
}

struct read_es_measurement_rp 
{
	uint16_t flags; /* Reserved for Future Use */
	uint8_t  sampling_function;
	uint8_t  measurement_period[3];
	uint8_t  update_interval[3];
	uint8_t  application;
	uint8_t  measurement_uncertainty;
} 
__packed;

static ssize_t read_es_measurement(struct bt_conn *conn, 
	    	   const struct bt_gatt_attr *attr, void *buf, 
			   uint16_t len, uint16_t offset)
{
	const struct es_measurement *value = attr->user_data;
	struct read_es_measurement_rp rsp =
	{
		.flags 			         = rsp.flags = sys_cpu_to_le16(value->flags),
		.sampling_function       = value->sampling_func,
		.measurement_period      = NULL,
		.update_interval         = NULL,
		.application             = value->application,
		.measurement_uncertainty = value->meas_uncertainty
	};

	sys_put_le24(value->meas_period, rsp.measurement_period);
	sys_put_le24(value->update_interval, rsp.update_interval);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &rsp, sizeof(rsp));
}

static ssize_t read_temp_valid_range(struct bt_conn *conn,
			   						 const struct bt_gatt_attr *attr, void *buf,
			   						 uint16_t len, uint16_t offset)
{
	const struct temperature_sensor *sensor = attr->user_data;
	uint16_t tmp[] = {sys_cpu_to_le16(sensor->lower_limit),
			          sys_cpu_to_le16(sensor->upper_limit)};

	return bt_gatt_attr_read(conn, attr, buf, len, offset, tmp, sizeof(tmp));
}

struct es_trigger_setting_seconds 
{
	uint8_t condition;
	uint8_t sec[60];
} 
__packed;

struct es_trigger_setting_reference {
	uint8_t condition;
	int16_t ref_val;
} 
__packed;

static ssize_t read_temp_trigger_setting (struct bt_conn *conn, 
			   const struct bt_gatt_attr *attr, void *buf, uint16_t len,
 			   uint16_t offset)
{
	const struct temperature_sensor *sensor = attr->user_data;

	switch (sensor->condition) 
	{
		/* Operand N/A */
		case ESS_TRIGGER_INACTIVE:
			__fallthrough;
		case ESS_VALUE_CHANGED:
			return bt_gatt_attr_read(conn, attr, buf, len, offset, 
				   &sensor->condition, sizeof(sensor->condition));

		/* Seconds */
		case ESS_FIXED_TIME_INTERVAL:
		__fallthrough;
		case ESS_NO_LESS_THAN_SPECIFIED_TIME: 
		{
			struct es_trigger_setting_seconds rp;
			rp.condition = sensor->condition;
			sys_put_le24(sensor->seconds, rp.sec);
			return bt_gatt_attr_read(conn, attr, buf, len, offset,
						 			 &rp, sizeof(rp));
		}

		/* Reference temperature */
		default: 
		{
			struct es_trigger_setting_reference rp;
			rp.condition = sensor->condition;
			rp.ref_val = sys_cpu_to_le16(sensor->ref_val);
			return bt_gatt_attr_read(conn, attr, buf, len, offset, &rp, 
									 sizeof(rp));
		}
	}
}

static bool check_condition(uint8_t condition, int16_t old_val, int16_t new_val,
			    int16_t ref_val)
{
	switch (condition) 
	{
		case ESS_TRIGGER_INACTIVE:
			return false;
		case ESS_FIXED_TIME_INTERVAL:
		case ESS_NO_LESS_THAN_SPECIFIED_TIME:
		/* TODO: Check time requirements */
			return false;
		case ESS_VALUE_CHANGED:
			return new_val != old_val;
		case ESS_LESS_THAN_REF_VALUE:
			return new_val < ref_val;
		case ESS_LESS_OR_EQUAL_TO_REF_VALUE:
			return new_val <= ref_val;
		case ESS_GREATER_THAN_REF_VALUE:
			return new_val > ref_val;
		case ESS_GREATER_OR_EQUAL_TO_REF_VALUE:
			return new_val >= ref_val;
		case ESS_EQUAL_TO_REF_VALUE:
			return new_val == ref_val;
		case ESS_NOT_EQUAL_TO_REF_VALUE:
			return new_val != ref_val;
	default:
		return false;
	}
}

static void update_temperature(struct bt_conn *conn,
			       			   const struct bt_gatt_attr *chrc, double value,
			       			   struct temperature_sensor *sensor)
{
	bool notify = check_condition(sensor->condition, sensor->temp_value, value,
				      			  sensor->ref_val);

	/* Update temperature value */
	sensor->temp_value = value;

	/* Trigger notification if conditions are met */
	if (notify) 
	{
		value = sys_cpu_to_le16(sensor->temp_value);
		bt_gatt_notify(conn, chrc, &value, sizeof(value));
	}
}

BT_GATT_SERVICE_DEFINE
(   
	ess_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),

	/* Temperature Sensor 1 */
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, 
	 					   BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       		   BT_GATT_PERM_READ, read_u16, NULL,
						   &sensor_1.temp_value),

	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
			   		   read_es_measurement, NULL, &sensor_1.meas),

	BT_GATT_CUD(SENSOR_1_NAME, BT_GATT_PERM_READ),

	BT_GATT_DESCRIPTOR(BT_UUID_VALID_RANGE, BT_GATT_PERM_READ,
			   		   read_temp_valid_range, NULL, &sensor_1),

	BT_GATT_DESCRIPTOR(BT_UUID_ES_TRIGGER_SETTING, BT_GATT_PERM_READ,
					   read_temp_trigger_setting, NULL, &sensor_1),

	BT_GATT_CCC(temp_ccc_cfg_changed,
		        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* Humidity Sensor */
	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY, 
						   BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			               BT_GATT_PERM_READ, read_u16, NULL,
						   &sensor_3.humid_value),

	BT_GATT_CUD(SENSOR_3_NAME, BT_GATT_PERM_READ),

	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
			   		   read_es_measurement, NULL, &sensor_3.meas),

	/* Soil Moisture Sensor */
	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY, 
						   BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			               BT_GATT_PERM_READ, read_u16, NULL, 
						   &sensor_4.humid_value),

	BT_GATT_CUD(SENSOR_3_NAME, BT_GATT_PERM_READ),

	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
			   		   read_es_measurement, NULL, &sensor_4.meas), 
);

static const struct bt_data ad[] = 
{
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0x00, 0x03),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		          BT_UUID_16_ENCODE(BT_UUID_ESS_VAL)),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err!=0) 
	{
		return -1;
	} 
	
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	//printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = 
{
	.connected    = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	//printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		//printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	//printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	//printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = 
{
	.passkey_display = auth_passkey_display,
	.passkey_entry   = NULL,
	.cancel          = auth_cancel,
};

void main(void)
{
	int err;
	struct gpio_dt_spec gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(sen0), gpios);
	if (!device_is_ready(gpio_dev.port)) 
	{
		return -1;
	}

	gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT | GPIO_ACTIVE_HIGH);
	err = bt_enable(NULL);

	if (err) 
	{
		return -1;
	}

	bt_ready();
	bt_conn_auth_cb_register(&auth_cb_display);
	const struct device *sht = DEVICE_DT_GET_ANY(sensirion_sht4x);
	volatile struct sensor_value hum;
	volatile struct sensor_value temp;
	adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));

	if (!adc_dev) 
	{
		return -1;
	}

	err = adc_channel_setup(adc_dev, &adc_channel_cfg);

	if (err!=0) 
	{
		return -1;
	}	
	
	while (true) 
	{
		gpio_pin_set_dt(&gpio_dev, 1);
	
		if (!device_is_ready(sht)) 
		{
			return;
		}

		#if CONFIG_APP_USE_HEATER
		struct sensor_value heater_p;
		struct sensor_value heater_d;

		heater_p.val1 = CONFIG_APP_HEATER_PULSE_POWER;
		heater_d.val1 = CONFIG_APP_HEATER_PULSE_DURATION;
		sensor_attr_set(sht, SENSOR_CHAN_ALL,
		                SENSOR_ATTR_SHT4X_HEATER_POWER, &heater_p);
		sensor_attr_set(sht, SENSOR_CHAN_ALL,
					    SENSOR_ATTR_SHT4X_HEATER_DURATION, &heater_d);
		#endif

		while (sensor_sample_fetch(sht)) 
		{
			int volatile dummy = 0;
		}

		sensor_channel_get(sht, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(sht, SENSOR_CHAN_HUMIDITY, &hum);

		#if CONFIG_APP_USE_HEATER
		/*
		 * Conditions in which it makes sense to activate the heater
		 * are application/environment specific.
		 *
		 * The heater should not be used above SHT4X_HEATER_MAX_TEMP (65 °C)
		 * as stated in the datasheet.
		 *
		 * The temperature data will not be updated here for obvious reasons.
		 **/
		if (hum.val1 > CONFIG_APP_HEATER_HUMIDITY_THRESH && 
		    temp.val1 < SHT4X_HEATER_MAX_TEMP) 
		{
			printf("Activating heater.\n");

			if (sht4x_fetch_with_heater(sht)) 
			{
				printf("Failed to fetch sample from SHT4X device\n");
				return;
			}

			sensor_channel_get(sht, SENSOR_CHAN_HUMIDITY, &hum);
		}
		#endif

		err = adc_sample(adc_gain_inv, &adc_val_bt);
		if (err!=0) 
		{
			return -1;
		}
		printk("TEMP RAW: %d", temp);
		int16_t bt_tmp_val = (int16_t)100*(sensor_value_to_double(&temp));
		int16_t bt_hum_val = (int16_t)100*(sensor_value_to_double(&hum));
		update_temperature(NULL, &ess_svc.attrs[2], bt_tmp_val, &sensor_1);
		sensor_3.humid_value = bt_hum_val;
		sensor_4.humid_value = (int16_t)(adc_val_bt/30);
		gpio_pin_set_dt(&gpio_dev, 0);

		k_sleep(K_SECONDS(60));
	}
}
