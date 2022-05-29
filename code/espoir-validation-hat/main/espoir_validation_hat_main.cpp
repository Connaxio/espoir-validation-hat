/* Copyright Connaxio inc. 2022.
 *
 * Author: Marc-Antoine Lalonde (@ma-lalonde)
 *
 */

#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "esp_adc_cal.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "soc/efuse_reg.h"

#include "driver/gpio.h"
#include "ADS1119.hpp"
#include "DACx0501.hpp"
#include "ADCDescriptor.hpp"

extern "C" {
void app_main(void);
}

TaskHandle_t eth_init_handle;
TaskHandle_t validation_handle;
esp_eth_mac_t *mac;

// List IO pins as digital or analog. IO18 and IO23 are excluded because they are the I2C pins.
uint8_t digital_io_count = 9;
uint8_t digital_io_pins[] = {
		GPIO_NUM_2,
		GPIO_NUM_4,
		GPIO_NUM_5,
		GPIO_NUM_9,
		GPIO_NUM_10,
		GPIO_NUM_12,
		GPIO_NUM_13,
		GPIO_NUM_14,
		GPIO_NUM_15 };
uint8_t analog_io_count = 5;
uint8_t analog_io_pins[] = { GPIO_NUM_34, GPIO_NUM_36, GPIO_NUM_37, GPIO_NUM_38, GPIO_NUM_39 };

// List ADC input pins
const uint8_t num_channels_adc1 = 5;
const uint8_t num_channels_adc2 = 6;
uint8_t adc1_pins[num_channels_adc1] = { 34, 36, 37, 38, 39 };
uint8_t adc2_pins[num_channels_adc2] = { 2, 4, 12, 13, 14, 15 };
bool write_two_point_adc_cal = true;
esp_err_t err_dhcp = ESP_FAIL;

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
	uint8_t mac_addr[6] = { 0 };
	/* we can get the ethernet driver handle from event data */
	esp_eth_handle_t eth_handle = *(esp_eth_handle_t*) event_data;

	switch (event_id) {
	case ETHERNET_EVENT_CONNECTED:
		esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
		ESP_LOGI("ETH", "Ethernet Link Up");
		ESP_LOGI("ETH", "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1],
				mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
		break;
	case ETHERNET_EVENT_DISCONNECTED:
		ESP_LOGI("ETH", "Ethernet Link Down");
		break;
	case ETHERNET_EVENT_START:
		ESP_LOGI("ETH", "Ethernet Started");
		break;
	case ETHERNET_EVENT_STOP:
		ESP_LOGI("ETH", "Ethernet Stopped");
		break;
	default:
		break;
	}
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
	ip_event_got_ip_t *event = (ip_event_got_ip_t*) event_data;
	const esp_netif_ip_info_t *ip_info = &event->ip_info;

	ESP_LOGI("ETH", "Ethernet Got IP Address");
	ESP_LOGI("ETH", "~~~~~~~~~~~");
	ESP_LOGI("ETH", "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
	ESP_LOGI("ETH", "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
	ESP_LOGI("ETH", "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
	ESP_LOGI("ETH", "~~~~~~~~~~~");
	err_dhcp = ESP_OK;
}

void ethernet_init(void *pvParameters) {
	// Initialize TCP/IP network interface (should be called only once in application)
	ESP_ERROR_CHECK(esp_netif_init());
	// Create default event loop that running in background
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	// Create new default instance of esp-netif for Ethernet
	esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
	esp_netif_t *eth_netif = esp_netif_new(&cfg);

	// Init MAC and PHY configs to default
	eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
	eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

	phy_config.phy_addr = 0;
	phy_config.reset_gpio_num = GPIO_NUM_NC;
	eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
	esp32_emac_config.smi_mdc_gpio_num = GPIO_NUM_32;
	esp32_emac_config.smi_mdio_gpio_num = GPIO_NUM_33;
	esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
	esp_eth_phy_t *phy = esp_eth_phy_new_ksz80xx(&phy_config);

	esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
	esp_eth_handle_t eth_handle = NULL;
	ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));
	/* attach Ethernet driver to TCP/IP stack */
	ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

	/* Force 10 mbps */
	/*
	 bool autonego = false;
	 eth_speed_t speed = ETH_SPEED_10M;
	 esp_eth_ioctl(eth_handle, ETH_CMD_S_AUTONEGO, &autonego);
	 esp_eth_ioctl(eth_handle, ETH_CMD_S_SPEED, &speed);
	 */

	// Register user defined event handers
	ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

	/* start Ethernet driver state machine */
	ESP_ERROR_CHECK(esp_eth_start(eth_handle));

	vTaskDelete(NULL);
}

void validate_board(void *pvParameters) {
	uint8_t settling_ticks = pdMS_TO_TICKS(50);

	/* Disable all GPIOs for next step */
	gpio_config_t gpio_conf = { .pin_bit_mask = (1ULL << GPIO_NUM_2), .mode = GPIO_MODE_DISABLE, .pull_up_en =
			GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE };

	for (uint8_t i = 0; i < digital_io_count; ++i) {
		gpio_conf.pin_bit_mask = (1ULL << digital_io_pins[i]);
		gpio_config(&gpio_conf);
	}

	for (uint8_t i = 0; i < analog_io_count; ++i) {
		gpio_conf.pin_bit_mask = (1ULL << analog_io_pins[i]);
		gpio_config(&gpio_conf);
	}

	/* Initialize validation board DAC and ADC */
	Sensor *adc = new ADS1119("validation_adc", GPIO_NUM_23, GPIO_NUM_18);
	DACx0501 *dac = new DACx0501("validation_dac", GPIO_NUM_23, GPIO_NUM_18);
	esp_err_t err = adc->init();
	ESP_LOGD("main", "ADC init: %s", esp_err_to_name(err));

	if (err == ESP_OK) {
		err = dac->init();
		ESP_LOGD("main", "DAC init: %s", esp_err_to_name(err));
	}
	if (err == ESP_OK) {
		dac->disable();
		ESP_LOGD("main", "DAC disable: %s", esp_err_to_name(err));
	}
	if (err == ESP_OK) {
		err = adc->update();
		ESP_LOGD("main", "ADC update: %s", esp_err_to_name(err));
	}
	printf("Init peripherals:\t%s\r\n", esp_err_to_name(err));

	/* Check which ADC calibration methods are burned in eFuse */
	for (uint8_t i = 0; i < ESP_ADC_CAL_VAL_MAX; ++i) {
		if (esp_adc_cal_check_efuse((esp_adc_cal_value_t) i) == ESP_OK) {
			if (i == ESP_ADC_CAL_VAL_EFUSE_VREF) {
				printf("ADC calibration available:\tEFUSE_VREF\r\n");
			} else if (i == ESP_ADC_CAL_VAL_EFUSE_TP) {
				printf("ADC calibration available:\tEFUSE_TP\r\n");
				// TP calibration already done. Do not write efuse.
				write_two_point_adc_cal = false;
				int16_t A1 = 0, B1 = 0, A2 = 0, B2 = 0;
				esp_efuse_read_field_blob(ESP_EFUSE_ADC1_TP_LOW, &A1, 7);
				esp_efuse_read_field_blob(ESP_EFUSE_ADC1_TP_HIGH, &B1, 9);
				esp_efuse_read_field_blob(ESP_EFUSE_ADC2_TP_LOW, &A2, 7);
				esp_efuse_read_field_blob(ESP_EFUSE_ADC2_TP_HIGH, &B2, 9);
				if ((A1 >> 6) == 1) {
					A1 |= 0xFF80;
				}
				if ((B1 >> 8) == 1) {
					B1 |= 0xFF00;
				}
				if ((A2 >> 6) == 1) {
					A2 |= 0xFF80;
				}
				if ((B2 >> 8) == 1) {
					B2 |= 0xFF00;
				}
				printf("EFUSE TP cal content: A1: %i | B1: %i | A2: %i | B2: %i\r\n", A1, B1, A2, B2);

			} else {
				printf("ADC calibration available:\t%i\r\n", i);
			}
		}
	}

	// Do ADC Two Point calibration
	if (err == ESP_OK) {
		uint32_t adc1_readings_150[num_channels_adc1] = { 0 };
		uint32_t adc2_readings_150[num_channels_adc2] = { 0 };
		uint32_t adc1_readings_850[num_channels_adc1] = { 0 };
		uint32_t adc2_readings_850[num_channels_adc2] = { 0 };
		float adc_ref_readings[2] = { 0 };
		float adc_ref_errors[2] = { 0 };
		uint8_t avg_pow = 5;	// Average readings with n = 2^avg_pow

		if (err == ESP_OK) {
			err = dac->enable();
		}

		/* Do readings for 150 mV */
		if (err == ESP_OK) {
			err = dac->set(150);
			vTaskDelay(settling_ticks);
		}
		if (err == ESP_OK) {
			adc_power_acquire();
			for (uint8_t i = 0; i < num_channels_adc1; ++i) {
				ADCDescriptor adc1((gpio_num_t) adc1_pins[i], ADC_WIDTH_BIT_12, ADC_ATTEN_DB_0);
				for (uint8_t j = 0; j < (1 << avg_pow); ++j) {
					adc1.update();
					adc1_readings_150[i] += adc1.getRawValue();
				}
				adc1_readings_150[i] = adc1_readings_150[i] >> avg_pow;
			}
			for (uint8_t i = 0; i < num_channels_adc2; ++i) {
				ADCDescriptor adc2((gpio_num_t) adc2_pins[i], ADC_WIDTH_BIT_12, ADC_ATTEN_DB_0);
				for (uint8_t j = 0; j < (1 << avg_pow); ++j) {
					adc2.update();
					adc2_readings_150[i] += adc2.getRawValue();
				}
				adc2_readings_150[i] = adc2_readings_150[i] >> avg_pow;
			}

			for (uint8_t j = 0; j < (1 << avg_pow); ++j) {
				static_cast<ADS1119*>(adc)->updateChannel(3);
				adc_ref_readings[0] += adc->getValue("ch3");
			}
			adc_ref_readings[0] /= (1 << avg_pow);

			adc_power_release();
            printf("ADC1 readings:\t%i\t%i\t%i\t%i\t%i\r\n", adc1_readings_150[0], adc1_readings_150[1],
                    adc1_readings_150[2], adc1_readings_150[3], adc1_readings_150[4]);
            printf("ADC2 readings:\t%i\t%i\t%i\t%i\t%i\t%i\r\n", adc2_readings_150[0], adc2_readings_150[1],
                    adc2_readings_150[2], adc2_readings_150[3], adc2_readings_150[4], adc2_readings_150[5]);
            printf("Ref ADC voltage:\t%.3f V\r\n", adc_ref_readings[0]);
			adc_ref_errors[0] = (adc_ref_readings[0] - 150) / 150;
			printf("Ref ADC 150 mV error:\t%.2f%%\r\n", 100.0f * adc_ref_errors[0]);
		}

		/* Do readings for 850 mV */
		if (err == ESP_OK) {
			err = dac->set(850);
			vTaskDelay(settling_ticks);
		}
		if (err == ESP_OK) {
			adc_power_acquire();

			for (uint8_t i = 0; i < num_channels_adc1; ++i) {
				ADCDescriptor adc1((gpio_num_t) adc1_pins[i], ADC_WIDTH_BIT_12, ADC_ATTEN_DB_0);
				for (uint8_t j = 0; j < (1 << avg_pow); ++j) {
					adc1.update();
					adc1_readings_850[i] += adc1.getRawValue();
				}
				adc1_readings_850[i] = adc1_readings_850[i] >> avg_pow;
			}
			for (uint8_t i = 0; i < num_channels_adc2; ++i) {
				ADCDescriptor adc2((gpio_num_t) adc2_pins[i], ADC_WIDTH_BIT_12, ADC_ATTEN_DB_0);
				for (uint8_t j = 0; j < (1 << avg_pow); ++j) {
					adc2.update();
					adc2_readings_850[i] += adc2.getRawValue();
				}
				adc2_readings_850[i] = adc2_readings_850[i] >> avg_pow;
			}

			for (uint8_t j = 0; j < (1 << avg_pow); ++j) {
				static_cast<ADS1119*>(adc)->updateChannel(3);
				adc_ref_readings[1] += adc->getValue("ch3");
			}
			adc_ref_readings[1] /= (1 << avg_pow);

			adc_power_release();
            printf("ADC1 readings:\t%i\t%i\t%i\t%i\t%i\r\n", adc1_readings_850[0], adc1_readings_850[1],
                    adc1_readings_850[2], adc1_readings_850[3], adc1_readings_850[4]);
            printf("ADC2 readings:\t%i\t%i\t%i\t%i\t%i\t%i\r\n", adc2_readings_850[0], adc2_readings_850[1],
                    adc2_readings_850[2], adc2_readings_850[3], adc2_readings_850[4], adc2_readings_850[5]);
            printf("Ref ADC voltage:\t%.3f V\r\n", adc_ref_readings[1]);
			adc_ref_errors[1] = (adc_ref_readings[1] - 850) / 850;
			printf("Ref ADC 850 mV error:\t%.2f%%\r\n", 100.0f * adc_ref_errors[1]);
		}

		// Make sure the reference DAC and ADC work properly
		if (abs(adc_ref_errors[0]) > 0.005 || abs(adc_ref_errors[1]) > 0.005) {
			err = ESP_ERR_INVALID_STATE;
		}
		printf("Reference DAC/ADC levels:\t%s\r\n", esp_err_to_name(err));

		if (err == ESP_OK) {
			/* Calculate mean and std dev for readings, make sure there are no outliers */
			float A1_mean = 0, B1_mean = 0, A2_mean = 0, B2_mean = 0;
			float A1_stdev = 0, B1_stdev = 0, A2_stdev = 0, B2_stdev = 0;
			for (uint8_t i = 0; i < num_channels_adc1; ++i) {
				A1_mean += adc1_readings_150[i];
				B1_mean += adc1_readings_850[i];
			}
			for (uint8_t i = 0; i < num_channels_adc2; ++i) {
				A2_mean += adc2_readings_150[i];
				B2_mean += adc2_readings_850[i];
			}

			A1_mean /= 1.0f * num_channels_adc1;
			B1_mean /= 1.0f * num_channels_adc1;
			A2_mean /= 1.0f * num_channels_adc2;
			B2_mean /= 1.0f * num_channels_adc2;

			for (uint8_t i = 0; i < num_channels_adc1; ++i) {
				A1_stdev += (adc1_readings_150[i] - A1_mean) * (adc1_readings_150[i] - A1_mean);
				B1_stdev += (adc1_readings_850[i] - B1_mean) * (adc1_readings_850[i] - B1_mean);
			}
			for (uint8_t i = 0; i < num_channels_adc2; ++i) {
				A2_stdev += (adc2_readings_150[i] - A2_mean) * (adc2_readings_150[i] - A2_mean);
				B2_stdev += (adc2_readings_850[i] - B2_mean) * (adc2_readings_850[i] - B2_mean);
			}

			A1_stdev = sqrt(1.0f * A1_stdev / num_channels_adc1);
			B1_stdev = sqrt(1.0f * B1_stdev / num_channels_adc1);
			A2_stdev = sqrt(1.0f * A2_stdev / num_channels_adc2);
			B2_stdev = sqrt(1.0f * B2_stdev / num_channels_adc2);

			printf("A1 mean / stdev:\t%.2f / %.2f\r\n", A1_mean, A1_stdev);
			printf("B1 mean / stdev:\t%.2f / %.2f\r\n", B1_mean, B1_stdev);
			printf("A2 mean / stdev:\t%.2f / %.2f\r\n", A2_mean, A2_stdev);
			printf("B2 mean / stdev:\t%.2f / %.2f\r\n", B2_mean, B2_stdev);

			/* Apply calibration if everything went well. */
			if (A1_stdev < 5 && B1_stdev < 5 && A2_stdev < 5 && B2_stdev < 5) {
				int16_t A1 = round((A1_mean - 278.0f) / 4.0f);
				int16_t B1 = round((B1_mean - 3265.0f) / 4.0f);
				int16_t A2 = round((A2_mean - 421.0f) / 4.0f);
				int16_t B2 = round((B2_mean - 3406.0f) / 4.0f);

				printf("A1: %i | B1: %i | A2: %i | B2: %i\r\n", A1, B1, A2, B2);

				printf("Write ADC TP cal: %i\r\n", write_two_point_adc_cal);
				if (write_two_point_adc_cal) {
					if (err == ESP_OK) {
						err = esp_efuse_write_field_blob(ESP_EFUSE_ADC1_TP_LOW, &A1, 7);
					}
					if (err == ESP_OK) {
						err = esp_efuse_write_field_blob(ESP_EFUSE_ADC1_TP_HIGH, &B1, 9);
					}
					if (err == ESP_OK) {
						err = esp_efuse_write_field_blob(ESP_EFUSE_ADC2_TP_LOW, &A2, 7);
					}
					if (err == ESP_OK) {
						err = esp_efuse_write_field_blob(ESP_EFUSE_ADC2_TP_HIGH, &B2, 9);
					}
					if (err == ESP_OK) {
						esp_efuse_write_reg(EFUSE_BLK0,
								(EFUSE_BLK0_RDATA3_REG - DR_REG_EFUSE_BASE) / sizeof(uint32_t),
								EFUSE_RD_BLK3_PART_RESERVE);
					}
					printf("Burning ADC Two point fuse: %s\r\n", esp_err_to_name(err));
				}

			} else {
				err = ESP_FAIL;
			}
		}
		printf("ADC Two Point calibration:\t%s\r\n", esp_err_to_name(err));
	}

	/* Check that all digital IOs can output properly. */
	if (err == ESP_OK) {
		err = dac->disable();
		vTaskDelay(settling_ticks);
	}
	if (err == ESP_OK) {
		int value = 0;
		for (uint8_t i = 0; i < digital_io_count; ++i) {
			gpio_conf.mode = GPIO_MODE_OUTPUT;
			gpio_conf.pin_bit_mask = (1ULL << digital_io_pins[i]);
			gpio_config(&gpio_conf);

			if (err == ESP_OK) {
				gpio_set_level((gpio_num_t) digital_io_pins[i], 0);
				vTaskDelay(settling_ticks);
				err = static_cast<ADS1119*>(adc)->updateChannel(3);
				value = adc->getValue("ch3");
				if (value > 100 || err != ESP_OK) {
					ESP_LOGI("VALIDATION", "ADC[%i] value: %i. Error:\t%s", i, value, esp_err_to_name(err));
					err = ESP_ERR_INVALID_RESPONSE;
				} else {
					gpio_set_level((gpio_num_t) digital_io_pins[i], 1);
					vTaskDelay(settling_ticks);
					err = static_cast<ADS1119*>(adc)->updateChannel(3);
					value = adc->getValue("ch3");
					if (value < 1500 || err != ESP_OK) {
						ESP_LOGI("VALIDATION", "ADC[%i] value: %i. Error:\t%s", i, value,
								esp_err_to_name(err));
						err = ESP_ERR_INVALID_RESPONSE;
					}
				}

			}
			gpio_conf.mode = GPIO_MODE_DISABLE;
			gpio_config(&gpio_conf);
		}
		printf("Digital IO output:\t%s\r\n", esp_err_to_name(err));
	}

	// Check that all IO can read properly
	if (err == ESP_OK) {
		// Use first IO pin as output
		gpio_conf.mode = GPIO_MODE_OUTPUT;
		gpio_conf.pin_bit_mask = (1ULL << digital_io_pins[0]);
		gpio_config(&gpio_conf);

		// Test all other digital IO
		gpio_conf.mode = GPIO_MODE_INPUT;
		for (uint8_t i = 1; i < digital_io_count; ++i) {
			gpio_conf.mode = GPIO_MODE_INPUT;
			gpio_conf.pin_bit_mask = (1ULL << digital_io_pins[i]);
			gpio_config(&gpio_conf);
			gpio_set_level((gpio_num_t) digital_io_pins[0], 1);
			vTaskDelay(settling_ticks);
			if (gpio_get_level((gpio_num_t) digital_io_pins[i]) != 1) {
				err = ESP_ERR_INVALID_RESPONSE;
				ESP_LOGI("VALIDATION", "IO read failed:\t%i.", digital_io_pins[i]);
			}
			gpio_set_level((gpio_num_t) digital_io_pins[0], 0);
			vTaskDelay(settling_ticks);
			if (gpio_get_level((gpio_num_t) digital_io_pins[i]) != 0) {
				err = ESP_ERR_INVALID_RESPONSE;
				ESP_LOGI("VALIDATION", "IO read failed:\t%i.", digital_io_pins[i]);
			}
		}

		// Test all analog IO
		for (uint8_t i = 0; i < analog_io_count; ++i) {
			gpio_conf.mode = GPIO_MODE_INPUT;
			gpio_conf.pin_bit_mask = (1ULL << analog_io_pins[i]);
			gpio_config(&gpio_conf);
			gpio_set_level((gpio_num_t) digital_io_pins[0], 1);
			vTaskDelay(settling_ticks);
			if (gpio_get_level((gpio_num_t) analog_io_pins[i]) != 1) {
				err = ESP_ERR_INVALID_RESPONSE;
				ESP_LOGI("VALIDATION", "IO read failed:\t%i.", analog_io_pins[i]);
			}
			gpio_set_level((gpio_num_t) digital_io_pins[0], 0);
			vTaskDelay(settling_ticks);
			if (gpio_get_level((gpio_num_t) analog_io_pins[i]) != 0) {
				err = ESP_ERR_INVALID_RESPONSE;
				ESP_LOGI("VALIDATION", "IO read failed:\t%i.", analog_io_pins[i]);
			}
		}

		// Test first digital IO
		gpio_conf.mode = GPIO_MODE_OUTPUT;
		gpio_conf.pin_bit_mask = (1ULL << digital_io_pins[1]);
		gpio_config(&gpio_conf);
		gpio_conf.mode = GPIO_MODE_INPUT;
		gpio_conf.pin_bit_mask = (1ULL << digital_io_pins[0]);
		gpio_config(&gpio_conf);

		gpio_set_level((gpio_num_t) digital_io_pins[1], 1);
		vTaskDelay(settling_ticks);
		if (gpio_get_level((gpio_num_t) digital_io_pins[0]) != 1) {
			err = ESP_ERR_INVALID_RESPONSE;
		}
		gpio_set_level((gpio_num_t) digital_io_pins[1], 0);
		vTaskDelay(settling_ticks);
		if (gpio_get_level((gpio_num_t) digital_io_pins[0]) != 0) {
			err = ESP_ERR_INVALID_RESPONSE;
		}
		printf("IO input:\t%s\r\n", esp_err_to_name(err));
	}

	if (err == ESP_OK) {
		/* Measure supply voltages */
		float error_5V = (adc->getValue("ch1") * 6.1 - 5000) / 5000;
		float error_3V3 = (adc->getValue("ch0") * 6.1 - 3300) / 3300;

		if (abs(error_5V) > 0.02f || abs(error_3V3) > 0.01f) {
			err = ESP_ERR_INVALID_STATE;
		}

		printf("5V error:\t%.2f%%\r\n", error_5V * 100.0f);
		printf("3.3V error:\t%.2f%%\r\n", error_3V3 * 100.0f);
		printf("Voltages:\t%s\r\n", esp_err_to_name(err));
	}

	delete (adc);
	delete (dac);
	while (err != ESP_OK || err_dhcp != ESP_OK){
		printf("I/Os: %s | DHCP: %s\r\n", esp_err_to_name(err), esp_err_to_name(err_dhcp));
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	printf("I/Os: %s | DHCP: %s\r\nQC PASS\r\n", esp_err_to_name(err), esp_err_to_name(err_dhcp));
	vTaskDelete(NULL);
}

void app_main(void) {
	xTaskCreate(ethernet_init, "ethInit_Task", 8192, NULL, 3, &eth_init_handle);
	xTaskCreate(validate_board, "validation_Task", 4096, NULL, 3, &validation_handle);

	while (true) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
