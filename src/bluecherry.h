/**
 * @file bluecherry.h
 * @author Daan Pape (daan@dptechnics.com)
 * @brief This code connects to the BlueCherry platform.
 * @version 1.0.0
 * @date 2025-07-25
 * @copyright Copyright (c) 2025 DPTechnics BV
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/lgpl-3.0.html>.
 */

#include <esp_err.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>

#ifndef BLUECHERRY_H
#define BLUECHERRY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief This return code is used by BlueCherry sync to signal if it want's to continue syncing.
 */
#define BLUECHERRY_SYNC_CONTINUE 0x100

/**
 * @brief The maximum size of a BlueCherry message payload.
 */
#define BLUECHERRY_MAX_MESSAGE_LEN 1024

/*
 * @brief The timeout in seconds for a SSL handshake to complete.
 */
#define SSL_HANDSHAKE_TIMEOUT_SEC 30

/**
 * @brief The maximum number of pending outgoing messages.
 */
static const UBaseType_t BLUECHERRY_MAX_PENDING_OUTGOING_MESSAGES = 32;

/**
 * @brief The different states the BlueCherry connection can be in.
 */
typedef enum {
  BLUECHERRY_STATE_UNINITIALIZED = 0,
  BLUECHERRY_STATE_AWAIT_CONNECTION,
  BLUECHERRY_STATE_CONNECTED_IDLE,
  BLUECHERRY_STATE_CONNECTED_AWAITING_RESPONSE,
  BLUECHERRY_STATE_CONNECTED_TIMED_OUT,
  BLUECHERRY_STATE_CONNECTED_RECEIVED_ACK,
  BLUECHERRY_STATE_CONNECTED_PENDING_MESSAGES,
} bluecherry_state;

/**
 * @brief Header of the function that handles incoming MQTT messages.
 *
 * This is the header of the function that is called when a new incoming MQTT message has arrived
 * from the BlueCherry cloud.
 *
 * @param topic The topic as the topic index.
 * @param len The length of the incoming data.
 * @param data The incoming data buffer.
 * @param args Optional user arguments, passed when the handler was installed.
 *
 * @return None
 */
typedef void (*bluecherry_msg_handler_t)(uint8_t topic, uint16_t len, const uint8_t *data,
                                         void *args);

/**
 * @brief Initialize the BlueCherry subsystem without ZTP.
 *
 * This function will initialize the BlueCherry IoT module without zero-touch provisioning enabled.
 *
 * @param device_cert The BlueCherry device certificate in PEM format.
 * @param device_key The BlueCherry device certificate's key in PEM format.
 * @param msg_handler The handler used for incoming messages or NULL to ignore them.
 * @param msg_handler_args Optional user pointer which is passed to the message handler.
 * @param auto_sync When set to true, the library will atomatically perform syncs in the background.
 *
 * @return ESP_OK on success.
 */
esp_err_t bluecherry_init(const char *device_cert, const char *device_key,
                          bluecherry_msg_handler_t msg_handler, void *msg_handler_args,
                          bool auto_sync);

/**
 * @brief Synchronize incoming and outgoing BlueCherry messages and perform OTA.
 *
 * This function will communicate with the BlueCherry cloud and send any enqueued MQTT messages,
 * check if there are any incoming MQTT messages and check for OTA updates.
 *
 * @return ESP_OK when finished, BLUECHERRY_SYNC_CONTINUE when it wants to continue.
 */
esp_err_t bluecherry_sync();

/**
 * @brief Enqueue an MQTT message for publishing.
 *
 * This function will add the MQTT message to the outgoing message queue. After bluecherry_sync, the
 * messages will be forwarded to the designated broker.
 *
 * @param topic The topic of the message, passed as the topic index.
 * @param len The length of the topic payload data.
 * @param data The topic payload data.
 * @param copy True when the data must be copied.
 *
 * @return ESP_OK on success.
 */
esp_err_t bluecherry_publish(uint8_t topic, uint16_t len, const uint8_t *data);

#ifdef __cplusplus
};
#endif

#endif