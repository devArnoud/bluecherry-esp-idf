/**
 * @file bluecherry.c
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

#include "bluecherry.h"

#include <time.h>
#include <netdb.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_mac.h>
#include <esp_task_wdt.h>
#include <lwip/sockets.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <mbedtls/platform.h>
#include <mbedtls/net_sockets.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/error.h>
#include <mbedtls/timing.h>

/**
 * @brief The logging tag for this BlueCherry module.
 */
static const char* TAG = "BlueCherry";

/**
 * @brief The hostname of the BlueCherry cloud.
 */
static const char* BLUECHERRY_HOST = "coap.bluecherry.io";

/**
 * @brief The port of the BlueCherry cloud.
 */
static const char* BLUECHERRY_PORT = "5684";

/**
 * @brief The priority used for automatically syncing with BlueCherry.
 */
static const UBaseType_t BLUECHERRY_SP = 10;

/**
 * @brief The size of the BlueCherry CoAP header.
 */
static const size_t BLUECHERRY_COAP_HEADER_SIZE = 5;

/**
 * @brief The size the the BlueCherry MQTT header.
 */
static const size_t BLUECHERRY_MQTT_HEADER_SIZE = 2;

/**
 * @brief The maximum number of CoAP retransmits.
 */
static const uint8_t BLUECHERRY_MAX_RETRANSMITS = 4;

/**
 * @brief The CoAP acknowledgement base timeout period.
 */
static const double BLUECHERRY_ACK_TIMEOUT = 2.0;

/**
 * @brief The CoAP acknowledgement timeout period randomness factor.
 */
static const double BLUECHERRY_ACK_RANDOM_FACTOR = 1.5;

/**
 * @brief The maximum number of milliseconds to wait for a datagram to arrive on a socket.
 */
static const uint32_t BLUECHERRY_SSL_READ_TIMEOUT = 100;

/**
 * @brief The BlueCherry CA root + intermediate certificate used for CoAP DTLS communication.
 */
static const char *BLUECHERRY_CA = "-----BEGIN CERTIFICATE-----\r\n\
MIIBlTCCATqgAwIBAgICEAAwCgYIKoZIzj0EAwMwGjELMAkGA1UEBhMCQkUxCzAJ\r\n\
BgNVBAMMAmNhMB4XDTI0MDMyNDEzMzM1NFoXDTQ0MDQwODEzMzM1NFowJDELMAkG\r\n\
A1UEBhMCQkUxFTATBgNVBAMMDGludGVybWVkaWF0ZTBZMBMGByqGSM49AgEGCCqG\r\n\
SM49AwEHA0IABJGFt28UrHlbPZEjzf4CbkvRaIjxDRGoeHIy5ynfbOHJ5xgBl4XX\r\n\
hp/r8zOBLqSbu6iXGwgjp+wZJe1GCDi6D1KjZjBkMB0GA1UdDgQWBBR/rtuEomoy\r\n\
49ovMAnj5Hpmk2gTGjAfBgNVHSMEGDAWgBR3Vw0Y1sUvMhkX7xySsX55tvsu8TAS\r\n\
BgNVHRMBAf8ECDAGAQH/AgEAMA4GA1UdDwEB/wQEAwIBhjAKBggqhkjOPQQDAwNJ\r\n\
ADBGAiEApN7DmuufC/aqyt6g2Y8qOWg6AXFUyTcub8/Y28XY3KgCIQCs2VUXCPwn\r\n\
k8jR22wsqNvZfbndpHthtnPqI5+yFXrY4A==\r\n\
-----END CERTIFICATE-----\r\n\
-----BEGIN CERTIFICATE-----\r\n\
MIIBmDCCAT+gAwIBAgIUDjfXeosg0fphnshZoXgQez0vO5UwCgYIKoZIzj0EAwMw\r\n\
GjELMAkGA1UEBhMCQkUxCzAJBgNVBAMMAmNhMB4XDTI0MDMyMzE3MzU1MloXDTQ0\r\n\
MDQwNzE3MzU1MlowGjELMAkGA1UEBhMCQkUxCzAJBgNVBAMMAmNhMFkwEwYHKoZI\r\n\
zj0CAQYIKoZIzj0DAQcDQgAEB00rHNthOOYyKj80cd/DHQRBGSbJmIRW7rZBNA6g\r\n\
fbEUrY9NbuhGS6zKo3K59zYc5R1U4oBM3bj6Q7LJfTu7JqNjMGEwHQYDVR0OBBYE\r\n\
FHdXDRjWxS8yGRfvHJKxfnm2+y7xMB8GA1UdIwQYMBaAFHdXDRjWxS8yGRfvHJKx\r\n\
fnm2+y7xMA8GA1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/BAQDAgGGMAoGCCqGSM49\r\n\
BAMDA0cAMEQCID7AcgACnXWzZDLYEainxVDxEJTUJFBhcItO77gcHPZUAiAu/ZMO\r\n\
VYg4UI2D74WfVxn+NyVd2/aXTvSBp8VgyV3odA==\r\n\
-----END CERTIFICATE-----\r\n";

/**
 * @brief The types of CoAP packets.
 */
typedef enum {
    BLUECHERRY_COAP_TYPE_CON = 0,
    BLUECHERRY_COAP_TYPE_NON = 1,
    BLUECHERRY_COAP_TYPE_ACK = 2,
    BLUECHERRY_COAP_TYPE_RST = 3
} _bluecherry_coap_type;

/**
 * @brief The types of CoAP responses.
 */
typedef enum {
    BLUECHERRY_COAP_RSP_VALID = 0x43,
    BLUECHERRY_COAP_RSP_CONTINUE = 0x61
} _bluecherry_coap_response;

/**
 * @brief This structure represents a scheduled BlueCherry message.
 */
typedef struct {
    /**
     * @brief The length of the data
     */
    size_t len;

    /**
     * @brief A pointer to the data.
     */
    uint8_t *data;
} _bluecherry_msg_t;

/**
 * @brief The operational data used by the BlueCherry cloud connection.
 */
typedef struct {
  /**
   * @brief The current state of the BlueCherry cloud connection.
   */
  bluecherry_state state;

  /**
   * @brief The Mbed TLS SSL context.
   */
  mbedtls_ssl_context ssl;

  /**
   * @brief The Mbed TLS SSL configuration.
   */
  mbedtls_ssl_config ssl_conf;

  /**
   * @brief The Mbed TLS random number generator context and state.
   */
  mbedtls_ctr_drbg_context ctr_drbg;

  /**
   * @brief The Mbed TLS entropy context.
   */
  mbedtls_entropy_context entropy;

  /**
   * @brief The server certificate.
   */
  mbedtls_x509_crt cacert;

  /**
   * @brief The device certificate.
   */
  mbedtls_x509_crt devcert;

  /**
   * @brief The device key.
   */
  mbedtls_pk_context devkey;

  /**
   * @brief The Mbed TLS delay context timer.
   */
  mbedtls_timing_delay_context timer;

  /**
   * @brief The socket used to communicate with the BlueCherry cloud.
   */
  int sock;

  /**
   * @brief The outgoing message queue.
   */
  QueueHandle_t out_queue;

  /**
   * @brief The message handler or NULL to ignore incoming messages.
   */
  bluecherry_msg_handler_t msg_handler;

  /**
   * @brief Optional user arguments to pass to the incoming message handler.
   */
  void *msg_handler_args;

  /**
   * @brief The current CoAP message id that is used.
   */
  int32_t cur_message_id;

  /**
   * @brief The last CoAP message id that was acknowledged from the cloud.
   */
  int32_t last_acked_message_id;

  /**
   * @brief The last CoAP transmission time.
   */
  time_t last_tx_time;

  /**
   * @brief The length of the last incoming buffer data.
   */
  size_t in_buf_len;

  /**
   * @brief The buffer to receive incoming server data in.
   */
  uint8_t in_buf[BLUECHERRY_MAX_MESSAGE_LEN];
} _bluecherry_t;

/**
 * @brief The operational data used by the BlueCherry cloud  connection.
 */
static _bluecherry_t _bluecherry_opdata = { 0 };

/**
 * @brief Read up to len bytes from the DTLS socket.
 * 
 * This function will read up to len bytes from the DTLS socket. The function will handle session
 * re-negotiations and retries autonomously. This function will block for a maximum of 
 * BLUECHERRY_SSL_READ_TIMEOUT milliseconds.
 * 
 * @param buf Pointer to a buffer to read the results in.
 * @param len The maximum number of bytes to read.
 * 
 * @return The number of bytes read from the socket.
 */
static int _bluecherry_mbed_dtls_read(unsigned char *buf, size_t len)
{
  int ret;

  while(true) {
    ret = mbedtls_ssl_read(&_bluecherry_opdata.ssl, buf, len);
    if(ret > 0) {
      return ret;
    }

    if(ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(10));  
      continue;
    }

    if(ret == MBEDTLS_ERR_SSL_TIMEOUT) {
      return ret;
    }

    ESP_LOGE(TAG, "Could not read from the BlueCherry cloud connection: -%04X", -ret);
    return ret;
  }
}

/**
 * @brief Write a buffer to the DTLS socket.
 * 
 * This function will write a buffer to the DTLS socket. This function will handle session
 * re-negotiations and retries autonomously.
 * 
 * @param buf Pointer to a buffer write to the network.
 * @param len The length of the data to write to the network.
 * 
 * @return The Mbed TLS result code.
 */
static int _bluecherry_mbed_dtls_write(const unsigned char *buf, size_t len)
{
  int ret;

  do {
    ret = mbedtls_ssl_write(&_bluecherry_opdata.ssl, buf, len);
    if(ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if(ret < 0) {
      ESP_LOGE(TAG, "Could not write to the BlueCherry cloud connection: -%04X", -ret);
      return ret;
    }

    return ret;
  } while(true);
}

/**
 * @brief Perform CoAP transmit and receive operations with the BlueCherry cloud.
 * 
 * This function will calculate and add the correct CoAP header to the message buffer and transmit
 * the data over the the Mbed TLS DTLS socket. The message buffer must have a free space of 
 * BLUECHERRY_COAP_HEADER_SIZE preceeding the valid data. After the buffer is transmitted the
 * function will try to read the acknowledgement and new data coming from the cloud.
 * 
 * @param msg The message to send or NULL to send empty sync packet.
 * 
 * @return ESP_OK on success. 
 */
static esp_err_t _bluecherry_coap_rxtx(_bluecherry_msg_t *msg)
{
  uint8_t no_payload_hdr[BLUECHERRY_COAP_HEADER_SIZE];
  uint8_t *data = msg == NULL ? no_payload_hdr : msg->data;
  size_t data_len = msg == NULL ? BLUECHERRY_COAP_HEADER_SIZE : msg->len;

  if(data_len < BLUECHERRY_COAP_HEADER_SIZE) {
    ESP_LOGE(TAG, "Cannot send CoAP message smaller than %dB", BLUECHERRY_COAP_HEADER_SIZE);
    return ESP_ERR_NO_MEM;
  }

  int32_t last_acked_message_id = _bluecherry_opdata.last_acked_message_id;
  if(last_acked_message_id > _bluecherry_opdata.cur_message_id) {
      last_acked_message_id -= 0xffff;
  }
  uint8_t missed_msg_count =  _bluecherry_opdata.cur_message_id - last_acked_message_id - 1;

  data[0] = 0x40;
  data[1] = missed_msg_count;
  data[2] =  _bluecherry_opdata.cur_message_id >> 8;
  data[3] =  _bluecherry_opdata.cur_message_id & 0xFF;
  data[4] = 0xFF;

  double timeout = BLUECHERRY_ACK_TIMEOUT * 
                   (1 + (rand() / (RAND_MAX + 1.0)) * 
                   (BLUECHERRY_ACK_RANDOM_FACTOR - 1));

  for(uint8_t attempt = 1; attempt <= BLUECHERRY_MAX_RETRANSMITS; ++attempt) {
    _bluecherry_opdata.last_tx_time = time(NULL);

    ESP_LOGD(TAG, "Sending CoAP message (attempt %u, timeout %.1fs)", attempt, timeout);

    if(_bluecherry_mbed_dtls_write(data, data_len) < 0) {
      return ESP_FAIL;
    }

    _bluecherry_opdata.state = BLUECHERRY_STATE_CONNECTED_AWAITING_RESPONSE;

    while(true) {
      int ret = _bluecherry_mbed_dtls_read(_bluecherry_opdata.in_buf, BLUECHERRY_MAX_MESSAGE_LEN);
      if(ret >= 0) {
        _bluecherry_opdata.in_buf_len = ret;
        _bluecherry_opdata.state = BLUECHERRY_STATE_CONNECTED_RECEIVED_ACK;
        return ESP_OK;
      } else if(ret != MBEDTLS_ERR_SSL_TIMEOUT) {
        return ESP_FAIL;
      }

      if (difftime(time(NULL), _bluecherry_opdata.last_tx_time) >= timeout) {
        break;
      }
    }

    timeout *= 2;
  }

  _bluecherry_opdata.state = BLUECHERRY_STATE_CONNECTED_TIMED_OUT;
  return ESP_ERR_TIMEOUT;
}

/**
 * @brief The entrypoint of the automatic BlueCherry synchronisation task.
 * 
 * This function implements the automatic BlueCherry syncronisation. 
 * 
 * @param args A NULL pointer.
 * 
 * @return None.
 */
static void _bluecherry_sync_task(void *args)
{
  while(true) {
    bluecherry_sync();
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

/**
 * @brief Send DTLS data over a socket.
 * 
 * This function is called by Mbed TLS to send encrypted data over the underlying socket.
 * 
 * @param ctx Pointer to the socket descriptor.
 * @param buf Pointer to the buffer containing the data to send.
 * @param len Length of the data to send, in bytes.
 * 
 * @return The number of bytes sent on success, or -1 on error.
 */
static int _bluecherry_dtls_send(void *ctx, const unsigned char *buf, size_t len)
{
  int sock = *(int*) ctx;
  return send(sock, buf, len, 0);
}

// static int _bluecherry_dtls_send(void *ctx, const unsigned char *buf, size_t len)
// {
//     int sock = *(int*) ctx;
//     int ret = send(sock, buf, len, 0);
//     if (ret < 0) {
//         if (errno == EAGAIN || errno == EWOULDBLOCK) {
//             return MBEDTLS_ERR_SSL_WANT_WRITE;
//         }
//         return -1;
//     }
//     return ret;
// }

/**
 * @brief Receive DTLS data from a socket.
 * 
 * This function is called by Mbed TLS when a read is required from the underlying socket.
 * 
 * @param ctx Pointer to the socket descriptor.
 * @param buf Pointer to the buffer where the received data will be stored.
 * @param len Maximum number of bytes to read into the buffer.
 * 
 * @return int Number of bytes received on success, 0 if the connection was closed, or -1 on error.
 */
static int _bluecherry_dtls_recv(void *ctx, unsigned char *buf, size_t len)
{
  int sock = *(int*) ctx;
  return recv(sock, buf, len, 0);
}

// static int _bluecherry_dtls_recv(void *ctx, unsigned char *buf, size_t len)
// {
//     int sock = *(int*) ctx;
//     int ret = recv(sock, buf, len, 0);
//     if (ret < 0) {
//         if (errno == EAGAIN || errno == EWOULDBLOCK) {
//             return MBEDTLS_ERR_SSL_WANT_READ;
//         }
//         return -1;
//     }
//     return ret;
// }

static esp_err_t bluecherry_connect(void)
{
    int ret;
    const int handshake_total_timeout_s = 10; /* tune as needed */

    ESP_LOGI(TAG, "Connecting to the BlueCherry platform");

    /* close old socket */
    if (_bluecherry_opdata.sock >= 0) {
        shutdown(_bluecherry_opdata.sock, 0);
        close(_bluecherry_opdata.sock);
        _bluecherry_opdata.sock = -1;
    }

    struct addrinfo hints = {0};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    struct addrinfo *res = NULL;
    ret = getaddrinfo(BLUECHERRY_HOST, BLUECHERRY_PORT, &hints, &res);
    if (ret != 0 || res == NULL) {
        ESP_LOGE(TAG, "DNS lookup failed: %d", ret);
        if (res) freeaddrinfo(res);
        return ESP_FAIL;
    }

    _bluecherry_opdata.sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (_bluecherry_opdata.sock < 0) {
        ESP_LOGE(TAG, "socket() failed: %s", strerror(errno));
        freeaddrinfo(res);
        return ESP_FAIL;
    }

    if (connect(_bluecherry_opdata.sock, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(TAG, "connect() failed: %s", strerror(errno));
        close(_bluecherry_opdata.sock);
        _bluecherry_opdata.sock = -1;
        freeaddrinfo(res);
        return ESP_FAIL;
    }
    freeaddrinfo(res);

    /* make non-blocking */
    // int flags = fcntl(_bluecherry_opdata.sock, F_GETFL, 0);
    // fcntl(_bluecherry_opdata.sock, F_SETFL, flags | O_NONBLOCK);

    /* reset SSL context and clear any previous session */
    mbedtls_ssl_session_reset(&_bluecherry_opdata.ssl);
    mbedtls_ssl_set_bio(&_bluecherry_opdata.ssl,
                        &_bluecherry_opdata.sock,
                        _bluecherry_dtls_send,
                        _bluecherry_dtls_recv,
                        NULL);

    /* handshake with bounded total timeout */
    time_t t_start = time(NULL);
    while ((ret = mbedtls_ssl_handshake(&_bluecherry_opdata.ssl)) != 0) {
        if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE ||
            ret == MBEDTLS_ERR_SSL_TIMEOUT) {
            if (difftime(time(NULL), t_start) >= handshake_total_timeout_s) {
                ESP_LOGE(TAG, "DTLS handshake timed out");
                return ESP_ERR_TIMEOUT;
            }

            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        ESP_LOGE(TAG, "DTLS handshake failed: -0x%04X", -ret);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BlueCherry cloud connection established successfully");
    _bluecherry_opdata.state = BLUECHERRY_STATE_CONNECTED_IDLE;

    return ESP_OK;
}

esp_err_t bluecherry_init(const char *device_cert,
                          const char *device_key,
                          bluecherry_msg_handler_t msg_handler,
                          void *msg_handler_args,
                          bool auto_sync)
{
  if(_bluecherry_opdata.state != BLUECHERRY_STATE_UNINITIALIZED) {
    return ESP_OK;
  }

  _bluecherry_opdata.msg_handler = msg_handler;
  _bluecherry_opdata.msg_handler_args = msg_handler_args;

  uint8_t mac[6];
  esp_err_t eret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
  if(eret != ESP_OK) {
    ESP_LOGE(TAG, "Could not read the MAC of the ESP32: %s", esp_err_to_name(eret));
    return ESP_FAIL;
  }

  _bluecherry_opdata.out_queue =
    xQueueCreate(BLUECHERRY_MAX_PENDING_OUTGOING_MESSAGES, sizeof(_bluecherry_msg_t));
  if(_bluecherry_opdata.out_queue == NULL) {
    ESP_LOGE(TAG, "Unable to create outgoing message queue");
    return ESP_FAIL;
  }

  mbedtls_ssl_init(&_bluecherry_opdata.ssl);
  mbedtls_ssl_config_init(&_bluecherry_opdata.ssl_conf);
  mbedtls_ctr_drbg_init(&_bluecherry_opdata.ctr_drbg);
  mbedtls_entropy_init(&_bluecherry_opdata.entropy);
  mbedtls_x509_crt_init(&_bluecherry_opdata.cacert);
  mbedtls_x509_crt_init(&_bluecherry_opdata.devcert);
  mbedtls_pk_init(&_bluecherry_opdata.devkey);

  int ret = mbedtls_ctr_drbg_seed(&_bluecherry_opdata.ctr_drbg,
                                  mbedtls_entropy_func,
                                  &_bluecherry_opdata.entropy,
                                  mac,
                                  sizeof(mac));
  if(ret != 0) {
    ESP_LOGE(TAG, "Could not seed RNG: -%04X", -ret);
    goto error;
  }

  ret = mbedtls_x509_crt_parse(&_bluecherry_opdata.cacert,
                               (const uint8_t*) BLUECHERRY_CA,
                               strlen(BLUECHERRY_CA) + 1);
  if(ret != 0) {
    ESP_LOGE(TAG, "Could not parse BlueCherry CA certificate: -%04X", -ret);
    goto error;
  }

  ret = mbedtls_x509_crt_parse(&_bluecherry_opdata.devcert,
                               (const uint8_t*) device_cert,
                               strlen(device_cert) + 1);
  if(ret != 0) {
    ESP_LOGE(TAG, "Could not parse BlueCherry device certificate: -%04X", -ret);
    goto error;
  }

  ret = mbedtls_pk_parse_key(&_bluecherry_opdata.devkey,
                             (const uint8_t*) device_key,
                             strlen(device_key) + 1,
                             NULL,
                             0,
                             mbedtls_entropy_func,
                             &_bluecherry_opdata.ctr_drbg);
  if(ret != 0) {
    ESP_LOGE(TAG, "Could not parse the BlueCherry device key: -%04X", -ret);
    goto error;
  }

  ret = mbedtls_ssl_config_defaults(&_bluecherry_opdata.ssl_conf,
                                    MBEDTLS_SSL_IS_CLIENT,
                                    MBEDTLS_SSL_TRANSPORT_DATAGRAM,
                                    MBEDTLS_SSL_PRESET_DEFAULT);
  if(ret != 0) {
    ESP_LOGE(TAG, "Could not configure DTLS defaults: -%04X", -ret);
    goto error;
  }

  mbedtls_ssl_conf_read_timeout(&_bluecherry_opdata.ssl_conf, BLUECHERRY_SSL_READ_TIMEOUT);
  mbedtls_ssl_conf_authmode(&_bluecherry_opdata.ssl_conf, MBEDTLS_SSL_VERIFY_REQUIRED);
  mbedtls_ssl_conf_ca_chain(&_bluecherry_opdata.ssl_conf, &_bluecherry_opdata.cacert, NULL);
  mbedtls_ssl_conf_rng(&_bluecherry_opdata.ssl_conf,
                       mbedtls_ctr_drbg_random,
                       &_bluecherry_opdata.ctr_drbg);

  ret = mbedtls_ssl_conf_own_cert(&_bluecherry_opdata.ssl_conf,
                                  &_bluecherry_opdata.devcert,
                                  &_bluecherry_opdata.devkey);
  if(ret != 0) {
    ESP_LOGE(TAG, "Could not configure BlueCherry device certificate in context: -%04X", -ret);
    goto error;
  }

  ret = mbedtls_ssl_setup(&_bluecherry_opdata.ssl,
                          &_bluecherry_opdata.ssl_conf);
  if(ret != 0) {
    ESP_LOGE(TAG, "Could not setup the SSL context for use: -%04X", -ret);
    goto error;
  }
  
  mbedtls_ssl_set_timer_cb(&_bluecherry_opdata.ssl,
                           &_bluecherry_opdata.timer,
                           mbedtls_timing_set_delay, mbedtls_timing_get_delay);

  ret = mbedtls_ssl_set_hostname(&_bluecherry_opdata.ssl, BLUECHERRY_HOST);
  if(ret != 0) {
    ESP_LOGE(TAG, "Could not set the hostname in the SSL context: -%04X", -ret);
    goto error;
  }

  _bluecherry_opdata.state = BLUECHERRY_STATE_AWAIT_CONNECTION;

  if (bluecherry_connect() != ESP_OK) {
    ESP_LOGE(TAG, "Could not connect to BlueCherry platform");
    goto error;
  }

  if(auto_sync) {
    BaseType_t ret = xTaskCreate(_bluecherry_sync_task, "bc_sync", 4096, NULL, BLUECHERRY_SP, NULL);
    if(ret != pdPASS) {
      shutdown(_bluecherry_opdata.sock, 0);
      close(_bluecherry_opdata.sock);
      vQueueDelete(_bluecherry_opdata.out_queue);
      _bluecherry_opdata.out_queue = NULL;
      return ESP_ERR_NO_MEM;
    }
  }

  return ESP_OK;

error:
  vQueueDelete(_bluecherry_opdata.out_queue);
  _bluecherry_opdata.out_queue = NULL;
  mbedtls_ssl_free(&_bluecherry_opdata.ssl);
  mbedtls_ssl_config_free(&_bluecherry_opdata.ssl_conf);
  mbedtls_ctr_drbg_free(&_bluecherry_opdata.ctr_drbg);
  mbedtls_entropy_free(&_bluecherry_opdata.entropy);
  mbedtls_x509_crt_free(&_bluecherry_opdata.cacert);
  mbedtls_x509_crt_free(&_bluecherry_opdata.devcert);
  mbedtls_pk_free(&_bluecherry_opdata.devkey);
  _bluecherry_opdata.state = BLUECHERRY_STATE_UNINITIALIZED;
  return ESP_FAIL;
}

esp_err_t bluecherry_sync()
{
  if (_bluecherry_opdata.state == BLUECHERRY_STATE_AWAIT_CONNECTION) {
    esp_err_t ret = bluecherry_connect();
    if (ret != ESP_OK) {
      return ret;
    }
  }

  if(_bluecherry_opdata.state == BLUECHERRY_STATE_UNINITIALIZED ||
     _bluecherry_opdata.state == BLUECHERRY_STATE_CONNECTED_AWAITING_RESPONSE) {
      ESP_LOGE(TAG, "Cannot sync in the current state");
    return ESP_ERR_INVALID_STATE;
  }

  /* Send pending messages or a poll to the BlueCherry cloud */
  _bluecherry_msg_t out_msg;
  if(xQueuePeek(_bluecherry_opdata.out_queue, &out_msg, 0) == pdPASS) {
    if(_bluecherry_coap_rxtx(&out_msg) == ESP_OK) {
      if(xQueueReceive(_bluecherry_opdata.out_queue, &out_msg, 0) == pdPASS) {
        ESP_LOGD(TAG, "Synchronized messages with cloud");
        free(out_msg.data);
      } else {
        ESP_LOGD(TAG, "Could not remove transmitted message from queue");
        return ESP_FAIL;
      }
    } else {
      ESP_LOGE(TAG, "Could not sync with cloud");
      _bluecherry_opdata.state = BLUECHERRY_STATE_AWAIT_CONNECTION;
      return ESP_ERR_NOT_FINISHED;
    }
  } else {
    if(_bluecherry_coap_rxtx(NULL) != ESP_OK) {
      ESP_LOGE(TAG, "Could not sync with cloud");
      _bluecherry_opdata.state = BLUECHERRY_STATE_AWAIT_CONNECTION;
      return ESP_ERR_NOT_FINISHED;
    }
  }

  /* Process the received acknowledgedment */
  bool want_resync = false;

  uint16_t offset = 0;

  uint8_t header = _bluecherry_opdata.in_buf[offset++];
  uint8_t version = (header >> 6) & 0x03;
  if (version != 1) {
    ESP_LOGE(TAG, "Received CoAP packet with version %d, expeced 1", version);
    return ESP_ERR_INVALID_VERSION;
  }

  uint8_t type = (header >> 4) & 0x03;
  uint8_t token_len = header & 0x0F;
  offset += token_len;
  uint8_t code = _bluecherry_opdata.in_buf[offset++];
  uint16_t msg_id = _bluecherry_opdata.in_buf[offset++];
  msg_id <<= 8;
  msg_id  |= _bluecherry_opdata.in_buf[offset++];
  offset++;

  if(type == BLUECHERRY_COAP_TYPE_ACK) {
    if(msg_id != _bluecherry_opdata.cur_message_id) {
      ESP_LOGE(TAG,
               "Received ACK for %"PRIu16" instead of %"PRIu32"",
               msg_id,
               _bluecherry_opdata.cur_message_id);
      return ESP_ERR_INVALID_STATE;
    }

    _bluecherry_opdata.last_acked_message_id = msg_id;
  }

  switch(code) {
    case BLUECHERRY_COAP_RSP_VALID:
      want_resync = false;
      break;
    
      case BLUECHERRY_COAP_RSP_CONTINUE:
      want_resync = true;
      break;

    default:
      ESP_LOGE(TAG, "Received invalid CoAP code %02X", code);
      return ESP_ERR_INVALID_RESPONSE;
  }
  
  while(offset < _bluecherry_opdata.in_buf_len) {
    uint8_t topic = _bluecherry_opdata.in_buf[offset++];
    uint8_t data_len = _bluecherry_opdata.in_buf[offset++];

    if(topic == 0x00) {
      want_resync = true;

      //TODO: handle bluecherry service events
      // if (_blueCherryProcessEvent(_bluecherry_opdata.in_buf + offset, data_len)) {
      //     _blueCherry.emitErrorEvent = true;
      //     _blueCherry.otaSize = 0;
      // }
    } else if(_bluecherry_opdata.msg_handler != NULL) {
      _bluecherry_opdata.msg_handler(topic,
                                     data_len,
                                     _bluecherry_opdata.in_buf + offset,
                                     _bluecherry_opdata.msg_handler_args);
    }
    
    offset += data_len;
  }

  _bluecherry_opdata.cur_message_id += 1;
  if(_bluecherry_opdata.cur_message_id == 0) {
    _bluecherry_opdata.cur_message_id = 1;
  }

  if(want_resync) {
    _bluecherry_opdata.state = BLUECHERRY_STATE_CONNECTED_PENDING_MESSAGES;
    return BLUECHERRY_SYNC_CONTINUE;
  }

  _bluecherry_opdata.state = BLUECHERRY_STATE_CONNECTED_IDLE;
  return ESP_OK;
}

esp_err_t bluecherry_publish(uint8_t topic, uint16_t len, const uint8_t *data)
{
  ESP_LOGD(TAG, "Scheduling publish on topic 0x%02X with %dB of data", topic, len);
  if(len > (BLUECHERRY_MAX_MESSAGE_LEN - 
           (BLUECHERRY_COAP_HEADER_SIZE + BLUECHERRY_MQTT_HEADER_SIZE))) {
    ESP_LOGE(TAG, "The message exceeds the maximum allowed size");
    return ESP_ERR_INVALID_SIZE;
  }

  size_t total_len = BLUECHERRY_COAP_HEADER_SIZE + BLUECHERRY_MQTT_HEADER_SIZE + len;

  uint8_t *data_cpy = malloc(total_len);
  if(data_cpy == NULL) {
    ESP_LOGE(TAG, "Could not allocate publish buffer: %s", strerror(errno));
    return ESP_ERR_NO_MEM;
  }

  (data_cpy + BLUECHERRY_COAP_HEADER_SIZE)[0] = topic;
  (data_cpy + BLUECHERRY_COAP_HEADER_SIZE)[1] = len & 0xFF;
  memcpy(data_cpy + BLUECHERRY_COAP_HEADER_SIZE + BLUECHERRY_MQTT_HEADER_SIZE, data, len);

  _bluecherry_msg_t msg = {
    .len = total_len,
    .data = data_cpy
  };

  if(xQueueSendToBack(_bluecherry_opdata.out_queue, &msg, 0) != pdTRUE) {
    free(data_cpy);
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}