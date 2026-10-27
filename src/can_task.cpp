/*
 * can_task.cpp -- CAN TX, SDO polling, bus health
 */

#include "globals.h"

// -- SDO helpers ---------------------------------------------------------------
static void sdo_send_write(uint16_t param_id, int32_t raw_value) {
    // raw_value is already x32 scaled
    uint16_t index    = SDO_BASE_INDEX | (param_id >> 8);
    uint8_t  subindex = (uint8_t)(param_id & 0xFF);
    twai_message_t tx = {0};
    tx.identifier       = SDO_TX_ID;
    tx.data_length_code = 8;
    tx.data[0] = 0x23;   // SDO expedited write, 4 bytes
    tx.data[1] = index & 0xFF;
    tx.data[2] = index >> 8;
    tx.data[3] = subindex;
    tx.data[4] = raw_value & 0xFF;
    tx.data[5] = (raw_value >> 8) & 0xFF;
    tx.data[6] = (raw_value >> 16) & 0xFF;
    tx.data[7] = (raw_value >> 24) & 0xFF;
    twai_transmit(&tx, pdMS_TO_TICKS(5));
    Serial.printf("SDO write param=%d raw=%d\n", (int)param_id, (int)raw_value);
}

static void sdo_send_read(uint16_t param_id) {
    uint16_t index    = SDO_BASE_INDEX | (param_id >> 8);
    uint8_t  subindex = (uint8_t)(param_id & 0xFF);
    twai_message_t tx = {0};
    tx.identifier       = SDO_TX_ID;
    tx.data_length_code = 8;
    tx.data[0] = SDO_CMD_READ;
    tx.data[1] = index & 0xFF;
    tx.data[2] = index >> 8;
    tx.data[3] = subindex;
    twai_transmit(&tx, pdMS_TO_TICKS(5));
}

static int32_t sdo_parse_response(const twai_message_t *msg, uint16_t param_id) {
    if (msg->identifier != SDO_RX_ID)    return -1;
    if (msg->data[0] == SDO_RESP_ABORT)  return -1;
    if (msg->data[0] != SDO_RESP_4B &&
        msg->data[0] != SDO_RESP_2B)     return -1;
    uint16_t index    = (uint16_t)(msg->data[1] | (msg->data[2] << 8));
    uint8_t  subindex = msg->data[3];
    uint16_t got_id   = (uint16_t)(((index & 0xFF) << 8) | subindex);
    if (got_id != param_id) return -1;
    return (int32_t)(msg->data[4] | (msg->data[5] << 8) |
                     (msg->data[6] << 16) | (msg->data[7] << 24));
}

// -- CAN task ------------------------------------------------------------------
void canTask(void* p) {
    TickType_t lastTx  = xTaskGetTickCount();
    TickType_t lastSdo = xTaskGetTickCount();
    uint8_t    sdoPg   = 1;

    const uint16_t sdoParamId[NUM_PAGES] = { 0, PARAM_GEAR, PARAM_MOTACTIVE };

    for (;;) {
        // -- Drain RX, handle SDO responses -----------------------------------
        twai_message_t msg;
        while (twai_receive(&msg, 0) == ESP_OK) {
            if (msg.identifier == SDO_RX_ID) {
                int32_t raw = sdo_parse_response(&msg, sdoParamId[sdoPg]);
                if (raw >= 0) {
                    int8_t vcu_val = (int8_t)(raw / 32);
                    Serial.printf("SDO pg=%d param=%d raw=%d val=%d conf=%d\n",
                        (int)sdoPg, (int)sdoParamId[sdoPg],
                        (int)raw, (int)vcu_val, (int)confirmed[sdoPg]);
                    if (confirmed[sdoPg] >= 0 && vcu_val == confirmed[sdoPg]) {
                        acked[sdoPg] = true;
                    }
                }
            }
        }

        // -- Bus health -------------------------------------------------------
        twai_status_info_t st;
        if (twai_get_status_info(&st) == ESP_OK) {
            if      (st.state == TWAI_STATE_BUS_OFF)  twai_initiate_recovery();
            else if (st.state == TWAI_STATE_STOPPED)  twai_start();
            txFail = st.tx_failed_count;
        }

        // -- TX every 20ms ----------------------------------------------------
        if (xTaskGetTickCount() - lastTx >= pdMS_TO_TICKS(20)) {
            lastTx = xTaskGetTickCount();
            for (int pg = 0; pg < NUM_PAGES; pg++) {
                if (pg == 0 && confirmed[0] < 0) continue;
                twai_message_t tx = {0};
                tx.identifier       = pageCanId[pg];
                tx.data_length_code = 8;
                if (pg == 0) {
                    tx.data[3] = ((uint8_t)confirmed[0] << 4) & 0xF0;
                    tx.data[7] = cnt312 & 0x0F;
                    cnt312     = (cnt312 + 1) & 0x0F;
                } else {
                    tx.data[0] = (uint8_t)(confirmed[pg] >= 0 ?
                        confirmed[pg] : dialPos[pg]);
                }
                if (pg == 0) {
                    if (twai_transmit(&tx, 0) == ESP_OK) txCount++;
                } else {
                    twai_transmit(&tx, 0);
                }
            }
        }

        // -- SDO write when pages 1/2 confirmed value changes ----------------
        {
            static int8_t lastConfirmed[NUM_PAGES] = {-1,-1,-1};
            for (int pg = 1; pg < NUM_PAGES; pg++) {
                if (confirmed[pg] >= 0 && confirmed[pg] != lastConfirmed[pg]) {
                    lastConfirmed[pg] = confirmed[pg];
                    // Write value x32 scaled
                    sdo_send_write(sdoParamId[pg], (int32_t)confirmed[pg] * 32);
                }
            }
        }

        // -- SDO poll every 500ms, alternate pages 1 and 2 -------------------
        if (xTaskGetTickCount() - lastSdo >= pdMS_TO_TICKS(SDO_POLL_MS)) {
            lastSdo = xTaskGetTickCount();
            sdoPg = (sdoPg == 1) ? 2 : 1;
            if (confirmed[sdoPg] >= 0) {
                sdo_send_read(sdoParamId[sdoPg]);
                Serial.printf("SDO poll pg=%d param=%d\n",
                    (int)sdoPg, (int)sdoParamId[sdoPg]);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// -- CAN init ------------------------------------------------------------------
void initCAN() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NO_ACK);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        Serial.println("CAN install failed"); return;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("CAN start failed"); return;
    }
    Serial.println("CAN OK 500kbps NO_ACK TX=GPIO2 RX=GPIO1");
}
