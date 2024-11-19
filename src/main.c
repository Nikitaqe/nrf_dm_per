/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic Distance Measurement sample
 */

#include <bluetooth/scan.h>
#include <bluetooth/services/ddfs.h>
#include <dk_buttons_and_leds.h>
#include <dm.h>
#include <stdint.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include "nrf.h"  // Для работы с регистрами периферии nRF52833
#include "peer.h"
#include "service.h"

#define TIMER_INTERVAL 1000  // Интервал таймера (в тактах)

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED2
#define CON_STATUS_LED DK_LED3
#define RUN_LED_BLINK_INTERVAL 1000
#define SLEEP_TIME 10

#define SUPPORT_DM_CODE 0xFF55AA5A

#define TIME_MAX 0x00ff
#define TIME_MIN 0x00fa

struct adv_mfg_data {
  uint16_t company_code;    /* Company Identifier Code. */
  uint32_t support_dm_code; /* To identify the device that supports distance
                               measurement. */
  uint32_t rng_seed; /* Random seed used for generating hopping patterns. */
} __packed;

static struct adv_mfg_data mfg_data;
struct bt_le_adv_param adv_param_conn = BT_LE_ADV_PARAM_INIT(
    BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_NOTIFY_SCAN_REQ, TIME_MIN,
    TIME_MAX, NULL);

struct bt_le_adv_param adv_param_noconn =
    BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_SCANNABLE |
                             BT_LE_ADV_OPT_NOTIFY_SCAN_REQ,
                         TIME_MIN, TIME_MAX, NULL);

struct bt_le_adv_param *adv_param = &adv_param_conn;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, (unsigned char *)&mfg_data,
            sizeof(mfg_data)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_DDFS_VAL),
};

static struct bt_le_scan_param scan_param = {
    .type = BT_LE_SCAN_TYPE_ACTIVE,
    .interval = BT_GAP_SCAN_FAST_INTERVAL,
    .window = BT_GAP_SCAN_FAST_WINDOW,
    .options = BT_LE_SCAN_OPT_NONE,
    .timeout = 0,
};

static struct bt_scan_init_param scan_init = {
    .connect_if_match = 0, .scan_param = &scan_param, .conn_param = NULL};

static uint32_t scanner_random_share;

static struct bt_le_ext_adv *adv;
static void adv_work_handle(struct k_work *item);
static K_WORK_DEFINE(adv_work, adv_work_handle);
static void adv_update_data(void);
static uint32_t scanner_addr_to_random_share(const bt_addr_t *p_scanner_addr);
static uint32_t get_id_addr_random_share(void);

static struct bt_scan_manufacturer_data scan_mfg_data = {
    .data = (unsigned char *)&mfg_data,
    .data_len =
        sizeof(mfg_data.company_code) + sizeof(mfg_data.support_dm_code),
};

static uint32_t get_id_addr_random_share(void) {
  bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
  size_t count = CONFIG_BT_ID_MAX;

  bt_id_get(addrs, &count);

  __ASSERT(count == 1, "The sample assumes a single ID addr");

  return scanner_addr_to_random_share(&addrs[0].a);
}

static uint32_t scanner_addr_to_random_share(const bt_addr_t *p_scanner_addr) {
  return (p_scanner_addr->val[0] | p_scanner_addr->val[1] << 8 |
          p_scanner_addr->val[2] << 16 | p_scanner_addr->val[3] << 24) +
         (p_scanner_addr->val[4] | p_scanner_addr->val[5] << 8);
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
                              struct bt_scan_filter_match *filter_match,
                              bool connectable) {
  bt_addr_le_t addr;

  bt_addr_le_copy(&addr, device_info->recv_info->addr);
  peer_supported_add(device_info->recv_info->addr);
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, NULL, NULL);

static void adv_scanned_cb(struct bt_le_ext_adv *adv,
                           struct bt_le_ext_adv_scanned_info *info) {
  struct dm_request req;

  if (peer_supported_test(info->addr)) {
    bt_addr_le_copy(&req.bt_addr, info->addr);
    req.role = DM_ROLE_REFLECTOR;
    req.ranging_mode = peer_ranging_mode_get();
    req.rng_seed =
        peer_rng_seed_get() + scanner_addr_to_random_share(&info->addr->a);
    req.start_delay_us = 0;
    req.extra_window_time_us = 0;

    dm_request_add(&req);
    adv_update_data();
  }
}

const static struct bt_le_ext_adv_cb adv_cb = {
    .scanned = adv_scanned_cb,
};

static void adv_update_data(void) {
  int err;

  if (!adv) {
    return;
  }
  mfg_data.rng_seed = peer_rng_seed_prepare();
  err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err) {
    printk("Failed setting adv data (err %d)\n", err);
  }
}

static int adv_start(void) {
  int err;
  struct bt_le_ext_adv_start_param ext_adv_start_param = {0};

  if (adv) {
    err = bt_le_ext_adv_stop(adv);
    if (err) {
      printk("Failed to stop extended advertising  (err %d)\n", err);
      return err;
    }
    err = bt_le_ext_adv_delete(adv);
    if (err) {
      printk("Failed to delete advertising set  (err %d)\n", err);
      return err;
    }
  }

  err = bt_le_ext_adv_create(adv_param, &adv_cb, &adv);
  if (err) {
    printk("Failed to create advertising set (err %d)\n", err);
    return err;
  }

  err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err) {
    printk("Failed setting adv data (err %d)\n", err);
    return err;
  }

  err = bt_le_ext_adv_start(adv, &ext_adv_start_param);
  if (err) {
    printk("Failed to start extended advertising  (err %d)\n", err);
    return err;
  }

  return err;
}

static int scan_start(void) {
  int err;

  bt_scan_init(&scan_init);
  bt_scan_cb_register(&scan_cb);

  err =
      bt_scan_filter_add(BT_SCAN_FILTER_TYPE_MANUFACTURER_DATA, &scan_mfg_data);
  if (err) {
    printk("Scanning filters cannot be set (err %d)\n", err);
    return err;
  }

  err = bt_scan_filter_enable(BT_SCAN_MANUFACTURER_DATA_FILTER, false);
  if (err) {
    printk("Filters cannot be turned on (err %d)\n", err);
    return err;
  }

  scanner_random_share = get_id_addr_random_share();

  err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
  if (err) {
    printk("Scanning failed to start (err %d)\n", err);
    return err;
  }

  return err;
}

static void adv_work_handle(struct k_work *item) { adv_start(); }

static void connected(struct bt_conn *conn, uint8_t conn_err) {
  adv_param = &adv_param_noconn;
  k_work_submit(&adv_work);

  dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
  adv_param = &adv_param_conn;
  k_work_submit(&adv_work);
  dk_set_led_off(CON_STATUS_LED);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static int bt_sync_init(void) {
  int err;
  printk("DM Bluetooth LE Synchronization initialization\n");

  mfg_data.company_code = sys_cpu_to_le16(CONFIG_BT_COMPANY_ID_NORDIC);
  mfg_data.support_dm_code = sys_cpu_to_le32(SUPPORT_DM_CODE);
  mfg_data.rng_seed = sys_cpu_to_le32(peer_rng_seed_prepare());

  err = adv_start();
  if (err) {
    printk("Failed to start advertising (err %d)\n", err);
    return err;
  }

  err = scan_start();
  if (err) {
    printk("Failed to start scanning (err %d)\n", err);
  }

  return err;
}

static void data_ready(struct dm_result *result) {
  if (result->status) {
    peer_update(result);
  }
}

static struct dm_cb dm_cb = {
    .data_ready = data_ready,
};

// void TIMER2_IRQHandler(void)
// {
//     if (NRF_TIMER2->EVENTS_COMPARE[0]) {
//         NRF_TIMER2->EVENTS_COMPARE[0] = 0;  // Сбросить событие
// 		printk("Timer 2 interrupt\n");
// 		NRF_POWER->SYSTEMOFF = 0;
//         // Здесь можно добавить код для выполнения действий по прерыванию
//     }
// }

// void timer_init(void)
// {
//     // Остановить таймер перед настройкой
//     NRF_TIMER1->TASKS_STOP = 1;
//     NRF_TIMER1->TASKS_CLEAR = 1;  // Очистить таймер

//     // // Установим прескалер для таймера (например, делитель частоты 1 МГц)
//     NRF_TIMER1->PRESCALER = 9;  // Прескалер 9 для частоты 1 МГц

//     // // Установим значение для сравнения (например, через 1000 тактов)
//     NRF_TIMER1->CC[0] = 1000;

//     // // Включим прерывание при сравнении
//     NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Enabled <<
//     TIMER_INTENSET_COMPARE0_Pos;

//     // // Разрешаем прерывание в NVIC
//     NVIC_EnableIRQ(10);

//     // // Запустим таймер
//     NRF_TIMER1->TASKS_START = 1;
// }

// void enter_idle_mode(void)
// {
//     // Процессор переходит в режим ожидания прерывания (Idle)
// 	//NRF_POWER->TASKS_LOWPWR = 0;
// 	printk("Enter idle mode\n");
//     __WFI();  // Wait For Interrupt (ожидание прерывания)
// }

int main(void) {
  int err;
  uint32_t blink_status = 0;
  struct dm_init_param init_param;

  printk("Starting Distance Measurement example\n");

  err = dk_leds_init();
  if (err) {
    printk("LEDs init failed (err %d)\n", err);
    return 0;
  }

  err = peer_init();
  if (err) {
    printk("Peer init failed (err %d)\n", err);
    return 0;
  }

  init_param.cb = &dm_cb;

  err = dm_init(&init_param);
  if (err) {
    printk("Distance measurement init failed (err %d)\n", err);
    return 0;
  }

  err = service_ddfs_init();
  if (err) {
    printk("DDF Service init failed (err %d)\n", err);
    return 0;
  }

  err = bt_enable(NULL);
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return 0;
  }

  err = bt_sync_init();
  if (err) {
    printk("Synchronisation init failed (err %d)\n", err);
    return 0;
  }

  // timer_init();

  for (;;) {
    dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
    k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
    printk("BLE\n");
  }
}

void test_run_cmd(void) {
  printk("BLE off\n");
  // enter_idle_mode();
  // nrf_power_system_off();
  // NRF_POWER->SYSTEMOFF = 1;//выключение полностью
  // k_sleep(K_SECONDS(SLEEP_TIME));
  // bt_disable();
}

void test_run_on(void) {
  printk("BLE on\n");
  bt_enable(NULL);
  bt_sync_init();
}

SHELL_CMD_REGISTER(off_ble, NULL, "Run the test", test_run_cmd);
SHELL_CMD_REGISTER(on_ble, NULL, "Run the test", test_run_on);