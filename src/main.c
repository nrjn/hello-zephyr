//
// Created by niranjan on 12.07.18.
//

#include <zephyr.h>
#include <misc/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <gpio.h>
#include <board.h>

#define MY_STACK_SIZE 1024
#define  MY_PRIORITY 5

int count1, count2;

struct device *led_button_device;
static struct gpio_callback gpio_cb;

static struct bt_conn *default_conn;

void blink_led(u32_t pin, int count)
{
  gpio_pin_write(led_button_device, (pin), count % 2);
}

k_tid_t b1_thread_id, b2_thread_id;

K_THREAD_STACK_DEFINE(b1_stack_area, MY_STACK_SIZE);
static struct k_thread b1_thread_data;

void b1_thread(void){
  while (1)
  {
    printk("thread 2\r\n");
    k_sleep(1000);
    blink_led(LED0_GPIO_PIN, count2);
    blink_led(LED3_GPIO_PIN, count2);
    count2++;
  }
}

K_THREAD_STACK_DEFINE(bms_thread_stack_area, MY_STACK_SIZE);
static struct k_thread bms_thread_data;

void b2_thread(void *a, void *b, void *c) {
  while (1)
  {
    printk("thread 1\r\n");
    k_sleep(1000);
    blink_led(LED1_GPIO_PIN, count1);
    blink_led(LED2_GPIO_PIN, count1);
    count1++;
  }
}

void button_pressed(struct device *gpiob, struct gpio_callback *cb, u32_t pins)
{
  if (pins == BIT(SW0_GPIO_PIN))
  {
    printk("Button1 pressed | Suspend 1 and 2\r\n");

    gpio_pin_write(led_button_device, LED0_GPIO_PIN, 1);
    gpio_pin_write(led_button_device, LED1_GPIO_PIN, 1);
    gpio_pin_write(led_button_device, LED2_GPIO_PIN, 1);
    gpio_pin_write(led_button_device, LED3_GPIO_PIN, 1);

    k_thread_suspend(b2_thread_id);
    k_thread_suspend(b1_thread_id);
  }
  else if (pins == BIT(SW1_GPIO_PIN))
  {
    printk("Button2 pressed | Suspend 1, resume 2\r\n");

    gpio_pin_write(led_button_device, LED1_GPIO_PIN, 1);
    gpio_pin_write(led_button_device, LED2_GPIO_PIN, 1);

    k_thread_suspend(b2_thread_id);
    k_thread_resume(b1_thread_id);
  }
  else if (pins == BIT(SW2_GPIO_PIN))
  {
    printk("Button3 pressed | suspend 2, resume 1\r\n");

    gpio_pin_write(led_button_device, LED0_GPIO_PIN, 1);
    gpio_pin_write(led_button_device, LED3_GPIO_PIN, 1);

    k_thread_suspend(b1_thread_id);
    k_thread_resume(b2_thread_id);
  }
  else if (pins == BIT(SW3_GPIO_PIN))
  {
    printk("Button4 pressed | resume 1 and 2\r\n");

    k_thread_resume(b2_thread_id);
    k_thread_resume(b1_thread_id);
  }
  else
    printk("What did you press %x\r\n", pins);
}

static void device_found(const bt_addr_le_t * addr, s8_t rssi, u8_t type, struct net_buf_simple * ad)
{
  char addr_str[BT_ADDR_LE_STR_LEN];

  if (default_conn)
  {
    return;
  }

  /* We are only interested in connectable events */
  if (type != BT_LE_ADV_IND && type != BT_LE_ADV_DIRECT_IND)
  {
    return;
  }

  bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
  printk("Device found: %s RSSI (%d)\r\n", addr_str, rssi);

  /* Connect only to devices in close proximity */
  if (rssi < -70)
  {
    return;
  }

  if (bt_le_scan_stop())
  {
    return;
  }

  default_conn = bt_conn_create_le(addr, BT_LE_CONN_PARAM_DEFAULT);
}

static void connected(struct bt_conn * conn, u8_t err)
{
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (err)
  {
    printk("Failed to connect to %s (%u)\r\n", addr, err);
    return;
  }

  if (conn != default_conn)
  {
    return;
  }

  printk("Connected to: %s\r\n", addr);

  struct bt_conn_info conn_info;
  bt_conn_get_info(conn, &conn_info);
 /* printk("Type(%d), Role(%d) \r\n src: %s \r\n Dest: %s \r\n, Int: %d, Latency: %d, Timeout: %d",
                          conn_info.type,
                          conn_info.role,
                          conn_info.le.src, conn_info.le.dst,
                          conn_info.le.interval, conn_info.le.latency, conn_info.le.timeout);
*/
  bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void disconnected(struct bt_conn * conn, u8_t reason)
{
  char addr[BT_ADDR_LE_STR_LEN];
  int err;

  if (conn != default_conn)
  {
    return;
  }

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  printk("Disconnected: %s reason(%d)\r\n", addr, reason);

  bt_conn_unref(default_conn);
  default_conn = NULL;

  /* We are not performing active scan here */
  err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
  if (err)
  {
    printk("Scanning failed to start (err %d)\r\n", err);
  }
}

struct bt_conn_cb connection_callbacks = {
        .connected = connected,
        .disconnected = disconnected,
};


void main (void)
{
  int err;

  b1_thread_id = k_thread_create(
          &b1_thread_data,
          b1_stack_area,
          K_THREAD_STACK_SIZEOF(b1_stack_area),
          b1_thread,
          NULL, NULL, NULL,
          MY_PRIORITY, 0, K_NO_WAIT);

  b2_thread_id = k_thread_create(
          &bms_thread_data, bms_thread_stack_area,
          K_THREAD_STACK_SIZEOF(bms_thread_stack_area),
          b2_thread,
          NULL, NULL, NULL,
          MY_PRIORITY, 0, K_NO_WAIT);

  led_button_device = device_get_binding(SW0_GPIO_NAME);
  if (!led_button_device)
  {
    printk("error\r\n");
    return;
  }

  gpio_pin_configure(led_button_device, SW0_GPIO_PIN,
                     (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
                      GPIO_INT_ACTIVE_LOW | GPIO_PUD_PULL_UP));
  gpio_pin_configure(led_button_device, SW1_GPIO_PIN,
                     (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
                      GPIO_INT_ACTIVE_LOW | GPIO_PUD_PULL_UP));
  gpio_pin_configure(led_button_device, SW2_GPIO_PIN,
                     (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
                      GPIO_INT_ACTIVE_LOW | GPIO_PUD_PULL_UP));
  gpio_pin_configure(led_button_device, SW3_GPIO_PIN,
                     (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
                      GPIO_INT_ACTIVE_LOW | GPIO_PUD_PULL_UP));

  gpio_pin_configure(led_button_device, LED0_GPIO_PIN, GPIO_DIR_OUT);
  gpio_pin_configure(led_button_device, LED1_GPIO_PIN, GPIO_DIR_OUT);
  gpio_pin_configure(led_button_device, LED2_GPIO_PIN, GPIO_DIR_OUT);
  gpio_pin_configure(led_button_device, LED3_GPIO_PIN, GPIO_DIR_OUT);

  gpio_init_callback(&gpio_cb, button_pressed,
                     (BIT(SW0_GPIO_PIN) | BIT(SW1_GPIO_PIN) |
                      BIT(SW2_GPIO_PIN) | BIT(SW3_GPIO_PIN)));
  gpio_add_callback(led_button_device, &gpio_cb);

  gpio_pin_enable_callback(led_button_device, SW0_GPIO_PIN);
  gpio_pin_enable_callback(led_button_device, SW1_GPIO_PIN);
  gpio_pin_enable_callback(led_button_device, SW2_GPIO_PIN);
  gpio_pin_enable_callback(led_button_device, SW3_GPIO_PIN);

  err = bt_enable(NULL);
  if (err)
  {
    printk("BLE INIT Failed (err %d)\r\n", err);
  }

  printk("BLE Initialized\r\n");

  bt_conn_cb_register(&connection_callbacks);

  err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
  if (err)
  {
     printk("Scanning failed to start (err %d)\r\n", err);
    return;
  }

  printk("Scanning sucessfully started\r\n");
}
