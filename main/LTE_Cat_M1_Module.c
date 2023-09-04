#include <stdio.h>

#include <uart_d9.h>
#include <utility_d9.h>

#include <mqtt_d9.h>

#include "esp_netif.h"
#include "esp_netif_ppp.h"

#include "esp_modem_api.h"

#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13

/* A7670E Setting */
#define USER_A7670E_PWR_GPIO    4
#define USER_A7670E_RESET       5
#define USER_A7670E_BAT_EN      12

#define USER_A7670E_PIN_DTR     25
#define USER_A7670E_TX_GPIO     26
#define USER_A7670E_RX_GPIO     27
#define USER_A7670E_RTS         -1
#define USER_A7670E_CTS         -1
#define USER_A7670E_BAUD_RATE    115200
/******************/

#define MODEM_UART_RX_BUFFER_SIZE 1024
#define MODEM_UART_TX_BUFFER_SIZE 512
#define MODEM_UART_EVENT_QUEUE_SIZE 30
#define MODEM_UART_EVENT_TASK_STACK_SIZE 4096
#define MODEM_UART_EVENT_TASK_PRIORITY 5

#define UART_PORT_NUM 2
#define READ_TIME_OUT 3

#define PACKET_READ_TICS (1000 / portTICK_PERIOD_MS)

#define TAG "MAIN"

bool check_flag = false;
bool ip_addr_flag = false;

static void on_ppp_changed(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "PPP state changed event %d", (int)event_id);
    if (event_id == NETIF_PPP_ERRORUSER) {
        /* User interrupted event from esp-netif */
        esp_netif_t *netif = event_data;
        ESP_LOGI(TAG, "User interrupted event from netif:%p", netif);
    }
}

static void on_ip_event(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "IP event! %d", (int)event_id);
    if (event_id == IP_EVENT_PPP_GOT_IP) {
        esp_netif_dns_info_t dns_info;

        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        esp_netif_t *netif = event->esp_netif;

        ESP_LOGI(TAG, "Modem Connect to PPP Server");
        ESP_LOGI(TAG, "==============");
        ESP_LOGI(TAG, "IP          : " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Netmask     : " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "Gateway     : " IPSTR, IP2STR(&event->ip_info.gw));
        esp_netif_get_dns_info(netif, 0, &dns_info);
        ESP_LOGI(TAG, "Name Server1: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        esp_netif_get_dns_info(netif, 1, &dns_info);
        ESP_LOGI(TAG, "Name Server2: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        ESP_LOGI(TAG, "==============");

        ESP_LOGI(TAG, "GOT ip event!!!");
        ip_addr_flag = true;
    } else if (event_id == IP_EVENT_PPP_LOST_IP) {
        ESP_LOGI(TAG, "Modem Disconnect from PPP Server");
    } else if (event_id == IP_EVENT_GOT_IP6) {
        ESP_LOGI(TAG, "GOT IPv6 event!");

        ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
        ESP_LOGI(TAG, "Got IPv6 address " IPV6STR, IPV62STR(event->ip6_info.ip));
    }
}

static void uart_setup()
{
    // uart / rs485
    uart_gpio_config_t uart_gpio_config = {
        .tx_gpio = USER_A7670E_TX_GPIO,
        .rx_gpio = USER_A7670E_RX_GPIO,
        .rts_gpio = USER_A7670E_RTS,
        .cts_gpio = USER_A7670E_CTS,
    };

    uart_config_t uart_config = {
        .baud_rate = USER_A7670E_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_init(UART_PORT_NUM, &uart_gpio_config, &uart_config, BUF_SIZE_1024, READ_TIME_OUT);
}

static void lte_module_boot(void)
{
    delay_ms(3000);
    ESP_LOGI(TAG, "LTE Module(%s) Initializing...", CONFIG_USER_LTE_MODULE);

    gpio_reset_pin(USER_A7670E_BAT_EN);
    gpio_set_direction(USER_A7670E_BAT_EN, GPIO_MODE_OUTPUT);

    gpio_set_level(USER_A7670E_BAT_EN, 1);

    //A7670 Reset
    gpio_reset_pin(USER_A7670E_RESET);
    gpio_set_direction(USER_A7670E_RESET, GPIO_MODE_OUTPUT);

    gpio_set_level(USER_A7670E_RESET, 0);
    delay_ms(100);
    gpio_set_level(USER_A7670E_RESET, 1);
    delay_ms(3000);
    gpio_set_level(USER_A7670E_RESET, 0);

    gpio_reset_pin(USER_A7670E_PWR_GPIO);
    gpio_set_direction(USER_A7670E_PWR_GPIO, GPIO_MODE_OUTPUT);

    gpio_set_level(USER_A7670E_PWR_GPIO, 0);
    delay_ms(100);
    gpio_set_level(USER_A7670E_PWR_GPIO, 1);
    delay_ms(1000);
    gpio_set_level(USER_A7670E_PWR_GPIO, 0);

    ESP_LOGI(TAG, "Wait...");
    delay_ms(5000);
}

static void boot_recv()
{
    for (int i = 0; i < 30; i++)
    {
        char *read_buf = (char *)calloc(BUF_SIZE_1024, sizeof(char));
        int buf_len = uart_read_bytes(UART_PORT_NUM, read_buf, BUF_SIZE_1024, PACKET_READ_TICS);

        if (buf_len > 0)
        {
            ESP_LOGI(TAG, "%s", (char*)read_buf);
            // if (strstr(read_buf, "*ATREADY") != NULL) {
            //     break;
            // }
            if (strstr(read_buf, "PB DONE") != NULL) { // 부팅 후 마지막으로 나오는 메시지 확인 
                break;
            }
        }
        delay_ms(500);
    }
}


static void check_AT(int retry_cnt)
{
    char *write_buf = "AT\r\n";
    int i;
    for (i = 0; i < retry_cnt; i++)
    {
        uart_write_bytes(UART_PORT_NUM, write_buf, 4);
        delay_ms(500);

        char *read_buf = (char *)calloc(BUF_SIZE_1024, sizeof(char));
        int buf_len = uart_read_bytes(UART_PORT_NUM, read_buf, BUF_SIZE_1024, PACKET_READ_TICS);

        if (buf_len > 0)
        {
            ESP_LOGI(TAG, "%s", (char*)read_buf);
            if (strstr(read_buf, "OK") != NULL) {
                check_flag = true;
                break;
            }
        }
        delay_ms(500);
    }

    if (i >= 10)
    {
        ESP_LOGI(TAG, "retry cnt: %d", i);
    }
}

static void send_AT(char *cmd)
{
    char write_buf[60] = "AT";
    char etx_write_buf[2] = "\r\n";

    strcat(write_buf, cmd);
    
    ESP_LOGI(TAG, "SEND AT: %s", write_buf);
    
    strcat(write_buf, etx_write_buf);
    
    uart_write_bytes(UART_PORT_NUM, write_buf, 50);
}

static int recv_AT(int timeout)
{
    uint8_t *read_buf = (uint8_t *)calloc(BUF_SIZE_1024, sizeof(char));
    int buf_len = uart_read_bytes(UART_PORT_NUM, read_buf, BUF_SIZE_1024, (timeout / portTICK_PERIOD_MS));

    if (buf_len <= 0) return -1;
    
    ESP_LOGI(TAG, "(%d)%s", buf_len, (char*)read_buf);
    return 0;
}

int get_sim_status() {
    send_AT("+CPIN?");

    char *read_buf = (char*)calloc(BUF_SIZE_1024, sizeof(char));
    int buf_len = uart_read_bytes(UART_PORT_NUM, read_buf, BUF_SIZE_1024, PACKET_READ_TICS);

    int status = -1;

    if (buf_len <= 0)
        return -1;
    
    status = 0;

    char cpin_ret[7][12] = { "READY", "SIM PIN", "SIM PUK", "PH-SIM PIN", "SIM PIN2", "SIM PUK2", "PH-NET PIN" };
    ESP_LOGI(TAG, "(%d)%s", buf_len, (char*)read_buf);

    for (int i = 0; i < 7; i++)
    {
        if (strstr(read_buf, cpin_ret[i]) != NULL)
        {
            status = i;
            break;
        }    
    }

    return status;
}

int lte_at_init(void)
{
    send_AT("E0");  // Echo Off
    if (recv_AT(1000) != 0) { return -1; }

#ifdef GSM_DEBUG
    send_AT("+CMEE=2");  // turn on verbose error codes
#else
    send_AT("+CMEE=0");  // turn off error codes
#endif
    recv_AT(1000);

    send_AT("+CGMM");
    recv_AT(1000);

    // OK는 오는데 타임 아웃 발생함
    // Disable time and time zone URC's
    send_AT("+CTZR=0");
    recv_AT(1000);

    // OK는 오는데 타임 아웃 발생함
    // Enable automatic time zome update
    send_AT("+CTZU=1");
    recv_AT(1000);

    get_sim_status();

    return 0;
}

void gprs_disconnect(void)
{
    send_AT("+NETCLOSE");
    recv_AT(1000);
}

bool gprs_connect(void) {
    gprs_disconnect(); 
    
    char apn_str[50] = { 0 };
    sprintf(apn_str, "+CGDCONT=1,\"IP\",\"%s\",\"0.0.0.0\",0,0", CONFIG_USER_APN);
    send_AT(apn_str);
    recv_AT(1000);

    // Select TCP/IP application mode (command mode)
    send_AT("+CIPMODE=0");
    recv_AT(1000);

    // Set Sending Mode - send without waiting for peer TCP ACK
    send_AT("+CIPSENDMODE=0");
    recv_AT(1000);

    // Configure socket parameters
    // AT+CIPCCFG= <NmRetry>, <DelayTm>, <Ack>, <errMode>, <HeaderType>,
    //            <AsyncMode>, <TimeoutVal>
    // NmRetry = number of retransmission to be made for an IP packet
    //         = 10 (default)
    // DelayTm = number of milliseconds to delay before outputting received data
    //          = 0 (default)
    // Ack = sets whether reporting a string "Send ok" = 0 (don't report)
    // errMode = mode of reporting error result code = 0 (numberic values)
    // HeaderType = which data header of receiving data in multi-client mode
    //            = 1 (+RECEIVE,<link num>,<data length>)
    // AsyncMode = sets mode of executing commands
    //           = 0 (synchronous command executing)
    // TimeoutVal = minimum retransmission timeout in milliseconds = 75000
    send_AT("+CIPCCFG=10,0,0,0,1,0,75000");
    recv_AT(1000);

    send_AT("+CIPTIMEOUT=75000,15000,15000");
    recv_AT(1000);

    send_AT("+NETOPEN");
    recv_AT(1000);

    return true;
}

void is_gprs_connect(void)
{
    send_AT("+NETOPEN?");
    recv_AT(1000);

    send_AT("+IPADDR");
    recv_AT(1000);
}

void get_signal_quality(void) // connection_check
{
    send_AT("+CSQ");
    recv_AT(1000);
}

static void mqtt_connected_callback(esp_mqtt_event_handle_t event)
{
    ESP_LOGI(TAG, "MQTT Connect");
}

static void mqtt_disconnected_callback(esp_mqtt_event_handle_t event)
{
    ESP_LOGI(TAG, "MQTT Disconnect");
}

static int mqtt_setup()
{
    if (ip_addr_flag)
    {
        set_mqtt_callback_function(MQTT_EVENT_CONNECTED, mqtt_connected_callback);
        set_mqtt_callback_function(MQTT_EVENT_DISCONNECTED, mqtt_disconnected_callback);

        char *client_id = calloc(20, sizeof(char));
        sprintf(client_id, "%s_%d", CONFIG_USER_DEVICE_TYPE, get_random_int(1, 9999));

        mqtt_app_start(CONFIG_USER_BROKER_HOST,
                       atoi(CONFIG_USER_BROKER_PORT),
                       client_id,
                       CONFIG_USER_BROKER_USER,
                       CONFIG_USER_BROKER_PASS);

        return 0;
    }
    return -1;
}

void app_main(void)
{
    lte_module_boot();

    delay_ms(1000);
    ESP_LOGI(TAG, "LTE Cat M1 Module Start");
    
    /* Init and register system/core components */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &on_ip_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, &on_ppp_changed, NULL));

    /* Configure the PPP netif */
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(CONFIG_USER_APN);
    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
    esp_netif_t *esp_netif = esp_netif_new(&netif_ppp_config);
    assert(esp_netif);

    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    /* setup UART specific configuration based on kconfig options */
    dte_config.uart_config.port_num = UART_NUM_2;
    dte_config.uart_config.tx_io_num = USER_A7670E_TX_GPIO;
    dte_config.uart_config.rx_io_num = USER_A7670E_RX_GPIO;
    dte_config.uart_config.rts_io_num = USER_A7670E_RTS;
    dte_config.uart_config.cts_io_num = USER_A7670E_CTS;
    dte_config.uart_config.flow_control = UART_HW_FLOWCTRL_DISABLE;
    dte_config.uart_config.rx_buffer_size = MODEM_UART_RX_BUFFER_SIZE;
    dte_config.uart_config.tx_buffer_size = MODEM_UART_TX_BUFFER_SIZE;
    dte_config.uart_config.event_queue_size = MODEM_UART_EVENT_QUEUE_SIZE;
    dte_config.task_stack_size = MODEM_UART_EVENT_TASK_STACK_SIZE;
    dte_config.task_priority = MODEM_UART_EVENT_TASK_PRIORITY;
    dte_config.dte_buffer_size = MODEM_UART_RX_BUFFER_SIZE / 2;

    ESP_LOGI(TAG, "Initializing esp_modem for the SIM7600 module...");
    esp_modem_dce_t *dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM7600, &dte_config, &dce_config, esp_netif);
    // uart_setup();
    delay_ms(1000);

    boot_recv();
    check_AT(10);

    int rssi, ber;
    esp_err_t err = esp_modem_get_signal_quality(dce, &rssi, &ber);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_modem_get_signal_quality failed with %d %s", err, esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Signal quality: rssi=%d, ber=%d", rssi, ber);

    if (check_flag) ESP_LOGI(TAG, "LTE Cat M1 Module check complete!");

    int ret = lte_at_init();
    if (ret == 0) {
        ESP_LOGI(TAG, "lte_at_init %d", ret);
    }
    /*
    2 Automatic
    13 GSM Only -> 한국은 GSM 불가
    14 WCDMA Only
    38 LTE Only
    */
    send_AT("+CNMP=38");
    
    send_AT("I");
    recv_AT(1000);

    get_signal_quality();

    // 0,1 : home network (국내 유심)
    // 0,5 : roaming (해외 유심)
    // 0,3 : denied (거부, 리셋후 재시도 필요)
    send_AT("+CGREG?");
    recv_AT(1000);

    gprs_connect();

    // AT mode -> PPP mode 변경
    err = esp_modem_set_mode(dce, ESP_MODEM_MODE_DATA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_modem_set_mode(ESP_MODEM_MODE_DATA) failed with %d", err);
        return;
    }
    ESP_LOGI(TAG, "Waiting for IP address");
    for (int i = 0; i < 20; i++)
    {
        if (ip_addr_flag) break;
        delay_ms(1000);
    }

    mqtt_setup();
    
    int cnt = 0;
    while(1)
    {
        char *pub_buf = calloc(40, sizeof(char));
        sprintf(pub_buf, "hello dev91 %d", cnt);
        mqtt_pub("dev91/lte/test", pub_buf, 40, 0, 0);
        ESP_LOGI(TAG, "publish cnt: %d", cnt);

        cnt++;
        delay_ms(10000);
    }
}
