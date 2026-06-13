/* ============================================================
   ESP32 通用固件 v2 — WiFi配网 + MQTT + OTA + 模块化传感器
   所有配置存在 NVS，同固件适配不同板子
   ============================================================ */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_mac.h"
#include "mqtt_client.h"

#define TAG "HUB"

/* ===== MQTT 默认地址 ===== */
#define MQTT_BROKER_DEFAULT "192.168.31.251"

/* ===== 雷达默认引脚 ===== */
#define RADAR_RX_DEFAULT  16
#define RADAR_TX_DEFAULT  17

/* ===== 全局 NVS 配置 (配网时写入, 启动时读取) ===== */
static char cfg_ssid[33]     = "";
static char cfg_pass[65]     = "";
static char cfg_mqtt[64]     = MQTT_BROKER_DEFAULT;
static char cfg_name[33]     = "";   // 设备名称, 默认用 MAC
static int32_t cfg_radar_en    = 0;
static int32_t cfg_radar_rx    = RADAR_RX_DEFAULT;
static int32_t cfg_radar_tx    = RADAR_TX_DEFAULT;
static int32_t cfg_radar_thres = 45;

/* ===== 传感器数据 ===== */
static int radar_ok, radar_state, radar_mv_en, radar_st_en, radar_dist, radar_light;

/* ===== MQTT ===== */
static esp_mqtt_client_handle_t mqtt = NULL;
static char mqtt_topic[64];
static int wifi_got_ip = 0, wifi_configured = 0;

/* ---------- NVS 工具 ---------- */
static void cfg_save_str(const char *key, const char *val)
{ nvs_handle_t h; nvs_open("cfg", NVS_READWRITE, &h); nvs_set_str(h, key, val); nvs_commit(h); nvs_close(h); }
static void cfg_save_int(const char *key, int val)
{ nvs_handle_t h; nvs_open("cfg", NVS_READWRITE, &h); nvs_set_i32(h, key, val); nvs_commit(h); nvs_close(h); }

static void cfg_load_all(void)
{
    nvs_handle_t h;
    if (nvs_open("cfg", NVS_READONLY, &h) != ESP_OK) return;
    size_t len;
    len = sizeof(cfg_ssid);   nvs_get_str(h, "ssid", cfg_ssid, &len);
    len = sizeof(cfg_pass);   nvs_get_str(h, "pass", cfg_pass, &len);
    len = sizeof(cfg_mqtt);   nvs_get_str(h, "mqtt", cfg_mqtt, &len);
    len = sizeof(cfg_name);   nvs_get_str(h, "name", cfg_name, &len);
    nvs_get_i32(h, "radar_en",  &cfg_radar_en);
    nvs_get_i32(h, "radar_rx",  &cfg_radar_rx);
    nvs_get_i32(h, "radar_tx",  &cfg_radar_tx);
    nvs_get_i32(h, "radar_thr", &cfg_radar_thres);
    nvs_close(h);
}

/* ---------- 雷达解析 ---------- */
static void radar_parse(uint8_t *d, int frm_len)
{
    if (d[0]!=0xF4||d[1]!=0xF3||d[2]!=0xF2||d[3]!=0xF1) return;
    if (d[7] != 0xAA) return;
    // 帧尾固定位置: 最后4字节是 F8 F7 F6 F5
    int tail = frm_len - 4;

    radar_ok = 1;
    radar_state = d[8];
    radar_mv_en = d[11];
    radar_st_en = d[14];

    static int low_cnt = 0;
    if (radar_mv_en < 30 && radar_st_en < cfg_radar_thres) {
        if (++low_cnt >= 3) radar_state = 0;
    } else { low_cnt = 0; }

    radar_dist  = (radar_state & 1) ? (d[9] | (d[10]<<8)) : ((radar_state & 2) ? (d[12] | (d[13]<<8)) : 0);
    // 光照过滤
    { static int ll=0; int r=d[tail-4]; if(r>0)ll=r; radar_light=ll; }

    // DEBUG: 打印原始帧字节帮助定位能量偏移
    {
        static int dc=0;
        if(++dc%30==0) {
            char hx[200]=""; char t[8];
            for(int q=6;q<18&&q<tail;q++){sprintf(t,"%02X ",d[q]);strcat(hx,t);}
            ESP_LOGI(TAG,"RAW[6..]:%s st=%d mv=%d se=%d",hx,radar_state,radar_mv_en,radar_st_en);
        }
    }
}

static void radar_task(void *arg)
{
    uint8_t buf[1024]; int blen = 0;
    uart_config_t cu = {.baud_rate=256000,.data_bits=UART_DATA_8_BITS,
        .parity=UART_PARITY_DISABLE,.stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
    uart_param_config(UART_NUM_2, &cu);
    uart_set_pin(UART_NUM_2, cfg_radar_tx, cfg_radar_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, 1024, 0, 0, NULL, 0);
    ESP_LOGI(TAG, "RADAR: RX=%d TX=%d", cfg_radar_rx, cfg_radar_tx);

    vTaskDelay(pdMS_TO_TICKS(500));
    uint8_t c1[]={0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04,0x03,0x02,0x01};
    uint8_t c2[]={0xFD,0xFC,0xFB,0xFA,0x02,0x00,0x62,0x00,0x04,0x03,0x02,0x01};
    uint8_t c3[]={0xFD,0xFC,0xFB,0xFA,0x02,0x00,0xFE,0x00,0x04,0x03,0x02,0x01};
    uart_write_bytes(UART_NUM_2, c1, sizeof(c1)); vTaskDelay(pdMS_TO_TICKS(200));
    uart_write_bytes(UART_NUM_2, c2, sizeof(c2)); vTaskDelay(pdMS_TO_TICKS(200));
    uart_write_bytes(UART_NUM_2, c3, sizeof(c3)); vTaskDelay(pdMS_TO_TICKS(200));

    int pub_tick = 0;
    while (1) {
        int n = uart_read_bytes(UART_NUM_2, buf+blen, sizeof(buf)-blen-1, pdMS_TO_TICKS(50));
        if (n > 0) {
            blen += n;
            for (int i = 0; i < blen-20; i++) {
                if (buf[i]!=0xF4||buf[i+1]!=0xF3||buf[i+2]!=0xF2||buf[i+3]!=0xF1) continue;
                for (int j = i+10; j < blen-3; j++) {
                    if (buf[j]==0xF8&&buf[j+1]==0xF7&&buf[j+2]==0xF6&&buf[j+3]==0xF5) {
                        radar_parse(buf+i, j-i+4);
                        int used = j+4; memmove(buf, buf+used, blen-used); blen -= used;
                        i = -1; break;
                    }
                }
                if (i >= 0) break;
            }
            if (blen > 900) { memmove(buf, buf+500, blen-500); blen -= 500; }
        }
        if (mqtt && ++pub_tick > 50) {
            pub_tick = 0;
            char payload[180];
            snprintf(payload, sizeof(payload),
                "{\"name\":\"%s\",\"st\":%d,\"mv\":%d,\"se\":%d,\"dst\":%d,\"lit\":%d,\"heap\":%lu}",
                cfg_name, radar_state, radar_mv_en, radar_st_en, radar_dist, radar_light,
                esp_get_free_heap_size()/1024);
            esp_mqtt_client_publish(mqtt, mqtt_topic, payload, 0, 0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ---------- WiFi ---------- */
static void wifi_handler(void *a, esp_event_base_t b, int32_t id, void *d)
{
    if (b == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (b == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) esp_wifi_connect();
    else if (b == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        wifi_got_ip = 1;
        ip_event_got_ip_t *e = d; ESP_LOGI(TAG, "IP:" IPSTR, IP2STR(&e->ip_info.ip));
        if (!mqtt) {
            uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
            if (!cfg_name[0]) snprintf(cfg_name, sizeof(cfg_name), "ESP32-%02X%02X%02X", mac[3], mac[4], mac[5]);
            snprintf(mqtt_topic, sizeof(mqtt_topic), "esp32/%02X%02X%02X%02X%02X%02X/data",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            char uri[80]; snprintf(uri, sizeof(uri), "mqtt://%s", cfg_mqtt);
            esp_mqtt_client_config_t mc = {.broker.address.uri = uri};
            mqtt = esp_mqtt_client_init(&mc); esp_mqtt_client_start(mqtt);
            ESP_LOGI(TAG, "MQTT: %s -> %s", mqtt_topic, cfg_mqtt);
        }
    }
}

static void wifi_fallback_check(void *arg)
{ vTaskDelay(pdMS_TO_TICKS(35000)); if (!wifi_got_ip) { nvs_flash_erase(); esp_restart(); } vTaskDelete(NULL); }

static void wifi_init(void)
{
    cfg_load_all();
    esp_netif_init(); esp_event_loop_create_default();
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_handler, NULL);
    wifi_init_config_t c = WIFI_INIT_CONFIG_DEFAULT(); esp_wifi_init(&c);

    if (cfg_ssid[0]) {
        wifi_configured = 1;
        esp_netif_create_default_wifi_sta();
        esp_wifi_set_mode(WIFI_MODE_STA);
        wifi_config_t w = {.sta={{0}}};
        memcpy(w.sta.ssid, cfg_ssid, 32); memcpy(w.sta.password, cfg_pass, 64);
        esp_wifi_set_config(WIFI_IF_STA, &w);
        esp_wifi_start();
        xTaskCreate(wifi_fallback_check, "wdt", 2048, NULL, 2, NULL);
    } else {
        // AP 配网模式
        esp_netif_create_default_wifi_sta();
        esp_netif_create_default_wifi_ap();
        esp_wifi_set_mode(WIFI_MODE_APSTA); esp_wifi_start(); vTaskDelay(pdMS_TO_TICKS(500));
        uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
        char apn[32]; snprintf(apn, sizeof(apn), "ESP32-%02X%02X%02X", mac[3], mac[4], mac[5]);
        wifi_config_t ap = {.ap={.password="12345678",.max_connection=4,.authmode=WIFI_AUTH_WPA_WPA2_PSK}};
        memcpy(ap.ap.ssid, apn, strlen(apn)); ap.ap.ssid_len = strlen(apn);
        esp_wifi_set_config(WIFI_IF_AP, &ap);
        ESP_LOGI(TAG, "AP: %s / 12345678", apn);
    }
}

/* ===== Web 服务器 ===== */
static char *urldec(char *s)
{ char *q=s; while(*q){if(*q=='+')*q=' ';if(*q=='%'&&q[1]&&q[2]){int v;sscanf(q+1,"%2x",&v);*q=(char)v;memmove(q+1,q+3,strlen(q+3)+1);}q++;} return s; }

// 配网 /save
static esp_err_t save_handler(httpd_req_t *req)
{
    char qs[512];
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) == ESP_OK) {
        char v[65];
        if (httpd_query_key_value(qs, "ssid", v, sizeof(v)) == ESP_OK) { urldec(v); cfg_save_str("ssid", v); }
        if (httpd_query_key_value(qs, "pass", v, sizeof(v)) == ESP_OK) { urldec(v); cfg_save_str("pass", v); }
        if (httpd_query_key_value(qs, "mqtt", v, sizeof(v)) == ESP_OK) { urldec(v); cfg_save_str("mqtt", v); }
        if (httpd_query_key_value(qs, "name", v, sizeof(v)) == ESP_OK) { urldec(v); cfg_save_str("name", v); }
        if (httpd_query_key_value(qs, "radar", v, sizeof(v)) == ESP_OK) { cfg_save_int("radar_en", atoi(v)); }
        httpd_resp_sendstr(req, "OK saved. Restarting...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
        return ESP_OK;
    }
    httpd_resp_sendstr(req, "ERR");
    return ESP_OK;
}

// 配置页面 (预填当前值)
static esp_err_t setup_page(httpd_req_t *req)
{
    char h[2500];
    snprintf(h, sizeof(h),
"<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>ESP32 配置</title><style>"
"body{font-family:Arial;background:#111;color:#eee;padding:20px;text-align:center}"
"h2{color:#58a6ff}.card{background:#1a1a2e;border-radius:10px;padding:16px;max-width:400px;margin:10px auto;text-align:left}"
"label{display:block;color:#8b949e;font-size:12px;margin-top:8px}"
"input,select{display:block;width:100%%;padding:8px;margin:4px 0;border-radius:4px;border:1px solid #30363d;background:#0d1117;color:#fff;font-size:14px;box-sizing:border-box}"
"button{background:#238636;color:#fff;border:none;padding:12px;width:100%%;border-radius:6px;font-size:16px;cursor:pointer;margin-top:12px}"
"#msg{font-size:13px;margin-top:8px;text-align:center}"
"</style></head><body><h2>ESP32 配置</h2>"
"<div class='card'>"
"<label>WiFi 名称</label><input id='ssid' value='%s'>"
"<label>WiFi 密码</label><input id='pass' type='password' value='%s'>"
"<label>MQTT Broker</label><input id='mqtt' value='%s'>"
"<label>设备名称</label><input id='name' value='%s' placeholder='如: 客厅雷达'>"
"<label>LD2410C 雷达</label><select id='radar'><option value='1' %s>启用</option><option value='0' %s>禁用</option></select>"
"<button onclick='go()'>保存</button><div id='msg'></div>"
"</div><script>"
"function go(){var p='ssid='+encodeURIComponent(document.getElementById('ssid').value)+'&pass='+encodeURIComponent(document.getElementById('pass').value)+'&mqtt='+encodeURIComponent(document.getElementById('mqtt').value)+'&name='+encodeURIComponent(document.getElementById('name').value)+'&radar='+document.getElementById('radar').value;document.getElementById('msg').textContent='saving...';fetch('/save?'+p).then(r=>r.text()).then(t=>{document.getElementById('msg').textContent=t})}"
"</script></body></html>",
    cfg_ssid, cfg_pass, cfg_mqtt, cfg_name,
    cfg_radar_en ? "selected" : "",
    cfg_radar_en ? "" : "selected");
    httpd_resp_sendstr(req, h);
    return ESP_OK;
}

// 状态 API
static esp_err_t api_data(httpd_req_t *req)
{
    char j[300];
    snprintf(j, sizeof(j),
        "{\"ok\":%d,\"st\":%d,\"mv\":%d,\"se\":%d,\"dst\":%d,\"lit\":%d,\"heap\":%lu,\"name\":\"%s\",\"ver\":\"%s %s\"}",
        radar_ok, radar_state, radar_mv_en, radar_st_en, radar_dist, radar_light,
        esp_get_free_heap_size()/1024, cfg_name, __DATE__, __TIME__);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, j, strlen(j));
    return ESP_OK;
}

// 主页仪表盘
static esp_err_t root_page(httpd_req_t *req)
{
    const char *h =
"<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>" "ESP32" "</title><style>"
"*{box-sizing:border-box}body{font-family:Arial;margin:16px;background:#111;color:#eee;text-align:center}"
"h2{color:#e94560;margin:8px 0}.card{background:#1a1a2e;border-radius:10px;padding:14px;margin:10px auto;max-width:400px}"
".big{font-size:28px;padding:8px}.row{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid #222}"
".l{color:#888}.v{font-weight:bold;color:#0af}.bar{height:6px;border-radius:3px;margin:3px 0;transition:width .3s}"
".t{display:inline-block;padding:2px 8px;border-radius:4px;font-size:11px;margin:0 3px}"
".on{background:#1a4;color:#fff}.off{background:#555;color:#999}"
"</style></head><body><h2>" "ESP32" "</h2>"
"<div class='card'><div class='big' id='pr'>--</div>"
"<div><span class='t' id='tm'>运动</span><span class='t' id='ts'>微动</span></div>"
"<div class='l'>运动 <span id='mv'>0</span></div><div class='bar'><div id='bm' style='width:0%;background:#e94560;height:6px;border-radius:3px'></div></div>"
"<div class='l'>微动 <span id='se'>0</span></div><div class='bar'><div id='bs' style='width:0%;background:#0af;height:6px;border-radius:3px'></div></div>"
"<div class='row'><span class='l'>距离</span><span class='v' id='dst'>--</span></div>"
"<div class='row'><span class='l'>光照</span><span class='v' id='lit'>--</span></div></div>"
"<div style='color:#555;font-size:11px;margin:12px'>ver <span id='ver'>--</span> | 内存 <span id='hp'>--</span>KB | "
"<a href='/ota' style='color:#58a6ff'>OTA</a> | "
"<a href='/cfg' style='color:#58a6ff'>配置</a> | "
"<a href='http://66.135.21.130:8088/dashboard' style='color:#58a6ff'>面板</a></div>"
"<script>function u(){fetch('/api').then(r=>r.json()).then(d=>{"
"document.getElementById('pr').textContent=d.st>0?'🟢 有人':'🔴 无人';"
"document.getElementById('mv').textContent=d.mv;document.getElementById('se').textContent=d.se;"
"document.getElementById('bm').style.width=Math.min(d.mv,100)+'%';"
"document.getElementById('bs').style.width=Math.min(d.se,100)+'%';"
"document.getElementById('dst').textContent=d.dst+' cm';"
"document.getElementById('lit').textContent=d.lit;"
"document.getElementById('ver').textContent=d.ver||'--';"
"document.getElementById('hp').textContent=d.heap+'KB';"
"var tm=document.getElementById('tm'),ts=document.getElementById('ts');"
"tm.className='t '+(d.mv>10?'on':'off');ts.className='t '+(d.se>10?'on':'off')"
"}).catch(function(){})}setInterval(u,500);u()</script></body></html>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, h, strlen(h));
    return ESP_OK;
}

// OTA 上传页
static esp_err_t ota_page(httpd_req_t *req)
{
    const char *h =
"<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>OTA</title><style>body{font-family:Arial;background:#111;color:#eee;text-align:center;padding:20px}"
"h2{color:#58a6ff}.card{background:#1a1a2e;border-radius:10px;padding:20px;max-width:400px;margin:20px auto}"
"input{display:block;margin:10px auto;color:#fff;font-size:14px}"
"button{background:#238636;color:#fff;border:none;padding:12px 30px;border-radius:6px;font-size:16px;cursor:pointer;margin:10px}"
".bar{height:6px;background:#21262d;border-radius:3px;overflow:hidden;max-width:300px;margin:10px auto}"
".bar div{height:100%;background:#238636;width:0;transition:width .3s}#msg{font-size:13px;margin-top:10px}"
"</style></head><body><h2>OTA 固件升级</h2><div class='card'>"
"<p style='color:#8b949e;font-size:13px'>选择 .bin 文件</p>"
"<input type='file' id='file' accept='.bin'><div class='bar'><div id='progress'></div></div>"
"<div id='msg'></div><button onclick='upload()'>开始升级</button>"
"</div><script>"
"function upload(){var f=document.getElementById('file').files[0];if(!f)return;"
"var x=new XMLHttpRequest();x.open('POST','/update',true);"
"x.upload.onprogress=function(e){if(e.lengthComputable){var p=Math.round(e.loaded/e.total*100);"
"document.getElementById('progress').style.width=p+'%';document.getElementById('msg').textContent='上传 '+p+'%'}};"
"x.onload=function(){if(x.status==200){document.getElementById('msg').textContent='OK! 重启中...'}"
"else{document.getElementById('msg').textContent='失败: '+x.status}};x.send(f)}"
"</script></body></html>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, h, strlen(h));
    return ESP_OK;
}

// OTA 接收
static esp_err_t ota_recv(httpd_req_t *req)
{
    int total = req->content_len;
    if (total <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no content"); return ESP_FAIL; }
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t oh; esp_ota_begin(part, total, &oh);
    uint8_t buf[1024]; int got = 0;
    while (got < total) {
        int r = httpd_req_recv(req, (char*)buf, sizeof(buf) > (total-got) ? (total-got) : sizeof(buf));
        if (r <= 0) { esp_ota_end(oh); return ESP_FAIL; }
        esp_ota_write(oh, buf, r); got += r;
    }
    esp_ota_end(oh); esp_ota_set_boot_partition(part);
    httpd_resp_sendstr(req, "OK");
    vTaskDelay(pdMS_TO_TICKS(2000)); esp_restart();
    return ESP_OK;
}

// MQTT 配置
static esp_err_t mqtt_page(httpd_req_t *req)
{
    char qs[256];
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) == ESP_OK) {
        char v[64];
        if (httpd_query_key_value(qs, "host", v, sizeof(v)) == ESP_OK && strlen(v) > 0) {
            cfg_save_str("mqtt", v); snprintf(cfg_mqtt, sizeof(cfg_mqtt), "%s", v);
            if (mqtt) { esp_mqtt_client_stop(mqtt); esp_mqtt_client_destroy(mqtt); mqtt = NULL; }
            vTaskDelay(pdMS_TO_TICKS(500));
            char uri[80]; snprintf(uri, sizeof(uri), "mqtt://%s", cfg_mqtt);
            esp_mqtt_client_config_t mc = {.broker.address.uri = uri};
            uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
            mqtt = esp_mqtt_client_init(&mc); esp_mqtt_client_start(mqtt);
            httpd_resp_sendstr(req, "OK reconnected");
            return ESP_OK;
        }
    }
    char p[1100]; snprintf(p, sizeof(p),
    "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>MQTT</title><style>body{font-family:Arial;background:#111;color:#eee;text-align:center;padding:20px}"
    "h2{color:#58a6ff}input{display:block;width:90%%;max-width:300px;margin:10px auto;padding:10px;"
    "border-radius:6px;border:1px solid #30363d;background:#0d1117;color:#fff;font-size:16px}"
    "button{background:#238636;color:#fff;border:none;padding:12px 30px;border-radius:6px;font-size:16px;cursor:pointer}"
    "#msg{margin-top:12px;font-size:14px;color:#3fb950}"
    "</style></head><body><h2>MQTT</h2><div style='color:#8b949e;font-size:13px'>当前: %s</div>"
    "<input id='host' placeholder='IP 或域名'><button onclick='go()'>保存</button><div id='msg'></div>"
    "<script>function go(){var h=document.getElementById('host').value;if(!h)return;"
    "fetch('/mqtt?host='+encodeURIComponent(h)).then(r=>r.text()).then(function(t){"
    "document.getElementById('msg').textContent=t})}</script></body></html>", cfg_mqtt);
    httpd_resp_sendstr(req, p);
    return ESP_OK;
}

/* ---------- 入口 ---------- */
void app_main(void)
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND)
        { nvs_flash_erase(); nvs_flash_init(); }

    wifi_init();
    vTaskDelay(pdMS_TO_TICKS(3000));

    if (wifi_configured) {
        if (cfg_radar_en) xTaskCreate(radar_task, "radar", 4096, NULL, 5, NULL);

        httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
        hc.max_uri_handlers = 7; hc.server_port = 80; hc.stack_size = 8192;
        hc.lru_purge_enable = true;      // 自动踢掉不活跃连接
        hc.max_open_sockets = 4;         // 限制并发连接数
        httpd_handle_t hs = NULL; httpd_start(&hs, &hc);
        httpd_register_uri_handler(hs, &(httpd_uri_t){.uri="/",.method=HTTP_GET,.handler=root_page});
        httpd_register_uri_handler(hs, &(httpd_uri_t){.uri="/api",.method=HTTP_GET,.handler=api_data});
        httpd_register_uri_handler(hs, &(httpd_uri_t){.uri="/ota",.method=HTTP_GET,.handler=ota_page});
        httpd_register_uri_handler(hs, &(httpd_uri_t){.uri="/update",.method=HTTP_POST,.handler=ota_recv});
        httpd_register_uri_handler(hs, &(httpd_uri_t){.uri="/mqtt",.method=HTTP_GET,.handler=mqtt_page});
        httpd_register_uri_handler(hs, &(httpd_uri_t){.uri="/cfg",.method=HTTP_GET,.handler=setup_page});
        httpd_register_uri_handler(hs, &(httpd_uri_t){.uri="/save",.method=HTTP_GET,.handler=save_handler});
    } else {
        // AP 配网模式
        httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
        hc.server_port = 80; hc.max_uri_handlers = 2;
        httpd_handle_t hs = NULL; httpd_start(&hs, &hc);
        httpd_register_uri_handler(hs, &(httpd_uri_t){.uri="/",.method=HTTP_GET,.handler=setup_page});
        httpd_register_uri_handler(hs, &(httpd_uri_t){.uri="/save",.method=HTTP_GET,.handler=save_handler});
        ESP_LOGI(TAG, "Setup AP: http://192.168.4.1");
    }

    while (1) vTaskDelay(1000);
}
