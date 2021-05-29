/*
  zh-TW.h - localization for Chinese (Traditional) - Taiwan for Tasmota

  Copyright (C) 2021  Dannydu

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _LANGUAGE_ZH_TW_H_
#define _LANGUAGE_ZH_TW_H_

/*************************** ATTENTION *******************************\
 *
 * Due to memory constraints only UTF-8 is supported.
 * To save code space keep text as short as possible.
 * Time and Date provided by SDK can not be localized (yet).
 * Use online command StateText to translate ON, OFF, HOLD and TOGGLE.
 * Use online command Prefix to translate cmnd, stat and tele.
 *
 * Updated until v9.3.1.1
\*********************************************************************/

//#define LANGUAGE_MODULE_NAME         // Enable to display "Module Generic" (ie Spanish), Disable to display "Generic Module" (ie English)

#define LANGUAGE_LCID 1028
// HTML (ISO 639-1) Language Code
#define D_HTML_LANGUAGE "zh"

// "2017-03-07T11:08:02" - ISO8601:2004
#define D_YEAR_MONTH_SEPARATOR "-"
#define D_MONTH_DAY_SEPARATOR "-"
#define D_DATE_TIME_SEPARATOR "T"
#define D_HOUR_MINUTE_SEPARATOR ":"
#define D_MINUTE_SECOND_SEPARATOR ":"

#define D_DAY3LIST "日  一  二  三  四  五  六  "
#define D_MONTH3LIST "1月 2月 3月 4月 5月 6月 7月 8月 9月 10月11月12月"

// Non JSON decimal separator
#define D_DECIMAL_SEPARATOR "."

// Common
#define D_ADMIN "Admin"
#define D_AIR_QUALITY "空氣品質"
#define D_AP "存取點"                    // Access Point
#define D_AS "名稱:"
#define D_AUTO "自動"
#define D_BATT "電池"                // Short for Battery
#define D_BLINK "閃爍"
#define D_BLINKOFF "閃爍關"
#define D_BOOT_COUNT "啟動計數"
#define D_BRIGHTLIGHT "亮度"
#define D_BSSID "BSSId"
#define D_BUTTON "按鈕"
#define D_BY "由"                    // Written by me
#define D_BYTES "大小:"
#define D_CELSIUS "攝氏"
#define D_CHANNEL "頻道"
#define D_CO2 "二氧化碳"
#define D_CODE "代碼"                // Button code
#define D_COLDLIGHT "冷光"
#define D_COMMAND "命令:"
#define D_CONNECTED "已連線"
#define D_CORS_DOMAIN "跨來源資源共享的網域(CORS Domain)"
#define D_COUNT "數量:"
#define D_COUNTER "Counter"
#define D_CT_POWER "CT Power"
#define D_CURRENT "電流"          // As in Voltage and Current
#define D_DATA "數據:"
#define D_DARKLIGHT "暗光"
#define D_DEBUG "偵錯"
#define D_DEWPOINT "Dew point"
#define D_DISABLED "已停用"
#define D_DISTANCE "距離"
#define D_DNS_SERVER "DNS伺服器"
#define D_DO "Disolved Oxygen"
#define D_DONE "完成"
#define D_DST_TIME "DST"
#define D_EC "EC"
#define D_ECO2 "eCO₂"
#define D_EMULATION "模擬"
#define D_ENABLED "已啟用"
#define D_ERASE "抹除"
#define D_ERROR "錯誤"
#define D_FAHRENHEIT "華氏"
#define D_FAILED "失敗了"
#define D_FALLBACK "Fallback"
#define D_FALLBACK_TOPIC "Fallback Topic"
#define D_FALSE "假"
#define D_FILE "檔案:"
#define D_FLOW_RATE "流量"
#define D_FRAGMENTATION "frag."      // Lower case abbreviated version of fragmentation used in "memory fragmentation"
#define D_FREE_MEMORY "可用的記憶體"
#define D_PSR_MAX_MEMORY "偽靜態隨機存取記憶體(PS-RAM)容量"
#define D_PSR_FREE_MEMORY "偽靜態隨機存取記憶體(PS-RAM)可用空間"
#define D_FREQUENCY "頻率"
#define D_GAS "氣體"
#define D_GATEWAY "閘道器"
#define D_GROUP "群組:"
#define D_HOST "主機"
#define D_HALL_EFFECT "Hall Effect"
#define D_HOSTNAME "主機名稱"
#define D_HUMIDITY "濕度"
#define D_ILLUMINANCE "照度"
#define D_IMMEDIATE "立即"      // Button immediate
#define D_INDEX "索引:"
#define D_INFO "資訊"
#define D_INFRARED "紅外線"
#define D_INITIALIZED "已初始化"
#define D_IP_ADDRESS "IP位址"
#define D_LIGHT "燈光"
#define D_LWT "LWT"
#define D_LQI "LQI"                  // Zigbee Link Quality Index
#define D_MODULE "模組"
#define D_MOISTURE "濕度"
#define D_MQTT "MQTT"
#define D_MULTI_PRESS "多重點擊"
#define D_NOISE "雜訊"
#define D_NONE "無"
#define D_O2 "Oxygen"
#define D_OFF "關閉"
#define D_OFFLINE "離線"
#define D_OK "好"
#define D_ON "開啟"
#define D_ONLINE "線上"
#define D_ORP "ORP"
#define D_PASSWORD "密碼"
#define D_PH "pH"
#define D_PORT "通訊埠"
#define D_POWER_FACTOR "功率因數"
#define D_POWERUSAGE "用電量"
#define D_POWERUSAGE_ACTIVE "有功功率"
#define D_POWERUSAGE_APPARENT "視在功率"
#define D_POWERUSAGE_REACTIVE "虛功率"
#define D_PRESSURE "氣壓"
#define D_PRESSUREATSEALEVEL "海平面氣壓"
#define D_PROGRAM_FLASH_SIZE "程式記憶體大小"
#define D_PROGRAM_SIZE "程式大小"
#define D_PROJECT "項目:"
#define D_RAIN "雨"
#define D_RANGE "範圍"
#define D_RECEIVED "已接收"
#define D_RESTART "重新啟動"
#define D_RESTARTING "正在重新啟動"
#define D_RESTART_REASON "重新啟動的原因"
#define D_RESTORE "讀取設定"
#define D_RETAINED "已保留"
#define D_RULE "規則"
#define D_SAVE "儲存"
#define D_SENSOR "感應器"
#define D_SSID "SSID"
#define D_START "啟動"
#define D_STD_TIME "標準時間"
#define D_STOP "停止"
#define D_SUBNET_MASK "子網域遮罩"
#define D_SUBSCRIBE_TO "訂閱"
#define D_UNSUBSCRIBE_FROM "退出訂閱自"
#define D_SUCCESSFUL "已成功"
#define D_SUNRISE "日出"
#define D_SUNSET "日落"
#define D_TEMPERATURE "溫度"
#define D_TO "到"
#define D_TOGGLE "切換"
#define D_TOPIC "主題"
#define D_TOTAL_USAGE "總使用量"
#define D_TRANSMIT "發送"
#define D_TRUE "真"
#define D_TVOC "TVOC"
#define D_UPGRADE "升級"
#define D_UPLOAD "上傳"
#define D_UPTIME "啟動時間"
#define D_USER "使用者名稱"
#define D_UTC_TIME "世界協調時間(UTC)"
#define D_UV_INDEX "紫外線指數"
#define D_UV_INDEX_1 "低"
#define D_UV_INDEX_2 "中"
#define D_UV_INDEX_3 "高"
#define D_UV_INDEX_4 "危險"
#define D_UV_INDEX_5 "BurnL1/2"
#define D_UV_INDEX_6 "BurnL3"
#define D_UV_INDEX_7 "超出範圍"
#define D_UV_LEVEL "紫外線等級"
#define D_UV_POWER "紫外線能量"
#define D_VERSION "版本"
#define D_VOLTAGE "電壓"
#define D_VOLUME "Volume"
#define D_WEIGHT "重量"
#define D_WARMLIGHT "暖光"
#define D_WEB_SERVER "網頁伺服器"

// tasmota.ino
#define D_WARNING_MINIMAL_VERSION "警告，這個版本並不支援將設定永久的儲存!"
#define D_LEVEL_10 "等級 1-0"
#define D_LEVEL_01 "等級 0-1"
#define D_SERIAL_LOGGING_DISABLED "已停用序列埠日誌"
#define D_SYSLOG_LOGGING_REENABLED "系統日誌已經重新啟用"

#define D_SET_BAUDRATE_TO "將鮑率設定為:"
#define D_RECEIVED_TOPIC "接收到的主題:"
#define D_DATA_SIZE "資料大小:"
#define D_ANALOG_INPUT "類比"

// support.ino
#define D_OSWATCH "osWatch"
#define D_BLOCKED_LOOP "Blocked Loop"
#define D_WPS_FAILED_WITH_STATUS "WPSconfig FAILED with status"
#define D_ACTIVE_FOR_3_MINUTES "啟動三分鐘"
#define D_FAILED_TO_START "無法啟動"
#define D_PATCH_ISSUE_2186 "Patch issue 2186"
#define D_CONNECTING_TO_AP "正在連線至存取點"
#define D_IN_MODE "in模式:"
#define D_CONNECT_FAILED_NO_IP_ADDRESS "連線失敗:未收到IP位置"
#define D_CONNECT_FAILED_AP_NOT_REACHED "連線失敗:無法連線至存取點"
#define D_CONNECT_FAILED_WRONG_PASSWORD "連線失敗"
#define D_CONNECT_FAILED_AP_TIMEOUT "連線失敗:存取點超時"
#define D_ATTEMPTING_CONNECTION "正在試圖連線中..."
#define D_CHECKING_CONNECTION "正在檢查連線中..."
#define D_QUERY_DONE "已完成查詢，找到MQTT服務了"
#define D_MQTT_SERVICE_FOUND "MQTT服務位於:"
#define D_FOUND_AT "位於"
#define D_SYSLOG_HOST_NOT_FOUND "找不到系統日誌主機"

// settings.ino
#define D_SAVED_TO_FLASH_AT "儲存至快閃記憶體，位於:"
#define D_LOADED_FROM_FLASH_AT "已從快閃記憶體中讀取，從:"
#define D_USE_DEFAULTS "使用預設值"
#define D_ERASED_SECTOR "抹除磁區"

// xdrv_02_webserver.ino
#define D_NOSCRIPT "為了要使用 Tasmota，請啟用 JavaScript"
#define D_MINIMAL_FIRMWARE_PLEASE_UPGRADE "MINIMAL韌體<br>請升級"
#define D_WEBSERVER_ACTIVE_ON "網頁伺服器已經啟動，位於:"
#define D_WITH_IP_ADDRESS "IP位址:"
#define D_WEBSERVER_STOPPED "網頁伺服器已經停止"
#define D_FILE_NOT_FOUND "照不到檔案"
#define D_REDIRECTED "已重新定向至閘道認證頁面"
#define D_WIFIMANAGER_SET_ACCESSPOINT_AND_STATION "Wifimanager set AccessPoint and keep Station"
#define D_WIFIMANAGER_SET_ACCESSPOINT "Wifimanager 已設定存取點"
#define D_TRYING_TO_CONNECT "正在將裝置連線至網路"

#define D_RESTART_IN "重新啟動倒數 "
#define D_SECONDS " 秒"
#define D_DEVICE_WILL_RESTART "裝置將會在幾秒內重新啟動"
#define D_BUTTON_TOGGLE "切換"
#define D_CONFIGURATION "設定"
#define D_INFORMATION "資訊"
#define D_FIRMWARE_UPGRADE "韌體升級"
#define D_MANAGEMENT "Consoles"
#define D_CONSOLE "控制台"
#define D_CONFIRM_RESTART "確認重新啟動"

#define D_CONFIGURE_MODULE "模組設定"
#define D_CONFIGURE_WIFI "WiFi設定"
#define D_CONFIGURE_MQTT "MQTT設定"
#define D_CONFIGURE_DOMOTICZ "Domoticz設定"
#define D_CONFIGURE_LOGGING "日誌設定"
#define D_CONFIGURE_OTHER "其他設定"
#define D_CONFIRM_RESET_CONFIGURATION "確定要重設設定"
#define D_RESET_CONFIGURATION "重設設定"
#define D_BACKUP_CONFIGURATION "備份設定"
#define D_RESTORE_CONFIGURATION "回復設定"
#define D_MAIN_MENU "主選單"

#define D_MODULE_PARAMETERS "模組參數"
#define D_MODULE_TYPE "模組類型"
#define D_PULLUP_ENABLE "無按鈕/上拉式開關(Switch pull-up)"
#define D_ADC "類比轉換數位(ADC)"
#define D_GPIO "GPIO"
#define D_SERIAL_IN "序列埠輸入(RX)"
#define D_SERIAL_OUT "序列埠輸出(TX)"

#define D_WIFI_PARAMETERS "Wifi設定"
#define D_SCAN_FOR_WIFI_NETWORKS "掃描無線網絡"
#define D_SCAN_DONE "掃描完畢"
#define D_NO_NETWORKS_FOUND "找不到任何網絡"
#define D_REFRESH_TO_SCAN_AGAIN "重新整理以重新掃描"
#define D_DUPLICATE_ACCESSPOINT "重覆的存取點"
#define D_SKIPPING_LOW_QUALITY "正在忽略訊號品質不佳的訊號"
#define D_RSSI "RSSI"
#define D_WEP "WEP"
#define D_WPA_PSK "WPA PSK"
#define D_WPA2_PSK "WPA2 PSK"
#define D_AP1_SSID "存取點 SSID"
#define D_AP1_SSID_HELP "Type or Select your WiFi Network"
#define D_AP2_SSID "存取點2 SSID"
#define D_AP2_SSID_HELP "Type your Alternative WiFi Network"
#define D_AP_PASSWORD "存取點 密碼"
#define D_AP_PASSWORD_HELP "Enter your WiFi Password"
#define D_SELECT_YOUR_WIFI_NETWORK "Select your WiFi Network"
#define D_SHOW_MORE_WIFI_NETWORKS "Scan for all WiFi Networks"
#define D_SHOW_MORE_OPTIONS "More Options"
#define D_CHECK_CREDENTIALS "Please, check your credentials"
#define D_SUCCESSFUL_WIFI_CONNECTION "Successful WiFi Connection"
#define D_NOW_YOU_CAN_CLOSE_THIS_WINDOW "Now you can close this window"
#define D_REDIRECTING_TO_NEW_IP "Redirecting to new device's IP address"

#define D_MQTT_PARAMETERS "MQTT設定"
#define D_CLIENT "客戶端"
#define D_FULL_TOPIC "完整主題"

#define D_LOGGING_PARAMETERS "日誌參數"
#define D_SERIAL_LOG_LEVEL "序列埠日誌等級"
#define D_MQTT_LOG_LEVEL "Mqtt 日誌等級"
#define D_WEB_LOG_LEVEL "Web 日誌等級"
#define D_SYS_LOG_LEVEL "Syslog 日誌等級"
#define D_MORE_DEBUG "More debug"
#define D_SYSLOG_HOST "Syslog 主機位址"
#define D_SYSLOG_PORT "Syslog 通訊埠"
#define D_TELEMETRY_PERIOD "Telemetry period"

#define D_OTHER_PARAMETERS "其他參數"
#define D_TEMPLATE "模板"
#define D_ACTIVATE "啟動"
#define D_DEVICE_NAME "裝置名稱"
#define D_WEB_ADMIN_PASSWORD "網頁上的管理員密碼"
#define D_MQTT_ENABLE "MQTT的啟用"
#define D_MQTT_TLS_ENABLE "MQTT TLS"
#define D_FRIENDLY_NAME "暱稱"
#define D_BELKIN_WEMO "貝爾金 WeMo"
#define D_HUE_BRIDGE "Hue 橋接器"
#define D_SINGLE_DEVICE "單個裝置"
#define D_MULTI_DEVICE "多重裝置"

#define D_CONFIGURE_TEMPLATE "模板設定"
#define D_TEMPLATE_PARAMETERS "模板參數"
#define D_TEMPLATE_NAME "名稱"
#define D_BASE_TYPE "基於"
#define D_TEMPLATE_FLAGS "選項"

#define D_SAVE_CONFIGURATION "儲存設定"
#define D_CONFIGURATION_SAVED "設定已儲存"
#define D_CONFIGURATION_RESET "設定已重設"

#define D_PROGRAM_VERSION "軟體版本"
#define D_BUILD_DATE_AND_TIME "編譯日期與時間"
#define D_CORE_AND_SDK_VERSION "核心與SDK版本"
#define D_FLASH_WRITE_COUNT "快閃記憶體寫入計數"
#define D_MAC_ADDRESS "MAC位址"
#define D_MQTT_HOST "MQTT主機"
#define D_MQTT_PORT "MQTT通訊埠"
#define D_MQTT_CLIENT "MQTT客戶端"
#define D_MQTT_USER "MQTT使用者名稱"
#define D_MQTT_TOPIC "MQTT 主題(Topic)"
#define D_MQTT_GROUP_TOPIC "MQTT 群組主題"
#define D_MQTT_FULL_TOPIC "MQTT 完整主題"
#define D_MQTT_NO_RETAIN "MQTT No Retain"
#define D_MDNS_DISCOVERY "mDNS 探索"
#define D_MDNS_ADVERTISE "mDNS 廣播"
#define D_ESP_CHIP_ID "ESP晶片ID"
#define D_FLASH_CHIP_ID "快閃記憶體ID"
#define D_FLASH_CHIP_SIZE "快閃記憶體大小"
#define D_FREE_PROGRAM_SPACE "可用的程式空間"

#define D_UPGRADE_BY_WEBSERVER "透過網頁升級"
#define D_OTA_URL "OTA網址"
#define D_START_UPGRADE "開始升級"
#define D_UPGRADE_BY_FILE_UPLOAD "透過檔案升級"
#define D_UPLOAD_STARTED "已開始上傳"
#define D_UPGRADE_STARTED "已開始升級"
#define D_UPLOAD_DONE "已上傳完成"
#define D_UPLOAD_TRANSFER "Upload transfer"
#define D_TRANSFER_STARTED "Transfer started"
#define D_UPLOAD_ERR_1 "沒選擇任何檔案"
#define D_UPLOAD_ERR_2 "可用空間不足"
#define D_UPLOAD_ERR_3 "Invalid file signature"
#define D_UPLOAD_ERR_4 "軟體刷入(Program flash)的大小超出實際記憶體的大小"
#define D_UPLOAD_ERR_5 "上傳緩衝器不相符"
#define D_UPLOAD_ERR_6 "上傳失敗。 啟用日誌記錄 3"
#define D_UPLOAD_ERR_7 "上傳取消"
#define D_UPLOAD_ERR_8 "檔案無效"
#define D_UPLOAD_ERR_9 "檔案太大了"
#define D_UPLOAD_ERR_10 "無法初始化 RF 晶片"
#define D_UPLOAD_ERR_11 "無法抹除 RF 晶片"
#define D_UPLOAD_ERR_12 "無法寫入 RF 晶片"
#define D_UPLOAD_ERR_13 "無法解碼 RF 韌體"
#define D_UPLOAD_ERR_14 "不相容"
#define D_UPLOAD_ERROR_CODE "上傳錯誤代碼"

#define D_ENTER_COMMAND "輸入命令"
#define D_ENABLE_WEBLOG_FOR_RESPONSE "如果回應如預期，啟用Weblog 2"
#define D_NEED_USER_AND_PASSWORD "需要 user=<使用者名稱>&password=<密碼>"

// xdrv_01_mqtt.ino
#define D_FINGERPRINT "驗證 TLS 指紋碼..."
#define D_TLS_CONNECT_FAILED_TO "TLS 連線失敗，因為"
#define D_RETRY_IN "重新嘗試倒數"
#define D_VERIFIED "驗證成功"
#define D_INSECURE "由於指紋碼無效，因此連線並未加密"
#define D_CONNECT_FAILED_TO "連線失敗:"

// xplg_wemohue.ino
#define D_MULTICAST_DISABLED "群播已停用"
#define D_MULTICAST_REJOINED "已(重新)加入群播"
#define D_MULTICAST_JOIN_FAILED "加入群播失敗"
#define D_FAILED_TO_SEND_RESPONSE "發送請求失敗"

#define D_WEMO "WeMo"
#define D_WEMO_BASIC_EVENT "WeMo 基本事件"
#define D_WEMO_EVENT_SERVICE "WeMo 事件服務"
#define D_WEMO_META_SERVICE "WeMo meta 服務"
#define D_WEMO_SETUP "WeMo 設定"
#define D_RESPONSE_SENT "請求已發送"

#define D_HUE "Hue"
#define D_HUE_BRIDGE_SETUP "Hue 設定"
#define D_HUE_API_NOT_IMPLEMENTED "Hue API 尚未實現"
#define D_HUE_API "Hue API"
#define D_HUE_POST_ARGS "Hue POST 參數"
#define D_3_RESPONSE_PACKETS_SENT "3 請求封包已發送"

// xdrv_07_domoticz.ino
#define D_DOMOTICZ_PARAMETERS "Domoticz 設定"
#define D_DOMOTICZ_IDX "Idx"
#define D_DOMOTICZ_KEY_IDX "Key idx"
#define D_DOMOTICZ_SWITCH_IDX "開關 idx"
#define D_DOMOTICZ_SENSOR_IDX "感應器 idx"
  #define D_DOMOTICZ_TEMP "溫度"
  #define D_DOMOTICZ_TEMP_HUM "溫度、濕度"
  #define D_DOMOTICZ_TEMP_HUM_BARO "溫度、濕度、氣壓"
  #define D_DOMOTICZ_POWER_ENERGY "功率、電量"
  #define D_DOMOTICZ_ILLUMINANCE "照度"
  #define D_DOMOTICZ_COUNT "計數/PM1"
  #define D_DOMOTICZ_VOLTAGE "電壓/PM2.5"
  #define D_DOMOTICZ_CURRENT "電流/PM10"
  #define D_DOMOTICZ_AIRQUALITY "空氣品質"
  #define D_DOMOTICZ_P1_SMART_METER "P1SmartMeter"
#define D_DOMOTICZ_UPDATE_TIMER "更新計時器"

// xdrv_09_timers.ino
#define D_CONFIGURE_TIMER "設定計時器"
#define D_TIMER_PARAMETERS "計時器參數"
#define D_TIMER_ENABLE "啟用"
#define D_TIMER_ARM "Arm"
#define D_TIMER_TIME "時間"
#define D_TIMER_DAYS "日數"
#define D_TIMER_REPEAT "重複"
#define D_TIMER_OUTPUT "輸出"
#define D_TIMER_ACTION "採取行動"

// xdrv_10_knx.ino
#define D_CONFIGURE_KNX "設定 KNX"
#define D_KNX_PARAMETERS "KNX 參數"
#define D_KNX_GENERAL_CONFIG "一般"
#define D_KNX_PHYSICAL_ADDRESS "實體位址"
#define D_KNX_PHYSICAL_ADDRESS_NOTE "( 在KNX網路中必須是獨一無二的值 )"
#define D_KNX_ENABLE "啟用 KNX"
#define D_KNX_GROUP_ADDRESS_TO_WRITE "要傳向群組位址的資料"
#define D_ADD "加入"
#define D_DELETE "移除"
#define D_REPLY "回覆"
#define D_KNX_GROUP_ADDRESS_TO_READ "要接收資料的群組位址"
#define D_RECEIVED_FROM "接收自"
#define D_KNX_COMMAND_WRITE "寫入"
#define D_KNX_COMMAND_READ "讀取"
#define D_KNX_COMMAND_OTHER "其他"
#define D_SENT_TO "傳送至"
#define D_KNX_WARNING "群組位址 ( 0 / 0 / 0 ) 已經被保留了，無法使用"
#define D_KNX_ENHANCEMENT "增強通訊"
#define D_KNX_TX_SLOT "KNX TX"
#define D_KNX_RX_SLOT "KNX RX"
#define D_KNX_TX_SCENE "KNX SCENE TX"
#define D_KNX_RX_SCENE "KNX SCENE RX"

// xdrv_23_zigbee
#define D_ZIGBEE_PERMITJOIN_ACTIVE "Devices allowed to join"
#define D_ZIGBEE_MAPPING_TITLE "Tasmota Zigbee Mapping"
#define D_ZIGBEE_NOT_STARTED "Zigbee not started"
#define D_ZIGBEE_MAPPING_IN_PROGRESS_SEC "Mapping in progress (%d s. remaining)"
#define D_ZIGBEE_MAPPING_NOT_PRESENT "No mapping"
#define D_ZIGBEE_MAP_REFRESH "Zigbee Map Refresh"
#define D_ZIGBEE_MAP   "Zigbee Map"
#define D_ZIGBEE_PERMITJOIN "Zigbee Permit Join"
#define D_ZIGBEE_GENERATE_KEY "generating random Zigbee network key"
#define D_ZIGBEE_UNKNOWN_DEVICE "Unknown device"
#define D_ZIGBEE_UNKNOWN_ATTRIBUTE "Unknown attribute"
#define D_ZIGBEE_INVALID_PARAM "Invalid parameter"
#define D_ZIGBEE_MISSING_PARAM "Missing parameters"
#define D_ZIGBEE_UNKNWON_ATTRIBUTE "Unknown attribute name (ignored): %s"
#define D_ZIGBEE_TOO_MANY_CLUSTERS "No more than one cluster id per command"
#define D_ZIGBEE_WRONG_DELIMITER "Wrong delimiter for payload"
#define D_ZIGBEE_UNRECOGNIZED_COMMAND "Unrecognized zigbee command: %s"
#define D_ZIGBEE_TOO_MANY_COMMANDS "Only 1 command allowed (%d)"
#define D_ZIGBEE_NO_ATTRIBUTE "No attribute in list"
#define D_ZIGBEE_UNSUPPORTED_ATTRIBUTE_TYPE "Unsupported attribute type"
#define D_ZIGBEE_JSON_REQUIRED "Config requires JSON objects"
#define D_ZIGBEE_RESET_1_OR_2 "1 or 2 to reset"
#define D_ZIGBEE_EEPROM_FOUND_AT_ADDRESS "ZBBridge EEPROM found at address"
#define D_ZIGBEE_RANDOMIZING_ZBCONFIG "Randomizing Zigbee parameters, please check with 'ZbConfig'"

// xdrv_03_energy.ino
#define D_ENERGY_TODAY "今日用電量"
#define D_ENERGY_YESTERDAY "昨日用電量"
#define D_ENERGY_TOTAL "總用電量"

// xdrv_27_shutter.ino
#define D_OPEN "開"
#define D_CLOSE "關"
#define D_DOMOTICZ_SHUTTER "百葉窗"

// xdrv_28_pcf8574.ino
#define D_CONFIGURE_PCF8574 "設定PCF8574"
#define D_PCF8574_PARAMETERS "PCF8574參數"
#define D_INVERT_PORTS "Invert Ports"
#define D_DEVICE "裝置"
#define D_DEVICE_INPUT "輸入"
#define D_DEVICE_OUTPUT "輸出"

// xsns_05_ds18b20.ino
#define D_SENSOR_BUSY "感應器忙碌中"
#define D_SENSOR_CRC_ERROR "感應器 CRC 校驗錯誤"
#define D_SENSORS_FOUND "找到感應器了"

// xsns_06_dht.ino
#define D_TIMEOUT_WAITING_FOR "等待超時"
#define D_START_SIGNAL_LOW "開始低電平(signal low)"
#define D_START_SIGNAL_HIGH "開始高電平(signal high)"
#define D_PULSE "脈衝"
#define D_CHECKSUM_FAILURE "校驗失敗"

// xsns_07_sht1x.ino
#define D_SENSOR_DID_NOT_ACK_COMMAND "感應器並未確認命令"
#define D_SHT1X_FOUND "發現 SHT1X 感應器了"

// xsns_18_pms5003.ino
#define D_STANDARD_CONCENTRATION "CF-1 PM"     // Standard Particle CF-1 Particle Matter
#define D_ENVIRONMENTAL_CONCENTRATION "PM"     // Environmetal Particle Matter
#define D_PARTICALS_BEYOND "粒子"

// xsns_27_apds9960.ino
#define D_GESTURE "手勢"
#define D_COLOR_RED "紅"
#define D_COLOR_GREEN "綠"
#define D_COLOR_BLUE "藍"
#define D_CCT "CCT"
#define D_PROXIMITY "距離"

// xsns_32_mpu6050.ino
#define D_AX_AXIS "加速度. X-軸"
#define D_AY_AXIS "加速度. Y-軸"
#define D_AZ_AXIS "加速度. Z-軸"
#define D_GX_AXIS "陀螺儀 X-軸"
#define D_GY_AXIS "陀螺儀 Y-軸"
#define D_GZ_AXIS "陀螺儀 Z-軸"

// xsns_34_hx711.ino
#define D_HX_CAL_REMOVE "移除重量"
#define D_HX_CAL_REFERENCE "載入參考重量"
#define D_HX_CAL_DONE "已同步"
#define D_HX_CAL_FAIL "同步失敗"
#define D_RESET_HX711 "重設比例"
#define D_CONFIGURE_HX711 "設定比例"
#define D_HX711_PARAMETERS "比例參數"
#define D_ITEM_WEIGHT "物品重量"
#define D_REFERENCE_WEIGHT "參考重量"
#define D_CALIBRATE "同步"
#define D_CALIBRATION "同步"

//xsns_35_tx20.ino
#define D_TX20_WIND_DIRECTION "風向"
#define D_TX20_WIND_SPEED "風速"
#define D_TX20_WIND_SPEED_MIN "最低風速"
#define D_TX20_WIND_SPEED_MAX "最高風速"
#define D_TX20_NORTH "北"
#define D_TX20_EAST "東"
#define D_TX20_SOUTH "南"
#define D_TX20_WEST "西"

// xsns_53_sml.ino
#define D_TPWRIN "Energy Total-In"
#define D_TPWROUT "Energy Total-Out"
#define D_TPWRCURR "Active Power-In/Out"
#define D_TPWRCURR1 "Active Power-In p1"
#define D_TPWRCURR2 "Active Power-In p2"
#define D_TPWRCURR3 "Active Power-In p3"
#define D_Strom_L1 "Current L1"
#define D_Strom_L2 "Current L2"
#define D_Strom_L3 "Current L3"
#define D_Spannung_L1 "Voltage L1"
#define D_Spannung_L2 "Voltage L2"
#define D_Spannung_L3 "Voltage L3"
#define D_METERNR "表號"
#define D_METERSID "服務ID"
#define D_GasIN "計數器"
#define D_H2oIN "計數器"
#define D_StL1L2L3 "Current L1+L2+L3"
#define D_SpL1L2L3 "Voltage L1+L2+L3/3"

// xsns_86_tfminiplus.ino
#define D_SIGNALSTRENGTH "Signal Strength"
#define D_CHIPTEMPERATURE "Chip Temperature"

// tasmota_template.h - keep them as short as possible to be able to fit them in GUI drop down box
#define D_SENSOR_NONE          "無"
#define D_SENSOR_USER          "使用者"
#define D_SENSOR_OPTION        "Option"
#define D_SENSOR_DHT11         "DHT11"
#define D_SENSOR_AM2301        "AM2301"
#define D_SENSOR_SI7021        "SI7021"
#define D_SENSOR_DS18X20       "DS18x20"
#define D_SENSOR_I2C_SCL       "I2C SCL"
#define D_SENSOR_I2C_SDA       "I2C SDA"
#define D_SENSOR_WS2812        "WS2812"
#define D_SENSOR_DFR562        "MP3 Player"
#define D_SENSOR_IRSEND        "IRsend"
#define D_SENSOR_SWITCH        "Switch"     // Suffix "1"
#define D_SENSOR_BUTTON        "Button"     // Suffix "1"
#define D_SENSOR_RELAY         "Relay"      // Suffix "1i"
#define D_SENSOR_LED           "Led"        // Suffix "1i"
#define D_SENSOR_LED_LINK      "LedLink"    // Suffix "i"
#define D_SENSOR_PWM           "PWM"        // Suffix "1"
#define D_SENSOR_COUNTER       "Counter"    // Suffix "1"
#define D_SENSOR_INPUT         "Input"
#define D_SENSOR_IRRECV        "IRrecv"
#define D_SENSOR_MHZ_RX        "MHZ Rx"
#define D_SENSOR_MHZ_TX        "MHZ Tx"
#define D_SENSOR_PZEM004_RX    "PZEM004 Rx"
#define D_SENSOR_PZEM016_RX    "PZEM016 Rx"
#define D_SENSOR_PZEM017_RX    "PZEM017 Rx"
#define D_SENSOR_PZEM0XX_TX    "PZEM0XX Tx"
#define D_SENSOR_SAIR_RX       "SAir Rx"
#define D_SENSOR_SAIR_TX       "SAir Tx"
#define D_SENSOR_SPI_CS        "SPI CS"
#define D_SENSOR_SPI_DC        "SPI DC"
#define D_SENSOR_SPI_MISO      "SPI MISO"
#define D_SENSOR_SPI_MOSI      "SPI MOSI"
#define D_SENSOR_SPI_CLK       "SPI CLK"
#define D_SENSOR_BACKLIGHT     "Backlight"
#define D_SENSOR_PMS5003_TX    "PMS5003 Tx"
#define D_SENSOR_PMS5003_RX    "PMS5003 Rx"
#define D_SENSOR_SDS0X1_RX     "SDS0X1 Rx"
#define D_SENSOR_SDS0X1_TX     "SDS0X1 Tx"
#define D_SENSOR_HPMA_RX       "HPMA Rx"
#define D_SENSOR_HPMA_TX       "HPMA Tx"
#define D_SENSOR_SBR_RX        "SerBr Rx"
#define D_SENSOR_SBR_TX        "SerBr Tx"
#define D_SENSOR_SR04_TRIG     "SR04 Tri/TX"
#define D_SENSOR_SR04_ECHO     "SR04 Ech/RX"
#define D_SENSOR_SDM72_TX      "SDM72 Tx"
#define D_SENSOR_SDM72_RX      "SDM72 Rx"
#define D_SENSOR_SDM120_TX     "SDMx20 Tx"
#define D_SENSOR_SDM120_RX     "SDMx20 Rx"
#define D_SENSOR_SDM630_TX     "SDM630 Tx"
#define D_SENSOR_SDM630_RX     "SDM630 Rx"
#define D_SENSOR_WE517_TX      "WE517 Tx"
#define D_SENSOR_WE517_RX      "WE517 Rx"
#define D_SENSOR_TM1637_CLK    "TM1637 CLK"
#define D_SENSOR_TM1637_DIO    "TM1637 DIO"
#define D_SENSOR_TM1638_CLK    "TM1638 CLK"
#define D_SENSOR_TM1638_DIO    "TM1638 DIO"
#define D_SENSOR_TM1638_STB    "TM1638 STB"
#define D_SENSOR_MAX7219_DIN    "MAX7219 DIN"
#define D_SENSOR_MAX7219_CS    "MAX7219 CS"
#define D_SENSOR_MAX7219_CLK    "MAX7219 CLK"
#define D_SENSOR_HX711_SCK     "HX711 SCK"
#define D_SENSOR_HX711_DAT     "HX711 DAT"
#define D_SENSOR_FTC532        "FTC532"
#define D_SENSOR_BS814_CLK     "BS814 CLK"
#define D_SENSOR_BS814_DAT     "BS814 DAT"
#define D_SENSOR_TX2X_TX       "TX2x"
#define D_SENSOR_RFSEND        "RFSend"
#define D_SENSOR_RFRECV        "RFrecv"
#define D_SENSOR_TUYA_TX       "Tuya Tx"
#define D_SENSOR_TUYA_RX       "Tuya Rx"
#define D_SENSOR_MGC3130_XFER  "MGC3130 Xfr"
#define D_SENSOR_MGC3130_RESET "MGC3130 Rst"
#define D_SENSOR_SSPI_MISO     "SSPI MISO"
#define D_SENSOR_SSPI_MOSI     "SSPI MOSI"
#define D_SENSOR_SSPI_SCLK     "SSPI SCLK"
#define D_SENSOR_SSPI_CS       "SSPI CS"
#define D_SENSOR_SSPI_DC       "SSPI DC"
#define D_SENSOR_RF_SENSOR     "RF Sensor"
#define D_SENSOR_AZ_RX         "AZ Rx"
#define D_SENSOR_AZ_TX         "AZ Tx"
#define D_SENSOR_MAX31855_CS   "MX31855 CS"
#define D_SENSOR_MAX31855_CLK  "MX31855 CLK"
#define D_SENSOR_MAX31855_DO   "MX31855 DO"
#define D_SENSOR_MAX31865_CS   "MX31865 CS"
#define D_SENSOR_NRG_SEL       "HLWBL SEL"  // Suffix "i"
#define D_SENSOR_NRG_CF1       "HLWBL CF1"
#define D_SENSOR_HLW_CF        "HLW8012 CF"
#define D_SENSOR_HJL_CF        "BL0937 CF"
#define D_SENSOR_MCP39F5_TX    "MCP39F5 Tx"
#define D_SENSOR_MCP39F5_RX    "MCP39F5 Rx"
#define D_SENSOR_MCP39F5_RST   "MCP39F5 Rst"
#define D_SENSOR_CSE7761_TX    "CSE7761 Tx"
#define D_SENSOR_CSE7761_RX    "CSE7761 Rx"
#define D_SENSOR_CSE7766_TX    "CSE7766 Tx"
#define D_SENSOR_CSE7766_RX    "CSE7766 Rx"
#define D_SENSOR_PN532_TX      "PN532 Tx"
#define D_SENSOR_PN532_RX      "PN532 Rx"
#define D_SENSOR_SM16716_CLK   "SM16716 CLK"
#define D_SENSOR_SM16716_DAT   "SM16716 DAT"
#define D_SENSOR_SM16716_POWER "SM16716 PWR"
#define D_SENSOR_P9813_CLK     "P9813 Clk"
#define D_SENSOR_P9813_DAT     "P9813 Dat"
#define D_SENSOR_MY92X1_DI     "MY92x1 DI"
#define D_SENSOR_MY92X1_DCKI   "MY92x1 DCKI"
#define D_SENSOR_ARIRFRCV      "ALux IrRcv"
#define D_SENSOR_ARIRFSEL      "ALux IrSel"
#define D_SENSOR_TXD           "Serial Tx"
#define D_SENSOR_RXD           "Serial Rx"
#define D_SENSOR_ROTARY        "Rotary"     // Suffix "1A"
#define D_SENSOR_HRE_CLOCK     "HRE Clock"
#define D_SENSOR_HRE_DATA      "HRE Data"
#define D_SENSOR_ADE7953_IRQ   "ADE7953 IRQ"
#define D_SENSOR_BUZZER        "Buzzer"
#define D_SENSOR_OLED_RESET    "OLED Reset"
#define D_SENSOR_ZIGBEE_TXD    "Zigbee Tx"
#define D_SENSOR_ZIGBEE_RXD    "Zigbee Rx"
#define D_SENSOR_ZIGBEE_RST    "Zigbee Rst"
#define D_SENSOR_SOLAXX1_TX    "SolaxX1 Tx"
#define D_SENSOR_SOLAXX1_RX    "SolaxX1 Rx"
#define D_SENSOR_IBEACON_TX    "iBeacon TX"
#define D_SENSOR_IBEACON_RX    "iBeacon RX"
#define D_SENSOR_RDM6300_RX    "RDM6300 RX"
#define D_SENSOR_RU5300_TX     "RU5300 TX"
#define D_SENSOR_RU5300_RX     "RU5300 RX"
#define D_SENSOR_CC1101_CS     "CC1101 CS"
#define D_SENSOR_A4988_DIR     "A4988 DIR"
#define D_SENSOR_A4988_STP     "A4988 STP"
#define D_SENSOR_A4988_ENA     "A4988 ENA"
#define D_SENSOR_A4988_MS1     "A4988 MS1"
#define D_SENSOR_OUTPUT_HI     "Output Hi"
#define D_SENSOR_OUTPUT_LO     "Output Lo"
#define D_SENSOR_AS608_TX      "AS608 Tx"
#define D_SENSOR_AS608_RX      "AS608 Rx"
#define D_SENSOR_DDS2382_TX    "DDS238-2 Tx"
#define D_SENSOR_DDS2382_RX    "DDS238-2 Rx"
#define D_SENSOR_DDSU666_TX    "DDSU666 Tx"
#define D_SENSOR_DDSU666_RX    "DDSU666 Rx"
#define D_SENSOR_SM2135_CLK    "SM2135 Clk"
#define D_SENSOR_SM2135_DAT    "SM2135 Dat"
#define D_SENSOR_DEEPSLEEP     "DeepSleep"
#define D_SENSOR_EXS_ENABLE    "EXS Enable"
#define D_SENSOR_CLIENT_TX    "Client TX"
#define D_SENSOR_CLIENT_RX    "Client RX"
#define D_SENSOR_CLIENT_RESET "Client RST"
#define D_SENSOR_GPS_RX        "GPS RX"
#define D_SENSOR_GPS_TX        "GPS TX"
#define D_SENSOR_HM10_RX       "HM10 RX"
#define D_SENSOR_HM10_TX       "HM10 TX"
#define D_SENSOR_LE01MR_RX     "LE-01MR Rx"
#define D_SENSOR_LE01MR_TX     "LE-01MR Tx"
#define D_SENSOR_BL0940_RX     "BL0940 Rx"
#define D_SENSOR_CC1101_GDO0   "CC1101 GDO0"
#define D_SENSOR_CC1101_GDO2   "CC1101 GDO2"
#define D_SENSOR_HRXL_RX       "HRXL Rx"
#define D_SENSOR_DYP_RX        "DYP Rx"
#define D_SENSOR_ELECTRIQ_MOODL "MOODL Tx"
#define D_SENSOR_AS3935        "AS3935"
#define D_SENSOR_WINDMETER_SPEED "WindMeter Spd"
#define D_SENSOR_TELEINFO_RX   "TInfo Rx"
#define D_SENSOR_TELEINFO_ENABLE "TInfo EN"
#define D_SENSOR_LMT01_PULSE   "LMT01 Pulse"
#define D_SENSOR_ADC_INPUT     "ADC Input"
#define D_SENSOR_ADC_TEMP      "ADC Temp"
#define D_SENSOR_ADC_LIGHT     "ADC Light"
#define D_SENSOR_ADC_BUTTON    "ADC Button"
#define D_SENSOR_ADC_RANGE     "ADC Range"
#define D_SENSOR_ADC_CT_POWER  "ADC CT Power"
#define D_SENSOR_ADC_JOYSTICK  "ADC Joystick"
#define D_SENSOR_ADC_PH        "ADC pH"
#define D_GPIO_WEBCAM_PWDN     "CAM_PWDN"
#define D_GPIO_WEBCAM_RESET    "CAM_RESET"
#define D_GPIO_WEBCAM_XCLK     "CAM_XCLK"
#define D_GPIO_WEBCAM_SIOD     "CAM_SIOD"
#define D_GPIO_WEBCAM_SIOC     "CAM_SIOC"
#define D_GPIO_WEBCAM_DATA     "CAM_DATA"
#define D_GPIO_WEBCAM_VSYNC    "CAM_VSYNC"
#define D_GPIO_WEBCAM_HREF     "CAM_HREF"
#define D_GPIO_WEBCAM_PCLK     "CAM_PCLK"
#define D_GPIO_WEBCAM_PSCLK    "CAM_PSCLK"
#define D_GPIO_WEBCAM_HSD      "CAM_HSD"
#define D_GPIO_WEBCAM_PSRCS    "CAM_PSRCS"
#define D_SENSOR_ETH_PHY_POWER "ETH POWER"
#define D_SENSOR_ETH_PHY_MDC   "ETH MDC"
#define D_SENSOR_ETH_PHY_MDIO  "ETH MDIO"
#define D_SENSOR_TCP_TXD       "TCP Tx"
#define D_SENSOR_TCP_RXD       "TCP Rx"
#define D_SENSOR_IEM3000_TX    "iEM3000 TX"
#define D_SENSOR_IEM3000_RX    "iEM3000 RX"
#define D_SENSOR_MIEL_HVAC_TX  "MiEl HVAC Tx"
#define D_SENSOR_MIEL_HVAC_RX  "MiEl HVAC Rx"
#define D_SENSOR_PROJECTOR_CTRL_TX  "DLP Tx"
#define D_SENSOR_PROJECTOR_CTRL_RX  "DLP Rx"
#define D_SENSOR_SHELLY_DIMMER_BOOT0 "SHD Boot 0"
#define D_SENSOR_SHELLY_DIMMER_RST_INV "SHD Reset"
#define D_SENSOR_RC522_RST     "RC522 Rst"
#define D_SENSOR_RC522_CS      "RC522 CS"
#define D_SENSOR_NRF24_CS      "NRF24 CS"
#define D_SENSOR_NRF24_DC      "NRF24 DC"
#define D_SENSOR_XPT2046_CS    "XPT2046 CS"
#define D_SENSOR_ILI9341_CS    "ILI9341 CS"
#define D_SENSOR_ILI9341_DC    "ILI9341 DC"
#define D_SENSOR_ILI9488_CS    "ILI9488 CS"
#define D_SENSOR_EPAPER29_CS   "EPaper29 CS"
#define D_SENSOR_EPAPER42_CS   "EPaper42 CS"
#define D_SENSOR_SSD1351_CS    "SSD1351 CS"
#define D_SENSOR_SSD1351_DC    "SSD1351 DC"
#define D_SENSOR_RA8876_CS     "RA8876 CS"
#define D_SENSOR_ST7789_CS     "ST7789 CS"
#define D_SENSOR_ST7789_DC     "ST7789 DC"
#define D_SENSOR_SSD1331_CS    "SSD1331 CS"
#define D_SENSOR_SSD1331_DC    "SSD1331 DC"
#define D_SENSOR_SDCARD_CS     "SDCard CS"
#define D_SENSOR_WIEGAND_D0    "Wiegand D0"
#define D_SENSOR_WIEGAND_D1    "Wiegand D1"
#define D_SENSOR_NEOPOOL_TX    "NeoPool Tx"
#define D_SENSOR_NEOPOOL_RX    "NeoPool Rx"
#define D_SENSOR_VL53L0X_XSHUT "VL53L0X XSHUT"
#define D_SENSOR_TFMINIPLUS_TX  "TFmini+ TX"
#define D_SENSOR_TFMINIPLUS_RX  "TFmini+ RX"
#define D_SENSOR_ZEROCROSS     "ZC Pulse"
#define D_SENSOR_HALLEFFECT    "HallEffect"
#define D_SENSOR_EPD_DATA      "EPD Data"

// Units
#define D_UNIT_AMPERE "安培"
#define D_UNIT_CELSIUS "C"
#define D_UNIT_CENTIMETER "cm"
#define D_UNIT_DEGREE "°"
#define D_UNIT_FAHRENHEIT "F"
#define D_UNIT_HERTZ "Hz"
#define D_UNIT_HOUR "時"
#define D_UNIT_GALLONS "gal"
#define D_UNIT_GALLONS_PER_MIN "g/m"
#define D_UNIT_INCREMENTS "inc"
#define D_UNIT_KELVIN "K"
#define D_UNIT_KILOMETER "km"
#define D_UNIT_KILOGRAM "kg"
#define D_UNIT_KILOMETER_PER_HOUR "km/h"  // or "km/h"
#define D_UNIT_KILOOHM "千歐姆"
#define D_UNIT_KILOWATTHOUR "千瓦小時"
#define D_UNIT_LITERS "L"
#define D_UNIT_LITERS_PER_MIN "L/m"
#define D_UNIT_LUX "lux"
#define D_UNIT_MICROGRAM_PER_CUBIC_METER "微克/立方公尺"
#define D_UNIT_MICROMETER "微米"
#define D_UNIT_MICROSECOND "微秒"
#define D_UNIT_MICROSIEMENS_PER_CM "µS/cm"
#define D_UNIT_MILLIAMPERE "毫安培"
#define D_UNIT_MILLILITERS "ml"
#define D_UNIT_MILLIMETER "mm"
#define D_UNIT_MILLIMETER_MERCURY "mmHg"
#define D_UNIT_MILLISECOND "毫秒"
#define D_UNIT_MILLIVOLT "mV"
#define D_UNIT_MINUTE "分"
#define D_UNIT_PARTS_PER_BILLION "ppb"
#define D_UNIT_PARTS_PER_DECILITER "每分升"
#define D_UNIT_PARTS_PER_MILLION "ppm"
#define D_UNIT_MILIGRAMS_PER_LITER "mg/L"
#define D_UNIT_PERCENT "%%"
#define D_UNIT_PRESSURE "百帕"
#define D_UNIT_SECOND "秒"
#define D_UNIT_SECTORS "磁區"
#define D_UNIT_VA "伏安(VA)"
#define D_UNIT_VAR "乏爾(VAr)"
#define D_UNIT_VOLT "伏特"
#define D_UNIT_WATT "瓦特"
#define D_UNIT_WATTHOUR "瓦小時"
#define D_UNIT_WATT_METER_QUADRAT "W/m²"

#define D_NEW_ADDRESS          "Setting address to"
#define D_OUT_OF_RANGE         "Out of Range"
#define D_SENSOR_DETECTED      "detected"

//SDM220, SDM120, SDM72, LE01MR
#define D_EXPORT_POWER    "Export Power"
#define D_IMPORT_POWER 	  "Import Power"
#define D_PHASE_ANGLE     "相量(Phase Angle)"
#define D_IMPORT_ACTIVE   "Import Active"
#define D_EXPORT_ACTIVE   "Export Active"
#define D_IMPORT_REACTIVE "Import Reactive"
#define D_EXPORT_REACTIVE "Export Reactive"
#define D_TOTAL_REACTIVE  "Total Reactive"
#define D_UNIT_KWARH      "kVArh"
#define D_UNIT_ANGLE      "Deg"
#define D_TOTAL_ACTIVE    "Total Active"

//SOLAXX1
#define D_PV1_VOLTAGE     "PV1 Voltage"
#define D_PV1_CURRENT     "PV1 Current"
#define D_PV1_POWER       "PV1 Power"
#define D_PV2_VOLTAGE     "PV2 Voltage"
#define D_PV2_CURRENT     "PV2 Current"
#define D_PV2_POWER       "PV2 Power"
#define D_SOLAR_POWER     "太陽能"
#define D_INVERTER_POWER  "Inverter Power"
#define D_STATUS          "狀態"
#define D_WAITING         "等待中"
#define D_CHECKING        "確認中"
#define D_WORKING         "運作中"
#define D_FAILURE         "錯誤"
#define D_SOLAX_ERROR_0   "無錯誤碼"
#define D_SOLAX_ERROR_1   "電網失聯(Grid Lost)錯誤"
#define D_SOLAX_ERROR_2   "電網(Grid)電壓錯誤"
#define D_SOLAX_ERROR_3   "電網(Grid)頻率錯誤"
#define D_SOLAX_ERROR_4   "光伏電壓(Pv Voltage)錯誤"
#define D_SOLAX_ERROR_5   "隔離錯誤"
#define D_SOLAX_ERROR_6   "過熱錯誤"
#define D_SOLAX_ERROR_7   "風扇錯誤"
#define D_SOLAX_ERROR_8   "其他裝置錯誤"

//xdrv_10_scripter.ino
#define D_CONFIGURE_SCRIPT     "編輯腳本"
#define D_SCRIPT               "編輯腳本"
#define D_SDCARD_UPLOAD        "上傳檔案"
#define D_UFSDIR               "記憶卡目錄"
#define D_UPL_DONE             "完成"
#define D_SCRIPT_CHARS_LEFT    "剩餘字元"
#define D_SCRIPT_CHARS_NO_MORE "放不下更多字元了"
#define D_SCRIPT_DOWNLOAD      "下載"
#define D_SCRIPT_ENABLE        "啟用腳本"
#define D_SCRIPT_UPLOAD        "上傳"
#define D_SCRIPT_UPLOAD_FILES  "上傳檔案"

//xdrv_50_filesystem.ino
#define D_MANAGE_FILE_SYSTEM   "Manage File system"
#define D_FS_SIZE              "Size"
#define D_FS_FREE              "Free"
#define D_NEW_FILE             "newfile.txt"
#define D_CREATE_NEW_FILE      "Create and edit new file"
#define D_EDIT_FILE            "Edit File"

//xsns_67_as3935.ino
#define D_AS3935_GAIN "gain:"
#define D_AS3935_ENERGY "能量:"
#define D_AS3935_DISTANCE "距離:"
#define D_AS3935_DISTURBER "干擾物:"
#define D_AS3935_VRMS "µVrms:"
#define D_AS3935_APRX "大約:"
#define D_AS3935_AWAY "遠離"
#define D_AS3935_LIGHT "閃電"
#define D_AS3935_OUT "閃電在範圍外"
#define D_AS3935_NOT "距離不確定"
#define D_AS3935_ABOVE "閃電在頭上"
#define D_AS3935_NOISE "偵測到雜訊"
#define D_AS3935_DISTDET "偵測到干擾物"
#define D_AS3935_INTNOEV "沒有任何事件觸發中斷!"
#define D_AS3935_FLICKER "IRQ flicker!"
#define D_AS3935_POWEROFF "Power Off"
#define D_AS3935_NOMESS "聽取中..."
#define D_AS3935_ON "開啟"
#define D_AS3935_OFF "關閉"
#define D_AS3935_INDOORS "室內"
#define D_AS3935_OUTDOORS "戶外"
#define D_AS3935_CAL_FAIL "同步失敗"
#define D_AS3935_CAL_OK "已同步為:"

//xsns_68_opentherm.ino
#define D_SENSOR_BOILER_OT_RX   "OpenTherm RX"
#define D_SENSOR_BOILER_OT_TX   "OpenTherm TX"

// xnrg_15_teleinfo Denky (Teleinfo)
#define D_CONTRACT        "Contract"
#define D_POWER_LOAD      "Power load"
#define D_CURRENT_TARIFF  "Current Tariff"
#define D_TARIFF          "Tariff"
#define D_OVERLOAD        "ADPS"
#define D_MAX_POWER       "Max Power"
#define D_MAX_CURRENT     "Max Current"

// xsns_79_as608.ino
#define D_FP_ENROLL_PLACEFINGER "Place finger"
#define D_FP_ENROLL_REMOVEFINGER "Remove finger"
#define D_FP_ENROLL_PLACESAMEFINGER "Place same finger again"
#define D_FP_ENROLL_RETRY "Error so retry"
#define D_FP_ENROLL_RESTART "Restart"
#define D_FP_ENROLL_ERROR "Error"
#define D_FP_ENROLL_RESET "Reset"
#define D_FP_ENROLL_ACTIVE "Active"
#define D_FP_ENROLL_INACTIVE "Inactive"
// Indexed by Adafruit_Fingerprint.h defines
#define D_FP_PACKETRECIEVEERR "Comms error"    // 0x01 Error when receiving data package
#define D_FP_NOFINGER ""                       // 0x02 No finger on the sensor
#define D_FP_IMAGEFAIL "Imaging error"         // 0x03 Failed to enroll the finger
#define D_FP_IMAGEMESS "Image too messy"       // 0x06 Failed to generate character file due to overly disorderly fingerprint image
#define D_FP_FEATUREFAIL "Fingerprint too small" // 0x07 Failed to generate character file due to the lack of character point or small fingerprint image
#define D_FP_NOMATCH "No match"                // 0x08 Finger doesn't match
#define D_FP_NOTFOUND "Did not find a match"   // 0x09 Failed to find matching finger
#define D_FP_ENROLLMISMATCH "Fingerprint did not match" // 0x0A Failed to combine the character files
#define D_FP_BADLOCATION "Bad location"        // 0x0B Addressed PageID is beyond the finger library
#define D_FP_DBRANGEFAIL "DB range error"      // 0x0C Error when reading template from library or invalid template
#define D_FP_UPLOADFEATUREFAIL "Upload feature error" // 0x0D Error when uploading template
#define D_FP_PACKETRESPONSEFAIL "Packet response error" // 0x0E Module failed to receive the following data packages
#define D_FP_UPLOADFAIL "Upload error"         // 0x0F Error when uploading image
#define D_FP_DELETEFAIL "Delete error"         // 0x10 Failed to delete the template
#define D_FP_DBCLEARFAIL "DB Clear error"      // 0x11 Failed to clear finger library
#define D_FP_PASSFAIL "Password error"         // 0x13 Find whether the fingerprint passed or failed
#define D_FP_INVALIDIMAGE "Image invalid"      // 0x15 Failed to generate image because of lac of valid primary image
#define D_FP_FLASHERR "Flash write error"      // 0x18 Error when writing flash
#define D_FP_INVALIDREG "Invalid number"       // 0x1A Invalid register number
#define D_FP_ADDRCODE "Address code"           // 0x20 Address code
#define D_FP_PASSVERIFY "Password verified"    // 0x21 Verify the fingerprint passed
#define D_FP_UNKNOWNERROR "Error"              // Any other error

// xsns_83_neopool.ino
#define D_NEOPOOL_MACH_NONE               "NeoPool"           // Machine names
#define D_NEOPOOL_MACH_HIDROLIFE          "Hidrolife (yellow)"
#define D_NEOPOOL_MACH_AQUASCENIC         "Aquascenic (blue)"
#define D_NEOPOOL_MACH_OXILIFE            "Oxilife (green)"
#define D_NEOPOOL_MACH_BIONET             "Bionet (light blue)"
#define D_NEOPOOL_MACH_HIDRONISER         "Hidroniser (red)"
#define D_NEOPOOL_MACH_UVSCENIC           "UVScenic (lilac)"
#define D_NEOPOOL_MACH_STATION            "Station (orange)"
#define D_NEOPOOL_MACH_BRILIX             "Brilix"
#define D_NEOPOOL_MACH_GENERIC            "Generic"
#define D_NEOPOOL_MACH_BAYROL             "Bayrol"
#define D_NEOPOOL_MACH_HAY                "Hay"
#define D_NEOPOOL_FILTRATION_MANUAL       "Manual"            // Filtration modes
#define D_NEOPOOL_FILTRATION_AUTO         "Auto"
#define D_NEOPOOL_FILTRATION_HEATING      "Heating"
#define D_NEOPOOL_FILTRATION_SMART        "Smart"
#define D_NEOPOOL_FILTRATION_INTELLIGENT  "Intelligent"
#define D_NEOPOOL_FILTRATION_BACKWASH     "Backwash"
#define D_NEOPOOL_FILTRATION_NONE         ""                  // Filtration speed level
#define D_NEOPOOL_FILTRATION_SLOW         "slow"
#define D_NEOPOOL_FILTRATION_MEDIUM       "medium"
#define D_NEOPOOL_FILTRATION_FAST         "fast"
#define D_NEOPOOL_TYPE                    "Type"              // Sensor & relais names
#define D_NEOPOOL_REDOX                   "Redox"
#define D_NEOPOOL_CHLORINE                "Chlorine"
#define D_NEOPOOL_CONDUCTIVITY            "Conductivity"
#define D_NEOPOOL_IONIZATION              "Ionization"
#define D_NEOPOOL_HYDROLYSIS              "Hydrolysis"
#define D_NEOPOOL_RELAY                   "Relay"
#define D_NEOPOOL_RELAY_FILTRATION        "Filtration"
#define D_NEOPOOL_RELAY_LIGHT             "Light"
#define D_NEOPOOL_RELAY_PH_ACID           "Acid pump"
#define D_NEOPOOL_RELAY_PH_BASE           "Base pump"
#define D_NEOPOOL_RELAY_RX                "Redox level"
#define D_NEOPOOL_RELAY_CL                "Chlorine pump"
#define D_NEOPOOL_RELAY_CD                "Brine pump"
#define D_NEOPOOL_TIME                    "Time"
#define D_NEOPOOL_FILT_MODE               "Filtration"
#define D_NEOPOOL_POLARIZATION            "Pol"               // Sensor status
#define D_NEOPOOL_PR_OFF                  "PrOff"
#define D_NEOPOOL_SETPOINT_OK             "Ok"
#define D_NEOPOOL_COVER                   "Cover"
#define D_NEOPOOL_SHOCK                   "Shock"
#define D_NEOPOOL_ALARM                   "! "
#define D_NEOPOOL_LOW                     "Low"
#define D_NEOPOOL_FLOW1                   "FL1"
#define D_NEOPOOL_FLOW2                   "FL2"
#define D_NEOPOOL_PH_HIGH                 "too high"          // ph Alarms
#define D_NEOPOOL_PH_LOW                  "too low"
#define D_NEOPOOL_PUMP_TIME_EXCEEDED      "pump time exceeded"

#endif  // _LANGUAGE_ZH_TW_H_
