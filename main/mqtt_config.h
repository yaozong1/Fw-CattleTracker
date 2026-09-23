#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

// MQTT服务器配置
#define MQTT_BROKER_HOST    "processengineeringsz.com"      // MQTT服务器地址
#define MQTT_BROKER_PORT    1883                        // MQTT服务器端口 (1883=非加密, 8883=SSL)

// MQTT客户端认证信息
#define MQTT_CLIENT_ID      "esp32_cattle_tracker_001"      // 客户端ID，确保唯一
#define MQTT_USERNAME       "testuser"             // MQTT用户名
#define MQTT_PASSWORD       "Hh3341136"             // MQTT密码

// MQTT主题配置
#define MQTT_TOPIC_PREFIX   "cattle/CT-001"                   // 主题前缀
#define MQTT_TOPIC_STATUS   MQTT_TOPIC_PREFIX "/status" // 状态主题
#define MQTT_TOPIC_DATA     MQTT_TOPIC_PREFIX "/data"   // 数据主题
#define MQTT_TOPIC_GPS      MQTT_TOPIC_PREFIX "/gps"    // GPS位置主题
#define MQTT_TOPIC_SENSOR   MQTT_TOPIC_PREFIX "/sensor" // 传感器数据主题
#define MQTT_TOPIC_BATTERY  MQTT_TOPIC_PREFIX "/battery" // 电池数据主题
#define MQTT_TOPIC_Device   MQTT_TOPIC_PREFIX "/device" // 传感器数据主题

// MQTT QoS等级
// 建议当前阶段使用QoS0以简化发布流程（无需msgId确认，移动网络更稳）
#define MQTT_QOS_LEVEL      0                           // 0=最多一次, 1=至少一次, 2=只有一次

// MQTT保持连接间隔（秒）
#define MQTT_KEEPALIVE      60

// MQTT重连设置
#define MQTT_RECONNECT_TIMEOUT_MS    5000               // 重连超时时间
#define MQTT_MAX_RECONNECT_ATTEMPTS  5                  // 最大重连次数

// 网络APN配置（适用于4G模组）
// 注意：如果APN_NAME为空字符串，将跳过APN设置
// 常见APN: "cmnet", "3gnet", "ctnet", "uninet", "wonet"
#define APN_NAME            ""                          // 网络接入点名称
#define APN_USERNAME        ""                          // APN用户名（通常为空）
#define APN_PASSWORD        ""                          // APN密码（通常为空）

#endif // MQTT_CONFIG_H