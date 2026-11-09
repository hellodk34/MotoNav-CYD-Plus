#pragma once

constexpr int GNSS_UART_NUMBER = 2;
constexpr int GNSS_RX_PIN = 35;
constexpr int GNSS_TX_PIN = -1;
constexpr uint32_t GNSS_BAUD = 9600;

constexpr int TOUCH_CS_PIN = 33;
constexpr int TOUCH_IRQ_PIN = 36;
constexpr int TOUCH_MOSI_PIN = 32;
constexpr int TOUCH_MISO_PIN = 39;
constexpr int TOUCH_CLK_PIN = 25;

constexpr int SD_CS_PIN = 5;
constexpr int SD_SCK_PIN = 18;
constexpr int SD_MISO_PIN = 19;
constexpr int SD_MOSI_PIN = 23;

constexpr uint32_t GNSS_DATA_TIMEOUT_MS = 3000;
constexpr uint32_t SCREEN_REFRESH_MS = 200;
constexpr uint32_t TOUCH_MIN_PRESS_MS = 60;
constexpr uint32_t MENU_HOLD_MS = 1500;

constexpr double SPEED_NOISE_FLOOR_KMH = 0.8;
constexpr double MOVING_SPEED_THRESHOLD_KMH = 2.0;
constexpr double MAX_VALID_SPEED_KMH = 220.0;
constexpr double MAX_HDOP_FOR_DISTANCE = 4.0;
constexpr double MAX_HDOP_FOR_TRACK = 5.0;
constexpr uint32_t MIN_SATELLITES_FOR_TRACK = 4;
constexpr double MIN_TRACK_POINT_DISTANCE_M = 1.5;
constexpr double MAX_TRACK_SEGMENT_DISTANCE_M = 100.0;
constexpr uint32_t MAX_TRACK_POINT_GAP_MS = 5000;
constexpr uint32_t MAX_MOTION_SAMPLE_MS = 1000;

constexpr double SPEED_SCREEN_ENTER_KMH = 5.0;
constexpr double SPEED_SCREEN_EXIT_KMH = 1.0;
constexpr uint32_t SPEED_SCREEN_ENTER_HOLD_MS = 800;
constexpr uint32_t SPEED_SCREEN_EXIT_HOLD_MS = 3000;

constexpr double MENU_MAX_SPEED_KMH = 2.0;
constexpr double TRACK_MOVING_THRESHOLD_KMH = 2.0;
constexpr uint32_t TRACK_POINT_INTERVAL_MS = 1000;
constexpr uint32_t TRACK_AUTO_PAUSE_MS = 10000;
constexpr uint32_t TRACK_FLUSH_INTERVAL_MS = 2000;
constexpr uint32_t TRACK_RECOVERED_NOTICE_MS = 5000;


constexpr double MAX_SPEED_JUMP_KMH_PER_S = 45.0;
constexpr double SPEED_FILTER_ALPHA_LOW = 0.22;
constexpr double SPEED_FILTER_ALPHA_NORMAL = 0.38;

// ---- 速度归零：静止检测 + 非对称 EMA + 迟滞（码表 Auto Pause 同款）----
constexpr double SPEED_STOP_THRESHOLD_KMH = 1.0;   // 原始速度低于此值判「疑似静止」
constexpr double SPEED_STOP_RESUME_KMH = 2.5;      // 静止后需超过此值才恢复显示（迟滞）
constexpr uint32_t SPEED_STOP_HOLD_MS = 2500;      // 连续低速多久后判定真正静止（置 0）
constexpr uint8_t SPEED_STOP_RESUME_SAMPLES = 2;   // 静止恢复去抖：连续几次超恢复阈值才退出静止（防单点毛刺）
constexpr double SPEED_FILTER_ALPHA_FALL = 0.5;    // 减速方向 EMA 系数（更快归零）

// ---- BMP280 气压计（I2C）----
constexpr int I2C_SDA_PIN = 22;                  // I2C 数据线（GPIO21 已被背光占用，改用 22/27）
constexpr int I2C_SCL_PIN = 27;                  // I2C 时钟线
constexpr uint8_t BMP280_ADDR = 0x76;            // 主地址（自动探测会再试 0x77）
constexpr uint8_t BMP280_ADDR_ALT = 0x77;
constexpr float SEA_LEVEL_HPA = 1013.25f;        // 相对海拔参考气压
constexpr float TEMP_OFFSET_C = 0.0f;            // 温度补偿（默认 0，发现系统性偏差再改这个数）

// ---- 数据合理性校验：I2C 偶发干扰会读到离谱值，超出范围则丢弃该采样并跳过写 <ele> ----
constexpr double BARO_MIN_TEMP_C = -50.0;        // 温度合理下限（BMP280 工作范围 -40~+85，留余量）
constexpr double BARO_MAX_TEMP_C = 100.0;        // 温度合理上限（车内暴晒可偏高，182°C 之类是干扰）
constexpr double ALT_MIN_M = -1000.0;            // 海拔合理下限（低于死海 -430m）
constexpr double ALT_MAX_M = 10000.0;            // 海拔合理上限（高于珠峰 8848m）

constexpr uint32_t BARO_READ_INTERVAL_MS = 100;  // BMP280 采样周期
constexpr double BARO_ALT_ALPHA = 0.15;          // 气压海拔低通系数（越大越跟手、越抖）
constexpr double BARO_GNSS_ANCHOR_ALPHA = 0.01;  // GNSS 绝对海拔慢锚定系数
constexpr uint32_t BARO_FAULT_THRESHOLD = 30;    // 连续读失败多少次判定传感器失效（100ms×30≈3s），失效时显示降级、恢复自动回来

// ---- 坡度：时间域 Vz/Vh（码表同款，抑制停车气压漂移→起步尖峰）----
constexpr double GRADE_WINDOW_S = 10.0;            // Vz 回归窗口（秒）
constexpr uint32_t GRADE_SAMPLE_INTERVAL_MS = 250; // 海拔采样周期（4Hz）
constexpr double GRADE_MIN_VH_KMH = 3.0;           // 水平速度门控：低于此值冻结坡度显示
constexpr uint8_t GRADE_MIN_SAMPLES = 8;           // 窗口内最少采样点数（约 2s@4Hz）
constexpr double GRADE_MAX_PERCENT = 40.0;         // 物理限幅：|坡度|超此值视为异常丢弃
constexpr double GRADE_OUTPUT_ALPHA = 0.25;        // 输出坡度 EMA 平滑系数
constexpr double GRADE_DISPLAY_HYSTERESIS = 0.2;   // 整数显示迟滞死区（显示单位：°或%，抑制 0↔1 边界跳变）

// ---- WiFi 配网与 GPX 自动上传 ----
constexpr uint32_t WIFI_STA_TIMEOUT_MS = 25000;      // 开机 STA 连接超时，失败进无密码 AP 配网
constexpr const char *WIFI_AP_SSID = "MotoNav-AP";    // 配网热点名（无密码，仅 2.4GHz）
constexpr uint32_t UPLOAD_RETRY_INTERVAL_MS = 300000; // 上传失败后周期重试间隔（5 分钟）
constexpr uint32_t UPLOAD_SEND_TIMEOUT_MS = 30000;    // 单个文件上传总超时
constexpr size_t UPLOAD_CHUNK_SIZE = 4096;            // GPX 流式上传分块字节数
constexpr uint16_t MAX_BATCH = 32;                    // 单批最多处理/记忆的待上传轨迹条数
