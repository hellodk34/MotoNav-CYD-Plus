#include <Arduino.h>
#include <TFT_eSPI.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <SD.h>
#include <FS.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

#include "config.h"
#include "network.h"
#include "upload.h"

TFT_eSPI tft;
TFT_eSprite speedSprite(&tft);
TinyGPSPlus gps;
HardwareSerial gnssSerial(GNSS_UART_NUMBER);
SPIClass touchSpi(HSPI);
// CYD microSD uses the ESP32 SPI3 controller (legacy VSPI).
// Numeric host 3 keeps compatibility with ESP32 Core 3.x where VSPI may be hidden.
SPIClass sdSpi(3);
File trackFile;
XPT2046_Touchscreen touch(TOUCH_CS_PIN, TOUCH_IRQ_PIN);
Preferences preferences;

bool nightTheme = true;
bool useMph = false;
bool settingsPage2 = false;   // 设置页分页：false=第1页(显示/单位/坡度/网络) true=第2页(网络/上传)
bool sdError = false;
bool savedNotice = false;
uint32_t savedNoticeMs = 0;
uint32_t savedPointCount = 0;
uint32_t nmeaChars = 0;
uint32_t lastNmeaByteMs = 0;
uint32_t lastScreenMs = 0;
uint32_t lastGnssUiMs = 0;
uint32_t touchStartedMs = 0;
bool touchWasDown = false;
bool longPressHandled = false;

enum UiMode { DRIVE_UI, MENU_UI, SETTINGS_UI, SUMMARY_UI, GNSS_UI, RIDES_UI };
UiMode uiMode = DRIVE_UI;
enum TrackState { TRACK_STOPPED, TRACK_WAIT_FIX, TRACK_RECORDING };
TrackState trackState = TRACK_STOPPED;
bool sdReady = false;
uint32_t lastTrackPointMs = 0;
uint32_t lastTrackFlushMs = 0;
uint32_t trackPointCount = 0;
String trackFileName;

constexpr const char *RIDES_INDEX_FILE = "/RIDES_INDEX.CSV";
constexpr uint8_t MAX_RIDE_HISTORY = 20;

struct RideRecord {
  String date;
  String time;
  String gpxFile;
  double distanceKm = 0.0;
  uint32_t totalSeconds = 0;
  uint32_t movingSeconds = 0;
  uint32_t stoppedSeconds = 0;
  double averageKmh = 0.0;
  double maximumKmh = 0.0;
  uint32_t points = 0;
  uint8_t uploaded = 0;      // 0=待上传 1=已上传 2=已忽略
};

RideRecord rideHistory[MAX_RIDE_HISTORY];
uint8_t rideHistoryCount = 0;
uint8_t selectedRideIndex = 0;

double smoothedSpeedKmh = 0.0;
double lastAcceptedRawSpeedKmh = 0.0;
uint32_t lastSpeedFilterMs = 0;
bool speedFilterReady = false;
bool speedStaticActive = false;       // 已判定静止（输出强制归零）
uint32_t speedStaticCandidateMs = 0;  // 疑似静止的起始时间（0 = 无）
uint8_t speedStaticResumeCount = 0;   // 静止恢复去抖计数（连续几次超恢复阈值）

// ---- BMP280 气压计状态 ----
Adafruit_BMP280 bmp;
bool bmpReady = false;
double baroAltitudeM = 0.0;      // 低通后的气压相对海拔
double baroOffsetM = 0.0;        // GNSS 慢锚定偏移（修正绝对海拔）
double baroTemperatureC = 0.0;
bool baroAltInitialized = false;
bool baroAnchorSet = false;
uint32_t lastBaroReadMs = 0;
bool baroHasValidSample = false;    // 是否已读到至少一次有效温压样本（区分「假死显示0」与真实数据）
uint32_t baroConsecutiveFails = 0;  // 连续读失败计数，用于运行中失效降级

// BMP280 是否「当前可用」：已初始化 && 至少读到过一次有效样本 && 没有连续失败超阈值。
// 用于把「假死/失效」与正常状态区分开，避免把初始值 0 当真数据显示。
bool baroHealthy() {
  return bmpReady && baroHasValidSample &&
         baroConsecutiveFails < BARO_FAULT_THRESHOLD;
}

uint8_t gradeStyle = 1;          // 坡度显示风格：0=关闭 1=百分比 2=角度（默认百分比）

// ---- 坡度窗口（时间域 Vz/Vh：码表同款，抑制停车气压漂移→起步尖峰）----
struct GradeSample { uint32_t tMs; double altM; };  // 采样时间戳 + 融合海拔快照
constexpr uint8_t GRADE_RING_SIZE = 128;
GradeSample gradeRing[GRADE_RING_SIZE];
uint8_t gradeRingHead = 0;       // 下一个写入位置
uint8_t gradeRingCount = 0;      // 当前有效点数
uint32_t lastGradeSampleMs = 0;  // 上次海拔采样时间
double gradeAltMedian[3];        // 海拔中值滤波缓冲（3 点）
uint8_t gradeAltMedianIdx = 0;
uint8_t gradeAltMedianCount = 0;
double gradeSmoothedPct = 0.0;   // 输出 EMA 后的坡度百分比
bool gradeHasValue = false;      // 是否已有有效坡度（用于停车冻结显示）
int gradeShownInt = 0;           // 整数显示迟滞：上一次显示的整数（随坡度风格解释单位）
uint8_t gradeShownStyle = 255;   // 迟滞状态对应的坡度风格（255=未初始化，切换风格时复位）

bool havePreviousTrackPosition = false;
double previousTrackLatitude = 0.0;
double previousTrackLongitude = 0.0;
uint32_t previousTrackPositionMs = 0;

// GPX <time> 跨午夜日期跟踪：GPS 的 date 与 time 来自不同 NMEA 语句，
// 午夜时 time 先翻到 00:00:00Z 而 date 可能仍停留在前一天，需自行纠偏。
bool gpsDateTracked = false;
int gpsTrackYear = 0;
int gpsTrackMonth = 0;
int gpsTrackDay = 0;
uint32_t lastGpsSecondsOfDay = 0;

constexpr double STOP_REMINDER_SPEED_KMH = 1.0;
constexpr double STOP_REMINDER_CLEAR_KMH = 3.0;
constexpr double MENU_AUTO_CLOSE_SPEED_KMH = 5.0;
constexpr uint32_t STOP_REMINDER_DELAY_MS = 15000UL;
uint32_t trackStoppedStartedMs = 0;
bool stopRecordReminderVisible = false;

constexpr int16_t TOUCH_X_MIN = 250;
constexpr int16_t TOUCH_X_MAX = 3850;
constexpr int16_t TOUCH_Y_MIN = 250;
constexpr int16_t TOUCH_Y_MAX = 3850;

enum ScreenMode { TRIP_SCREEN, SPEED_SCREEN };
ScreenMode screenMode = TRIP_SCREEN;
bool manualScreenMode = false;   // 手动切屏模式：true 时暂停自动切屏
uint32_t speedThresholdStartedMs = 0;
uint32_t stoppedStartedMs = 0;
int previousGaugeTick = -1;

bool tripActive = false;
uint32_t tripStartedMs = 0;
uint32_t tripFinishedMs = 0;
uint32_t lastMotionSampleMs = 0;

double summaryDistanceM = 0.0;
double summaryAverageSpeedKmh = 0.0;
double summaryMaximumSpeedKmh = 0.0;
uint32_t summaryTotalTimeMs = 0;
uint32_t summaryMovingTimeMs = 0;
uint32_t summaryStoppedTimeMs = 0;
uint32_t summaryPointCount = 0;
bool summarySavedOk = false;
String summaryTrackFileName;

constexpr const char *ACTIVE_TRACK_MARKER = "/ACTIVE_TRACK.TXT";
constexpr const char *ACTIVE_STATS_FILE = "/ACTIVE_STATS.CSV";
constexpr uint32_t STATS_SNAPSHOT_INTERVAL_MS = 10000UL;  // 每 10 秒快照累计统计，断电恢复用
uint32_t lastStatsSnapshotMs = 0;
uint32_t movingTimeMs = 0;
double tripDistanceM = 0.0;
double maximumSpeedKmh = 0.0;

String previousStatus;
String previousSpeed;
String previousTrip;
String previousAverage;
String previousMaximum;
String previousTimes;
String previousGnssQuality;
String previousGnssSatellites;
String previousGnssHdop;
String previousGnssAge;
String previousGnssAltitude;
String previousGnssLatitude;
String previousGnssLongitude;
String previousAltitude;
String previousGrade;
String previousTemperature;

constexpr int16_t GAUGE_CX = 160;
constexpr int16_t GAUGE_CY = 126;
constexpr int16_t GAUGE_OUTER_RX = 154;
constexpr int16_t GAUGE_OUTER_RY = 116;
constexpr int16_t GAUGE_INNER_RX = 136;
constexpr int16_t GAUGE_INNER_RY = 94;
constexpr float GAUGE_START_DEG = 160.0f;
constexpr float GAUGE_SWEEP_DEG = 220.0f;
constexpr int GAUGE_TICK_COUNT = 40;
constexpr double GAUGE_MAX_KMH = 240.0;   // 表盘上限（公里/小时）
constexpr double GAUGE_MAX_MPH = 150.0;   // 表盘上限（英里/小时，≈241 km/h）

// Forward declarations required by the Arduino sketch preprocessor.
void resetTrip(bool showNotice);
void drawRideSummary();
void showSpeedScreen();
void drawGnssDiagnostics();
void updateGnssDiagnostics();
void drawRideHistory();
void drawMenu();
bool validFix();
String durationText(uint32_t durationMs);
String statisticSpeedText(double speedKmh);
const char *gradeStyleText();
bool recoverRideStatistics(const String &gpxName);

uint16_t backgroundColor() { return nightTheme ? TFT_BLACK : TFT_WHITE; }
uint16_t primaryColor() { return nightTheme ? TFT_WHITE : TFT_BLACK; }
uint16_t secondaryColor() { return nightTheme ? TFT_LIGHTGREY : TFT_DARKGREY; }
uint16_t accentColor() { return nightTheme ? TFT_CYAN : TFT_BLUE; }

uint16_t labelColor() { return nightTheme ? TFT_WHITE : TFT_BLACK; }

// 红区/录制指示：夜间黑底上纯红(255,0,0)偏暗，混入少量绿蓝提亮更清晰；白天白底保持纯红。
uint16_t redlineColor() { return nightTheme ? tft.color565(255, 90, 90) : TFT_RED; }

// 表盘速度分区配色：绿色（低速）、蓝色（中速）、红色（超速警告）。
// 夜间黑底用偏亮色，白天白底用偏深色，保证两种主题下都清晰。
uint16_t greenColor() { return nightTheme ? tft.color565(0, 220, 70) : tft.color565(0, 140, 40); }
uint16_t blueColor() { return nightTheme ? tft.color565(90, 160, 255) : tft.color565(20, 90, 200); }

// 速度分区：0~60 km/h 绿，60~120 km/h 蓝，>120 km/h 红（换算成显示单位比较，兼容 MPH）。
constexpr double ZONE_GREEN_MAX_KMH = 60.0;
constexpr double ZONE_BLUE_MAX_KMH = 120.0;

uint16_t gaugeZoneColor(double displaySpeed) {
  const double kmh = useMph ? displaySpeed / 0.621371192 : displaySpeed;
  if (kmh > ZONE_BLUE_MAX_KMH) return redlineColor();
  if (kmh > ZONE_GREEN_MAX_KMH) return blueColor();
  return greenColor();
}

void drawBoldText(const String &text, int16_t x, int16_t y, uint8_t font,
                  uint16_t foreground, uint16_t background) {
  // Real bold GFX fonts replace the old doubled thin bitmap text.
  const GFXfont *boldFont = font >= 4 ? &FreeSansBold12pt7b
                                     : &FreeSansBold9pt7b;
  tft.setFreeFont(boldFont);
  tft.setTextColor(foreground, background);
  tft.drawString(text, x, y);
  tft.setFreeFont(nullptr);
}

double displaySpeed(double kmh) { return useMph ? kmh * 0.621371 : kmh; }
double displayDistance(double km) { return useMph ? km * 0.621371 : km; }
const char *speedUnitText() { return useMph ? "mph" : "km/h"; }
const char *distanceUnitText() { return useMph ? "mi" : "km"; }

void saveSettings() {
  preferences.putBool("night", nightTheme);
  preferences.putBool("mph", useMph);
  preferences.putUChar("grade", gradeStyle);
}

void loadSettings() {
  preferences.begin("motonav", false);
  nightTheme = preferences.getBool("night", true);
  useMph = preferences.getBool("mph", false);
  gradeStyle = preferences.getUChar("grade", 1);
  if (gradeStyle > 2) gradeStyle = 1;
}

const char *trackStateText() {
  switch (trackState) {
    case TRACK_WAIT_FIX: return "WAIT FIX";
    case TRACK_RECORDING: return "REC";
    default: return "STOP";
  }
}

void drawTrackBadge() {
  if (uiMode != DRIVE_UI || stopRecordReminderVisible) return;
  String badge = "STOP"; uint16_t color = secondaryColor();
  if (sdError) { badge = "SD ERROR"; color = redlineColor(); }
  else if (trackState == TRACK_RECORDING) { badge = String("REC ") + String(trackPointCount); color = redlineColor(); }
  else if (trackState == TRACK_WAIT_FIX) { badge = "WAIT FIX"; color = TFT_YELLOW; }
  else if (savedNotice && millis() - savedNoticeMs < 5000UL) { badge = String("SAVED ") + String(savedPointCount); color = TFT_GREEN; }

  const bool tripLayout = screenMode == TRIP_SCREEN;
  const uint16_t cellBg = tripLayout ? (nightTheme ? tft.color565(18,28,38) : tft.color565(222,231,238)) : backgroundColor();
  const int16_t cellX = tripLayout ? 105 : 4;
  const int16_t centerX = tripLayout ? 151 : 45;
  tft.fillRect(cellX, 6, 92, 20, cellBg);
  tft.setTextDatum(MC_DATUM); tft.setTextColor(color, cellBg);
  tft.drawString(badge, centerX, 16, 2);
}

bool initializeSd() {
  if (sdReady) return true;
  sdSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sdReady = SD.begin(SD_CS_PIN, sdSpi) && SD.cardType() != CARD_NONE;
  sdError = !sdReady;
  Serial.println(sdReady ? "microSD ready" : "ERROR: microSD initialization failed");
  return sdReady;
}

// ---- 东八区（UTC+8）本地时间换算 ----
bool isLeapYear(int y) {
  return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

int daysInMonth(int y, int m) {
  static const int days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (m == 2 && isLeapYear(y)) return 29;
  return days[m - 1];
}

// 计算 (y,m,d) 相对 1970-01-01 的 civil days，用于跨午夜时的日期比较（Howard Hinnant 算法）。
long daysFromCivil(int y, int m, int d) {
  y -= (m <= 2) ? 1 : 0;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153u * (m + (m > 2 ? -3 : 9)) + 2u) / 5u + d - 1u;
  const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
  return static_cast<long>(era) * 146097L + static_cast<long>(doe) - 719468L;
}

// 日期 +1 天，处理月末/年末进位。
void addOneDay(int &y, int &mo, int &d) {
  if (++d > daysInMonth(y, mo)) {
    d = 1;
    if (++mo > 12) {
      mo = 1;
      ++y;
    }
  }
}

// GPS 输出的是 UTC 时间，这里 +8 小时换算成东八区（中国标准时间），处理跨天/跨月/跨年。
void localDateTime(int &year, int &month, int &day,
                   int &hour, int &minute, int &second) {
  constexpr int TZ_OFFSET_HOURS = 8;
  year = gps.date.year();
  month = gps.date.month();
  day = gps.date.day();
  hour = gps.time.hour() + TZ_OFFSET_HOURS;
  minute = gps.time.minute();
  second = gps.time.second();

  while (hour >= 24) {
    hour -= 24;
    day++;
    if (day > daysInMonth(year, month)) {
      day = 1;
      month++;
      if (month > 12) {
        month = 1;
        year++;
      }
    }
  }
}

String makeTrackFileName() {
  char name[32];
  if (gps.date.isValid() && gps.time.isValid()) {
    int y, mo, d, h, mi, s;
    localDateTime(y, mo, d, h, mi, s);
    snprintf(name, sizeof(name), "/TRK_%04d%02d%02d_%02d%02d%02d.GPX", y, mo, d, h, mi, s);
  } else {
    snprintf(name, sizeof(name), "/TRK_%08lu.GPX",
             static_cast<unsigned long>(millis()));
  }
  return String(name);
}

String statisticsFileName(const String &gpxName) {
  String name = gpxName;
  if (name.endsWith(".GPX")) name.remove(name.length() - 4);
  return name + ".CSV";
}

bool writeActiveTrackMarker(const String &gpxName) {
  SD.remove(ACTIVE_TRACK_MARKER);
  File marker = SD.open(ACTIVE_TRACK_MARKER, FILE_WRITE);
  if (!marker) return false;
  marker.println(gpxName);
  marker.flush();
  const bool ok = !marker.getWriteError();
  marker.close();
  return ok;
}

void clearActiveTrackMarker() {
  if (SD.exists(ACTIVE_TRACK_MARKER)) SD.remove(ACTIVE_TRACK_MARKER);
}

void clearActiveStatsFile() {
  if (SD.exists(ACTIVE_STATS_FILE)) SD.remove(ACTIVE_STATS_FILE);
}

// 记录过程中周期性快照累计统计到 SD 卡，供断电后恢复时重建 CSV 与 RIDES 列表条目。
void writeActiveStatsSnapshot() {
  if (!sdReady) return;

  char dateBuffer[11] = "0000-00-00";
  char timeBuffer[9] = "00:00:00";
  if (gps.date.isValid() && gps.time.isValid()) {
    int y, mo, d, h, mi, s;
    localDateTime(y, mo, d, h, mi, s);
    snprintf(dateBuffer, sizeof(dateBuffer), "%04d-%02d-%02d", y, mo, d);
    snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", h, mi, s);
  }
  const uint32_t totalMs = (tripActive && tripStartedMs > 0)
      ? (millis() - tripStartedMs) : 0;

  SD.remove(ACTIVE_STATS_FILE);
  File f = SD.open(ACTIVE_STATS_FILE, FILE_WRITE);
  if (!f) return;
  // 字段：distanceM,movingTimeMs,totalTimeMs,maxSpeedKmh,pointCount,date,time
  f.printf("%.3f,%lu,%lu,%.2f,%lu,%s,%s\n",
           tripDistanceM,
           static_cast<unsigned long>(movingTimeMs),
           static_cast<unsigned long>(totalMs),
           maximumSpeedKmh,
           static_cast<unsigned long>(trackPointCount),
           dateBuffer, timeBuffer);
  f.flush();
  const bool ok = !f.getWriteError();
  f.close();
  if (!ok) clearActiveStatsFile();
}

bool gpxAlreadyClosed(File &file) {
  if (!file || file.size() == 0) return false;
  const size_t tailSize = min(static_cast<size_t>(file.size()), static_cast<size_t>(96));
  if (!file.seek(file.size() - tailSize)) return false;
  String tail;
  while (file.available()) tail += static_cast<char>(file.read());
  return tail.indexOf("</gpx>") >= 0;
}

void recoverInterruptedTrack() {
  if (!sdReady || !SD.exists(ACTIVE_TRACK_MARKER)) return;

  File marker = SD.open(ACTIVE_TRACK_MARKER, FILE_READ);
  if (!marker) return;
  String interruptedName = marker.readStringUntil('\n');
  marker.close();
  interruptedName.trim();
  if (interruptedName.length() == 0 || !SD.exists(interruptedName)) {
    clearActiveTrackMarker();
    clearActiveStatsFile();
    return;
  }

  File interrupted = SD.open(interruptedName, FILE_READ);
  const bool alreadyClosed = gpxAlreadyClosed(interrupted);
  interrupted.close();

  bool recovered = alreadyClosed;
  if (!alreadyClosed) {
    interrupted = SD.open(interruptedName, FILE_APPEND);
    if (interrupted) {
      interrupted.println("  </trkseg></trk>");
      interrupted.println("</gpx>");
      interrupted.flush();
      recovered = !interrupted.getWriteError();
      interrupted.close();
    }
  }

  if (recovered) {
    clearActiveTrackMarker();
    Serial.printf("Recovered interrupted GPX: %s\n", interruptedName.c_str());
    if (recoverRideStatistics(interruptedName)) {
      Serial.printf("Recovered ride statistics: %s\n", interruptedName.c_str());
    }
  }
}

String csvField(const String &line, uint8_t index) {
  int start = 0;
  for (uint8_t field = 0; field < index; ++field) {
    start = line.indexOf(',', start);
    if (start < 0) return "";
    start++;
  }
  int end = line.indexOf(',', start);
  if (end < 0) end = line.length();
  return line.substring(start, end);
}

bool appendRideIndex(const String &dateText, const String &timeText) {
  const bool newFile = !SD.exists(RIDES_INDEX_FILE);
  File index = SD.open(RIDES_INDEX_FILE, FILE_APPEND);
  if (!index) return false;
  if (newFile) {
    index.println("version,date,time,gpx_file,distance_km,total_seconds,moving_seconds,stopped_seconds,average_kmh,maximum_kmh,gpx_points,uploaded");
  }
  index.printf("1,%s,%s,%s,%.3f,%lu,%lu,%lu,%.2f,%.2f,%lu,0\n",
               dateText.c_str(), timeText.c_str(), summaryTrackFileName.c_str(),
               summaryDistanceM / 1000.0,
               static_cast<unsigned long>(summaryTotalTimeMs / 1000UL),
               static_cast<unsigned long>(summaryMovingTimeMs / 1000UL),
               static_cast<unsigned long>(summaryStoppedTimeMs / 1000UL),
               summaryAverageSpeedKmh, summaryMaximumSpeedKmh,
               static_cast<unsigned long>(summaryPointCount));
  index.flush();
  const bool ok = !index.getWriteError();
  index.close();
  return ok;
}

bool writeRideStatistics(const String &dateText, const String &timeText) {
  const String csvName = statisticsFileName(summaryTrackFileName);

  SD.remove(csvName);
  File stats = SD.open(csvName, FILE_WRITE);
  if (!stats) return false;
  stats.println("version,date,time,gpx_file,distance_km,total_seconds,moving_seconds,stopped_seconds,average_kmh,maximum_kmh,gpx_points,completed");
  stats.printf("1,%s,%s,%s,%.3f,%lu,%lu,%lu,%.2f,%.2f,%lu,1\n",
               dateText.c_str(), timeText.c_str(), summaryTrackFileName.c_str(),
               summaryDistanceM / 1000.0,
               static_cast<unsigned long>(summaryTotalTimeMs / 1000UL),
               static_cast<unsigned long>(summaryMovingTimeMs / 1000UL),
               static_cast<unsigned long>(summaryStoppedTimeMs / 1000UL),
               summaryAverageSpeedKmh, summaryMaximumSpeedKmh,
               static_cast<unsigned long>(summaryPointCount));
  stats.flush();
  const bool statsOk = !stats.getWriteError();
  stats.close();
  return statsOk && appendRideIndex(dateText, timeText);
}

bool saveRideStatistics() {
  if (!sdReady || summaryTrackFileName.length() == 0) return false;

  char dateBuffer[11] = "0000-00-00";
  char timeBuffer[9] = "00:00:00";
  if (gps.date.isValid() && gps.time.isValid()) {
    int y, mo, d, h, mi, s;
    localDateTime(y, mo, d, h, mi, s);
    snprintf(dateBuffer, sizeof(dateBuffer), "%04d-%02d-%02d", y, mo, d);
    snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", h, mi, s);
  }
  return writeRideStatistics(String(dateBuffer), String(timeBuffer));
}

// 断电后：从快照文件重建统计，生成 CSV 并补进 RIDES 列表条目。
bool recoverRideStatistics(const String &gpxName) {
  if (!sdReady || !SD.exists(ACTIVE_STATS_FILE)) return false;
  File f = SD.open(ACTIVE_STATS_FILE, FILE_READ);
  if (!f) return false;
  String line = f.readStringUntil('\n');
  f.close();
  line.trim();
  if (line.length() == 0) { clearActiveStatsFile(); return false; }

  // 字段：distanceM,movingTimeMs,totalTimeMs,maxSpeedKmh,pointCount,date,time
  const double distanceM = csvField(line, 0).toDouble();
  const uint32_t movingMs = static_cast<uint32_t>(csvField(line, 1).toInt());
  const uint32_t totalMs = static_cast<uint32_t>(csvField(line, 2).toInt());
  const double maxKmh = csvField(line, 3).toDouble();
  const uint32_t points = static_cast<uint32_t>(csvField(line, 4).toInt());
  const String dateText = csvField(line, 5);
  const String timeText = csvField(line, 6);

  summaryTrackFileName = gpxName;
  summaryDistanceM = distanceM;
  summaryTotalTimeMs = totalMs;
  summaryMovingTimeMs = min(movingMs, totalMs);
  summaryStoppedTimeMs = summaryTotalTimeMs > summaryMovingTimeMs
      ? summaryTotalTimeMs - summaryMovingTimeMs : 0;
  summaryAverageSpeedKmh = summaryMovingTimeMs > 0
      ? (distanceM / 1000.0) / (summaryMovingTimeMs / 3600000.0) : 0.0;
  summaryMaximumSpeedKmh = maxKmh;
  summaryPointCount = points;

  const bool ok = writeRideStatistics(dateText, timeText);
  clearActiveStatsFile();
  return ok;
}

bool parseRideIndexLine(const String &line, RideRecord &ride) {
  if (line.length() < 10 || line.startsWith("version")) return false;
  ride.date = csvField(line, 1);
  ride.time = csvField(line, 2);
  ride.gpxFile = csvField(line, 3);
  ride.distanceKm = csvField(line, 4).toDouble();
  ride.totalSeconds = static_cast<uint32_t>(csvField(line, 5).toInt());
  ride.movingSeconds = static_cast<uint32_t>(csvField(line, 6).toInt());
  ride.stoppedSeconds = static_cast<uint32_t>(csvField(line, 7).toInt());
  ride.averageKmh = csvField(line, 8).toDouble();
  ride.maximumKmh = csvField(line, 9).toDouble();
  ride.points = static_cast<uint32_t>(csvField(line, 10).toInt());
  ride.uploaded = static_cast<uint8_t>(constrain(csvField(line, 11).toInt(), 0, 2));
  return ride.gpxFile.length() > 0;
}

void loadRideHistory() {
  rideHistoryCount = 0;
  selectedRideIndex = 0;
  if (!initializeSd() || !SD.exists(RIDES_INDEX_FILE)) return;
  File index = SD.open(RIDES_INDEX_FILE, FILE_READ);
  if (!index) return;
  while (index.available()) {
    String line = index.readStringUntil('\n');
    line.trim();
    RideRecord ride;
    if (!parseRideIndexLine(line, ride)) continue;
    if (rideHistoryCount < MAX_RIDE_HISTORY) {
      rideHistory[rideHistoryCount++] = ride;
    } else {
      for (uint8_t i = 1; i < MAX_RIDE_HISTORY; ++i) {
        rideHistory[i - 1] = rideHistory[i];
      }
      rideHistory[MAX_RIDE_HISTORY - 1] = ride;
    }
  }
  index.close();
  if (rideHistoryCount > 0) selectedRideIndex = rideHistoryCount - 1;
}

bool openTrackFile() {
  sdError = false;
  if (!initializeSd()) {
    drawTrackBadge();
    return false;
  }
  trackFileName = makeTrackFileName();
  trackFile = SD.open(trackFileName, FILE_WRITE);
  if (!trackFile) {
    sdError = true;
    drawTrackBadge();
    return false;
  }
  trackFile.println("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
  trackFile.println("<gpx version=\"1.1\" creator=\"MotoNav-CYD-Plus\" xmlns=\"http://www.topografix.com/GPX/1/1\">");
  trackFile.println("  <trk><name>MotoNav ride</name><trkseg>");
  trackFile.flush();
  if (trackFile.getWriteError() || !writeActiveTrackMarker(trackFileName)) {
    trackFile.close();
    SD.remove(trackFileName);
    clearActiveTrackMarker();
    sdError = true;
    drawTrackBadge();
    return false;
  }
  lastTrackFlushMs = millis();
  lastTrackPointMs = 0;
  havePreviousTrackPosition = false;
  gpsDateTracked = false;
  trackPointCount = 0;
  savedNotice = false;
  lastStatsSnapshotMs = millis();
  writeActiveStatsSnapshot();
  return true;
}

void startTrack() {
  if (trackState != TRACK_STOPPED) return;
  sdError = false;
  savedNotice = false;
  resetTrip(false);
  if (!validFix()) {
    trackState = TRACK_WAIT_FIX;
  } else {
    trackState = openTrackFile() ? TRACK_RECORDING : TRACK_STOPPED;
  }
  drawTrackBadge();
}

void finishTrack() {
  const uint32_t finishMs = millis();
  summaryTrackFileName = trackFileName;
  summaryDistanceM = tripDistanceM;
  summaryTotalTimeMs = tripStartedMs > 0 ? finishMs - tripStartedMs : 0;
  summaryMovingTimeMs = min(movingTimeMs, summaryTotalTimeMs);
  summaryStoppedTimeMs = summaryTotalTimeMs - summaryMovingTimeMs;
  summaryAverageSpeedKmh = summaryMovingTimeMs > 0
      ? (summaryDistanceM / 1000.0) / (summaryMovingTimeMs / 3600000.0)
      : 0.0;
  summaryMaximumSpeedKmh = maximumSpeedKmh;
  summaryPointCount = trackPointCount;

  bool gpxSavedOk = false;
  if (trackFile) {
    trackFile.println("  </trkseg></trk>");
    trackFile.println("</gpx>");
    trackFile.flush();
    gpxSavedOk = !trackFile.getWriteError();
    trackFile.close();
  }
  if (gpxSavedOk) {
    clearActiveTrackMarker();
    clearActiveStatsFile();
  }

  const bool statisticsSavedOk = gpxSavedOk && saveRideStatistics();
  summarySavedOk = gpxSavedOk && statisticsSavedOk;
  savedPointCount = trackPointCount;
  savedNotice = summarySavedOk;
  savedNoticeMs = finishMs;
  sdError = !summarySavedOk && trackPointCount > 0;
  trackState = TRACK_STOPPED;
  tripActive = false;
  tripFinishedMs = finishMs;
  lastTrackPointMs = 0;
  drawTrackBadge();
  Upload.requestScan();   // 行程结束，若联网且开关开启则触发自动上传
}

bool trackPointQualityOk(uint32_t now, double latitude, double longitude) {
  if (gps.hdop.isValid() && gps.hdop.hdop() > MAX_HDOP_FOR_TRACK) return false;
  if (gps.satellites.isValid() &&
      gps.satellites.value() < MIN_SATELLITES_FOR_TRACK) return false;
  if (!havePreviousTrackPosition) return true;

  const uint32_t gapMs = now - previousTrackPositionMs;
  const double segmentM = TinyGPSPlus::distanceBetween(
      previousTrackLatitude, previousTrackLongitude, latitude, longitude);
  const double impliedKmh = gapMs > 0 ? segmentM * 3600.0 / gapMs : 9999.0;
  if (gapMs > MAX_TRACK_POINT_GAP_MS) {
    previousTrackLatitude = latitude;
    previousTrackLongitude = longitude;
    previousTrackPositionMs = now;
    return false;
  }
  if (segmentM < MIN_TRACK_POINT_DISTANCE_M) return false;
  if (segmentM > MAX_TRACK_SEGMENT_DISTANCE_M) return false;
  return impliedKmh <= MAX_VALID_SPEED_KMH;
}

// 返回可安全写入 GPX <ele> 的合理海拔（米）。数值异常（NaN/Inf/超出合理范围）返回 false。
bool gpxAltitudeMeters(double &out) {
  if (baroHealthy() && baroAnchorSet) {
    const double v = baroAltitudeM + baroOffsetM;
    if (isfinite(v) && v > ALT_MIN_M && v < ALT_MAX_M) { out = v; return true; }
  }
  if (gps.altitude.isValid()) {
    const double v = gps.altitude.meters();
    if (isfinite(v) && v > ALT_MIN_M && v < ALT_MAX_M) { out = v; return true; }
  }
  return false;
}

void writeTrackPoint(bool locationUpdated, double latitude, double longitude) {
  if (trackState == TRACK_WAIT_FIX && validFix()) {
    trackState = openTrackFile() ? TRACK_RECORDING : TRACK_STOPPED;
    havePreviousTrackPosition = false;
    drawTrackBadge();
  }
  if (trackState != TRACK_RECORDING || !validFix()) return;

  const uint32_t now = millis();
  if (!locationUpdated ||
      now - lastTrackPointMs + 100UL < TRACK_POINT_INTERVAL_MS ||
      !trackPointQualityOk(now, latitude, longitude)) return;

  trackFile.printf("    <trkpt lat=\"%.7f\" lon=\"%.7f\">\n",
                   latitude, longitude);
  // 海拔：优先用「GNSS 锚定 + 气压相对变化」的融合值（更平滑），
  // 未锚定时回退到纯 GNSS 海拔；数值异常（NaN/离谱）则不写 <ele>，保证 GPX 合法。
  double eleM;
  if (gpxAltitudeMeters(eleM)) {
    trackFile.printf("      <ele>%.2f</ele>\n", eleM);
  }
  if (gps.date.isValid() && gps.time.isValid()) {
    const uint32_t sod =
        static_cast<uint32_t>(gps.time.hour()) * 3600UL +
        static_cast<uint32_t>(gps.time.minute()) * 60UL +
        static_cast<uint32_t>(gps.time.second());
    if (!gpsDateTracked) {
      gpsTrackYear = gps.date.year();
      gpsTrackMonth = gps.date.month();
      gpsTrackDay = gps.date.day();
      gpsDateTracked = true;
    } else {
      // UTC 秒数回绕 ⇒ 跨过午夜，日期 +1 天（含月末/年末进位）。
      if (sod < lastGpsSecondsOfDay) {
        addOneDay(gpsTrackYear, gpsTrackMonth, gpsTrackDay);
      }
      // 若 GPS 的 date 已追平/前进（例如长时间失锁后恢复），以 GPS 为准。
      if (daysFromCivil(gps.date.year(), gps.date.month(), gps.date.day()) >
          daysFromCivil(gpsTrackYear, gpsTrackMonth, gpsTrackDay)) {
        gpsTrackYear = gps.date.year();
        gpsTrackMonth = gps.date.month();
        gpsTrackDay = gps.date.day();
      }
    }
    lastGpsSecondsOfDay = sod;
    trackFile.printf("      <time>%04d-%02d-%02dT%02d:%02d:%02dZ</time>\n",
                     gpsTrackYear, gpsTrackMonth, gpsTrackDay,
                     gps.time.hour(), gps.time.minute(), gps.time.second());
  }
  if (gps.hdop.isValid()) {
    trackFile.printf("      <hdop>%.2f</hdop>\n", gps.hdop.hdop());
  }
  if (gps.satellites.isValid()) {
    trackFile.printf("      <sat>%lu</sat>\n",
                     static_cast<unsigned long>(gps.satellites.value()));
  }
  trackFile.println("    </trkpt>");
  if (trackFile.getWriteError()) {
    trackFile.close(); sdReady = false; sdError = true;
    trackState = TRACK_STOPPED; drawTrackBadge(); return;
  }

  // 距离累加：按「实际写入 GPX 的点」累计，保证屏显 / CSV / 网站与 GPX 实测距离一致。
  if (havePreviousTrackPosition) {
    tripDistanceM += TinyGPSPlus::distanceBetween(
        previousTrackLatitude, previousTrackLongitude, latitude, longitude);
  }

  previousTrackLatitude = latitude;
  previousTrackLongitude = longitude;
  previousTrackPositionMs = now;
  havePreviousTrackPosition = true;
  lastTrackPointMs = now;
  trackPointCount++;
  drawTrackBadge();

  if (now - lastTrackFlushMs >= TRACK_FLUSH_INTERVAL_MS) {
    trackFile.flush();
    if (trackFile.getWriteError()) {
      trackFile.close(); sdReady = false; sdError = true;
      trackState = TRACK_STOPPED; drawTrackBadge(); return;
    }
    lastTrackFlushMs = now;
  }
}

void drawMenuButton(int16_t x, int16_t y, const String &title,
                    const String &subtitle, uint16_t color,
                    const String &hint = "") {
  const uint16_t tile = nightTheme ? tft.color565(20, 27, 35) : TFT_WHITE;
  const uint16_t titleColor = nightTheme ? TFT_WHITE : tft.color565(18, 28, 38);
  const uint16_t subtitleColor = nightTheme ? tft.color565(150, 164, 178)
                                            : tft.color565(82, 96, 110);

  if (!nightTheme) {
    tft.fillRoundRect(x + 2, y + 3, 146, 78, 10,
                      tft.color565(205, 214, 222));
  }
  tft.fillRoundRect(x, y, 146, 78, 10, tile);
  tft.fillRoundRect(x, y, 6, 78, 3, color);

  // 标题自适应降字号：用矢量粗体测宽，超出可用宽度则改用内置 16px 字体，避免右侧溢出
  const int16_t titleMaxWidth = 146 - 18 - 10;   // 左内边距 18 + 右安全边距 10
  tft.setFreeFont(&FreeSansBold9pt7b);
  const int16_t boldWidth = tft.textWidth(title);
  tft.setFreeFont(nullptr);

  // Match the glyph background to the tile. The old screen background here
  // caused visible rectangular boxes around menu text in daylight mode.
  tft.setTextDatum(TL_DATUM);
  if (boldWidth <= titleMaxWidth) {
    drawBoldText(title, x + 18, y + 18, 2, titleColor, tile);
  } else {
    tft.setTextColor(titleColor, tile);
    tft.drawString(title, x + 18, y + 20, 2);   // 降级为内置 font 2（16px），完整显示
  }
  tft.setTextColor(subtitleColor, tile);
  tft.drawString(subtitle, x + 18, y + 51, 2);
  if (hint.length() > 0) {           // 可选第三行小字提示
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(subtitleColor, tile);
    tft.drawString(hint, x + 18, y + 68, 1);
  }
}

const char *uploadStateText(uint8_t v) {
  switch (v) {
    case 1: return "UPLOADED";
    case 2: return "SKIPPED";
    default: return "PENDING";
  }
}

uint16_t uploadStateColor(uint8_t v) {
  switch (v) {
    case 1: return TFT_GREEN;
    case 2: return TFT_ORANGE;
    default: return TFT_CYAN;
  }
}

void drawSettings() {
  uiMode = SETTINGS_UI;
  const uint16_t bg = nightTheme ? TFT_BLACK : tft.color565(235, 240, 244);
  tft.fillScreen(bg);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(nightTheme ? TFT_WHITE : tft.color565(18, 28, 38), bg);
  tft.drawString(settingsPage2 ? "NETWORK" : "SETTINGS", 10, 7, 4);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(accentColor(), bg);
  tft.drawString(settingsPage2 ? "2 / 2" : "1 / 2", 310, 11, 2);

  if (!settingsPage2) {
    drawMenuButton(8, 38, "THEME", nightTheme ? "NIGHT" : "DAY", TFT_ORANGE);
    drawMenuButton(166, 38, "UNITS", useMph ? "MPH / MI" : "KM/H / KM", TFT_GREEN);
    drawMenuButton(8, 124, "GRADE", gradeStyleText(), TFT_CYAN);
    drawMenuButton(166, 124, "PAGE 2", "WIFI / UPLOAD", TFT_BLUE);
    tft.setTextDatum(BC_DATUM);
    tft.setTextColor(secondaryColor(), bg);
    tft.drawString("SETTINGS ARE SAVED", 160, 220, 2);
    tft.drawString("TITLE: MOTONAV", 160, 239, 1);
    return;
  }

  // 第 2 页：网络 / 上传
  drawMenuButton(8, 38, "WIFI", motoNet.statusText(),
                 motoNet.connected() ? TFT_GREEN : TFT_BLUE);
  drawMenuButton(166, 38, "AUTO UPLOAD", Upload.enabled() ? "ON" : "OFF",
                 Upload.enabled() ? TFT_GREEN : secondaryColor());
  drawMenuButton(8, 124, "PENDING", String(Upload.pendingCount()) + " GPX",
                 TFT_ORANGE, "TAP TO UPLOAD");
  drawMenuButton(166, 124, "BACK", "PAGE 1", secondaryColor());

  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(secondaryColor(), bg);
  if (motoNet.connected()) {
    tft.drawString(String("WIFI ") + motoNet.ssid(), 160, 220, 2);
    tft.drawString(String("IP ") + motoNet.ipText(), 160, 237, 2);
  } else if (motoNet.apMode()) {
    tft.drawString(String("HOTSPOT: ") + WIFI_AP_SSID, 160, 220, 2);
    tft.drawString("CONNECT PHONE, THEN BROWSE", 160, 237, 1);
  } else {
    tft.drawString(motoNet.statusText(), 160, 220, 2);
    tft.drawString("HOLD: MENU", 160, 237, 1);
  }
}

void drawMenu() {
  uiMode = MENU_UI;
  const uint16_t bg = nightTheme ? TFT_BLACK : tft.color565(235, 240, 244);
  tft.fillScreen(bg);
  tft.setTextDatum(TL_DATUM);
  drawBoldText("MENU", 10, 17, 4, labelColor(), bg);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(accentColor(), bg);
  tft.drawString("V1.2.0", 310, 11, 2);

  const String trackAction = trackState == TRACK_STOPPED ? "START TRACK" :
                             trackState == TRACK_WAIT_FIX ? "CANCEL / WAIT FIX" :
                             String("FINISH / ") + String(trackPointCount) + " PTS";
  drawMenuButton(8, 38, "TRACK", trackAction,
                 trackState == TRACK_RECORDING ? TFT_RED : accentColor());
  drawMenuButton(166, 38, "RIDES", "SAVED TRIPS", TFT_GREEN);
  drawMenuButton(8, 124, "GNSS", "LIVE DIAGNOSTICS", TFT_ORANGE);
  drawMenuButton(166, 124, "SETTINGS", "THEME / UNITS", secondaryColor());

  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(secondaryColor(), bg);
  tft.drawString("HOLD OR TAP TITLE TO RETURN", 160, 239, 1);
}

String gnssQualityText() {
  if (!validFix()) return "NO FIX";
  const bool satellitesOk = !gps.satellites.isValid() ||
                            gps.satellites.value() >= MIN_SATELLITES_FOR_TRACK;
  const bool hdopOk = !gps.hdop.isValid() ||
                      gps.hdop.hdop() <= MAX_HDOP_FOR_DISTANCE;
  return satellitesOk && hdopOk ? "GOOD" : "WEAK";
}

void drawDiagnosticCard(int16_t x, int16_t y, int16_t w,
                        const String &label, const String &value,
                        uint16_t accent) {
  const uint16_t panel = nightTheme ? tft.color565(18,28,38)
                                    : tft.color565(222,231,238);
  const uint16_t border = nightTheme ? tft.color565(55,78,94)
                                     : tft.color565(165,181,192);
  tft.fillRoundRect(x,y,w,48,7,panel);
  tft.drawRoundRect(x,y,w,48,7,border);
  tft.fillRoundRect(x+4,y+4,5,40,2,accent);
  tft.setTextDatum(TL_DATUM);
  drawBoldText(label,x+15,y+13,2,labelColor(),panel);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(primaryColor(),panel);
  tft.drawString(value,x+w-8,y+28,2);
}

void drawGnssCardFrame(int16_t x, int16_t y, int16_t w,
                       const String &label, uint16_t accent) {
  const uint16_t panel = nightTheme ? tft.color565(18,28,38)
                                    : tft.color565(222,231,238);
  const uint16_t border = nightTheme ? tft.color565(55,78,94)
                                     : tft.color565(165,181,192);
  tft.fillRoundRect(x,y,w,48,7,panel);
  tft.drawRoundRect(x,y,w,48,7,border);
  tft.fillRoundRect(x+4,y+4,5,40,2,accent);
  tft.setTextDatum(TC_DATUM);
  drawBoldText(label,x+w/2+3,y+4,2,labelColor(),panel);
}

void updateGnssCardValue(int16_t x, int16_t y, int16_t w,
                         const String &value, String &previous) {
  if (value == previous) return;
  const uint16_t panel = nightTheme ? tft.color565(18,28,38)
                                    : tft.color565(222,231,238);
  tft.fillRect(x+12,y+25,w-18,19,panel);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(primaryColor(),panel);
  tft.drawString(value,x+w/2+3,y+34,2);
  previous = value;
}

void updateGnssDiagnostics() {
  if (uiMode != GNSS_UI) return;

  const String quality = gnssQualityText();
  const uint16_t qualityColor = quality=="GOOD" ? TFT_GREEN :
                                quality=="WEAK" ? TFT_ORANGE : TFT_RED;
  if (quality != previousGnssQuality) {
    tft.fillRoundRect(6,4,308,30,7,qualityColor);
    const uint16_t headerText = quality=="GOOD" ? TFT_BLACK : TFT_WHITE;
    tft.setTextDatum(ML_DATUM);
    drawBoldText("GNSS",14,19,4,headerText,qualityColor);
    tft.setTextDatum(MR_DATUM);
    drawBoldText(quality,306,19,2,headerText,qualityColor);
    previousGnssQuality = quality;
  }

  const String sats = gps.satellites.isValid() ? String(gps.satellites.value()) : "--";
  const String hdop = gps.hdop.isValid() ? String(gps.hdop.hdop(),1) : "--";
  const String age = gps.location.isValid() ? String(gps.location.age())+" ms" : "--";
  const String altitude = gps.altitude.isValid() ? String(gps.altitude.meters(),1)+" m" : "--";
  updateGnssCardValue(6,40,151,sats,previousGnssSatellites);
  updateGnssCardValue(163,40,151,hdop,previousGnssHdop);
  updateGnssCardValue(6,93,151,age,previousGnssAge);
  updateGnssCardValue(163,93,151,altitude,previousGnssAltitude);

  const String latitude = gps.location.isValid() ? String(gps.location.lat(),6) : "--";
  const String longitude = gps.location.isValid() ? String(gps.location.lng(),6) : "--";
  const uint16_t panel = nightTheme ? tft.color565(18,28,38)
                                    : tft.color565(222,231,238);
  if (latitude != previousGnssLatitude) {
    tft.fillRect(70,151,234,20,panel);
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(primaryColor(),panel);
    tft.drawString(latitude,304,161,2);
    previousGnssLatitude = latitude;
  }
  if (longitude != previousGnssLongitude) {
    tft.fillRect(70,175,234,20,panel);
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(primaryColor(),panel);
    tft.drawString(longitude,304,185,2);
    previousGnssLongitude = longitude;
  }
}

void drawGnssDiagnostics() {
  uiMode = GNSS_UI;
  const uint16_t bg = backgroundColor();
  const uint16_t panel = nightTheme ? tft.color565(18,28,38)
                                    : tft.color565(222,231,238);
  const uint16_t border = nightTheme ? tft.color565(55,78,94)
                                     : tft.color565(165,181,192);

  // Clear and draw the fixed layout only once when entering this screen.
  tft.fillScreen(bg);
  drawGnssCardFrame(6,40,151,"SATELLITES",TFT_GREEN);
  drawGnssCardFrame(163,40,151,"HDOP",TFT_ORANGE);
  drawGnssCardFrame(6,93,151,"FIX AGE",TFT_CYAN);
  drawGnssCardFrame(163,93,151,"ALTITUDE",TFT_BLUE);

  tft.fillRoundRect(6,146,308,66,7,panel);
  tft.drawRoundRect(6,146,308,66,7,border);
  tft.setTextDatum(ML_DATUM);
  drawBoldText("LAT",16,161,2,labelColor(),panel);
  drawBoldText("LON",16,185,2,labelColor(),panel);
  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(secondaryColor(),bg);
  tft.drawString("TAP: MENU",160,238,1);

  previousGnssQuality = "";
  previousGnssSatellites = "";
  previousGnssHdop = "";
  previousGnssAge = "";
  previousGnssAltitude = "";
  previousGnssLatitude = "";
  previousGnssLongitude = "";
  updateGnssDiagnostics();
  lastGnssUiMs = millis();
}

void drawRideHistory() {
  uiMode = RIDES_UI;
  const uint16_t bg=backgroundColor(),panel=nightTheme?tft.color565(18,28,38):tft.color565(222,231,238);
  tft.fillScreen(bg);
  tft.setTextDatum(ML_DATUM);
  drawBoldText("RIDES",10,18,4,labelColor(),bg);

  if(rideHistoryCount==0){
    tft.setTextDatum(MC_DATUM);
    drawBoldText("NO SAVED RIDES",160,112,2,labelColor(),bg);
    tft.setTextColor(secondaryColor(),bg);tft.drawString("Finish a GPX track first",160,144,2);
  } else {
    const RideRecord &ride=rideHistory[selectedRideIndex];
    tft.setTextDatum(MR_DATUM);tft.setTextColor(accentColor(),bg);
    tft.drawString(String(selectedRideIndex+1)+"/"+String(rideHistoryCount),310,12,2);
    tft.fillRoundRect(6,38,308,48,7,panel);
    tft.setTextDatum(ML_DATUM);drawBoldText(ride.date,16,57,2,labelColor(),panel);
    tft.setTextDatum(MR_DATUM);drawBoldText(ride.time,304,57,2,labelColor(),panel);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(uploadStateColor(ride.uploaded), panel);
    tft.drawString(uploadStateText(ride.uploaded),160,79,2);

    drawDiagnosticCard(6,92,151,"DISTANCE",String(displayDistance(ride.distanceKm),2)+" "+distanceUnitText(),TFT_GREEN);
    drawDiagnosticCard(163,92,151,"TOTAL",durationText(ride.totalSeconds*1000UL),TFT_CYAN);
    drawDiagnosticCard(6,145,98,"AVG",statisticSpeedText(ride.averageKmh),TFT_CYAN);
    drawDiagnosticCard(111,145,98,"MAX",statisticSpeedText(ride.maximumKmh),TFT_ORANGE);
    drawDiagnosticCard(216,145,98,"POINTS",String(ride.points),TFT_GREEN);
  }
  tft.setTextDatum(BC_DATUM);tft.setTextColor(secondaryColor(),bg);
  tft.drawString("L: PREV   R: NEXT   HOLD: SKIP   TITLE: MENU",160,239,1);
}


void closeMenu() {
  uiMode = DRIVE_UI;
  settingsPage2 = false;
  redrawTheme();
  drawTrackBadge();
}

void drawStopRecordReminder() {
  stopRecordReminderVisible = true;
  tft.fillScreen(TFT_BLACK);

  // The whole red panel is one large, unmistakable STOP button.
  tft.fillRoundRect(12, 20, 296, 194, 18, TFT_RED);
  tft.drawRoundRect(12, 20, 296, 194, 18, TFT_WHITE);
  tft.drawRoundRect(14, 22, 292, 190, 16, TFT_WHITE);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_RED);
  tft.drawString("STOP", 160, 73, 4);
  tft.drawString("RECORD?", 160, 125, 4);
  tft.drawString("HOLD TO STOP", 160, 168, 2);
  tft.drawString("TAP TO RETURN", 160, 192, 2);
}

void hideStopRecordReminder() {
  if (!stopRecordReminderVisible) return;
  stopRecordReminderVisible = false;
  redrawTheme();
  drawTrackBadge();
}

void updateSafetyUi() {
  const uint32_t now = millis();
  const double speed = filteredSpeedKmh(validFix());

  if ((uiMode == MENU_UI || uiMode == SETTINGS_UI || uiMode == GNSS_UI) &&
      speed >= MENU_AUTO_CLOSE_SPEED_KMH) {
    closeMenu();
  }

  if ((uiMode == SUMMARY_UI || uiMode == RIDES_UI) &&
      speed >= MENU_AUTO_CLOSE_SPEED_KMH) {
    showSpeedScreen();
  }

  if (trackState != TRACK_RECORDING) {
    trackStoppedStartedMs = 0;
    hideStopRecordReminder();
    return;
  }

  if (speed > STOP_REMINDER_CLEAR_KMH) {
    trackStoppedStartedMs = 0;
    hideStopRecordReminder();
    return;
  }

  if (speed <= STOP_REMINDER_SPEED_KMH) {
    if (trackStoppedStartedMs == 0) trackStoppedStartedMs = now;
    if (!stopRecordReminderVisible &&
        uiMode == DRIVE_UI &&
        now - trackStoppedStartedMs >= STOP_REMINDER_DELAY_MS) {
      drawStopRecordReminder();
    }
  } else {
    trackStoppedStartedMs = 0;
  }
}

void handleMenuTap(int16_t x, int16_t y) {
  if (y < 38) { closeMenu(); return; }   // 标题栏点按 MENU：返回行程页
  if (y >= 38 && y <= 116 && x < 160) {
    if (trackState == TRACK_STOPPED) { startTrack(); drawMenu(); }
    else if (trackState == TRACK_WAIT_FIX) {
      trackState = TRACK_STOPPED; resetTrip(false); drawMenu();
    } else { finishTrack(); drawRideSummary(); }
  } else if (y >= 38 && y <= 116) {
    loadRideHistory(); drawRideHistory();
  } else if (y >= 124 && y <= 202 && x < 160) {
    drawGnssDiagnostics();
  } else if (y >= 124 && y <= 202) {
    drawSettings();
  }
}

void handleRideHistoryTap(int16_t x, int16_t y) {
  if (y < 38) { drawMenu(); return; }   // 标题栏点按：返回菜单
  if (rideHistoryCount == 0) { drawMenu(); return; }
  if (x < 160) {
    selectedRideIndex = selectedRideIndex == 0
        ? rideHistoryCount - 1 : selectedRideIndex - 1;
  } else {
    selectedRideIndex = (selectedRideIndex + 1) % rideHistoryCount;
  }
  drawRideHistory();
}

// RIDES 历史长按：在当前行程「已忽略 / 待上传」间切换并写回 CSV
void toggleRideUploadIgnore() {
  if (rideHistoryCount == 0) { drawMenu(); return; }
  RideRecord &ride = rideHistory[selectedRideIndex];
  ride.uploaded = (ride.uploaded == 2) ? 0 : 2;
  Upload.markUploaded(ride.gpxFile, ride.uploaded);
  drawRideHistory();
}

void handleSettingsTap(int16_t x, int16_t y) {
  if (y < 38) { closeMenu(); return; }   // 标题栏点按：返回行程屏 (MOTONAV)
  if (settingsPage2) {
    if (y >= 38 && y <= 116 && x < 160) {
      // WIFI：未连接时进入无密码配网热点
      if (!motoNet.connected()) motoNet.startConfigPortal();
      drawSettings();
    } else if (y >= 38 && y <= 116) {
      // 自动上传开关
      Upload.setEnabled(!Upload.enabled());
      drawSettings();
    } else if (y >= 124 && y <= 202 && x < 160) {
      // 立即手动上传（绕过自动上传开关，仅需联网）
      Upload.requestManualUpload();
      drawSettings();
    } else if (y >= 124 && y <= 202) {
      settingsPage2 = false;
      drawSettings();
    }
    return;
  }

  if (y >= 38 && y <= 116 && x < 160) {
    nightTheme = !nightTheme;
    saveSettings();
    drawSettings();
  } else if (y >= 38 && y <= 116) {
    useMph = !useMph;
    saveSettings();
    drawSettings();
  } else if (y >= 124 && y <= 202 && x < 160) {
    // 坡度显示风格：百分比 → 角度 → 关闭，循环切换
    gradeStyle = (gradeStyle + 1) % 3;
    saveSettings();
    drawSettings();
  } else if (y >= 124 && y <= 202) {
    settingsPage2 = true;
    drawSettings();
  }
}

bool nmeaActive() {
  return nmeaChars > 0 && millis() - lastNmeaByteMs < GNSS_DATA_TIMEOUT_MS;
}

bool validFix() {
  return gps.location.isValid() && gps.location.age() < GNSS_DATA_TIMEOUT_MS;
}

void updateSpeedFilter() {
  if (!gps.speed.isUpdated() || !gps.speed.isValid()) return;
  const uint32_t now = millis();
  const double raw = gps.speed.kmph();
  if (raw > MAX_VALID_SPEED_KMH) return;

  if (!speedFilterReady) {
    smoothedSpeedKmh = raw;
    lastAcceptedRawSpeedKmh = raw;
    lastSpeedFilterMs = now;
    speedFilterReady = true;
    return;
  }

  const double measuredElapsedS = (now - lastSpeedFilterMs) / 1000.0;
  const double elapsedS = measuredElapsedS < 0.2 ? 0.2 : measuredElapsedS;
  const double allowedJump = MAX_SPEED_JUMP_KMH_PER_S * elapsedS + 5.0;
  if (fabs(raw - lastAcceptedRawSpeedKmh) > allowedJump) return;

  // 静止态：需连续多次超过恢复阈值才退出静止，避免单点 GNSS 速度毛刺误触发
  if (speedStaticActive) {
    if (raw > SPEED_STOP_RESUME_KMH) {
      if (++speedStaticResumeCount >= SPEED_STOP_RESUME_SAMPLES) {
        speedStaticActive = false;
        speedStaticResumeCount = 0;
        smoothedSpeedKmh = raw;        // 去抖通过，从当前速度平滑起步，避免从 0 突跳
        lastAcceptedRawSpeedKmh = raw;
        lastSpeedFilterMs = now;
      } else {
        smoothedSpeedKmh = 0.0;        // 仍在去抖计数中，继续保持归零
        lastSpeedFilterMs = now;
      }
    } else {
      speedStaticResumeCount = 0;      // 未达恢复阈值，复位去抖计数
      smoothedSpeedKmh = 0.0;
      lastSpeedFilterMs = now;
    }
    return;
  }

  // 非静止态：检测进入静止（连续低速达到 SPEED_STOP_HOLD_MS 即判定停车）
  if (raw < SPEED_STOP_THRESHOLD_KMH) {
    if (speedStaticCandidateMs == 0) {
      speedStaticCandidateMs = now;
    } else if (now - speedStaticCandidateMs >= SPEED_STOP_HOLD_MS) {
      speedStaticActive = true;
      speedStaticCandidateMs = 0;
      speedStaticResumeCount = 0;
      smoothedSpeedKmh = 0.0;
      lastAcceptedRawSpeedKmh = raw;
      lastSpeedFilterMs = now;
      return;
    }
    // 未满持续时长：继续用快速衰减的低通往下压
  } else {
    speedStaticCandidateMs = 0;        // 又动起来了，复位候选计时
  }

  // 非对称 EMA：减速方向用更快的系数，让速度更快归零/跟手
  const double alpha = raw < smoothedSpeedKmh
      ? SPEED_FILTER_ALPHA_FALL
      : (raw < 8.0 ? SPEED_FILTER_ALPHA_LOW : SPEED_FILTER_ALPHA_NORMAL);
  smoothedSpeedKmh += alpha * (raw - smoothedSpeedKmh);
  lastAcceptedRawSpeedKmh = raw;
  lastSpeedFilterMs = now;
}

double filteredSpeedKmh(bool fix) {
  if (!fix || !speedFilterReady) return 0.0;
  return smoothedSpeedKmh < SPEED_NOISE_FLOOR_KMH ? 0.0 : smoothedSpeedKmh;
}

String gnssStatus(bool fix) {
  if (fix) return "GNSS FIX";
  return nmeaActive() ? "SEARCHING" : "NO DATA";
}

String speedText(bool fix) {
  const double speed = displaySpeed(filteredSpeedKmh(fix));
  return speed < 100.0 ? String(speed, 1) : String(speed, 0);
}

String distanceText() {
  const double distance = displayDistance(tripDistanceM / 1000.0);
  if (distance < 10.0) return String(distance, 2);
  if (distance < 100.0) return String(distance, 1);
  return String(distance, 0);
}

String statisticSpeedText(double speedKmh) {
  const double speed = displaySpeed(speedKmh);
  return speed < 100.0 ? String(speed, 1) : String(speed, 0);
}

String durationText(uint32_t durationMs) {
  const uint32_t totalMinutes = durationMs / 60000UL;
  const uint32_t hours = totalMinutes / 60UL;
  const uint32_t minutes = totalMinutes % 60UL;
  char value[12];
  if (hours < 100) {
    snprintf(value, sizeof(value), "%02lu:%02lu",
             static_cast<unsigned long>(hours),
             static_cast<unsigned long>(minutes));
  } else {
    snprintf(value, sizeof(value), "%luh",
             static_cast<unsigned long>(hours));
  }
  return String(value);
}

double averageSpeedKmh() {
  if (movingTimeMs == 0) return 0.0;
  return (tripDistanceM / 1000.0) / (movingTimeMs / 3600000.0);
}

uint32_t totalTimeMs() {
  if (tripActive) return millis() - tripStartedMs;
  if (tripStartedMs > 0 && tripFinishedMs >= tripStartedMs) {
    return tripFinishedMs - tripStartedMs;
  }
  return 0;
}

void invalidateDynamicValues() {
  previousStatus = "";
  previousSpeed = "";
  previousTrip = "";
  previousAverage = "";
  previousMaximum = "";
  previousTimes = "";
}

void drawGaugeTick(int tick, bool active) {
  const float stepDeg = GAUGE_SWEEP_DEG / GAUGE_TICK_COUNT;
  const float centerDeg = GAUGE_START_DEG + (stepDeg * tick);
  const float halfSegmentDeg = stepDeg * 0.39f;
  const float startAngle = (centerDeg - halfSegmentDeg) * DEG_TO_RAD;
  const float endAngle = (centerDeg + halfSegmentDeg) * DEG_TO_RAD;

  const int16_t outerStartX = GAUGE_CX + cosf(startAngle) * GAUGE_OUTER_RX;
  const int16_t outerStartY = GAUGE_CY + sinf(startAngle) * GAUGE_OUTER_RY;
  const int16_t outerEndX = GAUGE_CX + cosf(endAngle) * GAUGE_OUTER_RX;
  const int16_t outerEndY = GAUGE_CY + sinf(endAngle) * GAUGE_OUTER_RY;
  const int16_t innerStartX = GAUGE_CX + cosf(startAngle) * GAUGE_INNER_RX;
  const int16_t innerStartY = GAUGE_CY + sinf(startAngle) * GAUGE_INNER_RY;
  const int16_t innerEndX = GAUGE_CX + cosf(endAngle) * GAUGE_INNER_RX;
  const int16_t innerEndY = GAUGE_CY + sinf(endAngle) * GAUGE_INNER_RY;

  const uint16_t inactive = nightTheme ? tft.color565(30, 40, 48)
                                        : tft.color565(190, 200, 205);
  const double stepSpeed = (useMph ? GAUGE_MAX_MPH : GAUGE_MAX_KMH) / GAUGE_TICK_COUNT;
  const uint16_t liveColor = gaugeZoneColor(tick * stepSpeed);
  const uint16_t color = active ? liveColor : inactive;

  // Two triangles form one thick annular segment.
  tft.fillTriangle(outerStartX, outerStartY, outerEndX, outerEndY,
                   innerStartX, innerStartY, color);
  tft.fillTriangle(innerStartX, innerStartY, outerEndX, outerEndY,
                   innerEndX, innerEndY, color);

  if ((tick % 10) == 0) {
    const float centerAngle = centerDeg * DEG_TO_RAD;
    const int speedMark = useMph
        ? static_cast<int>(round(tick * (GAUGE_MAX_MPH / GAUGE_TICK_COUNT)))
        : static_cast<int>(round(tick * (GAUGE_MAX_KMH / GAUGE_TICK_COUNT)));
    const int16_t labelRx = tick == 20 ? 121 : 125;
    const int16_t labelRy = tick == 20 ? 72 : 75;
    const int16_t lx = GAUGE_CX + cosf(centerAngle) * labelRx;
    const int16_t ly = GAUGE_CY + sinf(centerAngle) * labelRy;
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(gaugeZoneColor(speedMark), backgroundColor());
    tft.drawString(String(speedMark), lx, ly, 2);
  }
}

int gaugeTickForSpeed(double speedKmh) {
  const double speed = displaySpeed(speedKmh);
  const double gaugeMaximum = useMph ? GAUGE_MAX_MPH : GAUGE_MAX_KMH;
  const double limited = constrain(speed, 0.0, gaugeMaximum);
  return static_cast<int>(round(limited * GAUGE_TICK_COUNT / gaugeMaximum));
}

void drawGaugeScale(bool force = false) {
  const int currentTick = gaugeTickForSpeed(filteredSpeedKmh(validFix()));
  if (!force && currentTick == previousGaugeTick) return;
  for (int tick = 0; tick <= GAUGE_TICK_COUNT; ++tick) {
    drawGaugeTick(tick, tick <= currentTick);
  }
  previousGaugeTick = currentTick;
}

void drawSpeedometerStatic() {
  const uint16_t bg = backgroundColor();
  tft.fillScreen(bg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(secondaryColor(), bg);   // 单位文字用中性灰，不随速度分区变色
  tft.drawString(useMph ? "MPH" : "KM/h", 160, 194, 4);
  previousGaugeTick = -1;
  previousSpeed = "";
  previousStatus = "";
  previousAltitude = "";
  previousGrade = "";
  previousTemperature = "";
  drawGaugeScale(true);
}

void drawStaticScreen() {
  const uint16_t bg=backgroundColor(), panel=nightTheme?tft.color565(18,28,38):tft.color565(222,231,238), border=nightTheme?tft.color565(55,78,94):tft.color565(165,181,192);
  tft.fillScreen(bg);
  tft.fillRoundRect(6,4,308,26,6,panel); tft.drawRoundRect(6,4,308,26,6,border);
  tft.drawFastVLine(104,7,20,border); tft.drawFastVLine(198,7,20,border);
  tft.setTextDatum(MC_DATUM); drawBoldText("MOTONAV",55,16,2,labelColor(),panel);
  tft.setTextColor(secondaryColor(),bg); tft.drawString(speedUnitText(),160,117,2);

  const int16_t x[3]={6,109,212}; const uint16_t accent[3]={TFT_GREEN,accentColor(),TFT_ORANGE}; const char* label[3]={"TRIP","AVG","MAX"};
  for(int i=0;i<3;++i){tft.fillRoundRect(x[i],128,98,50,7,panel);tft.drawRoundRect(x[i],128,98,50,7,border);tft.fillRoundRect(x[i]+4,132,5,42,2,accent[i]);tft.setTextDatum(TC_DATUM);drawBoldText(label[i],x[i]+53,132,2,labelColor(),panel);}

  tft.fillRoundRect(6,182,308,43,7,panel);tft.drawRoundRect(6,182,308,43,7,border);tft.drawFastVLine(160,187,33,border);
  tft.setTextDatum(TL_DATUM);drawBoldText("TOTAL",16,186,2,labelColor(),panel);drawBoldText("MOVING",170,186,2,labelColor(),panel);
  tft.setTextDatum(BC_DATUM);tft.setTextColor(secondaryColor(),bg);tft.drawString("HOLD OR TAP TITLE FOR MENU",160,239,1);
  invalidateDynamicValues();
}

void drawStatus(bool fix, bool force) {
  const String status=gnssStatus(fix); if(!force&&status==previousStatus)return;
  const uint16_t panel=nightTheme?tft.color565(18,28,38):tft.color565(222,231,238);
  tft.fillRect(200,6,111,20,panel);tft.setTextDatum(MC_DATUM);tft.setTextColor(fix?TFT_GREEN:TFT_ORANGE,panel);tft.drawString(status,255,16,2);previousStatus=status;
}

void drawSpeed(bool fix, bool force) {
  const String value = speedText(fix);
  if (!force && value == previousSpeed) return;
  speedSprite.fillSprite(backgroundColor());
  speedSprite.setTextDatum(MC_DATUM);
  speedSprite.setTextColor(primaryColor(), backgroundColor());
  speedSprite.drawString(value, 95, 39, 8);
  speedSprite.pushSprite(65, 29);
  previousSpeed = value;
}

void drawStatistic(const String &value,String &previous,int16_t centerX,bool force){
  if(!force&&value==previous)return;const uint16_t panel=nightTheme?tft.color565(18,28,38):tft.color565(222,231,238);
  tft.fillRect(centerX-39,149,78,26,panel);tft.setTextDatum(MC_DATUM);tft.setTextColor(primaryColor(),panel);tft.drawString(value,centerX,162,4);previous=value;
}

void drawTimes(bool force){
  const String value=durationText(totalTimeMs())+"|"+durationText(movingTimeMs);if(!force&&value==previousTimes)return;
  const uint16_t panel=nightTheme?tft.color565(18,28,38):tft.color565(222,231,238);
  tft.fillRect(12,203,142,19,panel);tft.fillRect(166,203,142,19,panel);tft.setTextDatum(MC_DATUM);tft.setTextColor(primaryColor(),panel);
  tft.drawString(durationText(totalTimeMs()),83,212,4);tft.drawString(durationText(movingTimeMs),237,212,4);previousTimes=value;
}

void drawGaugeSpeed(bool fix, bool force) {
  const String value = speedText(fix);
  if (!force && value == previousSpeed) return;
  speedSprite.fillSprite(backgroundColor());
  speedSprite.setTextDatum(MC_DATUM);
  speedSprite.setTextColor(primaryColor(), backgroundColor());
  speedSprite.drawString(value, 95, 39, 8);
  speedSprite.pushSprite(65, 91);
  previousSpeed = value;

  // The speed sprite overlaps the gauge edges; restore the thick arc afterward.
  drawGaugeScale(true);
}

// ---------------- BMP280 气压计 / 海拔 / 坡度 / 温度 ----------------

// TFT_eSPI 的 16px 位图字体里，反引号 0x60 被映射为 ° 符号
//（Font16.c 定义了 TFT_ESPI_GRAVE_IS_DEGREE），温度显示用这个字符。
const char DEG_SYMBOL = '\x60';

void initializeBaro() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (bmp.begin(BMP280_ADDR) || bmp.begin(BMP280_ADDR_ALT)) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X8,
                    Adafruit_BMP280::FILTER_X8,
                    Adafruit_BMP280::STANDBY_MS_1);
    bmpReady = true;
    Serial.println("BMP280 ready");
  } else {
    Serial.println("ERROR: BMP280 not found (check VCC/GND/SDA=22/SCL=27)");
  }
}

void updateBaro() {
  if (!bmpReady) return;
  const uint32_t now = millis();
  if (now - lastBaroReadMs < BARO_READ_INTERVAL_MS) return;
  lastBaroReadMs = now;

  const double tempC = bmp.readTemperature() + TEMP_OFFSET_C;
  const double rawAlt = bmp.readAltitude(SEA_LEVEL_HPA);

  // I2C 偶发干扰会读到离谱的温压值（例如 182°C、NaN、超大/负海拔），
  // 超出合理范围则整次丢弃，避免污染滤波状态和后续 GPX <ele>。
  if (!isfinite(tempC) || !isfinite(rawAlt) ||
      tempC < BARO_MIN_TEMP_C || tempC > BARO_MAX_TEMP_C ||
      rawAlt < ALT_MIN_M || rawAlt > ALT_MAX_M) {
    baroConsecutiveFails++;
    return;
  }
  baroConsecutiveFails = 0;
  baroHasValidSample = true;
  baroTemperatureC = tempC;

  if (!baroAltInitialized) {
    baroAltitudeM = rawAlt;
    baroAltInitialized = true;
    return;
  }

  baroAltitudeM += BARO_ALT_ALPHA * (rawAlt - baroAltitudeM);

  // 只有 GNSS 定位良好时，才用其绝对海拔慢锚定融合值
  if (gps.altitude.isValid() && gps.hdop.isValid() &&
      gps.hdop.hdop() < MAX_HDOP_FOR_DISTANCE) {
    const double gpsAlt = gps.altitude.meters();
    if (isfinite(gpsAlt) && gpsAlt > ALT_MIN_M && gpsAlt < ALT_MAX_M) {
      if (!baroAnchorSet) {
        baroOffsetM = gpsAlt - baroAltitudeM;
        baroAnchorSet = true;
      } else {
        const double fused = baroAltitudeM + baroOffsetM;
        baroOffsetM += BARO_GNSS_ANCHOR_ALPHA * (gpsAlt - fused);
      }
    }
  }
}

bool haveAltitude() {
  return baroHealthy() || gps.altitude.isValid();
}

// ---- 坡度：时间域 Vz/Vh（海拔对时间回归求 Vz，再除以多普勒 Vh）----

// 用于坡度计算的海拔快照（米）。优先用「低通后的气压相对海拔」：
// 短期变化准，且坡度只关心相对变化，与 GNSS 绝对锚定无关。
bool gradeAltitudeM(double &out) {
  if (baroHealthy()) {
    const double v = baroAltitudeM;
    if (isfinite(v) && v > ALT_MIN_M && v < ALT_MAX_M) { out = v; return true; }
  }
  if (gps.altitude.isValid()) {
    const double v = gps.altitude.meters();
    if (isfinite(v) && v > ALT_MIN_M && v < ALT_MAX_M) { out = v; return true; }
  }
  return false;
}

// 时间域坡度采样：固定周期采集「融合海拔」→ 中值滤波 → 仅在移动时写入环形缓冲。
// 静止时只更新中值滤波状态、不写缓冲，从而「停车冻结」，且不被气压漂移污染。
void updateGradeWindow() {
  double altM;
  if (!gradeAltitudeM(altM)) return;      // 无有效海拔来源（无 BMP 也无 GNSS）
  const uint32_t now = millis();
  if (now - lastGradeSampleMs < GRADE_SAMPLE_INTERVAL_MS) return;
  lastGradeSampleMs = now;

  // 3 点中值滤波：压车内气压短时尖峰（开关门、阵风等）
  gradeAltMedian[gradeAltMedianIdx] = altM;
  gradeAltMedianIdx = (gradeAltMedianIdx + 1) % 3;
  if (gradeAltMedianCount < 3) gradeAltMedianCount++;
  double medianAlt = altM;
  if (gradeAltMedianCount == 3) {
    const double a = gradeAltMedian[0], b = gradeAltMedian[1], c = gradeAltMedian[2];
    if (a <= b) medianAlt = (b <= c) ? b : ((a <= c) ? c : a);
    else        medianAlt = (a <= c) ? a : ((b <= c) ? c : b);
  }

  // 仅在「移动」时写入采样点：静止时 Vh=0 无法算坡度，也避免把停车漂移纳入窗口
  if (filteredSpeedKmh(validFix()) < GRADE_MIN_VH_KMH) return;

  gradeRing[gradeRingHead].tMs = now;
  gradeRing[gradeRingHead].altM = medianAlt;
  gradeRingHead = (gradeRingHead + 1) % GRADE_RING_SIZE;
  if (gradeRingCount < GRADE_RING_SIZE) gradeRingCount++;
}

// 对最近 GRADE_WINDOW_S 秒内的采样点做「海拔 vs 时间」最小二乘 → 垂直速度 Vz，
// 再除以水平速度 Vh（多普勒地速）得坡度百分比。Vh 低于门控时返回 false（冻结显示）。
bool computeGradePercent(double &pct) {
  const bool fix = validFix();
  const double vhKmh = filteredSpeedKmh(fix);
  if (vhKmh < GRADE_MIN_VH_KMH) return false;   // 低速/静止：冻结，不更新
  const double vh = vhKmh / 3.6;                // m/s

  const uint32_t now = millis();
  const uint32_t windowStartMs = now - (uint32_t)(GRADE_WINDOW_S * 1000.0);

  // 平移坐标轴（时间减窗口起点、海拔减首个样本），避免大数相减丢精度
  double baseT = 0.0, baseA = 0.0;
  double sumT = 0.0, sumA = 0.0, sumTT = 0.0, sumTA = 0.0;
  int n = 0;
  for (uint8_t i = 0; i < gradeRingCount; ++i) {
    const int idx = ((int)gradeRingHead + GRADE_RING_SIZE - gradeRingCount + i) % GRADE_RING_SIZE;
    const uint32_t tMs = gradeRing[idx].tMs;
    if (tMs < windowStartMs) continue;
    const double t = (double)(tMs - windowStartMs);  // 相对窗口起点的毫秒数
    const double a = gradeRing[idx].altM;
    if (n == 0) { baseT = t; baseA = a; }
    const double x = t - baseT;
    const double y = a - baseA;
    sumT += x; sumA += y; sumTT += x * x; sumTA += x * y; ++n;
  }
  if (n < GRADE_MIN_SAMPLES) return false;

  const double denom = n * sumTT - sumT * sumT;
  if (fabs(denom) < 1e-9) return false;
  const double vzPerMs = (n * sumTA - sumT * sumA) / denom;  // m per ms
  const double vz = vzPerMs * 1000.0;                        // m/s
  const double rawPct = (vz / vh) * 100.0;
  if (!isfinite(rawPct) || fabs(rawPct) > GRADE_MAX_PERCENT) return false;

  // 输出 EMA 平滑
  if (!gradeHasValue) {
    gradeSmoothedPct = rawPct;
    gradeHasValue = true;
  } else {
    gradeSmoothedPct += GRADE_OUTPUT_ALPHA * (rawPct - gradeSmoothedPct);
  }
  pct = gradeSmoothedPct;
  return true;
}

String altitudeText() {
  if (!haveAltitude()) return "--";
  const double alt = baroHealthy() ? (baroAltitudeM + baroOffsetM)
                                   : gps.altitude.meters();
  if (!isfinite(alt) || alt < ALT_MIN_M || alt > ALT_MAX_M) return "--";
  return String(static_cast<int>(round(alt))) + " m";
}

// 整数显示迟滞：连续值需越过整数边界并留出死区后才更新显示整数，
// 避免坡度值恰好在 0°/1° 等边界附近时来回跳。
int gradeQuantize(double vDisplay) {
  if (gradeShownStyle != gradeStyle) {
    gradeShownStyle = gradeStyle;
    gradeShownInt = static_cast<int>(round(vDisplay));
    return gradeShownInt;
  }
  const double hyst = GRADE_DISPLAY_HYSTERESIS;
  if (vDisplay >= gradeShownInt + 0.5 + hyst) {
    gradeShownInt = static_cast<int>(round(vDisplay));
  } else if (vDisplay <= gradeShownInt - 0.5 - hyst) {
    gradeShownInt = static_cast<int>(round(vDisplay));
  }
  return gradeShownInt;
}

String gradeText() {
  if (gradeStyle == 0) return "";        // 关闭：不显示
  if (!haveAltitude()) return "--";      // 无气压计也无 GNSS 海拔
  double pct;
  const bool fresh = computeGradePercent(pct);
  // 尚无有效坡度（起步不足 / 静止前从未移动）→ "--"；否则停车时冻结显示最后一次有效值
  if (!fresh && !gradeHasValue) return "--";
  const double value = fresh ? pct : gradeSmoothedPct;

  // 换算到显示单位（百分比直接用，角度用 atan），再经迟滞取整
  const double vDisplay = (gradeStyle == 1)
      ? value
      : atan(value / 100.0) * 57.2957795131;
  const int v = gradeQuantize(vDisplay);

  char buf[16];
  if (gradeStyle == 1) {
    // 百分比：整数，正数不带 +、负数带 -、零无号（对齐比亚迪车机观感）
    snprintf(buf, sizeof(buf), "%d%%", v);
  } else {
    // 角度：整数°，正数不带 +、负数带 -、零无号
    snprintf(buf, sizeof(buf), "%d", v);
  }
  return String(buf);
}

String temperatureText() {
  if (!baroHealthy()) return "--";
  if (!isfinite(baroTemperatureC) ||
      baroTemperatureC < BARO_MIN_TEMP_C || baroTemperatureC > BARO_MAX_TEMP_C) {
    return "--";
  }
  return String(static_cast<int>(round(baroTemperatureC))) + DEG_SYMBOL + "C";
}

const char *gradeStyleText() {
  switch (gradeStyle) {
    case 2: return "DEGREES";
    case 0: return "OFF";
    default: return "PERCENT";
  }
}

void drawAltitudeValue(bool force) {
  const String value = altitudeText();
  if (!force && value == previousAltitude) return;
  const uint16_t bg = backgroundColor();
  tft.fillRect(4, 208, 104, 32, bg);
  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(primaryColor(), bg);
  tft.drawString(value, 12, 236, 4);
  previousAltitude = value;
}

void drawGradeValue(bool force) {
  const String value = gradeText();
  if (!force && value == previousGrade) return;
  const uint16_t bg = backgroundColor();
  tft.fillRect(220, 208, 96, 32, bg);
  if (value.length() == 0) { previousGrade = value; return; }
  tft.setTextDatum(BR_DATUM);
  tft.setTextColor(primaryColor(), bg);
  if (gradeStyle == 2 && value != "--") {
    // 角度：数字右对齐留出 ° 的位置，再画一个小圆圈当 ° 符号
    tft.drawString(value, 300, 236, 4);
    tft.drawCircle(306, 213, 3, primaryColor());
  } else {
    tft.drawString(value, 308, 236, 4);
  }
  previousGrade = value;
}

void drawTemperatureValue(bool force) {
  const String value = temperatureText();
  if (!force && value == previousTemperature) return;
  const uint16_t bg = backgroundColor();
  tft.fillRect(224, 6, 96, 20, bg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(secondaryColor(), bg);
  tft.drawString(value, 290, 16, 2);
  previousTemperature = value;
}

void updateDynamicScreen(bool force = false) {
  if (uiMode != DRIVE_UI || stopRecordReminderVisible) return;
  const bool fix = validFix();
  if (screenMode == SPEED_SCREEN) {
    drawGaugeScale(force);
    drawGaugeSpeed(fix, force);
    drawAltitudeValue(force);
    drawGradeValue(force);
    drawTemperatureValue(force);
    return;
  }
  drawStatus(fix, force);
  drawSpeed(fix, force);
  drawStatistic(distanceText(), previousTrip, 59, force);
  drawStatistic(statisticSpeedText(averageSpeedKmh()), previousAverage, 162, force);
  drawStatistic(statisticSpeedText(maximumSpeedKmh), previousMaximum, 265, force);
  drawTimes(force);
  drawTrackBadge();
}

void startTripIfNeeded() {
  if (tripActive || trackState == TRACK_STOPPED || !validFix()) return;
  tripActive = true;
  tripStartedMs = millis();
  lastMotionSampleMs = tripStartedMs;
}

void updateTripStatistics() {
  const uint32_t now = millis();
  startTripIfNeeded();

  if (tripActive) {
    const uint32_t sampleMs = now - lastMotionSampleMs;
    if (sampleMs <= MAX_MOTION_SAMPLE_MS &&
        filteredSpeedKmh(validFix()) >= MOVING_SPEED_THRESHOLD_KMH) {
      movingTimeMs += sampleMs;
    }
    lastMotionSampleMs = now;
  }

  const bool fix = validFix();
  const double speed = filteredSpeedKmh(fix);
  const bool hdopGood = !gps.hdop.isValid() ||
                        gps.hdop.hdop() <= MAX_HDOP_FOR_DISTANCE;
  const bool satellitesGood = !gps.satellites.isValid() ||
                              gps.satellites.value() >= MIN_SATELLITES_FOR_TRACK;
  if (fix && hdopGood && satellitesGood &&
      speed >= MOVING_SPEED_THRESHOLD_KMH &&
      speed <= MAX_VALID_SPEED_KMH) {
    maximumSpeedKmh = max(maximumSpeedKmh, speed);
  }
}

void resetTrip(bool showNotice = true) {
  tripActive = false;
  tripDistanceM = 0.0;
  movingTimeMs = 0;
  maximumSpeedKmh = 0.0;
  tripStartedMs = 0;
  tripFinishedMs = 0;
  lastMotionSampleMs = millis();
  invalidateDynamicValues();

  if (showNotice) {
    const uint16_t bg = backgroundColor();
    tft.fillRect(70, 84, 180, 28, bg);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(accentColor(), bg);
    tft.drawString("TRIP RESET", 160, 98, 2);
    delay(350);
    updateDynamicScreen(true);
  }
}


void drawRideSummary() {
  uiMode=SUMMARY_UI;
  const uint16_t bg=backgroundColor(),panel=nightTheme?tft.color565(18,28,38):tft.color565(222,231,238),border=nightTheme?tft.color565(55,78,94):tft.color565(165,181,192),state=summarySavedOk?TFT_GREEN:TFT_RED;
  tft.fillScreen(bg);
  tft.fillRoundRect(6,4,308,28,7,state);tft.setTextDatum(MC_DATUM);drawBoldText(summarySavedOk?"RIDE SAVED":"SAVE ERROR",160,18,2,summarySavedOk?TFT_BLACK:TFT_WHITE,state);

  tft.fillRoundRect(6,36,308,51,8,panel);tft.drawRoundRect(6,36,308,51,8,border);
  // Lower the complete distance row for better visual centering with the bold font.
  tft.setTextDatum(ML_DATUM);drawBoldText("DISTANCE",18,63,2,labelColor(),panel);
  tft.setTextDatum(MC_DATUM);tft.setTextColor(primaryColor(),panel);tft.drawString(String(displayDistance(summaryDistanceM/1000.0),2),201,61,6);
  tft.setTextDatum(MR_DATUM);drawBoldText(distanceUnitText(),304,63,2,labelColor(),panel);

  const int16_t x[3]={6,109,212};const char* tl[3]={"TOTAL","MOVING","STOPPED"};const String tv[3]={durationText(summaryTotalTimeMs),durationText(summaryMovingTimeMs),durationText(summaryStoppedTimeMs)};
  for(int i=0;i<3;++i){tft.fillRoundRect(x[i],92,98,54,7,panel);tft.drawRoundRect(x[i],92,98,54,7,border);tft.setTextDatum(TC_DATUM);drawBoldText(tl[i],x[i]+49,97,2,labelColor(),panel);tft.setTextDatum(MC_DATUM);tft.setTextColor(primaryColor(),panel);tft.drawString(tv[i],x[i]+49,126,4);}

  const char* sl[3]={"AVG","MAX","POINTS"};const String sv[3]={statisticSpeedText(summaryAverageSpeedKmh),statisticSpeedText(summaryMaximumSpeedKmh),String(summaryPointCount)};const uint16_t sa[3]={TFT_CYAN,TFT_ORANGE,TFT_GREEN};
  for(int i=0;i<3;++i){tft.fillRoundRect(x[i],151,98,61,7,panel);tft.drawRoundRect(x[i],151,98,61,7,border);tft.fillRoundRect(x[i]+4,155,5,53,2,sa[i]);tft.setTextDatum(TC_DATUM);drawBoldText(sl[i],x[i]+53,155,2,labelColor(),panel);tft.setTextDatum(MC_DATUM);tft.setTextColor(primaryColor(),panel);tft.drawString(sv[i],x[i]+53,188,4);}

  tft.setTextDatum(BC_DATUM);tft.setTextColor(secondaryColor(),bg);tft.drawString("TAP TO CONTINUE",160,239,1);
}


void showTripScreen() {
  uiMode = DRIVE_UI;
  screenMode = TRIP_SCREEN;
  manualScreenMode = false;   // 回到行程屏即恢复自动切屏
  drawStaticScreen();
  updateDynamicScreen(true);
}

void showSpeedScreen() {
  uiMode = DRIVE_UI;
  screenMode = SPEED_SCREEN;
  drawSpeedometerStatic();
  updateDynamicScreen(true);
}

void redrawTheme() {
  if (uiMode == MENU_UI) {
    drawMenu();
  } else if (uiMode == SETTINGS_UI) {
    drawSettings();
  } else if (uiMode == SUMMARY_UI) {
    drawRideSummary();
  } else if (uiMode == GNSS_UI) {
    drawGnssDiagnostics();
  } else if (uiMode == RIDES_UI) {
    drawRideHistory();
  } else if (screenMode == SPEED_SCREEN) {
    showSpeedScreen();
  } else {
    showTripScreen();
  }
}

void updateAutomaticScreenMode() {
  if (uiMode != DRIVE_UI) return;
  if (manualScreenMode) return;   // 手动切屏期间暂停自动切换
  const uint32_t now = millis();
  const double speed = filteredSpeedKmh(validFix());

  if (screenMode == TRIP_SCREEN) {
    stoppedStartedMs = 0;
    if (speed >= SPEED_SCREEN_ENTER_KMH) {
      if (speedThresholdStartedMs == 0) speedThresholdStartedMs = now;
      if (now - speedThresholdStartedMs >= SPEED_SCREEN_ENTER_HOLD_MS) {
        speedThresholdStartedMs = 0;
        showSpeedScreen();
      }
    } else {
      speedThresholdStartedMs = 0;
    }
    return;
  }

  speedThresholdStartedMs = 0;
  if (speed <= SPEED_SCREEN_EXIT_KMH) {
    if (stoppedStartedMs == 0) stoppedStartedMs = now;
    if (now - stoppedStartedMs >= SPEED_SCREEN_EXIT_HOLD_MS) {
      stoppedStartedMs = 0;
      showTripScreen();
    }
  } else {
    stoppedStartedMs = 0;
  }
}

void handleTouch() {
  const bool down = touch.touched();
  const uint32_t now = millis();
  static int16_t tapX = 0;
  static int16_t tapY = 0;

  if (down && !touchWasDown) {
    TS_Point p = touch.getPoint();
    tapX = constrain(map(p.x, TOUCH_X_MIN, TOUCH_X_MAX, 0, 319), 0, 319);
    tapY = constrain(map(p.y, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, 239), 0, 239);
    touchWasDown = true;
    longPressHandled = false;
    touchStartedMs = now;
  }

  if (down && touchWasDown && !longPressHandled &&
      now - touchStartedMs >= MENU_HOLD_MS) {
    longPressHandled = true;
    if (stopRecordReminderVisible) {
      // 长按：确认停止记录
      stopRecordReminderVisible = false;
      trackStoppedStartedMs = 0;
      finishTrack();
      drawRideSummary();
    } else if (uiMode == MENU_UI || uiMode == SETTINGS_UI) closeMenu();
    else if (uiMode == GNSS_UI) drawMenu();
    else if (uiMode == RIDES_UI) toggleRideUploadIgnore();
    else if (filteredSpeedKmh(validFix()) < MENU_MAX_SPEED_KMH) drawMenu();
  }

  if (!down && touchWasDown) {
    const uint32_t heldMs = now - touchStartedMs;
    touchWasDown = false;
    if (!longPressHandled && heldMs >= TOUCH_MIN_PRESS_MS) {
      if (stopRecordReminderVisible) {
        // 短按：返回驾驶屏，不停止记录（若仍静止，稍后会再次提醒）
        trackStoppedStartedMs = 0;
        hideStopRecordReminder();
      } else if (uiMode == MENU_UI) handleMenuTap(tapX, tapY);
      else if (uiMode == SETTINGS_UI) handleSettingsTap(tapX, tapY);
      else if (uiMode == GNSS_UI) drawMenu();
      else if (uiMode == RIDES_UI) handleRideHistoryTap(tapX, tapY);
      else if (uiMode == SUMMARY_UI) showTripScreen();
      else if (screenMode == TRIP_SCREEN) {
        if (tapY < 38 && filteredSpeedKmh(validFix()) < MENU_MAX_SPEED_KMH) {
          // 点击标题栏 MOTONAV：进入菜单（与长按一致，带速度门控，避免电阻屏误触）
          drawMenu();
        } else {
          // 短按：手动进入椭圆速度仪表屏，并暂停自动切屏
          showSpeedScreen();
          manualScreenMode = true;
        }
      } else {
        // 短按：从仪表屏切回行程屏（showTripScreen 会恢复自动切屏）
        showTripScreen();
      }
    }
  }
}

void drawStartupTestValue(int value) {
  const uint16_t bg = backgroundColor();
  tft.fillRect(88, 83, 144, 76, bg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(primaryColor(), bg);
  tft.drawString(String(value), 160, 119, 7);
}



void drawPixelWheel(TFT_eSprite &sprite, int16_t cx, int16_t cy,
                    uint16_t tire, uint16_t rim, bool phase) {
  // 30 px knobby wheel. Rectangular clusters keep the sprite genuinely pixel-art.
  sprite.fillRect(cx - 10, cy - 14, 20, 3, tire);
  sprite.fillRect(cx - 10, cy + 12, 20, 3, tire);
  sprite.fillRect(cx - 14, cy - 10, 3, 20, tire);
  sprite.fillRect(cx + 12, cy - 10, 3, 20, tire);
  sprite.fillRect(cx - 12, cy - 12, 5, 5, tire);
  sprite.fillRect(cx + 8, cy - 12, 5, 5, tire);
  sprite.fillRect(cx - 12, cy + 8, 5, 5, tire);
  sprite.fillRect(cx + 8, cy + 8, 5, 5, tire);
  sprite.drawCircle(cx, cy, 10, rim);
  sprite.fillRect(cx - 3, cy - 3, 7, 7, rim);
  sprite.fillRect(cx - 1, cy - 1, 3, 3, tire);
  if (phase) {
    sprite.drawLine(cx, cy, cx - 8, cy - 6, rim);
    sprite.drawLine(cx, cy, cx + 8, cy + 6, rim);
    sprite.drawLine(cx, cy, cx + 6, cy - 8, rim);
    sprite.drawLine(cx, cy, cx - 6, cy + 8, rim);
  } else {
    sprite.drawLine(cx, cy, cx + 8, cy - 6, rim);
    sprite.drawLine(cx, cy, cx - 8, cy + 6, rim);
    sprite.drawLine(cx, cy, cx - 6, cy - 8, rim);
    sprite.drawLine(cx, cy, cx + 6, cy + 8, rim);
  }
}

void drawPixelThickLine(TFT_eSprite &sprite, int16_t x0, int16_t y0,
                        int16_t x1, int16_t y1, uint8_t width,
                        uint16_t color) {
  // Uses only the long-established drawLine API for broad TFT_eSPI compatibility.
  const int16_t half = width / 2;
  if (abs(x1 - x0) >= abs(y1 - y0)) {
    for (int16_t offset = -half; offset <= half; ++offset)
      sprite.drawLine(x0, y0 + offset, x1, y1 + offset, color);
  } else {
    for (int16_t offset = -half; offset <= half; ++offset)
      sprite.drawLine(x0 + offset, y0, x1 + offset, y1, color);
  }
}

void drawPixelEnduro(TFT_eSprite &sprite, uint8_t frame) {
  const uint16_t transparent = TFT_MAGENTA;
  const uint16_t outline = tft.color565(8, 12, 18);
  const uint16_t tire = tft.color565(18, 22, 28);
  const uint16_t rim = tft.color565(170, 190, 198);
  const uint16_t metal = tft.color565(105, 125, 132);
  const uint16_t lightMetal = tft.color565(205, 216, 218);
  const uint16_t body = tft.color565(0, 182, 205);
  const uint16_t bodyDark = tft.color565(0, 82, 118);
  const uint16_t highlight = tft.color565(255, 158, 24);
  const uint16_t engine = tft.color565(70, 78, 84);
  sprite.fillSprite(transparent);

  const int16_t bob = (frame == 1 || frame == 2) ? 1 : 0;
  const int16_t rearX = 23, frontX = 94, wheelY = 54;

  drawPixelWheel(sprite, rearX, wheelY, tire, rim, frame & 1);
  drawPixelWheel(sprite, frontX, wheelY - bob, tire, rim, frame & 1);

  // Swingarm, frame and rear shock.
  drawPixelThickLine(sprite, rearX, wheelY, 59, 49 + bob, 3, lightMetal);
  drawPixelThickLine(sprite, 40, 29 + bob, 59, 49 + bob, 3, outline);
  drawPixelThickLine(sprite, 59, 49 + bob, 66, 27 + bob, 3, outline);
  drawPixelThickLine(sprite, 66, 27 + bob, 40, 29 + bob, 3, outline);
  drawPixelThickLine(sprite, 48, 30 + bob, 55, 44 + bob, 3, highlight);

  // Single-cylinder engine, cooling fins, header and skid plate.
  sprite.fillRect(49, 34 + bob, 18, 16, outline);
  sprite.fillRect(52, 35 + bob, 13, 13, engine);
  for (int y = 37; y <= 43; y += 3) sprite.drawFastHLine(52, y + bob, 13, lightMetal);
  sprite.fillRect(55, 46 + bob, 11, 5, metal);
  drawPixelThickLine(sprite, 65, 38 + bob, 73, 48 + bob, 2, highlight);
  drawPixelThickLine(sprite, 73, 48 + bob, 67, 54 + bob, 2, highlight);
  sprite.fillRect(49, 51 + bob, 22, 3, lightMetal);

  // Tank, long enduro seat and angular side panels.
  sprite.fillTriangle(48, 21 + bob, 70, 19 + bob, 66, 34 + bob, outline);
  sprite.fillTriangle(50, 23 + bob, 68, 21 + bob, 64, 32 + bob, body);
  sprite.fillRect(31, 19 + bob, 22, 6, outline);
  sprite.fillRect(33, 20 + bob, 20, 3, tft.color565(36, 42, 48));
  sprite.fillTriangle(34, 25 + bob, 53, 24 + bob, 46, 38 + bob, bodyDark);
  sprite.fillTriangle(46, 27 + bob, 62, 32 + bob, 49, 41 + bob, body);
  sprite.fillRect(50, 28 + bob, 10, 3, highlight);

  // Raised exhaust and small tail assembly.
  drawPixelThickLine(sprite, 43, 31 + bob, 20, 26 + bob, 5, lightMetal);
  drawPixelThickLine(sprite, 22, 26 + bob, 10, 24 + bob, 5, metal);
  sprite.fillRect(7, 22 + bob, 8, 5, outline);
  sprite.fillRect(5, 19 + bob, 5, 3, TFT_RED);
  sprite.drawLine(9, 27 + bob, 4, 37 + bob, outline);

  // Tall golden fork, wheel brake and high front mudguard.
  drawPixelThickLine(sprite, 76, 19 + bob, frontX - 4, wheelY - bob, 4, highlight);
  drawPixelThickLine(sprite, 80, 19 + bob, frontX + 1, wheelY - bob, 3, metal);
  sprite.drawCircle(frontX, wheelY - bob, 6, lightMetal);
  sprite.fillRect(78, 33 + bob, 28, 4, body);
  sprite.fillRect(101, 35 + bob, 12, 3, bodyDark);

  // Headlight mask, lamp, handlebar and mirror.
  sprite.fillTriangle(70, 12 + bob, 82, 14 + bob, 78, 28 + bob, outline);
  sprite.fillTriangle(72, 14 + bob, 80, 15 + bob, 77, 25 + bob, body);
  sprite.fillRect(79, 17 + bob, 4, 7, TFT_WHITE);
  drawPixelThickLine(sprite, 73, 13 + bob, 79, 7 + bob, 2, outline);
  sprite.drawFastHLine(75, 7 + bob, 12, outline);
  sprite.fillRect(83, 5 + bob, 8, 4, body);
  sprite.drawLine(76, 9 + bob, 74, 2 + bob, outline);
  sprite.fillRect(71, bob, 7, 5, outline);

  // Animated chain glint and suspension movement add life without deforming the bike.
  if (frame & 1) sprite.fillRect(31, 50, 4, 2, highlight);
  else sprite.fillRect(38, 49, 4, 2, highlight);
}

const uint8_t *pixelTitleGlyph(char c) {
  // Five columns, seven rows; only glyphs needed by "READY TO RIDE" are stored.
  static const uint8_t A[5] = {0x7E,0x09,0x09,0x09,0x7E};
  static const uint8_t D[5] = {0x7F,0x41,0x41,0x22,0x1C};
  static const uint8_t E[5] = {0x7F,0x49,0x49,0x49,0x41};
  static const uint8_t I[5] = {0x41,0x41,0x7F,0x41,0x41};
  static const uint8_t O[5] = {0x3E,0x41,0x41,0x41,0x3E};
  static const uint8_t R[5] = {0x7F,0x09,0x19,0x29,0x46};
  static const uint8_t T[5] = {0x01,0x01,0x7F,0x01,0x01};
  static const uint8_t Y[5] = {0x03,0x04,0x78,0x04,0x03};
  switch (c) { case 'A': return A; case 'D': return D; case 'E': return E;
    case 'I': return I; case 'O': return O; case 'R': return R;
    case 'T': return T; case 'Y': return Y; default: return nullptr; }
}

void drawPixelTitleLayer(const char *text, int16_t startX, int16_t y,
                         uint8_t scale, int16_t offsetX, int16_t offsetY,
                         uint16_t topColor, uint16_t bottomColor) {
  int16_t x = startX;
  while (*text) {
    const uint8_t *glyph = pixelTitleGlyph(*text++);
    if (glyph) {
      for (uint8_t col = 0; col < 5; ++col) {
        for (uint8_t row = 0; row < 7; ++row) {
          if (glyph[col] & (1 << row)) {
            tft.fillRect(x + col * scale + offsetX, y + row * scale + offsetY,
                         scale, scale, row < 3 ? topColor : bottomColor);
          }
        }
      }
    }
    x += 6 * scale;
  }
}

void drawReadyToRideTitle() {
  const char *title = "READY TO RIDE";
  constexpr uint8_t scale = 3;
  constexpr int16_t width = 13 * 6 * scale - scale;
  const int16_t x = (320 - width) / 2;
  const int16_t y = 104;
  // Red extrusion, black separator, then a hot arcade gradient face.
  drawPixelTitleLayer(title, x, y, scale, 5, 6,
                      tft.color565(120, 12, 8), tft.color565(72, 4, 8));
  drawPixelTitleLayer(title, x, y, scale, 2, 2, TFT_BLACK, TFT_BLACK);
  drawPixelTitleLayer(title, x, y, scale, 0, 0,
                      TFT_YELLOW, tft.color565(255, 92, 0));
  tft.fillRect(43, 133, 234, 3, tft.color565(0, 190, 220));
  tft.fillRect(71, 138, 178, 2, tft.color565(0, 78, 128));
}

void rotateEnduroSprite(TFT_eSprite &source, TFT_eSprite &target,
                        float angleDegrees) {
  const uint16_t transparent = TFT_MAGENTA;
  target.fillSprite(transparent);

  // Rotate around the rear axle so the motorcycle performs a real wheelie.
  // The taller destination sprite leaves room for the raised front wheel.
  constexpr int16_t sourcePivotX = 23;
  constexpr int16_t sourcePivotY = 54;
  constexpr int16_t targetPivotX = 23;
  constexpr int16_t targetPivotY = 82;
  const float radians = angleDegrees * DEG_TO_RAD;
  const float cosine = cosf(radians);
  const float sine = sinf(radians);

  for (int16_t sy = 0; sy < source.height(); ++sy) {
    for (int16_t sx = 0; sx < source.width(); ++sx) {
      const uint16_t color = source.readPixel(sx, sy);
      if (color == transparent) continue;
      const int16_t dx = sx - sourcePivotX;
      const int16_t dy = sy - sourcePivotY;
      const int16_t tx = targetPivotX + lroundf(dx * cosine - dy * sine);
      const int16_t ty = targetPivotY + lroundf(dx * sine + dy * cosine);
      if (tx >= 0 && tx < target.width() && ty >= 0 && ty < target.height()) {
        target.drawPixel(tx, ty, color);
        // Fill occasional one-pixel holes caused by nearest-neighbour rotation.
        if (angleDegrees < -1.0f && tx + 1 < target.width())
          target.drawPixel(tx + 1, ty, color);
      }
    }
  }
}

float startupWheelieAngle(int16_t motorcycleX) {
  // Lift, hold briefly in the centre of the display, then lower smoothly.
  if (motorcycleX < 54 || motorcycleX > 224) return 0.0f;
  if (motorcycleX < 102) return -24.0f * (motorcycleX - 54) / 48.0f;
  if (motorcycleX <= 174) return -24.0f;
  return -24.0f * (224 - motorcycleX) / 50.0f;
}

void runMotorcycleAnimation() {
  const uint16_t bg = backgroundColor();
  TFT_eSprite baseSprite(&tft);
  TFT_eSprite animatedSprite(&tft);
  TFT_eSprite frameSprite(&tft);
  baseSprite.setColorDepth(16);
  animatedSprite.setColorDepth(16);
  frameSprite.setColorDepth(16);
  if (baseSprite.createSprite(118, 72) == nullptr) return;
  if (animatedSprite.createSprite(118, 102) == nullptr) {
    baseSprite.deleteSprite();
    return;
  }
  // Compose the complete moving area off-screen, then transfer it in one pass.
  // This removes the visible erase/redraw cycle that caused flicker.
  if (frameSprite.createSprite(320, 136) == nullptr) {
    animatedSprite.deleteSprite();
    baseSprite.deleteSprite();
    return;
  }

  tft.fillScreen(bg);
  tft.setTextDatum(TC_DATUM);
  drawBoldText("MOTONAV", 160, 16, 4, labelColor(), bg);

  const uint16_t road = nightTheme ? tft.color565(35, 45, 52)
                                   : tft.color565(185, 190, 194);
  // Eight-pixel steps provide 50% more frames than the previous fast version.
  // The single buffered transfer keeps the total pass fast and flicker-free.
  for (int16_t x = -120, frame = 0; x <= 326; x += 8, ++frame) {
    frameSprite.fillSprite(bg);
    frameSprite.drawFastHLine(0, 119, 320, road);
    for (int16_t mark = (frame * 12) % 50 - 50; mark < 320; mark += 50) {
      frameSprite.fillRect(mark, 129, 25, 3, road);
    }
    drawPixelEnduro(baseSprite, frame % 4);
    rotateEnduroSprite(baseSprite, animatedSprite, startupWheelieAngle(x));
    // Rear axle remains at screen Y=170 throughout the wheelie.
    animatedSprite.pushToSprite(&frameSprite, x, 26, TFT_MAGENTA);
    frameSprite.pushSprite(0, 62);
    delay(1);
  }
  frameSprite.deleteSprite();
  animatedSprite.deleteSprite();
  baseSprite.deleteSprite();
}

void runStartupAnimation() {
  const uint16_t bg = backgroundColor();

  tft.fillScreen(bg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(accentColor(), bg);
  tft.drawString("MOTONAV", 160, 88, 4);
  tft.setTextColor(secondaryColor(), bg);
  tft.drawString("IGNITION ON", 160, 132, 2);
  tft.setTextColor(TFT_ORANGE, bg);
  tft.drawString("MOTONAV V1.2.0", 160, 158, 2);
  delay(400);

  drawSpeedometerStatic();
  for (int tick = 0; tick <= GAUGE_TICK_COUNT; ++tick) {
    drawGaugeTick(tick, true);
    drawStartupTestValue(tick * 4);
    delay(16);
  }
  for (int tick = GAUGE_TICK_COUNT; tick >= 0; --tick) {
    drawGaugeTick(tick, false);
    drawStartupTestValue(tick * 4);
    delay(11);
  }

  runMotorcycleAnimation();

  tft.fillScreen(bg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_CYAN, bg);
  tft.drawString("MOTONAV", 160, 65, 4);
  drawReadyToRideTitle();
  delay(3000);
}

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("MotoNav-CYD-Plus V1.2.0 Stable");

  tft.init();
  tft.setRotation(1);
  // CYD2USB 用的是 ST7789 面板，颜色不需要反相；上游 V1.1.2 是给 ILI9341 写的，
  // 那里 invertDisplay(true) 才对。这里若继续 true，红↔青、绿↔品红、黑白全部反色。
  tft.invertDisplay(false);

  touchSpi.begin(TOUCH_CLK_PIN, TOUCH_MISO_PIN, TOUCH_MOSI_PIN, TOUCH_CS_PIN);
  touch.begin(touchSpi);
  touch.setRotation(1);

  gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  initializeSd();
  recoverInterruptedTrack();
  loadSettings();
  motoNet.begin();       // 后台启动 WiFi：有凭据自动连，否则开无密码配网热点（动画期间异步连接）
  Upload.begin();        // 读取自动上传开关
  initializeBaro();
  runStartupAnimation();

  // Allocate the permanent speed sprite after the large temporary startup
  // framebuffer has been released, keeping peak RAM usage comfortably lower.
  speedSprite.setColorDepth(16);
  if (speedSprite.createSprite(190, 82) == nullptr) {
    Serial.println("ERROR: speed sprite allocation failed");
  }
  showTripScreen();
  drawTrackBadge();
}

void loop() {
  while (gnssSerial.available()) {
    const char c = static_cast<char>(gnssSerial.read());
    gps.encode(c);
    nmeaChars++;
    lastNmeaByteMs = millis();
  }

  const bool locationUpdated = gps.location.isUpdated();
  const double latitude = gps.location.lat();
  const double longitude = gps.location.lng();

  updateSpeedFilter();
  updateBaro();
  updateGradeWindow();
  handleTouch();
  updateTripStatistics();
  writeTrackPoint(locationUpdated, latitude, longitude);
  if (trackState == TRACK_RECORDING &&
      millis() - lastStatsSnapshotMs >= STATS_SNAPSHOT_INTERVAL_MS) {
    lastStatsSnapshotMs = millis();
    writeActiveStatsSnapshot();
  }
  updateSafetyUi();
  updateAutomaticScreenMode();

  motoNet.tick();   // 非阻塞：WiFi 状态机 + 配网页面的 DNS/HTTP 处理
  Upload.tick();    // 非阻塞：后台扫描并流式上传待上传 GPX

  if (savedNotice && millis() - savedNoticeMs >= 5000UL) {
    savedNotice = false;
    drawTrackBadge();
  }

  if (uiMode == GNSS_UI) {
    // GNSS data changes slowly; update only changed value regions at 2 Hz.
    if (millis() - lastGnssUiMs >= 500UL) {
      lastGnssUiMs = millis();
      updateGnssDiagnostics();
    }
  } else if (millis() - lastScreenMs >= SCREEN_REFRESH_MS) {
    lastScreenMs = millis();
    updateDynamicScreen();
  }
}
