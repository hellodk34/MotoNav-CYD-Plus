#pragma once

#include <Arduino.h>
#include <SD.h>
#include <FS.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include "config.h"

// GPX 自动上传管理器：
//   1) 扫描 SD 卡 RIDES_INDEX.CSV 中 uploaded==0 的行程；
//   2) 以「批次」模型处理：触发后一次性抓取全部待上传文件列表，逐个 HTTPS 流式上传，
//      成功标记为 1，失败保持 0（本批不再立即重试，留给下一周期）；
//   3) 上传目标可同时是 TRACKS 后台（X-API-Key）与 WebDAV（Basic 认证），二者并存：
//      一个文件需向「所有已配置目标」都上传成功才标记 uploaded=1；
//   4) 整批处理完（无论成功/失败）推送一条 ntfy 通知，含各目标「成功 n 条，失败 m 条」。
// 非阻塞状态机，主循环每轮调用 tick()，绝不阻塞。
class UploadManager {
public:
  void begin();                    // 读取 auto_upload 开关 + 判定上传目标
  void tick();                     // 每轮 loop() 调用一次（非阻塞状态机）
  void requestScan();              // 触发一次「自动」扫描上传（开机/行程结束后，受开关约束）
  void requestManualUpload();      // 手动触发（点 PENDING 按钮）：绕过 auto_upload 开关，仅需联网
  bool enabled() const { return _enabled; }
  void setEnabled(bool on);        // 切换开关并持久化
  uint16_t pendingCount();         // 统计待上传条目数（读 SD CSV）
  uint8_t lastResult() const { return _lastResult; }   // 0=无 1=全成功 2=有失败
  bool busy() const { return _step != UP_IDLE; }
  String statusText() const;       // 供设置页显示
  // 重写 RIDES_INDEX.CSV 中某条行程的 uploaded 字段（0/1/2），供 UI 切换「忽略」用。
  void markUploaded(const String &gpxFile, uint8_t value);

private:
  void beginBatch();               // 抓取全部待上传文件，开始批次
  void resumeBatch();              // 非阻塞推进批次：每轮至多一次网络连接（打开文件/切换目标/收尾）
  void finalizeBatch();            // 整批结束 → 触发 ntfy 推送
  void notifyStart();              // 组装 JSON 并发送 ntfy POST
  void notifyStep();               // 读取 ntfy 响应

  bool _enabled = false;
  bool _scanRequested = false;
  bool _manualScanRequested = false;
  uint8_t _lastResult = 0;         // 0 none, 1 all-ok, 2 some-fail
  uint32_t _lastAttemptMs = 0;

  // 上传目标判定（编译期 secret.h 常量 → 运行期开关）
  bool _tracksConfigured = false;
  bool _webDavConfigured = false;

  enum Step { UP_IDLE, UP_SENDING, UP_READ_STATUS, UP_NOTIFY } _step = UP_IDLE;
  File _file;
  WiFiClientSecure _client;
  uint8_t _txBuf[UPLOAD_CHUNK_SIZE]; // 上传发送缓冲（成员，避免占用主循环栈）
  String _fileName;                // gpx_file 完整路径（/TRK_xxx.GPX）
  String _displayName;             // 仅文件名（multipart filename / WebDAV 路径末尾）
  String _suffix;                  // multipart 结尾边界（WebDAV PUT 为空串）
  size_t _fileSize = 0;
  size_t _sent = 0;
  uint32_t _stepStartMs = 0;

  // 批次模型
  String _pendingList[MAX_BATCH];  // 本批待上传文件路径
  uint16_t _pendingCount = 0;
  uint16_t _pendingIndex = 0;
  int8_t _target = -1;             // 当前文件正在处理的目标：-1=尚未开始 0=TRACKS 1=WebDAV
  bool _fileTracksOk = false;      // 当前文件对 TRACKS 的上传结果
  bool _fileWebDavOk = false;      // 当前文件对 WebDAV 的上传结果
  uint16_t _batchOkTracks = 0;
  uint16_t _batchFailTracks = 0;
  uint16_t _batchOkWebDav = 0;
  uint16_t _batchFailWebDav = 0;
  bool _batchActive = false;       // 批次进行中（跨多轮 tick 推进）

  // WebDAV 解析结果 + Basic 认证
  String _webDavHost;
  uint16_t _webDavPort = 443;
  String _webDavPath;              // 目录基址路径（结尾带 /）
  String _webDavAuth;              // "Basic <base64(user:pass)>"

  // ntfy 推送
  String _notifyHost;
  uint16_t _notifyPort = 443;
  String _notifyPath;
  String _notifyBody;
};

extern UploadManager Upload;
