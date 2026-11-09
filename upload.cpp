#include "upload.h"
#include "config.h"
#include "secret.h"
#include "network.h"

UploadManager Upload;

// 覆盖 ESP32 core 的 weak 函数 getArduinoLoopTaskStackSize()：
// 上传/推送的 TLS 握手（mbedTLS）+ 发送缓冲在 loop() 主循环里执行，
// 而默认 loopTask 栈只有 8KB，会触发 "Stack canary watchpoint triggered (loopTask)" 崩溃。
// 这里把主循环栈加大到 20KB。（放在 .cpp 里避免 .ino 预处理器的宏/原型扫描干扰）
size_t getArduinoLoopTaskStackSize(void) {
  return 20480;
}

// RIDES_INDEX.CSV 路径（与 .ino 保持一致）
static const char RIDES_INDEX_FILE[] = "/RIDES_INDEX.CSV";

// Let's Encrypt 根证书 ISRG Root X1，用于校验 tracks 服务 / ntfy.sh / WebDAV 的 TLS 证书链。
static const char ISRG_ROOT_X1[] =
"-----BEGIN CERTIFICATE-----\n"
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
"h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
"0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
"A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
"T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
"B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
"B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
"KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
"OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
"jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
"qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
"rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
"HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
"hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
"ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
"3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
"NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
"ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
"TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
"jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
"oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
"4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
"mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
"emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
"-----END CERTIFICATE-----\n";

// 读取 CSV 行的第 index 列（0 基）。缺列返回空串。
static String csvFieldAt(const String &line, uint8_t index) {
  int start = 0;
  for (uint8_t f = 0; f < index; ++f) {
    start = line.indexOf(',', start);
    if (start < 0) return "";
    ++start;
  }
  int end = line.indexOf(',', start);
  if (end < 0) end = line.length();
  return line.substring(start, end);
}

// 把一行的第 12 列（index 11，uploaded）替换为 value。旧格式（11 列）自动补一列。
static String withUploaded(const String &line, uint8_t value) {
  int commas = 0;
  int idx = -1;
  for (int i = 0; i < (int)line.length(); ++i) {
    if (line.charAt(i) == ',') {
      ++commas;
      if (commas == 11) { idx = i; break; }
    }
  }
  String prefix = (idx >= 0) ? line.substring(0, idx + 1) : line + ",";
  return prefix + String((int)value);
}

// 标准 Base64 编码（用于 WebDAV HTTP Basic 认证头）。
static String base64Encode(const String &in) {
  static const char tbl[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String out;
  out.reserve(((in.length() + 2) / 3) * 4);
  const size_t n = in.length();
  size_t i = 0;
  while (i + 3 <= n) {
    uint32_t v = ((uint32_t)(uint8_t)in[i] << 16) |
                 ((uint32_t)(uint8_t)in[i + 1] << 8) |
                 ((uint32_t)(uint8_t)in[i + 2]);
    out += tbl[(v >> 18) & 0x3F];
    out += tbl[(v >> 12) & 0x3F];
    out += tbl[(v >> 6) & 0x3F];
    out += tbl[v & 0x3F];
    i += 3;
  }
  if (i + 1 == n) {
    uint32_t v = (uint32_t)(uint8_t)in[i] << 16;
    out += tbl[(v >> 18) & 0x3F];
    out += tbl[(v >> 12) & 0x3F];
    out += "==";
  } else if (i + 2 == n) {
    uint32_t v = ((uint32_t)(uint8_t)in[i] << 16) |
                 ((uint32_t)(uint8_t)in[i + 1] << 8);
    out += tbl[(v >> 18) & 0x3F];
    out += tbl[(v >> 12) & 0x3F];
    out += tbl[(v >> 6) & 0x3F];
    out += '=';
  }
  return out;
}

// 解析 URL → host / port / path（仅支持 https）。
static void parseHttpUrl(const String &url, String &host, uint16_t &port, String &path) {
  String s = url;
  host = "";
  port = 443;
  path = "/";
  if (s.startsWith("https://")) s.remove(0, 8);
  else if (s.startsWith("http://")) s.remove(0, 7);
  int slash = s.indexOf('/');
  String authority = (slash >= 0) ? s.substring(0, slash) : s;
  path = (slash >= 0) ? s.substring(slash) : "/";
  int colon = authority.lastIndexOf(':');
  if (colon > 0) {
    host = authority.substring(0, colon);
    port = authority.substring(colon + 1).toInt();
    if (port == 0) port = 443;
  } else {
    host = authority;
  }
}

void UploadManager::begin() {
  Preferences prefs;
  prefs.begin("netcfg", true);
  _enabled = prefs.getBool("auto_upload", false);
  prefs.end();
  Serial.printf("[up] auto_upload = %s\n", _enabled ? "ON" : "OFF");

  // 上传目标判定（编译期 secret.h 常量 → 运行期开关）
  const String tracksKey = TRACKS_API_KEY;
  _tracksConfigured = tracksKey.length() > 0 &&
                      tracksKey != "PASTE_YOUR_API_KEY_HERE";

  const String davUrl = WEBDAV_SERVER_URL;
  _webDavConfigured = davUrl.startsWith("https://") &&
                      String(WEBDAV_USERNAME).length() > 0 &&
                      String(WEBDAV_PASSWORD).length() > 0;
  if (_webDavConfigured) {
    parseHttpUrl(davUrl, _webDavHost, _webDavPort, _webDavPath);
    if (_webDavPath.length() == 0) _webDavPath = "/";
    if (!_webDavPath.endsWith("/")) _webDavPath += "/";
    _webDavAuth = "Basic " + base64Encode(String(WEBDAV_USERNAME) + ":" +
                                          String(WEBDAV_PASSWORD));
  }
  Serial.printf("[up] targets: tracks=%s webdav=%s\n",
                _tracksConfigured ? "ON" : "OFF",
                _webDavConfigured ? "ON" : "OFF");
}

void UploadManager::setEnabled(bool on) {
  _enabled = on;
  Preferences prefs;
  prefs.begin("netcfg", false);
  prefs.putBool("auto_upload", on);
  prefs.end();
  Serial.printf("[up] auto_upload -> %s\n", on ? "ON" : "OFF");
}

uint16_t UploadManager::pendingCount() {
  if (!SD.exists(RIDES_INDEX_FILE)) return 0;
  File f = SD.open(RIDES_INDEX_FILE, FILE_READ);
  if (!f) return 0;
  uint16_t n = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() < 10 || line.startsWith("version")) continue;
    if (csvFieldAt(line, 11).toInt() == 0) ++n;
  }
  f.close();
  return n;
}

void UploadManager::markUploaded(const String &gpxFile, uint8_t value) {
  if (!SD.exists(RIDES_INDEX_FILE)) return;
  const char *tmp = "/RIDES_INDEX.TMP";
  File in = SD.open(RIDES_INDEX_FILE, FILE_READ);
  if (!in) return;
  File out = SD.open(tmp, FILE_WRITE);
  if (!out) { in.close(); return; }

  while (in.available()) {
    String line = in.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (csvFieldAt(line, 3) == gpxFile) {
      out.print(withUploaded(line, value));
    } else {
      out.print(line);
    }
    out.print('\n');
  }
  in.close();
  out.flush();
  const bool ok = !out.getWriteError();
  out.close();
  if (ok) {
    SD.remove(RIDES_INDEX_FILE);
    SD.rename(tmp, RIDES_INDEX_FILE);
  } else {
    SD.remove(tmp);
  }
}

void UploadManager::requestScan() {
  _scanRequested = true;
}

void UploadManager::requestManualUpload() {
  _manualScanRequested = true;
}

// 抓取全部 uploaded==0 的行程路径，存入 _pendingList，开始批次。
void UploadManager::beginBatch() {
  _pendingCount = 0;
  _pendingIndex = 0;
  _target = -1;
  _batchOkTracks = 0; _batchFailTracks = 0;
  _batchOkWebDav = 0; _batchFailWebDav = 0;
  _batchActive = true;

  // 没有任何已配置的上传目标 → 不启动批次，也不标记任何文件（避免误把文件置为「已上传」）
  if (!_tracksConfigured && !_webDavConfigured) {
    _batchActive = false;
    _step = UP_IDLE;
    Serial.println("[up] no upload target configured (skipped)");
    return;
  }

  if (SD.exists(RIDES_INDEX_FILE)) {
    File idx = SD.open(RIDES_INDEX_FILE, FILE_READ);
    if (idx) {
      while (idx.available() && _pendingCount < MAX_BATCH) {
        String line = idx.readStringUntil('\n');
        line.trim();
        if (line.length() < 10 || line.startsWith("version")) continue;
        if (csvFieldAt(line, 11).toInt() == 0) {
          String target = csvFieldAt(line, 3);
          if (target.length() > 0) _pendingList[_pendingCount++] = target;
        }
      }
      idx.close();
    }
  }

  Serial.printf("[up] batch: %u file(s) pending\n", _pendingCount);
  resumeBatch();
}

// 非阻塞推进批次：每轮 tick 至多发起一次网络连接；纯 SD 的「跳过缺失文件 / 切换目标 /
// 收尾文件」可安全地在函数内循环完成。
void UploadManager::resumeBatch() {
  while (true) {
    // 1) 尚未开始当前文件 → 打开下一条待上传文件（纯 SD，可循环）
    if (_target < 0) {
      if (_pendingIndex >= _pendingCount) { finalizeBatch(); return; }
      _fileName = _pendingList[_pendingIndex];
      _displayName = _fileName;
      int slash = _fileName.lastIndexOf('/');
      if (slash >= 0) _displayName = _fileName.substring(slash + 1);

      if (!SD.exists(_fileName)) {
        Serial.printf("[up] missing: %s\n", _displayName.c_str());
        if (_tracksConfigured) ++_batchFailTracks;
        if (_webDavConfigured) ++_batchFailWebDav;
        ++_pendingIndex;
        continue;
      }
      File probe = SD.open(_fileName, FILE_READ);
      if (!probe) {
        Serial.printf("[up] open fail: %s\n", _displayName.c_str());
        if (_tracksConfigured) ++_batchFailTracks;
        if (_webDavConfigured) ++_batchFailWebDav;
        ++_pendingIndex;
        continue;
      }
      _fileSize = probe.size();
      probe.close();
      _fileTracksOk = false;
      _fileWebDavOk = false;
      _target = 0;
    }

    // 2) 跳过未配置的目标
    while (_target <= 1 &&
           !((_target == 0 && _tracksConfigured) ||
             (_target == 1 && _webDavConfigured))) {
      ++_target;
    }

    // 3) 所有目标处理完 → 收尾当前文件（纯 SD，可循环到下一文件）
    if (_target > 1) {
      const bool ok = (!_tracksConfigured || _fileTracksOk) &&
                      (!_webDavConfigured || _fileWebDavOk);
      if (ok) {
        markUploaded(_fileName, 1);
        if (_tracksConfigured) ++_batchOkTracks;
        if (_webDavConfigured) ++_batchOkWebDav;
        Serial.printf("[up] ok: %s\n", _displayName.c_str());
      } else {
        if (_tracksConfigured) { if (_fileTracksOk) ++_batchOkTracks; else ++_batchFailTracks; }
        if (_webDavConfigured) { if (_fileWebDavOk) ++_batchOkWebDav; else ++_batchFailWebDav; }
        Serial.printf("[up] fail: %s\n", _displayName.c_str());
      }
      ++_pendingIndex;
      _target = -1;
      continue;
    }

    // 4) 打开文件流并连接当前目标（本函数唯一一次网络连接）
    _file = SD.open(_fileName, FILE_READ);
    if (!_file) {
      Serial.printf("[up] reopen fail: %s\n", _displayName.c_str());
      if (_target == 0) _fileTracksOk = false; else _fileWebDavOk = false;
      ++_target;
      continue;
    }
    _sent = 0;

    _client.stop();
    _client.setCACert(ISRG_ROOT_X1);
    _client.setTimeout(5);

    bool connected;
    if (_target == 0) {
      connected = _client.connect(TRACKS_API_HOST, TRACKS_API_PORT);
    } else {
      connected = _client.connect(_webDavHost.c_str(), _webDavPort);
    }

    if (!connected) {
      _file.close();
      Serial.printf("[up] connect failed (%s): %s\n",
                    _target == 0 ? "tracks" : "webdav", _displayName.c_str());
      if (_target == 0) _fileTracksOk = false; else _fileWebDavOk = false;
      ++_target;
      return;   // 连接失败：下一轮 tick 再试下一个目标，避免本 tick 连续连接
    }

    if (_target == 0) {
      const String boundary = "----MotoNavBoundary";
      const String prefix =
          "--" + boundary +
          "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"" + _displayName + "\"\r\n" +
          "Content-Type: application/gpx+xml\r\n\r\n";
      _suffix = "\r\n--" + boundary + "--\r\n";
      const size_t bodyLen = prefix.length() + _fileSize + _suffix.length();
      _client.print("POST /api/ingest HTTP/1.1\r\n");
      _client.print("Host: "); _client.print(TRACKS_API_HOST); _client.print("\r\n");
      _client.print("X-API-Key: "); _client.print(TRACKS_API_KEY); _client.print("\r\n");
      _client.print("Content-Type: multipart/form-data; boundary="); _client.print(boundary); _client.print("\r\n");
      _client.print("Content-Length: "); _client.print((unsigned long)bodyLen); _client.print("\r\n");
      _client.print("Connection: close\r\n\r\n");
      _client.print(prefix);
    } else {
      _suffix = "";
      _client.print("PUT "); _client.print(_webDavPath + _displayName); _client.print(" HTTP/1.1\r\n");
      _client.print("Host: "); _client.print(_webDavHost); _client.print("\r\n");
      _client.print("Authorization: "); _client.print(_webDavAuth); _client.print("\r\n");
      _client.print("Content-Type: application/gpx+xml\r\n");
      _client.print("Content-Length: "); _client.print((unsigned long)_fileSize); _client.print("\r\n");
      _client.print("Connection: close\r\n\r\n");
    }

    _step = UP_SENDING;
    _stepStartMs = millis();
    Serial.printf("[up] sending %s -> %s (%u bytes)\n", _displayName.c_str(),
                  _target == 0 ? "tracks" : "webdav", (unsigned)_fileSize);
    return;
  }
}

// 整批结束 → 组装 ntfy 推送（无论成功/失败都推）。
void UploadManager::finalizeBatch() {
  _batchActive = false;
  const uint16_t totalOk = _batchOkTracks + _batchOkWebDav;
  const uint16_t totalFail = _batchFailTracks + _batchFailWebDav;
  _lastResult = (totalFail == 0 && totalOk > 0) ? 1 : (totalFail > 0 ? 2 : 0);
  _lastAttemptMs = millis();

  // 上传代码实际没做任何事（既无成功也无失败）→ 不推送
  if (totalOk == 0 && totalFail == 0) {
    _step = UP_IDLE;
    return;
  }

  notifyStart();
}

// 组装 JSON 并发送 ntfy POST（非阻塞，随后由 notifyStep 收尾）。
void UploadManager::notifyStart() {
  _notifyBody = "{\"topic\":\"";
  _notifyBody += NTFY_SH_PUSH_TOPIC;
  _notifyBody += "\",\"title\":\"CYD GPX 上传结果\",\"message\":\"cyd gpx文件上传情况：";
  if (_tracksConfigured) {
    _notifyBody += "tracks成功"; _notifyBody += _batchOkTracks;
    _notifyBody += "条失败"; _notifyBody += _batchFailTracks; _notifyBody += "条";
  }
  if (_webDavConfigured) {
    if (_tracksConfigured) _notifyBody += "；";
    _notifyBody += "webdav成功"; _notifyBody += _batchOkWebDav;
    _notifyBody += "条失败"; _notifyBody += _batchFailWebDav; _notifyBody += "条";
  }
  _notifyBody += "\",\"priority\":";
  const bool anyFail = (_batchFailTracks + _batchFailWebDav) > 0;
  _notifyBody += (anyFail ? 4 : 3);
  _notifyBody += "}";

  parseHttpUrl(NTFY_SH_SERVER, _notifyHost, _notifyPort, _notifyPath);

  _client.stop();
  _client.setCACert(ISRG_ROOT_X1);
  _client.setTimeout(5);
  if (!_client.connect(_notifyHost.c_str(), _notifyPort)) {
    _client.stop();
    _step = UP_IDLE;
    Serial.println("[up] ntfy connect failed (notify skipped)");
    return;
  }

  _client.print("POST "); _client.print(_notifyPath); _client.print(" HTTP/1.1\r\n");
  _client.print("Host: "); _client.print(_notifyHost); _client.print("\r\n");
  _client.print("Content-Type: application/json\r\n");
  _client.print("Content-Length: "); _client.print((unsigned long)_notifyBody.length()); _client.print("\r\n");
  _client.print("Connection: close\r\n\r\n");
  _client.print(_notifyBody);

  _step = UP_NOTIFY;
  _stepStartMs = millis();
}

// 读取 ntfy 响应，收尾。
void UploadManager::notifyStep() {
  bool done = false;
  bool ok = false;
  if (_client.available()) {
    String statusLine = _client.readStringUntil('\n');
    statusLine.trim();
    int sp = statusLine.indexOf(' ');
    String code = sp >= 0 ? statusLine.substring(sp + 1, sp + 4) : "";
    ok = (code == "200");
    done = true;
  } else if (!_client.connected() || millis() - _stepStartMs > UPLOAD_SEND_TIMEOUT_MS) {
    done = true;
  }
  if (!done) return;

  _client.stop();
  _step = UP_IDLE;
  Serial.printf("[up] 已推送 ntfy 通知：成功%u条，失败%u条%s\n",
                _batchOkTracks + _batchOkWebDav,
                _batchFailTracks + _batchFailWebDav,
                ok ? "" : "（服务器未确认）");
}

void UploadManager::tick() {
  static bool wasConnected = false;
  const bool nowConnected = motoNet.connected();
  if (nowConnected && !wasConnected && !_batchActive) {
    _scanRequested = true;         // 刚连上网 → 触发一次上传扫描（批次进行中不重复触发）
  }
  wasConnected = nowConnected;

  if (_step == UP_SENDING) {
    if (_file.available()) {
      size_t n = _file.read(_txBuf, UPLOAD_CHUNK_SIZE);
      if (n > 0) {
        size_t off = 0;
        while (off < n) {
          size_t w = _client.write(_txBuf + off, n - off);
          if (w == 0) break;       // 对端关闭或写入失败
          off += w;
        }
        _sent += off;
        if (off < n) {
          _file.close(); _client.stop();
          Serial.printf("[up] send aborted: %s\n", _displayName.c_str());
          if (_target == 0) _fileTracksOk = false; else _fileWebDavOk = false;
          ++_target;
          _step = UP_IDLE;         // 交给下一轮 tick 继续下一个目标
          return;
        }
      }
      if (millis() - _stepStartMs > UPLOAD_SEND_TIMEOUT_MS) {
        _file.close(); _client.stop();
        Serial.printf("[up] send timeout: %s\n", _displayName.c_str());
        if (_target == 0) _fileTracksOk = false; else _fileWebDavOk = false;
        ++_target;
        _step = UP_IDLE;
      }
      return;
    }
    // 文件读完，补结尾边界（WebDAV 为空串），进入等待响应
    _file.close();
    _client.print(_suffix);
    _step = UP_READ_STATUS;
    _stepStartMs = millis();
    return;
  }

  if (_step == UP_READ_STATUS) {
    if (_client.available()) {
      String statusLine = _client.readStringUntil('\n');
      statusLine.trim();
      int sp = statusLine.indexOf(' ');
      String code = sp >= 0 ? statusLine.substring(sp + 1, sp + 4) : "";
      const bool ok = (code == "200" || code == "201" || code == "204");
      _client.stop();
      if (_target == 0) _fileTracksOk = ok; else _fileWebDavOk = ok;
      Serial.printf("[up] %s (%s): %s\n",
                    ok ? "ok" : "fail", statusLine.c_str(), _displayName.c_str());
      ++_target;
      _step = UP_IDLE;             // 下一轮 tick 继续下一个目标/文件
    } else if (!_client.connected() ||
               millis() - _stepStartMs > UPLOAD_SEND_TIMEOUT_MS) {
      _client.stop();
      Serial.printf("[up] no response (%s): %s\n",
                    _target == 0 ? "tracks" : "webdav", _displayName.c_str());
      if (_target == 0) _fileTracksOk = false; else _fileWebDavOk = false;
      ++_target;
      _step = UP_IDLE;
    }
    return;
  }

  if (_step == UP_NOTIFY) {
    notifyStep();
    return;
  }

  // 批次进行中：继续推进（即使中途掉线也要把本批收尾，失败项留给下一周期重试）
  if (_batchActive) {
    resumeBatch();
    return;
  }

  // 手动触发（点 PENDING 按钮）：绕过 auto_upload 开关，仅要求已联网
  if (_manualScanRequested) {
    _manualScanRequested = false;
    if (motoNet.connected()) {
      beginBatch();
    } else {
      Serial.println("[up] manual upload requested but offline (skipped)");
    }
    return;
  }

  // 空闲态：自动上传仅在联网 + 开关开启时尝试
  if (!_enabled) return;
  if (!motoNet.connected()) return;

  if (_scanRequested) {
    _scanRequested = false;
    beginBatch();
    return;
  }
  // 上次有失败后周期重试
  if (_lastResult == 2 && millis() - _lastAttemptMs >= UPLOAD_RETRY_INTERVAL_MS) {
    beginBatch();
  }
}

String UploadManager::statusText() const {
  if (_step == UP_NOTIFY) return "NOTIFY";
  if (_step != UP_IDLE) return "UPLOADING";
  switch (_lastResult) {
    case 1: return "LAST OK";
    case 2: return "LAST FAIL";
    default: return "IDLE";
  }
}
