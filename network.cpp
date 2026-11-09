#include "network.h"
#include "config.h"

MotoNet motoNet;

// 配网页：附近热点下拉 + 手动输入。保持轻量，移动端友好。
static const char PORTAL_HEAD[] =
  "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>MotoNav WiFi</title>"
  "<style>body{font-family:sans-serif;background:#0f1720;color:#e2e8f0;padding:16px}"
  ".box{max-width:420px;margin:0 auto;background:#1e293b;border-radius:12px;padding:20px}"
  "h1{font-size:20px;margin:0 0 4px}p{font-size:13px;color:#94a3b8;margin:0 0 8px}"
  "label{display:block;font-size:14px;margin:12px 0 4px}"
  "select,input{width:100%;box-sizing:border-box;padding:10px;border-radius:8px;"
  "border:1px solid #334155;background:#0f1720;color:#e2e8f0;font-size:16px}"
  "button{width:100%;margin-top:16px;padding:12px;border:none;border-radius:8px;"
  "background:#22d3ee;color:#0f1720;font-size:16px;font-weight:700}"
  "</style></head><body><div class='box'><h1>MotoNav 配网</h1>"
  "<p>选择手机热点（需 2.4GHz）并填写密码</p>"
  "<form method='POST' action='/'>"
  "<label>选择 WiFi</label><select name='ssid_select' id='sel' onchange='chg()'>";

static const char PORTAL_TAIL[] =
  "<option value='__other__'>手动输入（隐藏网络）</option></select>"
  "<div id='manual' style='display:none'><label>SSID</label>"
  "<input name='ssid' placeholder='WiFi 名称'></div>"
  "<label>密码</label><input type='password' name='password' placeholder='WiFi 密码（无密码留空）'>"
  "<button type='submit'>保存并连接</button></form>"
  "<script>function chg(){document.getElementById('manual').style.display="
  "document.getElementById('sel').value=='__other__'?'block':'none';}chg();</script>"
  "</div></body></html>";

// 转义 SSID 里的 HTML 特殊字符，防止注入 / 属性断裂
static String htmlEscape(const String &s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s.charAt(i);
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '\'': out += "&#39;"; break;
      case '"': out += "&quot;"; break;
      default: out += c;
    }
  }
  return out;
}

void MotoNet::begin() {
  loadPrefs();
  if (_ssid.length() > 0) {
    enterStaConnecting();
  } else {
    Serial.println("[net] no saved wifi -> AP mode");
    enterApMode();
  }
}

void MotoNet::loadPrefs() {
  Preferences prefs;
  prefs.begin("netcfg", true);
  _ssid = prefs.getString("ssid", "");
  _pass = prefs.getString("pass", "");
  prefs.end();
  if (_ssid.length() > 0) {
    Serial.printf("[net] loaded wifi %s\n", _ssid.c_str());
  }
}

void MotoNet::savePrefs(const String &ssid, const String &pass) {
  _ssid = ssid;
  _pass = pass;
  Preferences prefs;
  prefs.begin("netcfg", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

void MotoNet::enterStaConnecting() {
  _state = NET_STA_CONNECTING;
  _staStartMs = millis();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false);
  WiFi.begin(_ssid.c_str(), _pass.c_str());
  Serial.printf("[net] connecting to %s ...\n", _ssid.c_str());
}

void MotoNet::enterApMode() {
  _state = NET_AP_MODE;
  WiFi.mode(WIFI_AP_STA);          // AP+STA 共存，保证软 AP 期间仍能扫描附近热点
  WiFi.disconnect(false);
  WiFi.softAP(WIFI_AP_SSID, "");   // 无密码
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("[net] AP mode: %s ip=%s\n", WIFI_AP_SSID, ip.toString().c_str());

  _dns.start(53, "*", ip);
  _server.on("/", HTTP_POST, [this]() { serveSubmit(); });
  _server.onNotFound([this]() { servePortal(); });
  _server.begin();

  _scanOptions = "";
  _scanning = (WiFi.scanNetworks(true) == WIFI_SCAN_RUNNING);
}

void MotoNet::startConfigPortal() {
  if (_state == NET_AP_MODE) return;
  enterApMode();
}

void MotoNet::reconnect() {
  _dns.stop();
  _server.stop();
  WiFi.softAPdisconnect(true);
  if (_ssid.length() > 0) enterStaConnecting();
  else enterApMode();
}

void MotoNet::handleScanResult(int count) {
  _scanOptions = "";
  for (int i = 0; i < count; ++i) {
    String ssid;
    uint8_t enc;
    int32_t rssi;
    uint8_t *bssid;
    int32_t channel;
    if (WiFi.getNetworkInfo(i, ssid, enc, rssi, bssid, channel)) {
      if (ssid.length() == 0) continue;
      _scanOptions += "<option value='" + htmlEscape(ssid) + "'>" +
                      htmlEscape(ssid) + " (" + String(rssi) + "dBm)</option>";
    }
  }
  WiFi.scanDelete();
}

void MotoNet::servePortal() {
  String html = PORTAL_HEAD;
  html += _scanOptions;
  html += PORTAL_TAIL;
  _server.send(200, "text/html; charset=utf-8", html);
}

void MotoNet::serveSubmit() {
  String sel = _server.arg("ssid_select");
  String ssid = _server.arg("ssid");
  String pass = _server.arg("password");
  if (sel.length() > 0 && sel != "__other__") ssid = sel;
  ssid.trim();
  if (ssid.length() == 0) {
    _server.send(200, "text/html; charset=utf-8",
                 "<meta charset='utf-8'>SSID 不能为空 <a href='/'>返回</a>");
    return;
  }
  savePrefs(ssid, pass);
  _server.send(200, "text/html; charset=utf-8",
               String("<meta charset='utf-8'>已保存，正在连接 <b>") +
               htmlEscape(ssid) + "</b> ...<br>成功后配网热点将自动关闭。");
  Serial.printf("[net] saved wifi %s, reconnecting\n", ssid.c_str());
  _reconnectRequested = true;      // 响应发出后由 tick() 统一重连，避免在 handler 内改 WiFi 模式
}

void MotoNet::tick() {
  switch (_state) {
    case NET_STA_CONNECTING: {
      if (WiFi.status() == WL_CONNECTED) {
        _state = NET_STA_CONNECTED;
        Serial.printf("[net] connected, ip=%s\n", WiFi.localIP().toString().c_str());
      } else if (millis() - _staStartMs >= WIFI_STA_TIMEOUT_MS) {
        Serial.println("[net] STA connect timeout -> AP mode");
        enterApMode();
      }
      break;
    }
    case NET_STA_CONNECTED: {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[net] connection lost, reconnecting");
        enterStaConnecting();
      }
      break;
    }
    case NET_AP_MODE: {
      _dns.processNextRequest();
      _server.handleClient();
      if (_scanning) {
        int16_t r = WiFi.scanComplete();
        if (r == WIFI_SCAN_RUNNING) {
          // still scanning
        } else {
          _scanning = false;
          if (r >= 0) handleScanResult(r);
          else _scanOptions = "";
        }
      }
      if (_reconnectRequested) {
        _reconnectRequested = false;
        reconnect();
      }
      break;
    }
    default:
      break;
  }
}

String MotoNet::statusText() const {
  switch (_state) {
    case NET_STA_CONNECTED:  return "ONLINE";
    case NET_STA_CONNECTING: return "CONNECTING";
    case NET_AP_MODE:        return "AP MODE";
    default:                 return "OFF";
  }
}
