#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

// 网络状态
enum NetState {
  NET_OFF = 0,          // 尚未启动
  NET_STA_CONNECTING,   // STA 正在连接已保存热点
  NET_STA_CONNECTED,    // 已连上互联网
  NET_AP_MODE           // 无密码 AP 配网模式
};

// 非阻塞 WiFi 管理器：开机自动连已保存热点，连不上则开无密码 AP 供手机配网。
// 主循环每轮调用 tick()，绝不阻塞（配网页面的 DNS/HTTP 请求逐条在后台处理）。
// 注意：类名/全局名避开 ESP32 core 3.x 自带的 NetworkManager / Network。
class MotoNet {
public:
  void begin();                    // 读取配置并启动（有凭据 → STA 连接，否则进 AP）
  void tick();                     // 每轮 loop() 调用一次
  bool connected() const { return _state == NET_STA_CONNECTED; }
  bool apMode() const { return _state == NET_AP_MODE; }
  NetState state() const { return _state; }
  const String &ssid() const { return _ssid; }
  bool hasConfig() const { return _ssid.length() > 0; }
  String ipText() const { return WiFi.localIP().toString(); }
  String statusText() const;       // 简短状态，供设置页瓦片副标题
  void startConfigPortal();        // 手动进入 AP 配网模式
  void reconnect();                // 退出 AP，用当前凭据重新连 STA

private:
  void loadPrefs();
  void savePrefs(const String &ssid, const String &pass);
  void enterStaConnecting();
  void enterApMode();
  void handleScanResult(int count);
  void servePortal();
  void serveSubmit();

  NetState _state = NET_OFF;
  String _ssid;
  String _pass;
  uint32_t _staStartMs = 0;
  bool _scanning = false;
  String _scanOptions;             // 缓存的附近热点 <option> 列表
  bool _reconnectRequested = false;
  WebServer _server;
  DNSServer _dns;
};

extern MotoNet motoNet;
