# 自动上传与 ntfy 推送说明

本文说明 MotoNav-CYD-Plus 中「连网后自动上传 GPX 轨迹」与「上传结果 ntfy 推送」的完整逻辑，对应源码：

- `network.h` / `network.cpp` —— WiFi 连网状态机（类 `MotoNet`，全局 `motoNet`）
- `upload.h` / `upload.cpp` —— 上传与推送状态机（类 `UploadManager`，全局 `Upload`）
- `secret.h` —— 密钥与推送配置（`TRACKS_API_*` / `NTFY_SH_*`）
- `config.h` —— `WIFI_*` / `UPLOAD_*` / `MAX_BATCH` 常量

---

## 1. 总体流程

```
开机
 └─ MotoNet.begin()：读已存凭据
      ├─ 有凭据 ──► STA 连接（最多 25s）
      │               ├─ 连上 ──►「刚连上网」边沿事件
      │               └─ 超时 ──► 进无密码 AP 配网（此后 STA 空闲，不轮询）
      └─ 无凭据 ──► 进无密码 AP 配网

「刚连上网」/ 行程结束 / 手动触发 / 失败周期重试
 └─ UploadManager 触发一次「批次」上传
      ├─ 一次性抓取全部 uploaded==0 的行程
      ├─ 逐个流式 HTTPS 上传（成功标记 uploaded=1，失败保持 0）
      └─ 整批结束 ──► 若实际尝试过 ──► 推送一条 ntfy 通知
```

两条主线——`motoNet.tick()`（WiFi）与 `Upload.tick()`（上传/推送）——都在主循环 `loop()` 里每轮调用，全部是非阻塞状态机，互不阻塞。

---

## 2. WiFi 连网（前置）

`MotoNet` 是非阻塞状态机，状态定义见 `network.h` 的 `NetState`：

| 状态 | 含义 | 行为 |
|---|---|---|
| `NET_OFF` | 未启动 | `begin()` 前的初始态 |
| `NET_STA_CONNECTING` | STA 连接中 | 每轮查 `WiFi.status()`；连上 → `NET_STA_CONNECTED`；超过 `WIFI_STA_TIMEOUT_MS` → 进 AP |
| `NET_STA_CONNECTED` | 已连上互联网 | 检测到掉线 → 重新进入 `NET_STA_CONNECTING` |
| `NET_AP_MODE` | 无密码配网热点 | 处理 DNS 劫持 + 配网页；扫描一次附近热点；STA 空闲 |

关键设计（对应「少做无用功」诉求）：

1. **开机只试一次**：有凭据 → `WiFi.begin()` 异步连接，最多 `WIFI_STA_TIMEOUT_MS`（25 秒）。这期间只被动查 `WiFi.status()`，不主动扫描、不额外计算。
2. **连不上就停**：25 秒内连不上（或本来就没凭据）→ 进入 `NET_AP_MODE`，软 AP 发 beacon 供手机配网，**STA 完全空闲**：不扫描、不重连、不后台折腾。
3. **已连上后掉线会重连**：`NET_STA_CONNECTED` 里检测到掉线会重新走 `enterStaConnecting()`，再给 25 秒；仍连不上才落回 AP 模式。这是「连上过之后」的唯一自动重连，且同样有超时上限，不会无限循环。
4. **配网页扫描只扫一次**：进 AP 模式时用 `WiFi.scanNetworks(true)` 异步扫一遍填充下拉列表，之后不再扫。
5. **无线协议栈跑 core 0**，主循环（GPS / 显示 / 记录）在 core 1，基本不抢算力。

配网交互：手机连上 `MotoNav-AP`（无密码，仅 2.4GHz），浏览器打开任意页面被 DNS 劫持到配网页 → 选择或输入 SSID + 密码 → 提交 → `savePrefs()` 写 NVS（命名空间 `netcfg`，键 `ssid` / `pass`）→ 由 `tick()` 统一 `reconnect()` 退出 AP、连 STA。

> 凭据只存**一组**（家里 WiFi 或手机热点二选一，切换需重新配网一次），不做 WiFiMulti 多热点自动挑选，以把开销压到最小。

---

## 3. 「连上网」如何触发上传（边沿检测）

`Upload.tick()` 里维护一个 `static bool wasConnected`，对 `motoNet.connected()` 做**上升沿检测**：

```cpp
const bool nowConnected = motoNet.connected();
if (nowConnected && !wasConnected && !_batchActive) {
  _scanRequested = true;   // 刚连上网 → 触发一次上传扫描
}
wasConnected = nowConnected;
```

- 只在「断开 → 连上」的这一拍置位 `_scanRequested`，不会在连接保持期间反复触发。
- 批次进行中（`_batchActive`）不重复触发，避免断线重连造成一秒钟推多条。

`_scanRequested` 只是「待办标记」，真正的上传在 `tick()` 空闲态统一执行，且要同时满足 `_enabled`（开关开）与 `motoNet.connected()`（在线）。

---

## 4. 自动上传的触发时机

一共有四类触发来源，最终都落到 `beginBatch()`：

| 触发来源 | 入口 | 说明 |
|---|---|---|
| 开机后首次连上 | 边沿检测置 `_scanRequested` | 家里测试 / 手机热点开启后自动跑一次 |
| 行程结束 | `finishTrack()` 里调 `Upload.requestScan()` | 正常停止记录、断电恢复补完统计后都会走到 |
| 手动 | 设置页第 2 页「立即上传」按钮调 `Upload.requestScan()` | 调试 / 手动补传 |
| 失败周期重试 | `tick()` 里 `_lastResult==2` 且距上次结束 ≥ `UPLOAD_RETRY_INTERVAL_MS`（5 分钟） | 上一批有失败时，5 分钟后自动重试仍为 `0` 的文件 |

`requestScan()` 只是置位 `_scanRequested`，**不立即联网**。没网时标记会一直留着，等连上网后（边沿又置位一次）在空闲态统一执行；因此「行程结束时没网，之后才连上」也能补传。

---

## 5. 批次上传流程（UploadManager）

上传采用**批次模型**，核心是「一次抓全、逐个传、失败不立即重试、整批一次推送」。状态机 `Step` 有四态：`UP_IDLE` / `UP_SENDING` / `UP_READ_STATUS` / `UP_NOTIFY`。

### 5.1 `beginBatch()` —— 抓取待上传列表

1. 重置计数（`_pendingCount/_pendingIndex` 及分目标的 `_batchOk*/_batchFail*`），置 `_batchActive = true`。
2. 若 TRACKS 与 WebDAV 均未配置 → 直接放弃本批，**不标记任何文件、不推送**。
3. 读 SD 卡 `RIDES_INDEX.CSV`，把 `uploaded == 0` 的行（第 4 列 `gpx_file` 路径）收集进 `_pendingList[]`，最多 `MAX_BATCH`（32）条。
4. 否则调 `resumeBatch()` 开始推进（每条文件依次向「已配置的目标」上传）。

### 5.2 `resumeBatch()` —— 每次 tick 最多连接一个目标

- **快速跳过**缺失 / 打不开的文件（纯 SD 操作，可安全循环，计入「对应已配置目标」的失败）；
- 每条文件按「TRACKS → WebDAV」顺序**逐目标**上传：一次只对当前文件的一个目标做 TLS 连接；
- 连接失败则计入该目标失败、`++_target`、`_step = UP_IDLE`，**下一个目标留到下一轮 tick**——绝不在同一次调用里循环连接多个目标，避免网络不通时阻塞主循环；
- 一条文件需向**所有已配置目标**都成功才 `markUploaded(file, 1)`，否则保持 `0`（下一周期重试；TRACKS 服务端按内容去重，重复上传无害）。

### 5.3 `UP_SENDING` —— 流式上传

- 每轮从文件读 `UPLOAD_CHUNK_SIZE`（4096 字节）写入 socket，直到文件读完；
- 写完补 multipart 结束边界，转 `UP_READ_STATUS`；
- 单文件总超时 `UPLOAD_SEND_TIMEOUT_MS`（30 秒），超时或写失败 → 计入失败，继续下一条。

上传请求格式：

```
POST /api/ingest HTTP/1.1
Host: tracks.example.com
X-API-Key: <TRACKS_API_KEY>
Content-Type: multipart/form-data; boundary=----MotoNavBoundary
Content-Length: <n>
Connection: close

--...（multipart 字段名 file，文件名取 GPX 文件名，Content-Type: application/gpx+xml）
```

### 5.4 `UP_READ_STATUS` —— 判定结果

- 读 HTTP 状态行：`200` / `201`（TRACKS）或 `200` / `201` / `204`（WebDAV PUT）→ 该目标成功；
- 其他（如 `401` Key/密码错）或无响应 → 该目标失败；
- 记录到 `_fileTracksOk` / `_fileWebDavOk`，`++_target` 并回 `UP_IDLE`，**由下一轮 tick 的 `resumeBatch()` 继续下一个目标**。失败的同一条本批不再重试（这是相对旧版「失败紧循环重试」的修复）。

### 5.5 `finalizeBatch()` —— 收尾

- `_batchActive = false`；
- 记录 `_lastResult`：全成功 → `1`；有失败 → `2`；无任何尝试 → `0`；
- 记录 `_lastAttemptMs`（供 5 分钟周期重试计时）；
- 若各目标成功与失败合计都为 0（上传代码没实际做事）→ 直接回 `UP_IDLE`，**不推送**；
- 否则调 `notifyStart()`。

---

## 6. RIDES_INDEX.CSV 的 `uploaded` 字段

每行末尾第 12 列（索引 11）为 `uploaded`：

| 值 | 含义 | 由谁写入 |
|---|---|---|
| `0` | 待上传 | 新行程默认；上传失败保持 |
| `1` | 已上传 | 上传成功后 `markUploaded(file, 1)` |
| `2` | 已忽略 | 「RIDES」历史里长按切换，`markUploaded(file, 2)` |

- 被忽略（`2`）的行程**不会**进入待上传列表，也不会上传；
- 旧版 `RIDES_INDEX.CSV` 是 11 列（无 `uploaded` 列），读取时缺列按 `0`（待上传）处理，因此升级后旧轨迹会被自动识别为待上传；
- `markUploaded()` 通过「读原文件 → 写临时文件 → 原子替换」的方式改写 CSV，避免断电损坏索引。

---

## 7. ntfy 推送逻辑

上传结果通过 **ntfy 的 JSON 发布接口**推送（你 `byd_checkin_autox` / `ithome_push` 里同款接口）：POST 到服务器根 `/`，body 为 JSON。

### 7.1 推送时机

- 只要**本批实际尝试过上传**（`_batchOk` 或 `_batchFail` 至少一个 > 0），整批结束就推**一条**；
- 多条轨迹全部上传完**只推一条**，不是逐条推；
- **无论成功 / 失败都推**；完全没有待上传文件（没做事）则不推。

### 7.2 推送内容

`notifyStart()` 组装 JSON body：

```json
{
  "topic":   "cyd_gpx_auto_upload_notify",
  "title":   "CYD GPX 上传结果",
  "message": "cyd gpx文件上传情况：tracks成功n条失败m条；webdav成功n条失败m条",
  "priority": 3
}
```

- `title`：`CYD GPX 上传结果`
- `message`：按「实际配置的目标」拼接——仅 TRACKS：`cyd gpx文件上传情况：tracks成功{n}条失败{m}条`；仅 WebDAV：`cyd gpx文件上传情况：webdav成功{n}条失败{m}条`；两者都配：`…：tracks成功{n}条失败{m}条；webdav成功{n}条失败{m}条`
- `priority`：**全部目标均无失败 = 3**（默认），**任一目标有失败 = 4**（更醒目）
- `topic`：取 `NTFY_SH_PUSH_TOPIC`

串口日志（`notifyStep()` 收尾时）：

```
[up] 已推送 ntfy 通知：成功3条，失败1条
```

### 7.3 非阻塞实现

- `notifyStart()`：解析 `NTFY_SH_SERVER` 得到 host / port / path，复用同一个 `WiFiClientSecure` 连接、发送 POST，转 `UP_NOTIFY`；
- `notifyStep()`：读状态行，`200` 视为服务器确认；超时 / 断开则收尾并带「（服务器未确认）」后缀；
- 连接失败不重试、不阻塞，直接跳过（`notify skipped`）。

### 7.4 TLS 校验

上传（TRACKS / WebDAV）与 ntfy 推送共用一个内置的 Let's Encrypt 根证书 **ISRG Root X1**，不跳过证书校验。WebDAV 服务器若用自签 / 内部 CA 证书，需把 `upload.cpp` 里的 `ISRG_ROOT_X1` 换成对应根证书 PEM。

---

## 8. 配置项汇总

### 8.1 `secret.h`（已加入 `.gitignore`，模板见 `secret.h.example`）

| 配置 | 默认 | 说明 |
|---|---|---|
| `TRACKS_API_HOST` | `tracks.example.com` | 上传服务主机名 |
| `TRACKS_API_PORT` | `443` | 上传服务端口（HTTPS） |
| `TRACKS_API_KEY` | （占位符） | 上传认证，请求头 `X-API-Key`；留空/占位符则不启用 TRACKS 上传 |
| `WEBDAV_SERVER_URL` | （空） | WebDAV 目录基址（如 `https://dav.example.com/moto/`），设备 PUT 到该目录下 |
| `WEBDAV_USERNAME` | （空） | WebDAV Basic 认证用户名 |
| `WEBDAV_PASSWORD` | （空） | WebDAV Basic 认证密码 |
| `NTFY_SH_SERVER` | `https://ntfy.sh` | ntfy 服务器（官方 / 自建实例） |
| `NTFY_SH_PUSH_TOPIC` | `cyd_gpx_auto_upload_notify` | 推送主题 |

### 8.2 `config.h`

| 常量 | 默认 | 说明 |
|---|---:|---|
| `WIFI_STA_TIMEOUT_MS` | 25000 | 开机 STA 连接超时，失败进 AP 配网 |
| `WIFI_AP_SSID` | `MotoNav-AP` | 配网热点名（无密码，仅 2.4GHz） |
| `UPLOAD_RETRY_INTERVAL_MS` | 300000 | 失败后周期重试间隔（5 分钟） |
| `UPLOAD_SEND_TIMEOUT_MS` | 30000 | 单个文件上传总超时 |
| `UPLOAD_CHUNK_SIZE` | 4096 | GPX 流式上传分块字节数 |
| `MAX_BATCH` | 32 | 单批最多记忆 / 处理的待上传条数 |

### 8.3 其他

- 自动上传开关存 NVS 命名空间 `netcfg`，键 `auto_upload`（`Upload.begin()` 读取，`setEnabled()` 写入）。

---

## 9. 使用自建 ntfy 实例

默认用官方 `https://ntfy.sh`。要换自建实例，只改 `secret.h` 一处：

```cpp
#define NTFY_SH_SERVER "https://ntfy.example.com"
```

`parseHttpUrl()` 支持 `https://主机[:端口][/路径]` 形式，例如 `https://ntfy.example.com` 或带端口的 `https://host:8443`。

注意两点：

1. **证书**：自建实例若用 Let's Encrypt 证书，内置的 ISRG Root X1 可直接校验；若用自签 / 内部 CA，需把 `upload.cpp` 里的 `ISRG_ROOT_X1` 换成对应根证书 PEM。
2. **主题**：手机 ntfy App 里订阅与 `NTFY_SH_PUSH_TOPIC` 同名的主题即可收到推送。

---

## 10. 行为一览

| 场景 | 结果 |
|---|---|
| 开机有凭据且热点在 | 连上后自动上传全部待上传轨迹，推一条「成功 n，失败 0」 |
| 开机无凭据 / 25s 连不上 | 进 `MotoNav-AP` 配网；STA 空闲不再扫描重连，不耗算力 |
| 家里 WiFi 首测（3 条旧轨迹） | 配网后连上 → 边沿触发 → 3 条（旧 11 列按 0）全部上传，推一条 |
| 上传中网络中断 | 当前文件失败，剩余文件逐个失败；本批收尾推一条「成功 a，失败 b」，5 分钟后重试 |
| 某条返回 401（Key 错） | 该条计入失败、保持 `0`，继续下一条；推「失败」通知 |
| 重复上传同一轨迹 | 服务端按内容去重返回 `200`，按成功处理并标记 `1` |
| 没有待上传文件 | 触发后不执行任何网络请求，**不推送** |
| 自动上传开关关闭 | `tick()` 空闲态直接返回，不发起任何上传 / 推送 |
| 「RIDES」里长按忽略某条 | 该行 `uploaded=2`，永不进入上传列表 |
| 只配 TRACKS（未配 WebDAV） | 每条只传 TRACKS，成功置 `1`，通知只报 `tracks` |
| 只配 WebDAV（未填 TRACKS Key） | 每条只 PUT 到 WebDAV，成功置 `1`，通知只报 `webdav` |
| TRACKS + WebDAV 都配 | 每条依次传两处，**两处都成功才置 `1`**；通知同时报 `tracks` 与 `webdav` |
| 两者都未配 | 触发后不启动批次、不标记、不推送 |

---

## 11. 相关代码位置

| 功能 | 位置 |
|---|---|
| WiFi 状态机 `MotoNet` | `network.h` / `network.cpp` |
| 上传 + 推送状态机 `UploadManager` | `upload.h` / `upload.cpp` |
| 上传触发（行程结束） | `MotoNav-CYD-Plus.ino` 的 `finishTrack()`（`Upload.requestScan()`） |
| 设置页开关 / 手动触发 | `MotoNav-CYD-Plus.ino` 的 `handleSettingsTap()` |
| 主循环接入 | `MotoNav-CYD-Plus.ino` 的 `setup()` / `loop()`（`motoNet.tick()` / `Upload.tick()`） |
| 证书 `ISRG_ROOT_X1` | `upload.cpp` |
| 密钥 / 推送配置 | `secret.h` |
| 网络 / 上传常量 | `config.h` |

---

## 12. WebDAV 上传（可选，与 TRACKS 并存）

在 `secret.h` 里增加三项（模板见 `secret.h.example`）：

```cpp
#define WEBDAV_SERVER_URL "https://dav.example.com/moto/"
#define WEBDAV_USERNAME  "你的用户名"
#define WEBDAV_PASSWORD  "你的密码"
```

要点：

1. **触发时机与 TRACKS 完全一致**：自动上传开关打开、或设置页点「立即上传」时执行；没有单独的 WebDAV 开关。
2. **上传方式**：HTTP `PUT` 到 `<WEBDAV_SERVER_URL>/<文件名>.GPX`，头带 `Authorization: Basic <base64(user:pass)>`、`Content-Type: application/gpx+xml`、`Content-Length` 为文件字节数；成功码认 `200` / `201` / `204`。
3. **目录要求**：`WEBDAV_SERVER_URL` 指向的集合目录需在服务端**预先存在**（固件不会自动 MKCOL）；结尾斜杠可省略，固件会自动补齐。
4. **启用判定**：URL（须 `https://`）、用户名、密码三项都非空才算已配置；否则跳过 WebDAV 上传。
5. **只上传 GPX**：上传列表只含 `.GPX`（源自 `RIDES_INDEX.CSV` 的 `gpx_file` 列），`.CSV` 统计文件从不参与上传。
6. **证书**：与 TRACKS/ntfy 共用内置的 Let's Encrypt 根证书；自签/内部 CA 需替换 `ISRG_ROOT_X1`（见 §7.4）。
