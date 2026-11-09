# GPX 记录相关说明

本文说明 MotoNav-CYD-Plus 的 **GPX 轨迹记录**机制：记录生命周期、轨迹点写入与质量过滤、海拔写入规则、文件命名、刷盘策略、断电恢复、CSV 统计与历史索引。对应源码 `MotoNav-CYD-Plus.ino` 中 `startTrack()` / `writeTrackPoint()` / `finishTrack()` / `trackPointQualityOk()` / `recoverInterruptedTrack()` 等函数，以及 `config.h` 中 `TRACK_*` / `MAX_*` / `MIN_*` 常量。

---

## 1. 记录生命周期

轨迹记录是一个显式的状态机（`TrackState`）：

```
TRACK_STOPPED ──START TRACK──▶ TRACK_WAIT_FIX ──(定位良好)──▶ TRACK_RECORDING ──FINISH──▶ TRACK_STOPPED
     ▲                                                           │
     └────────────────────（完成/保存/断电恢复）────────────────────┘
```

| 状态 | 含义 | 触发 |
|---|---|---|
| `TRACK_STOPPED` | 未记录 | 开机 / 结束记录 |
| `TRACK_WAIT_FIX` | 等待定位 | 用户点了 START TRACK 但尚无定位 |
| `TRACK_RECORDING` | 正在写 GPX | 定位良好后自动开始 |

- **开始**（`startTrack`）：先 `resetTrip(false)` 清零行程统计；若尚无有效定位则进入 `TRACK_WAIT_FIX`，否则立即打开 GPX 文件进入 `TRACK_RECORDING`。
- **等待定位期间**：每次有定位更新且质量合格，自动从 `TRACK_WAIT_FIX` 转入 `TRACK_RECORDING`。
- **结束**（`finishTrack`）：写入 GPX 结尾标签、刷盘、保存 CSV 统计、清除临时恢复文件。

---

## 2. GPX 文件结构与写入

### 2.1 文件头（`openTrackFile`）

打开文件后立即写入标准 **GPX 1.1** 头：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="MotoNav-CYD" xmlns="http://www.topografix.com/GPX/1/1">
  <trk><name>MotoNav ride</name><trkseg>
```

### 2.2 轨迹点（`writeTrackPoint`）

每个轨迹点写为：

```xml
    <trkpt lat="31.2345678" lon="121.4567890">
      <ele>23.45</ele>
      <time>2026-10-29T06:30:00Z</time>
      <hdop>0.90</hdop>
      <sat>12</sat>
    </trkpt>
```

字段规则：

| 字段 | 写入条件 | 格式 |
|---|---|---|
| `lat` / `lon` | 总是写 | `%.7f`（约 1.1 cm 精度） |
| `<ele>` 海拔 | 数值合理时才写（见 §3） | `%.2f` |
| `<time>` | GPS 日期/时间有效时 | UTC，`YYYY-MM-DDTHH:MM:SSZ` |
| `<hdop>` | `gps.hdop` 有效时 | `%.2f` |
| `<sat>` 卫星数 | `gps.satellites` 有效时 | 整数 |

> `<time>` 始终是 **UTC（带 `Z`）**，符合 GPX 规范，与文件名的「东八区本地时间」不同，勿混淆。
>
> 由于 GPS 的日期（来自 RMC 语句）与时间（来自 GGA 语句）是两条不同的 NMEA 报文，跨 UTC 午夜时 `time` 会先翻到 `00:00:00Z`，而 `date` 可能仍停留在前一天。固件在写 `<time>` 时内部跟踪「上一秒数」，检测到秒数回绕即自动把日期 `+1` 天（含月末/年末进位）；若 GPS 日期已追平/前进（如长时间失锁后恢复）则以 GPS 为准。这样午夜前后几分钟写出的日期始终正确。

### 2.3 文件尾（`finishTrack`）

结束时补上：

```xml
  </trkseg></trk>
</gpx>
```

---

## 3. 海拔 `<ele>` 写入规则（`gpxAltitudeMeters`）

保证 GPX 永远合法可导入，海拔只在「数值可靠」时才写：

1. 优先用融合海拔 `baroAltitudeM + baroOffsetM`（**要求 BMP280 就绪且已完成 GNSS 锚定** `baroAnchorSet`）；
2. 否则回退到纯 GNSS 海拔 `gps.altitude.meters()`；
3. 数值必须是有限值（非 NaN/Inf）且落在 `[-1000, +10000]` m 内。

三者都不满足时，**该点不写 `<ele>`**（整段 GPX 依然合法）。这样即使 I2C 偶发读到垃圾海拔，也不会污染 GPX。

---

## 4. 轨迹点质量过滤（`trackPointQualityOk`）

写入前逐点校验，丢弃异常点，防止跳变/噪声污染轨迹：

| 过滤项 | 阈值 | 行为 |
|---|---:|---|
| HDOP | > `MAX_HDOP_FOR_TRACK`（5.0） | 丢弃 |
| 卫星数 | < `MIN_SATELLITES_FOR_TRACK`（4） | 丢弃 |
| 时间间隔 | > `MAX_TRACK_POINT_GAP_MS`（5000 ms） | 当作断点，更新参考点，丢弃本点 |
| 相邻点距离 | < `MIN_TRACK_POINT_DISTANCE_M`（1.5 m） | 丢弃（太密） |
| 相邻点距离 | > `MAX_TRACK_SEGMENT_DISTANCE_M`（100 m） | 丢弃（跳变） |
| 推算速度 | > `MAX_VALID_SPEED_KMH`（220） | 丢弃（速度尖峰） |

### 写入节奏

- **最小间隔**：`TRACK_POINT_INTERVAL_MS = 1000 ms`，即最快约 1 秒写一个点。
- 距离过滤与时间过滤叠加，实际写入是「≥ 1 秒 且 间距 ≥ 1.5 m 且 质量合格」才落盘。

---

## 5. 文件命名（东八区）

- 文件名格式：`/TRK_YYYYMMDD_HHMMSS.GPX`，其中日期/时间用 **东八区（UTC+8）本地时间**（`localDateTime()`，`TZ_OFFSET_HOURS = 8`，自动处理跨天/跨月/跨年）。
- 若 GPS 时间尚未有效，退化为 `/TRK_<millis>.GPX`。

配套文件：

| 文件 | 说明 |
|---|---|
| `TRK_*.CSV` | 与 GPX 同名的统计文件（见 §7） |
| `RIDES_INDEX.CSV` | 全部行程的紧凑索引 |
| `ACTIVE_TRACK.TXT` | 记录中的临时恢复标记（存当前 GPX 文件名） |
| `ACTIVE_STATS.CSV` | 记录中的统计快照（每 10 秒更新） |

---

## 6. 刷盘与断电恢复

### 6.1 刷盘（flush）

- `TRACK_FLUSH_INTERVAL_MS = 2000 ms`：每 2 秒把缓冲区写入 SD 卡一次，兼顾数据安全与写入寿命。
- 写失败（如 SD 异常）会立即停止记录并标记 `SD ERROR`，避免继续写坏文件。

### 6.2 断电恢复流程

记录过程中随时可能断电。为把损失降到最小：

1. **标记**：开始记录时写 `ACTIVE_TRACK.TXT`，记录当前 GPX 文件名。
2. **快照**：记录中每 `STATS_SNAPSHOT_INTERVAL_MS = 10000 ms`（10 秒）把累计统计快照写进 `ACTIVE_STATS.CSV`，字段为：
   ```
   distanceM,movingTimeMs,totalTimeMs,maxSpeedKmh,pointCount,date,time
   ```
3. **断电**。
4. **下次开机**（`recoverInterruptedTrack`）：
   - 读到 `ACTIVE_TRACK.TXT` → 找到未闭合的 GPX；
   - 若 GPX 没有 `</gpx>` 结尾，自动补上 `</trkseg></trk></gpx>`；
   - 再读 `ACTIVE_STATS.CSV`，重建 `.CSV` 统计并补进 `RIDES_INDEX.CSV`，让这条行程出现在历史列表。
5. **正常结束**时两个临时文件都会被删除。

> **恢复精度**：断电瞬间最多丢失最后 10 秒内的增量（主要是总时长、以及最后 10 秒还在移动的距离）。停稳后再断电基本无损。

---

## 7. CSV 统计与历史索引

### 7.1 单段行程统计 `TRK_*.CSV`（`writeRideStatistics`）

```
version,date,time,gpx_file,distance_km,total_seconds,moving_seconds,stopped_seconds,average_kmh,maximum_kmh,gpx_points,completed
1,2026-10-29,14:30:00,/TRK_...,12.345,3600,3200,400,38.6,102.3,2450,1
```

字段含：版本、日期/时间（东八区）、GPX 文件名、总里程 km、总/移动/停车时长秒、平均/最高速度、轨迹点数、是否完成。

### 7.2 历史索引 `RIDES_INDEX.CSV`（`appendRideIndex`）

```
version,date,time,gpx_file,distance_km,total_seconds,moving_seconds,stopped_seconds,average_kmh,maximum_kmh,gpx_points,uploaded
1,2026-10-29,14:30:00,/TRK_...,12.345,3600,3200,400,38.6,102.3,2450,0
```

设备内「RIDES」历史列表读取此文件，显示最近 **20** 条行程摘要（`MAX_RIDE_HISTORY = 20`）。末列 `uploaded` 表示上传状态：`0`=待上传 / `1`=已上传 / `2`=已忽略（详见《自动上传与ntfy推送.md》）。

---

## 8. 异常防护汇总

| 场景 | 处理 |
|---|---|
| 速度尖峰（> 220 km/h） | 轨迹点丢弃 / 距离不累计 |
| 距离跳变（段距 > 100 m） | 轨迹点丢弃 / 距离不累计 |
| 点过密（间距 < 1.5 m） | 轨迹点丢弃 |
| 海拔异常（NaN/超范围） | 该点不写 `<ele>`，GPX 仍合法 |
| 卫星 < 4 / HDOP > 5 | 轨迹点丢弃 |
| SD 写失败 | 停止记录 + `SD ERROR` 提示 |
| 突然断电 | 开机自动补 GPX 结尾 + 重建 CSV 统计 |

---

## 9. 关键参数（`config.h`）

| 常量 | 默认 | 说明 |
|---|---:|---|
| `MIN_SATELLITES_FOR_TRACK` | 4 | 轨迹点最少卫星数 |
| `MAX_HDOP_FOR_TRACK` | 5.0 | 轨迹点 HDOP 上限 |
| `MIN_TRACK_POINT_DISTANCE_M` | 1.5 | 轨迹点最小间距 |
| `MAX_TRACK_SEGMENT_DISTANCE_M` | 100.0 | 轨迹点最大段距 |
| `MAX_TRACK_POINT_GAP_MS` | 5000 | 轨迹点最大时间间隔（超时算断点） |
| `MAX_VALID_SPEED_KMH` | 220.0 | 超过视为非法速度 |
| `TRACK_POINT_INTERVAL_MS` | 1000 | 轨迹点写入间隔 |
| `TRACK_FLUSH_INTERVAL_MS` | 2000 | 刷盘间隔 |
| `STATS_SNAPSHOT_INTERVAL_MS` | 10000 | 断电恢复快照间隔 |
| `MAX_RIDE_HISTORY` | 20 | 设备内历史列表条数 |

---

## 10. 兼容性

GPX 为标准 GPX 1.1，可在以下工具直接打开：

- **QGIS**、**Google Earth Pro**、**GPXSee**、**OsmAnd**、**BaseCamp** 等；
- 含 `<ele>`（海拔）、`<time>`、`<hdop>`、`<sat>` 扩展字段，缺失时（如海拔异常点）仍合法。

---

## 11. 相关代码位置

| 函数 | 位置 |
|---|---|
| `startTrack()` / `finishTrack()` | `MotoNav-CYD-Plus.ino` |
| `writeTrackPoint()` / `trackPointQualityOk()` | `MotoNav-CYD-Plus.ino` |
| `openTrackFile()` / `makeTrackFileName()` / `localDateTime()` | `MotoNav-CYD-Plus.ino` |
| `gpxAltitudeMeters()` | `MotoNav-CYD-Plus.ino` |
| `writeActiveStatsSnapshot()` / `recoverInterruptedTrack()` / `recoverRideStatistics()` | `MotoNav-CYD-Plus.ino` |
| `writeRideStatistics()` / `appendRideIndex()` | `MotoNav-CYD-Plus.ino` |
| `TRACK_*` / `MAX_*` / `MIN_*` 常量 | `config.h` |
