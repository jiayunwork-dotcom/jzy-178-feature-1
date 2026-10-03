# HTTP API

所有接口返回 JSON。错误形如 `{"error":"...","field":"..."}`，字段类问题为 400，
重复时间戳 409，时间戳倒退 422，资源不存在 404。

## 运行档

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/profiles` | 列出全部运行档（内置+自建） |
| GET | `/api/profiles/:name` | 取单个运行档 |
| POST | `/api/profiles` | 新建运行档（字段见 profiles.md） |

## 会话

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/sessions` | 列出会话 ID |
| POST | `/api/sessions` | 创建会话，body `{"id":"可选","profile_name":"enroute"}`（缺 id 自动生成） |
| GET | `/api/sessions/:id` | 会话状态、统计与逐历元记录 |
| POST | `/api/sessions/:id/epochs` | 提交单个历元 |
| POST | `/api/sessions/:id/epochs/batch` | 批量提交（`epochs` 数组，≤3600，整段原子） |

另有 `GET /healthz`。

## 历元请求体

```json
{
  "timestamp": 100,
  "approx": [4096000.0, 3096000.0, 3830000.0],
  "satellites": [
    {"system": "GPS", "id": 1, "pos": [x, y, z], "pr": 22123456.7, "sigma": 1.2},
    {"system": "GAL", "id": 1, "pos": [x, y, z], "pr": 23523456.7, "sigma": 1.2},
    {"system": "BDS", "id": 1, "pos": [x, y, z], "pr": 21623456.7, "sigma": 1.2}
  ]
}
```

- `system`：卫星所属系统，仅接受精确大写 `GPS` / `GAL` / `BDS`。
  **缺省（旧客户端/旧数据）按 `GPS` 处理。**
- 编号只在**同一系统内**不可重复：GPS 3 与北斗 3 是两颗不同的星，可同时出现。
- `timestamp`：单调时间戳；相同则 409、倒退则 422，均不推进状态。
- `approx`：可选概略位置；省略时沿用上一历元定位，首历元由卫星反推地表点。
- 定位至少需要 `3 + 系统数` 颗星（3 个位置未知量 + 每个系统 1 个钟差）；
  `sigma>0`；坐标/伪距不得为 NaN/无穷。
- 未知 `system` 报 `satellites[i].system`；同系统重号报 `satellites[i].id`；
  批量中再带历元下标（如 `epochs[3].satellites[1].system`）。

## 多模时间基准

同一历元三套系统的伪距一起参与定位、检测、排除与保护级。除位置外，解算为每个
参与系统各估一个钟差未知量：GPS 列作为基准（其系数即接收机钟差），Galileo/北斗
列的系数是该系统时相对 GPS 时的偏差。该偏差**按历元自由估计、不跨历元携带**
（理由与代价见 design.md §1.2）：某系统全部伪距同加常数时，台阶在出现的第一个
历元即被该系统钟差完全吸收，位置与残差不变、不检出。

## 历元响应（节选）

```json
{
  "seq": 9, "timestamp": 100,
  "profile_name": "enroute", "pfa": 1e-5, "pmd": 1e-3, "hal": 3704,
  "dof": 21, "chi_square_threshold": 40.0,
  "mode": "excluded",
  "sse": 64.2, "hpl": 10.7,
  "excluded_id": 4, "excluded_system": "GPS",
  "isolated": [{"system":"GPS","id":4}],
  "position": {"ecef":[...],"lon":116.39,"lat":39.90,"alt":50.1},
  "clock_bias": 12.3, "iterations": 4, "converged": true,
  "system_clocks": [
    {"system":"GPS","clock_bias":12.3,"used":true,"sat_count":8},
    {"system":"GAL","clock_bias":-45.6,"used":true,"sat_count":9},
    {"system":"BDS","clock_bias":78.9,"used":true,"sat_count":9}
  ],
  "sat_results": [
    {"system":"GPS","id":1,"resid":0.2,"sigma":1.2,"stdres":0.16},
    {"system":"BDS","id":3,"resid":0.1,"sigma":1.2,"stdres":0.08}
  ],
  "trials": [{"system":"GPS","excluded_id":1,"sse":40.1,"passes":false,"used":26}],
  "alert": false, "bad_streak": 0, "good_streak": 1, "raim_available": true
}
```

- `system_clocks`：三套系统本历元的时间偏差（相对 GPS 时，米）与是否实际参与、
  入解星数；未参与的系统 `used=false`。
- `mode` ∈ `ok` / `unavailable` / `detected` / `excluded`。
- 为兼容旧消费方，**只含 GPS 时**响应保持升级前形态：隔离引用写裸编号
  （`"isolated":[4]`），`sat_results`/`trials` 的 GPS 星省略 `system`，
  `excluded_system` 省略。

## 会话状态响应中的统计

```json
{
  "stats": {
    "epochs": 24,
    "raim_available_epochs": 24,
    "alert_epochs": 0,
    "detections": 1,
    "alert_episodes": 0,
    "false_alarm_episodes": 0,
    "raim_availability": 1.0,
    "service_availability": 1.0
  },
  "state": {"isolated": {"GPS:4": {"id":4,"normal_streak":2,"since_epoch_seq":9}},
            "alert": {"active":false,"bad_streak":0,"good_streak":1}},
  "last_ts": 24
}
```

隔离星的键是 `"系统:编号"`（如 `GPS:4`、`BDS:3`）；升级前会话文件里的裸编号键
`"4"` 读取时按 GPS 解释。已经落盘的旧历元记录不会被改写，新历元按新格式追加。

## 批量与逐历元一致性 / 重启续跑

对同一会话，把 N 个历元放进 `/epochs/batch` 与逐个 POST 到 `/epochs` 产生的
**每一条记录、隔离状态、告警状态与统计完全相同**；批量中任一历元非法会整段
拒收，错误字段带历元下标。服务重启后对同一 `--data` 目录继续提交，结果与不中断
一致（本版时间偏差为逐历元白估计，无额外跨历元状态）。
