# 机载 GNSS 接收机完好性监测（RAIM/FDE）回放服务

本服务在地面用录好的飞行数据验证机载 GNSS 完好性监测软件。下一阶段试飞的接收机
为**多模**（GPS / Galileo / 北斗同收）：同一历元三套系统的伪距一起进入加权最小
二乘定位、卡方检测、逐星试排与保护级；除位置外为**每套参与系统各估一个钟差**
（GPS 为基准），系统间时间偏差**按历元自由估计、不跨历元携带**（选型理由见
docs/design.md §1.2）。单历元这层沿用既有逻辑（仅把 4 未知量推广为 3+系统数），
跨历元仍保留：

- 被排除的星（按**系统+编号**区分）进入**隔离**，必须连续满足若干历元“本星残差
  正常且加回后整体检验通过”才恢复，隔离期间不参与解算、但仍逐历元算检验量；
- 告警采用**持续性窗口**与**恶劣超限立即告警**的组合判定，抑制残差尖峰闪烁。

**向后兼容是硬约束**：没有系统标识的旧星/旧会话一律按 GPS 直接接着提交；已经
落盘的历元记录不被改写；只含 GPS 的输入逐历元结果与升级前严格一致
（黄金回放 + 全套旧测试钉死）。

后端 Go 1.22 + Echo，运行档与会话状态以文件形式持久化，支持重启续跑。

## 目录结构（各包单一职责）

| 包 | 职责 |
|---|---|
| `pkg/gnss` | 系统标识（GPS/GAL/BDS）与“系统+编号”卫星身份（隔离/排除/落盘的统一键） |
| `pkg/geo` | WGS84 ECEF↔经纬度高、ENU 旋转矩阵 |
| `pkg/lsq` | 多模加权最小二乘迭代定位（每系统一个钟差列）、残差投影 S、DOP、RAIM 斜率、字段校验 |
| `pkg/chisq` | 自研卡方/非中心卡方 CDF 与分位数（不完全伽马函数，**不引统计库**） |
| `pkg/detect` | 残差平方和卡方检测、逐星排除（唯一可解释才排除）、冗余度随系统数推广 |
| `pkg/protect` | 斜率法水平保护级 HPL（非中心卡方漏检门限，多钟差列几何） |
| `pkg/profile` | 具名运行档（航路/终端区/NPA + 调用方自建） |
| `pkg/statem` | 跨历元隔离/恢复计数（系统+编号）、告警持续性状态机（纯逻辑） |
| `pkg/session` | 会话编排、系统标识兼容/校验、时间戳去重/倒退、统计、文件持久化 |
| `pkg/server` | Echo HTTP 路由 |
| `internal/sim` | 固定种子的确定性仿真（单模与多模，测试与回放演示共用） |
| `cmd/server` | 服务入口 |
| `cmd/replaydemo` | 三种告警判定回放对比，输出 `docs/replay_results.json`（GPS-only 黄金文件） |

## 快速开始

```bash
# 构建并运行（数据目录即运行档与会话落盘处，应挂载持久卷）
go build -o raim-server ./cmd/server
./raim-server --addr :8080 --data ./data

# Docker 两阶段构建
docker build -t raim:latest .
docker run -p 8080:8080 -v $(pwd)/data:/data raim:latest

# 跑测试
CGO_ENABLED=0 go test ./...

# 三种告警判定回放对比（固定种子，结果与 docs/replay_results.json 一致）
go run ./cmd/replaydemo --seed 20260930
```

详细接口见 [docs/api.md](docs/api.md)，算法与跨历元选型见
[docs/design.md](docs/design.md)，运行档参数与出处见 [docs/profiles.md](docs/profiles.md)。
