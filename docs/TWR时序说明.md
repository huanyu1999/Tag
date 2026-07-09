# 标签 TWR 时序说明

> 文件覆盖范围：`Core/application/dw_instance.h`、`Core/application/dw_main.c`、`Core/application/dw_instance_tag.c`

---

## 一、时序常量定义

**文件：`Core/application/dw_instance.h`，第 113–149 行**

```c
#define PRE_TIMEOUT                     5       // 前导码超时（PAC 单位）

// 按基站数量选 slot 时长（单位 ms）
#define ONE_SLOT_TIME_MS_110K           28      // 1个标签与2个基站完成完整TWR所需时间
#define ONE_SLOT_TIME_MS_850K           8
#define ONE_SLOT_TIME_MS_6P8M           9

// 6.8 Mbps 速率时序（UUS：微秒等效单位，1 UUS × 65536 = DW 计数器刻度）
#define FIRST_RESP_SEND_6P8M            900     // POLL_TX → 第1个RESP_RX 预期偏移
#define DATA_INTERVAL_TIME_6P8M         1100    // 相邻两条消息间隔
#define RESP_RX_TIMEOUT_6P8M            450     // 标签等 RESP 的超时窗口
#define FINAL_RX_TIMEOUT_6P8M           600     // 基站等 FINAL 的超时窗口
#define TAG_FINALE_SEND_BACK_6P8M       100     // 标签延后发 FINAL 的额外余量

// 850 Kbps
#define FIRST_RESP_SEND_850K            1300
#define DATA_INTERVAL_TIME_850K         1600
#define RESP_RX_TIMEOUT_850K            1000
#define FINAL_RX_TIMEOUT_850K           1300
#define TAG_FINALE_SEND_BACK_850K       300

// 110 Kbps
#define FIRST_RESP_SEND_110K            3000
#define DATA_INTERVAL_TIME_110K         3900
#define RESP_RX_TIMEOUT_110K            3800
#define FINAL_RX_TIMEOUT_110K           6000
#define TAG_FINALE_SEND_BACK_110K       1080
```

**说明：**

| 常量 | 含义 |
|---|---|
| `ONE_SLOT_TIME_MS_*` | 超级帧内单个 slot 的持续时间，决定标签测距频率 |
| `FIRST_RESP_SEND_*` | POLL 发出后，标签预期收到第一个 RESP 的 UWB 时刻偏移 |
| `DATA_INTERVAL_TIME_*` | 多基站顺序回 RESP 的间距；RESP[n] 时刻 = POLL_TX + FIRST + n×INTERVAL |
| `RESP_RX_TIMEOUT_*` | 标签每次开窗等 RESP 的最大等待时间，超时触发 `RX_TIMEOUT` 回调 |
| `TAG_FINALE_SEND_BACK_*` | FINAL 预定发出时刻在"最后一个 RESP 理论收完"之后再加的安全余量 |

---

## 二、时序变量初始化

**文件：`Core/application/dw_main.c`，第 208–232 行（DW1000）/ 第 390–406 行（DW3000）**

```c
// 以 850K 为例（dw_main.c:225）
if(inst_dataRate == DWT_BR_850K)
{
    inst_one_slot_time    = ONE_SLOT_TIME_MS_850K;   // 8 ms
    inst_resp_rx_timeout  = RESP_RX_TIMEOUT_850K;    // 标签等RESP超时（写入DW硬件）
    inst_final_rx_timeout = FINAL_RX_TIMEOUT_850K;   // 基站等FINAL超时（由基站使用）
    inst_data_interval    = DATA_INTERVAL_TIME_850K; // 相邻消息间隔（UUS，整型存储）
    // POLL_TX → 最后一个RESP收完的DW计数器跨度
    inst_poll2final_time  = (FIRST_RESP_SEND_850K
                             + MAX_AHCHOR_NUMBER * inst_data_interval)
                            * UUS_TO_DWT_TIME;       // ×65536 转为 DW 刻度
}
```

**`inst_poll2final_time` 计算示意（850K，2 基站）：**

```
inst_poll2final_time = (1300 + 2×1600) × 65536
                     = 4500 × 65536
                     = 294,912,000 (DW 计数器刻度，约 4.615 ms)
```

此值用于 `STA_SEND_FINAL` 中精确定位 FINAL 的发送时刻。

---

## 三、防冲突超级帧 / Slot 机制

**文件：`Core/application/dw_instance_tag.c`，第 12–15 行 & 第 115–126 行**

```c
// 全局时序状态（dw_instance_tag.c:12）
static int tagSleepCorrection_ms = 0;   // A0基站下发的校准量（ms），对齐各标签slot边界
uint32_t volatile next_period_time = 0; // 标签下次 TWR 周期的绝对起始 tick（ms）
uint8_t Correction_flag = 0;            // 是否已收到 A0 的时序校准

// 入口随机退避（dw_instance_tag.c:115）
if (inst_slot_number > 1)
{
    diff_time += (tag_id + 1);          // 以 tag_id 为步长累积退避量
    if(diff_time > inst_one_slot_time * inst_slot_number / 2)
        diff_time = 0;                  // 超过超级帧一半则重置
}
```

**三层防冲突设计：**

1. **固定 slot 分配**：`next_period_time` 由 A0 基站通过 RESP 帧中的 `tagSleepCorrection_ms` 字段校准，将每个标签锁定到超级帧内各自的时隙。

2. **随机退避**：首次上电（`Correction_flag=0`）尚未收到 A0 校准时，`next_period_time` 叠加 `diff_time`（步长为 `tag_id+1`），避免多标签同时发 POLL 持续冲突。

3. **超级帧周期**：超级帧长度 = `inst_one_slot_time × inst_slot_number`（= 8 ms × 50 = **400 ms**），每个标签在此周期内恰好完成一次完整 TWR。

---

## 四、核心时序计算函数

**文件：`Core/application/dw_instance_tag.c`，第 38–107 行**

### 4.1 `tag_calc_resp_rx_time` — 计算第 N 个 RESP 的延迟开窗时刻

```c
// dw_instance_tag.c:43（DW1000 版），DW3000 版在第 80 行，逻辑相同
static uint32_t tag_calc_resp_rx_time(uint64_t poll_ts, uint8_t resp_index)
{
    // 以 850K 为例
    resp_rx_time = (poll_ts
                    + (FIRST_RESP_SEND_850K            // POLL_TX → RESP[0] 预期偏移（UUS）
                       + resp_index * inst_data_interval) // 第 resp_index 个RESP的额外偏移
                      * UUS_TO_DWT_TIME)               // 转为 DW 计数器刻度
                   >> 8;                               // 取高 32 位写入延迟寄存器
    return resp_rx_time;
}
```

- `>>8`：DW1000/DW3000 的延迟时间寄存器（`DX_TIME`）存放 40 位时间戳的高 32 位（即 bit[39:8]），低 8 位固定为 0，分辨率为 8 ns。
- `resp_index=0`：开窗时刻 = `poll_ts + 1300×65536`
- `resp_index=1`：开窗时刻 = `poll_ts + (1300+1600)×65536`

### 4.2 `tag_calc_final_tx_time` — 计算 FINAL 的延迟发送时刻

```c
// dw_instance_tag.c:61（DW1000 版），DW3000 版在第 94 行
static uint64_t tag_calc_final_tx_time(uint64_t poll_ts)
{
    // 以 850K 为例
    final_tx_time = (poll_ts
                     + inst_poll2final_time             // 覆盖所有RESP收完所需总时长
                     + TAG_FINALE_SEND_BACK_850K        // 额外安全余量（300 UUS）
                       * UUS_TO_DWT_TIME)
                    >> 8;
    return final_tx_time;
}
```

---

## 五、状态机中的时序执行

**文件：`Core/application/dw_instance_tag.c`，第 109–430 行**

### 5.1 `STA_SEND_POLL` — 立即发 POLL，延迟开第一个 RESP 接收窗

```c
// dw_instance_tag.c:152
range_time = portGetTickCnt();                    // 记录本 slot 的 ms 时钟起点

dwt_writetxdata(POLL_MSG_LEN + FCS_LEN, tx_poll_msg, 0);
dwt_writetxfctrl(POLL_MSG_LEN + FCS_LEN, 0, 1);
tx_status = TX_WAIT;
dwt_starttx(DWT_START_TX_IMMEDIATE);              // 立即发 POLL
while (tx_status == TX_WAIT);                    // 自旋等待 TX 中断确认
poll_tx_ts = get_tx_timestamp_u64();              // 读取 POLL 的精确 40 位发送时间戳

dwt_setrxtimeout(inst_resp_rx_timeout);           // 设置 RESP 超时窗口
dwt_setpreambledetecttimeout(PRE_TIMEOUT);        // 前导码超时（5 PAC）
uint32_t resp_rx_time = tag_calc_resp_rx_time(poll_tx_ts, 0); // 计算 RESP[0] 开窗时刻
dwt_setdelayedtrxtime(resp_rx_time);              // 写入延迟接收时刻
dwt_rxenable(DWT_START_RX_DELAYED);              // 在 resp_rx_time 时刻自动开窗
state = STA_WAIT_RESP;
```

### 5.2 `STA_RECV_RESP` — 收完一个 RESP，顺序开下一个接收窗

```c
// dw_instance_tag.c:233
resp_rx_ts[recv_anc_id] = get_rx_timestamp_u64(); // 保存该基站 RESP 的精确接收时间戳

resp_expect--;
if(resp_expect == 0)          // 所有 RESP 都等完
{
    state = STA_SEND_FINAL;
}
else                          // 还有 RESP 未收，开下一个延迟接收窗
{
    // resp_index = 已处理的 RESP 数量
    uint32_t resp_rx_time = tag_calc_resp_rx_time(poll_tx_ts,
                                (MAX_AHCHOR_NUMBER - resp_expect));
    dwt_setdelayedtrxtime(resp_rx_time);
    dwt_rxenable(DWT_START_RX_DELAYED);
    state = STA_WAIT_RESP;
}
```

### 5.3 `STA_SEND_FINAL` — 延迟发 FINAL，打包三类时间戳

```c
// dw_instance_tag.c:317
uint64_t final_tx_time = tag_calc_final_tx_time(poll_tx_ts);
dwt_setdelayedtrxtime((uint32)final_tx_time);

// 用预测时刻反推精确时间戳（低 9 位清零后补天线延时）
final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + ant_dly;

// 三类时间戳写入 FINAL 帧，基站用其计算飞行时间 ToF
memcpy(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX],  &poll_tx_ts,  5); // POLL 发送时刻
memcpy(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts, 5); // FINAL 发送时刻
for(int i=0; i<MAX_AHCHOR_NUMBER; i++)
    memcpy(&tx_final_msg[FINAL_MSG_RESP1_RX_TS_IDX + i*5],
           &resp_rx_ts[i], 5);                           // 各基站 RESP 接收时刻

dwt_writetxdata(FIANL_MSG_LEN + FCS_LEN, tx_final_msg, 0);
dwt_writetxfctrl(FIANL_MSG_LEN + FCS_LEN, 0, 1);
tx_status = TX_WAIT;
dwt_starttx(DWT_START_TX_DELAYED);       // 在精确计算的时刻延迟发出
while(tx_status == TX_WAIT);

// 设置下个 TWR 周期起始时刻
next_period_time = range_time
                 + inst_one_slot_time * inst_slot_number // 一个超级帧后
                 + tagSleepCorrection_ms;                // 叠加 A0 下发的校准偏移
tagSleepCorrection_ms = 0;
state = STA_IDLE;
```

**`final_tx_ts` 的构造方式说明：**  
`final_tx_time`（寄存器写入值）左移 8 位还原为 40 位时间戳，低 9 位（传播分辨率 = 0）后加天线延时 `ant_dly`，得到基站收到 FINAL 时能反推的精确发出时刻，用于 DS-TWR 飞行时间公式。

### 5.4 `STA_IDLE` — 软件定时器等待下一个周期

```c
// dw_instance_tag.c:393
if(nowtime >= next_period_time)   // ms 级 tick 到点
{
    dwt_forcetrxoff();
    state = STA_SEND_POLL;        // 启动下一轮 TWR
}
```

---

## 六、时序中断回调

**文件：`Core/application/dw_instance_tag.c`，第 443–496 行**

```c
void tag_tx_conf_cb(...) { tx_status = TX_OK;      }  // TX 完成 → 解除 STA_SEND_POLL / STA_SEND_FINAL 自旋
void tag_rx_ok_cb(...)   { rx_status = RX_OK;
                           dwt_readrxdata(rx_buffer,...); }  // RX 成功 → 推进 STA_WAIT_RESP
void tag_rx_to_cb(...)   { rx_status = RX_TIMEOUT; }  // 接收超时 → 同样推进（无数据）
void tag_rx_err_cb(...)  { rx_status = RX_ERROR;   }  // 帧错误 → 同样推进
```

这四个回调由 DW1000/DW3000 硬件中断触发，是状态机推进的唯一驱动力。`STA_WAIT_RESP` 在 `rx_status` 被任意一个回调置非零后，下一轮 `tag_app()` 调用即进入 `STA_RECV_RESP`。

---

## 七、完整时序图

以 **850K、2 基站** 为例，一次完整 TWR 周期的时间轴：

```
ms 时钟（portGetTickCnt）
│
├─────── next_period_time ──────────► 进入 STA_SEND_POLL
│
│  DW 40位硬件时钟（分辨率 ~15.65 ps，计数范围约 17.2 s 溢出）
│
│  poll_tx_ts ─── 立即发 POLL
│       │
│       ├── +1300 UUS (~85 µs) ── 开窗等 RESP[A0] → 收到后记 resp_rx_ts[0]
│       │
│       ├── +2900 UUS (~190 µs) ─ 开窗等 RESP[A1] → 收到后记 resp_rx_ts[1]
│       │
│       └── +4800 UUS (~315 µs) ─ 延时发 FINAL（含 poll_tx_ts / resp_rx_ts[0..1] / final_tx_ts）
│
└── next_period_time += 8 ms × 50 + tagSleepCorrection_ms
                      = 400 ms（下一轮 TWR 周期起点）
```

**各时刻公式（850K）：**

| 事件 | DW 时间戳（40 bit） |
|---|---|
| POLL TX | `poll_tx_ts`（硬件读取） |
| RESP[0] RX 开窗 | `poll_tx_ts + 1300×65536`（>>8 写寄存器） |
| RESP[1] RX 开窗 | `poll_tx_ts + 2900×65536`（>>8 写寄存器） |
| FINAL TX 调度 | `poll_tx_ts + (4500+300)×65536`（>>8 写寄存器） |
| FINAL TX 时间戳 | `(final_tx_time & 0xFFFFFFFE) << 8 + ant_dly` |

---

## 八、关键变量速查

| 变量 | 定义位置 | 含义 |
|---|---|---|
| `next_period_time` | `dw_instance_tag.c:14` | 下次 TWR 周期 ms 起点（软件定时器） |
| `tagSleepCorrection_ms` | `dw_instance_tag.c:13` | A0 基站下发的 slot 校准量（ms） |
| `diff_time` | `dw_instance_tag.c:27` | 未校准时的随机退避量（ms） |
| `poll_tx_ts` | `dw_instance_tag.c:18` | POLL 发送精确时间戳（40 bit DW 刻度） |
| `resp_rx_ts[i]` | `dw_instance_tag.c:19` | 各基站 RESP 接收时间戳数组 |
| `final_tx_ts` | `dw_instance_tag.c:20` | FINAL 发送精确时间戳（写入帧） |
| `inst_poll2final_time` | `dw_main.c:29` | POLL→最后RESP 的 DW 计数器跨度 |
| `inst_one_slot_time` | `dw_main.c:25` | 单 slot 时长（ms），按速率赋值 |
| `inst_slot_number` | `dw_main.c:21` | 系统最大标签容量（= 超级帧 slot 数） |
| `inst_resp_rx_timeout` | `dw_main.c:27` | 标签等 RESP 的超时窗口（UUS） |
| `inst_data_interval` | `dw_main.c:30` | 相邻消息间隔（UUS），按速率赋值 |
| `range_time` | `dw_main.c:13` | 本轮 POLL 发出时的 ms tick，用于计算 next_period_time |

---

---

# 基站 TWR 时序说明

> 文件覆盖范围：`Anchor_RTOS/01_Core/App/Src/dw_instance_anchor.c`、`Anchor_RTOS/01_Core/App/Src/dw_main.c`  
> 时序常量定义与标签共享同一套 `dw_instance.h`，见上文第一节，此处不再重复。

---

## 九、基站端架构概述

基站固件运行在 FreeRTOS 上，TWR 处理由两个任务协同完成，与标签的裸机轮询状态机完全不同：

**文件：`Anchor_RTOS/01_Core/App/Src/dw_main.c`，第 495–558 行**

```
硬件中断(PB0 EXTI0)
    │
    ▼
task_uwb（高优先级）
    osSemaphoreAcquire(sema_uwbInt)  ← 中断里 Release
    process_deca_irq()               ← 调用 dwt_isr() 分发回调
    txcallback / rxcallback / rxTimeoutCallback / rxfailedcallback
        │  将事件类型写入 queue_uwbEvent
        ▼
task_twrRun（普通优先级）
    osMessageQueueGet(queue_uwbEvent)
    current_Algorithm->onEvent()     ← 调用 twrAnchor_onEvent()
        │  按 f_code 分支处理 POLL / RESP / FINAL
        ▼
    osMessageQueuePut(queue_processDis, distance)
        │
        ▼
task_minHeapManage  ← 维护标签距离最小堆，超时清除
task_getMinDis      ← 定期读最小距离，推送给报警逻辑
```

**与标签的关键区别：**

| | 标签 | 基站 |
|---|---|---|
| 运行环境 | 裸机，`tag_app()` 轮询 | RTOS，事件队列驱动 |
| 时序基准 | `poll_tx_ts`（自己发出的时间戳） | `poll_rx_ts`（收到 POLL 的时间戳） |
| RESP 槽位 | 按 `resp_index` 顺序开窗 | 按 `anc_id` 固定槽位发送 |
| 距离计算 | 无（读 RESP 里的 prev_range） | 收到 FINAL 后执行 DS-TWR 公式 |

---

## 十、基站时序变量初始化

**文件：`Anchor_RTOS/01_Core/App/Src/dw_main.c`，第 424–447 行（DW1000）/ 第 347–362 行（DW3000）**

```c
// 以 850K 为例（dw_main.c:440，DW1000）
else if (inst_dataRate == DWT_BR_850K)
{
    inst_one_slot_time    = ONE_SLOT_TIME_MS_850K;    // 8 ms
    inst_final_rx_timeout = FINAL_RX_TIMEOUT_850K;    // 基站等FINAL超时（1300 UUS）
    inst_resp_rx_timeout  = RESP_RX_TIMEOUT_850K;     // 接收其他基站RESP的超时窗口
    inst_data_interval    = DATA_INTERVAL_TIME_850K;  // 相邻消息间隔（1600 UUS）
    inst_poll2final_time  = ((FIRST_RESP_SEND_850K
                              + MAX_AHCHOR_NUMBER * inst_data_interval)
                             * UUS_TO_DWT_TIME);      // 与标签侧完全相同的值
}
```

`inst_poll2final_time` 的值与标签侧完全一致（4500×65536 刻度 ≈ 4.615 ms），标签用它定位 FINAL 发送时刻，基站用它定位 FINAL 接收开窗时刻，两端从同一公式出发保证对齐。

---

## 十一、基站接收 POLL → 准备并延迟发 RESP

**文件：`Anchor_RTOS/01_Core/App/Src/dw_instance_anchor.c`**

### 11.1 接收 POLL，初始化本轮时序状态（第 151–175 行）

```c
// case RTLS_MSG_TAG_POLL（dw_instance_anchor.c:151）
range_time   = portGetTickCnt();           // 记录收到POLL的ms tick（仅用于A0校准计算）
poll_rx_ts   = get_rx_timestamp_u64();     // 读取POLL的精确40位接收时间戳 ← 本轮时序基准

dev->remainingRespToRx = MAX_AHCHOR_NUMBER - 1; // 还需接收的其他基站RESP数量（不含自己）
handleResp_times = MAX_AHCHOR_NUMBER;      // 需要处理的RESP轮次总数（发+收）
dev->respTxIndex = 0x01 << anc_id;        // 发送RESP的位置标志：A0→0x01，A1→0x02，A2→0x04

anch_perpareAnc2TagResp();                // 将RESP帧数据写入DW发送缓冲区
anch_txRespOrRxReEnable();               // 决策：现在发RESP？还是先接收其他基站的RESP？
```

**`respTxIndex` 机制说明：**  
`respTxIndex = 0x01 << anc_id`，在 `anch_txRespOrRxReEnable()` 里每轮右移一位。当 bit0 为 1 时轮到自己发送。因此：
- A0（`respTxIndex=0x01`）：收到 POLL 后第一轮 bit0=1，立即发 RESP
- A1（`respTxIndex=0x02`）：第一轮 bit0=0，先开窗收 A0 的 RESP；第二轮右移后 bit0=1，再发自己的 RESP
- A2（`respTxIndex=0x04`）：类推，等前两个 RESP 都处理完再发

### 11.2 计算并执行延迟发 RESP（第 489–527 行）

```c
// anch_txRespOrRxReEnable()（dw_instance_anchor.c:489）
if (dev->respTxIndex & 0x01)    // 轮到本基站发送RESP
{
    dev->respTxIndex = 0;
    // 850K 为例
    resp_tx_time = (poll_rx_ts
                    + (FIRST_RESP_SEND_850K              // POLL_RX → RESP[0] 预期偏移
                       + anc_id * inst_data_interval)    // 本基站在第 anc_id 个槽位
                      * UUS_TO_DWT_TIME
                    + ANC_RESP_SEND_BACK_850K            // 基站额外余量（300 UUS）
                      * UUS_TO_DWT_TIME);

    resp_tx_time = resp_tx_time >> 8;                   // 取高32位写入DX_TIME
    dwt_setdelayedtrxtime((uint32)resp_tx_time);
    dwt_starttx(DWT_START_TX_DELAYED);                  // 在精确时刻延迟发出RESP
    handleResp_times--;
}
```

**与标签侧的关键对比：**

| | 标签（等 RESP） | 基站（发 RESP） |
|---|---|---|
| 时间基准 | `poll_tx_ts`（发出时刻） | `poll_rx_ts`（收到时刻） |
| 槽位参数 | `resp_index`（0,1,2...） | `anc_id`（固定，0,1,2...） |
| 额外余量 | `TAG_FINALE_SEND_BACK` | `ANC_RESP_SEND_BACK`（300 UUS） |
| 方向 | 开窗 RX | 延迟 TX |

两端用不同基准（TX vs RX 时间戳）、但相同的偏移常量计算各自的时刻，天线延时（`ant_dly`）已在 DW 硬件层自动补偿，无需手动加减。

---

## 十二、基站接收其他基站的 RESP → 开延迟接收窗

**文件：`Anchor_RTOS/01_Core/App/Src/dw_instance_anchor.c`，第 548–584 行**

```c
// anch_txRespOrRxReEnable() 中的"继续接收"分支
else    // 接收其他基站的 RESP（帧过滤关闭，接收所有数据）
{
    dwt_enableframefilter(DWT_FF_NOTYPE_EN);   // 关帧过滤，能收任意帧

    // resp_index = MAX_AHCHOR_NUMBER - handleResp_times（已处理轮次的补数）
    resp_rx_time = (poll_rx_ts
                    + ((FIRST_RESP_SEND_850K
                        + (MAX_AHCHOR_NUMBER - handleResp_times) * inst_data_interval)
                       * UUS_TO_DWT_TIME));

    resp_rx_time = resp_rx_time >> 8;
    dwt_setdelayedtrxtime(resp_rx_time);
    dwt_setrxtimeout(inst_resp_rx_timeout);       // 等RESP超时窗口（1000 UUS @ 850K）
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
    dwt_rxenable(DWT_START_RX_DELAYED);

    handleResp_times--;                           // 处理轮次减一
    dev->respTxIndex = dev->respTxIndex >> 1;     // 右移一位，判断下轮是否轮到自己发
}
```

**说明：**  
基站也需要监听其他基站的 RESP，原因有两个：
1. 通过 RESP 帧的 `PREV_DIS` 字段获取其他基站对当前标签的上一轮测距值（汇总上报）。
2. 维护 `handleResp_times` 计数，确保所有 RESP 都处理完毕后再开 FINAL 接收窗。

---

## 十三、基站延迟开窗等 FINAL

**文件：`Anchor_RTOS/01_Core/App/Src/dw_instance_anchor.c`，第 530–541 行**

```c
// remainingRespToRx == 0，所有 RESP 处理完毕，开 FINAL 接收窗
if (dev->remainingRespToRx == 0)
{
    uint64_t final_rx_time = (poll_rx_ts + inst_poll2final_time); // 与标签发FINAL的时刻对齐
    final_rx_time = final_rx_time >> 8;
    dwt_setdelayedtrxtime((uint32)final_rx_time);
    dwt_setrxtimeout(inst_final_rx_timeout);       // FINAL 接收超时（1300 UUS @ 850K）
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
    dwt_rxenable(DWT_START_RX_DELAYED);            // 在 poll_rx_ts + inst_poll2final_time 处开窗
}
```

**对齐逻辑：**

```
标签：FINAL_TX 时刻 = poll_tx_ts + inst_poll2final_time + TAG_FINALE_SEND_BACK × 65536
基站：FINAL_RX 开窗 = poll_rx_ts + inst_poll2final_time

由于 poll_rx_ts ≈ poll_tx_ts + 传播时间 + ant_dly（约几十 ns）
TAG_FINALE_SEND_BACK（300 UUS ≈ 19.6 µs）作为余量将基站的开窗提前，
确保标签发出时基站接收机已经就绪。
```

---

## 十四、DS-TWR 飞行时间计算

**文件：`Anchor_RTOS/01_Core/App/Src/dw_instance_anchor.c`，第 178–230 行**

基站收到 FINAL 后立即执行双边双向测距（DS-TWR）公式：

```c
// case RTLS_MSG_TAG_FINAL（dw_instance_anchor.c:178）

// 从 FINAL 帧中取出标签侧三个时间戳（低32位，单位 DW 刻度）
final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX],   &poll_tx_ts_32);  // 标签发POLL
final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP1_RX_TS_IDX
                             + anc_id * (FINAL_MSG_TS_LEN+1)], &resp_rx_ts_32); // 标签收本站RESP
final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX],  &final_tx_ts_32); // 标签发FINAL

// 基站侧三个时间戳（自己持有）
poll_rx_ts_32  = (uint32_t)poll_rx_ts;    // 基站收POLL
resp_tx_ts_32  = (uint32_t)resp_tx_ts;    // 基站发RESP（TX中断回调里读取）
final_rx_ts_32 = (uint32_t)final_rx_ts;   // 基站收FINAL（当前RX回调里读取）

// DS-TWR 四个时间差
Ra = resp_rx_ts_32  - poll_tx_ts_32;   // 标签侧：POLL发出 → RESP收到（含传播×2+处理）
Rb = final_rx_ts_32 - resp_tx_ts_32;   // 基站侧：RESP发出 → FINAL收到（含传播×2+处理）
Da = final_tx_ts_32 - resp_rx_ts_32;   // 标签侧：RESP收到 → FINAL发出（标签处理时延）
Db = resp_tx_ts_32  - poll_rx_ts_32;   // 基站侧：POLL收到 → RESP发出（基站处理时延）

// DS-TWR 公式（消除时钟偏差）
tof_dtu = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);

tof = tof_dtu * DWT_TIME_UNITS;          // 转换为秒（1 DW 刻度 ≈ 15.65 ps）
distance_now_m = tof * SPEED_OF_LIGHT;   // 距离 = 飞行时间 × 光速

// DW1000 修正多径偏差
distance_now_m -= dwt_getrangebias(inst_ch, distance_now_m, inst_prf);

// 结果存入 prev_range，下个周期写入 RESP 帧回传给标签
prev_range[recv_tag_id] = (int32_t)(distance_now_m * 1000); // 单位 mm
```

**DS-TWR 公式几何含义：**

```
TAG ────POLL──►──────────────────────────────────────────────────
         │(Ra)                              ▲(Da)
ANC ─────▼──────────────────RESP──►────────┤
                     (Db)│          (Rb)   ▼
TAG ─────────────────────►──────────────FINAL──►
```

| 变量 | 路径含义 |
|---|---|
| `Ra` = resp_rx - poll_tx | 标签：POLL 发出 → 收到 RESP（一来一回 + 基站处理） |
| `Rb` = final_rx - resp_tx | 基站：RESP 发出 → 收到 FINAL（一来一回 + 标签处理） |
| `Da` = final_tx - resp_rx | 标签侧处理时延（RESP 收到到 FINAL 发出） |
| `Db` = resp_tx - poll_rx | 基站侧处理时延（POLL 收到到 RESP 发出） |

四个量代入公式后，时钟漂移（两端晶振速率差）在分子中相消，得到纯净的单程飞行时间。

---

## 十五、A0 基站下发时序校准

**文件：`Anchor_RTOS/01_Core/App/Src/dw_instance_anchor.c`，第 630–654 行**

A0 基站（`anc_id == 0`）在每次打包 RESP 时，根据当前收到 POLL 的实际时刻与期望时刻的差，计算校准量并写入 RESP 帧，标签据此修正 `next_period_time`：

```c
// anch_perpareAnc2TagResp()（dw_instance_anchor.c:630）
if (anc_id == 0)
{
    int sframePeriod_ms = inst_one_slot_time * inst_slot_number; // 超级帧总长 = 8×50 = 400 ms
    int slotDuration_ms = inst_one_slot_time;                    // 单 slot = 8 ms

    // 当前标签实际落在超级帧的哪个位置
    int currentSlotTime  = range_time % sframePeriod_ms;
    // 该 tag_id 的标签理论上应该落在哪个位置
    int expectedSlotTime = recv_tag_id * slotDuration_ms;
    // 偏差 = 期望 - 实际
    int error = expectedSlotTime - currentSlotTime;

    // 保证校准后的等待时间在 [0, 1.5×超级帧] 范围内
    if (error < (-(sframePeriod_ms >> 1)))
        tagSleepCorrection_ms = sframePeriod_ms + error; // 负偏差过大，加一整帧补偿
    else
        tagSleepCorrection_ms = error;

    // 写入 RESP 帧（大端，占2字节）
    tx_resp_msg[RESP_MSG_SLEEP_COR_IDX]     = (tagSleepCorrection_ms >> 8) & 0xFF;
    tx_resp_msg[RESP_MSG_SLEEP_COR_IDX + 1] = tagSleepCorrection_ms & 0xFF;
    tx_resp_msg[RESP_MSG_GROUP_IDX] = group_id | 0x80;  // 最高bit=1，标识校准基站
}
```

**标签侧收到后的处理（`dw_instance_tag.c:246–248`）：**

```c
// 识别校准基站（RESP_MSG_GROUP_IDX 最高bit=1）
if((rx_buffer[RESP_MSG_GROUP_IDX] & 0x80) == 0x80)
{
    tagSleepCorrection_ms = (int16)(
        ((uint16)rx_buffer[RESP_MSG_SLEEP_COR_IDX] << 8)
        + rx_buffer[RESP_MSG_SLEEP_COR_IDX + 1]);
    Correction_flag = 1;
}
// 发完 FINAL 后叠加到下次周期（dw_instance_tag.c:360）
next_period_time = range_time + inst_one_slot_time * inst_slot_number + tagSleepCorrection_ms;
```

**校准收敛过程：**

```
初始（未校准）：各标签随机发 POLL，可能互相冲突
              ↓ A0 在第一次成功通信后计算偏差并下发
第1个周期：标签修正 next_period_time，向目标 slot 靠近
              ↓ 若偏差较大需多次迭代
稳定后：tag_id=0 → slot 0（0~8 ms），tag_id=1 → slot 1（8~16 ms），...
        所有标签在 400 ms 超级帧内均匀分布，无冲突
```

---

## 十六、标签与基站时序对齐总图

以 **850K、2 基站（A0/A1）、1 标签（T0）** 为例：

```
时间轴（DW 40位硬件时钟，左对齐为 POLL_TX / POLL_RX 时刻）

T0（标签）
  poll_tx_ts ──────────────────────────────────────────────────────────►
       │ 发 POLL（立即）
       │
       │        +1300 UUS          +2900 UUS          +4800 UUS
       │           │                  │                   │
       │    开窗等RESP[A0]      开窗等RESP[A1]       延迟发FINAL
       │       (RX)                (RX)                 (TX)
       │         ▲                  ▲                    │
       └─────────┼──────────────────┼────────────────────┘

A0（基站 anc_id=0）
  poll_rx_ts ──────────────────────────────────────────────────────────►
       │ 收 POLL
       │
       │  +1300+0×1600+300 UUS    +4500 UUS（开窗等FINAL）
       │    = +1600 UUS
       │         │                      │
       │    延迟发 RESP[A0]        延迟开窗等 FINAL
       │       (TX)                   (RX)
       │                               ▲
       │                           收到FINAL
       │                               │
       │                       执行 DS-TWR 公式
       │                       prev_range[T0] = 距离(mm)

A1（基站 anc_id=1）
  poll_rx_ts ──────────────────────────────────────────────────────────►
       │ 收 POLL
       │
       │  先接收A0的RESP    +1300+1×1600+300 = +3200 UUS
       │  （开延迟接收窗）          │
       │       (RX)           延迟发 RESP[A1]
       │                          (TX)
       │                               +4500 UUS
       │                                  │
       │                          延迟开窗等 FINAL
       │                                 (RX)
       │                                  ▲
       │                              收到FINAL → DS-TWR
```

**时刻公式汇总（850K）：**

| 事件 | 时间戳公式 |
|---|---|
| A0 发 RESP | `poll_rx_ts + (1300 + 0×1600 + 300) × 65536` → `>>8` |
| A1 发 RESP | `poll_rx_ts + (1300 + 1×1600 + 300) × 65536` → `>>8` |
| T0 等 RESP[A0] 开窗 | `poll_tx_ts + (1300 + 0×1600) × 65536` → `>>8` |
| T0 等 RESP[A1] 开窗 | `poll_tx_ts + (1300 + 1×1600) × 65536` → `>>8` |
| T0 发 FINAL | `poll_tx_ts + (4500 + 300) × 65536` → `>>8` |
| A0/A1 等 FINAL 开窗 | `poll_rx_ts + 4500 × 65536` → `>>8` |

`ANC_RESP_SEND_BACK`（+300 UUS）使基站 RESP 稍晚于标签开窗时刻，`TAG_FINALE_SEND_BACK`（+300 UUS）使 FINAL 稍晚于基站 FINAL 开窗时刻，两个余量共同保证收发双方不会因调度抖动而错过。

---

## 十七、基站端关键变量速查

| 变量 | 定义位置 | 含义 |
|---|---|---|
| `poll_rx_ts` | `dw_instance_anchor.c:38` | 本轮 POLL 接收精确时间戳（本轮所有时刻的基准） |
| `resp_tx_ts` | `dw_instance_anchor.c:39` | 本基站 RESP 发送时间戳（TX中断回调读取，用于ToF） |
| `final_rx_ts` | `dw_instance_anchor.c:40` | FINAL 接收精确时间戳（用于ToF计算） |
| `prev_range[tag_id]` | `dw_instance_anchor.c:41` | 各标签上一轮测距结果（mm），写入下次RESP回传 |
| `handleResp_times` | `dw_instance_anchor.c:45` | 还需处理的RESP轮次数（发+收），从MAX_AHCHOR_NUMBER递减 |
| `dev->respTxIndex` | `dw_main.c:741` | 位图，bit右移判断本轮是否轮到自己发RESP |
| `dev->remainingRespToRx` | `dw_main.c:739` | 还需接收的其他基站RESP数量 |
| `dev->wait4final` | `dw_instance_anchor.c:170` | 所有RESP处理完毕，进入等FINAL状态标志 |
| `resp_valid` | `dw_instance_anchor.c:44` | FINAL帧内各基站RESP有效位图（bit N=本基站有效才计算ToF） |
| `sema_uwbInt` | `dw_main.c:211` | 硬件中断 → task_uwb 的同步信号量 |
| `queue_uwbEvent` | `dw_main.c:243` | task_uwb → task_twrRun 的事件队列 |
| `queue_processDis` | `dw_main.c:235` | task_twrRun → task_minHeapManage 的距离队列 |
