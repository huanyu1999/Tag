# Tag 工程重构方案:CMake 构建 + TREK 时序适配 + 睡眠唤醒 + Discovery(5 阶段)

> **目的**:参照 Anchor_RTOS 的 TREK1000 重构(`../Anchor_RTOS/docs/trek1000_refactor_plan.md`),
> 为标签(Tag,STM32F103 裸机)工程:①打通 CMake 双目标构建;②按新版
> `../Anchor_RTOS/docs/TWR_TIMING.md` 适配 TREK 空口时序;③实现 DW 深睡 + MCU WFI 睡眠唤醒;
> ④实现 TREK 风格 Discovery(blink → RNG_INIT 动态分配 tag_id/slot);⑤清理旧代码与 Keil 工程。
>
> **本文档是本项工作跨电脑交接的唯一事实来源**。每完成一个阶段,更新本文档的进度标记并随代码一起提交。

---

## 工作流程约定(与 Anchor_RTOS 一致)

1. **每个 Phase 结束必须停下**:由用户本人编译(`cmake -B build && cmake --build build`,
   TAG_DW1000/TAG_DW3000 双目标)并烧录真机验证,通过后才进入下一阶段;编译、烧录均由用户执行;
2. 每阶段独立提交,格式:`参考Trek1000代码进行重构, tag phase X: <内容> [YYYY-MM-DD HH:MM]`;
3. 阶段内如需中断,提交信息注明断点位置,并同步更新本文档"当前断点"一节;
4. 源码注释用中文。

## 进度总览(2026-07-10 创建)

| 阶段 | 内容 | 状态 | 提交 |
|------|------|------|------|
| Phase 0 | CMake 双目标构建打通(接 dw_instance 副本) | ✅ 真机通过 | ae572e5 |
| Phase 1 | TREK 时序适配(TWR_TIMING.md §5) | ✅ 真机通过(已联调,测距+boot log 正常) | ae572e5 |
| Phase 2 | 睡眠唤醒(DW 深睡 + MCU WFI + nextPeriod 公式) | 🟡 代码完成,待真机 | 待提交 |
| Phase 3 | Discovery(blink/RNG_INIT)+ 失联回退 | ⬜ 未开始 | — |
| Phase 4 | 清理(旧驱动树/旧副本/Keil 退役)+ 文档 | ⬜ 未开始 | — |

### 当前断点

Phase 0 + Phase 1 **已真机通过并提交(ae572e5)**:与 Phase B 基站联调,测距与 boot log 均正常。

Phase 2(睡眠唤醒)代码已完成,双目标编译通过、应用层告警清零
(TAG_DW1000 flash 62.4%,TAG_DW3000 flash 69.0%),**待真机验证**。
下一步:用户烧录验证(见下方"Phase 2 验证"),通过后进入 Phase 3(discovery,需基站 Phase C)。

另:VS Code 构建/烧录任务见 `.vscode/tasks.json`(CMake 双目标 + OpenOCD/ST-Link V2,
被 .gitignore 排除,换电脑需重建)。

#### Phase 0/1 实施记录(与原计划的差异)

1. **工作区曾被前次会话搞坏**:`Core/application/dw_instance.h` 被整体覆盖成了**基站的头文件**
   (include 了标签工程不存在的 `board_dw1000.h` / `iwdg.h` / `../01_Core/...`,还把 `ANCRANGE` 打开),
   无法编译。已 `git checkout HEAD --` 回退后重做。`instance.h` / `instance_tag.c` 当时也从工作区消失,
   一并从 HEAD 恢复(按计划它们留到 Phase 4 再删)。
2. **`sfConfig` 在基站侧是死代码**(只定义、无任何读取点,基站 `TWR_TIMING.md:78` 自己也承认)。
   标签侧**接成了活配置**:`sf_config_init()` 由 `inst_one_slot_time` / `inst_slot_number` 填充,
   状态机里 8 处 `inst_one_slot_time * inst_slot_number` 全部改读 `sfConfig.tagPeriod_ms`(=600ms),
   `pollTxToFinalTxDly_us` 由 `twr_set_replydelay()` 公式导出(不是基站那个写死的 6100)。
   → Phase 2 的 `tag_round_end()` / 睡眠周期直接在此收拢。
3. **修掉的两个实质 bug**(不只是照搬公式):
   - 旧代码开 resp 接收窗时**没有减前导码**。delayed RX 编程的是接收机开机时刻,而 resp 的 RMARKER
     落在槽时刻、前导码在其之前就开始飞 —— 不提前一个前导码开窗会漏掉整个前导码。已按
     `TWR_TIMING.md:82-83` 契约改为 `poll_tx + (k+1)×fixedReplyDelay − preambleDuration32h`;
   - 旧代码把 **µs 值当 symbol** 传给 `dwt_setrxtimeout`(原生单位 1.0256µs),约 2.5% 偏差。
     已改传 `twr_timings.fwto4RespFrame_sy`;
   - 顺带:`STA_RECV_RESP` 里读未初始化的 `dwt_rxdiag_t`(`dwt_readdiagnostics` 是注释掉的),
     `rx_power` 拿的是栈上垃圾值,已移除该死代码。
4. **天线延时未同步**:标签保持单值 `ANT_DLY=16485`,没有拆成基站的 `ANT_DLY_DW1000=16549` /
   `ANT_DLY_DW3000=16347`。天线延时是逐板标定量、会直接平移测距值;本阶段要验证的是时序公式是否对齐,
   混入距离偏移会让回归无法归因。**留待 Phase 1 真机测距通过后单独标定**。
5. `POLL_MSG_LEN` 标签 26 / 基站 16 不一致,但**无害且不要动**:它不参与 `twr_set_replydelay`
   (槽间隔只由 RESP 帧长决定),基站按 `RX_FINFO` 实际长度收帧;标签的 26 是给 `user_data[10]` 留的。

#### Phase 2 实施记录

1. **新增 `Core/application/dw_power.c`**:`tag_dw_sleep_config()`(init 调一次) /
   `tag_dw_entersleep()` / `tag_dw_wakeup()`,把 DW1000 与 DW3000 的深睡唤醒差异封装掉,
   状态机不感知芯片。
2. **`dw_apply_runtime_config()`(dw_main.c,新)**:深睡**不保留**天线延时 / TX功率 / PANID /
   帧过滤 / PA-LNA / 中断掩码,唤醒后必须整套重下。这一份被 **init 与唤醒路径共用**,
   避免"改了 init 忘了改唤醒"的漂移 —— 那类 bug 的表现是上电第一轮正常、睡醒之后就不对了。
   init 里原先散落的这些调用已全部改为调它。
3. **`tag_dw_wakeup()` 里额外重下 `dwt_setaddress16(tag_id)`**:短地址与 PANID 同在 PANADR
   寄存器,深睡是否保留不确定。丢了的表现是帧过滤把基站发给本标签的 resp 全部拒掉 ——
   "睡醒后再也测不出距离"。一条 SPI 写,买保险。
4. **DW1000 的 EXTI0 坑**(CLAUDE.md 点名过):`port_wakeup_IC_fast()` 内部最后一步
   `setup_DW1000RSTnIRQ(0)` 会 `HAL_NVIC_DisableIRQ(EXTI0_IRQn)`,而 DW1000 的 IRQ(PB0) 与
   RSTn(PA0) 共用 EXTI0。唤醒后不重新 `HAL_NVIC_EnableIRQ(EXTI0_IRQn)`,所有 `dwt_*` 回调
   永久失效,标签卡死在 STA_WAIT_RESP 的 50ms 看门狗上不断重启。已在 `tag_dw_wakeup()` 里补。
5. **`tag_round_end()`(dw_instance_tag.c)**:6 处散落的 `next_period_time` 赋值收拢为一份,
   顺带修掉它们互相不一致的问题(错误分支漏了 tagSleepCorrection,成功分支漏了 diff_time)。
   公式即 TREK 的 `nextPeriod = tagSleepTime + tagSleepCorrection + tagSleepRnd`。
   Phase 3 的 discovery 回退也从这里分流。
6. **`STA_IDLE`**:到点前 `DW_WAKEUP_LEAD_MS`(4ms,唤醒 ~2.2ms + 重下配置余量)先唤醒 DW;
   到点若发现还在睡(唤醒窗被长中断挤掉)则补唤醒兜底;其余时间 `__WFI()` 让 MCU 进 Sleep。
   **WFI 只放 STA_IDLE**:TWR 交换内的 500us 级忙等保持忙等,不动。

#### Phase 2 验证(待用户执行)

- [ ] 测距结果与 Phase 1 完全一致(成功率/距离);**唤醒后首轮距离无系统性偏移**
      —— 偏移数米 = 天线延时没重下
- [ ] 报警 LED/蜂鸣器 4Hz 正常(WFI 不影响 TIM2 中断)
- [ ] 电流计:睡眠窗电流显著下降(DW 深睡 µA 级 + MCU Sleep)
- [ ] 长跑:correction 收敛、周期不漂,无反复重启(重启 = EXTI0 没恢复,回调不来撞看门狗)
- [ ] **DW3000 深睡路径本项目首次跑**,单独安排长跑(基站有过 ARFE 静默停收的教训)

#### Phase 1 验证(已通过)

- [ ] **boot log 对账**:标签 RTT 的 `TWR timing:` 与基站逐项一致。**基站侧需先把
      `../Anchor_RTOS/01_Core/App/Src/dw_main.c:434-440` 那段被注释掉的 `log_i` 取消注释**才能对比。
      本地按同公式核算的期望值(850K/PLEN1024/N=3/TURNAROUND=500):
      preamble **1058us**、replyDelay **1801us(32h=449632)**、pollTx2Final **≈7205us(32h=1798528)**、
      fwtoResp **1287sy**、fwtoFinal **1759sy**、单次 T2A 交换 **8786us < 12000us slot** ✓
      (注:`TWR_TIMING.md §4` 里写的 fwtoResp≈1289 / fwtoFinal≈1720 是文档里的近似值;双端跑的是
      同一份公式和同一组帧长宏,故实际值必然一致,以 boot log 实测为准。)
- [ ] 三基站 T2A:测距成功率/距离稳定性 ≥ 旧版;30min 长跑无 `FINAL tx FAILED` 刷屏;基站侧 A2A 不受影响
- [ ] 报警链路:`distance_flag` 红灯+蜂鸣、`lost_flag`(5s)黄灯照旧

### 联调依赖

```
Tag Phase 0(CMake) ──→ Tag Phase 1(时序) ──→ Tag Phase 2(睡眠) ──→ Tag Phase 4(清理)
                              │                                          ↑
                    基站 Phase B 真机验证(同一次联调)          Tag Phase 3(discovery)
                                                                    ↑
                                                          基站 Phase C(未开始)
```

- Tag Phase 1 与基站 Phase B 是**同一次真机联调**(T2A 回归 + SWO/RTT 实测收紧
  `RX_RESPONSE_TURNAROUND`,收紧值双端同步改);
- Tag Phase 3 端到端联调依赖基站 Phase C(discovery 基站侧,未开始);标签侧代码可先行开发,
  用"blink 超时→睡→重发"自闭环冒烟 —— 故睡眠(P2)排在 discovery(P3)之前。

---

## 背景与已确认决策

基站工程已到 Phase B(TREK 单轨时序,旧时序宏彻底删除),**Tag 不适配就无法与新基站测距**。
Tag 现状:`tag_app()` 仅 5 状态(IDLE/SEND_POLL/WAIT_RESP/RECV_RESP/SEND_FINAL),
`tag_id` 硬编码(`dw_main.c` 的 `TAG_ID 0x07`),无任何睡眠代码,主循环纯忙等。

| 决策点 | 结论(用户已确认) |
|---|---|
| 构建系统 | 全面转 CMake,Keil 目标不再维护 |
| 时序适配 | 纳入本计划作前置阶段(Phase 1) |
| 低功耗深度 | DW 深睡 + MCU Sleep/WFI;STOP+RTC 留作后续独立阶段 |
| 失联行为 | lost_flag(5s 无任何基站)后退回 discovery 重新 blink 注册;黄灯告警保留 |
| 代码落点 | 双目标副本 `Core/application/dw_instance.h` + `dw_instance_tag.c`(旧 `instance.*` 弃用待删) |

### 已核实的关键事实(2026-07-10 探查)

- **CMake 骨架已存在但从未编译过**:`CMakeLists.txt`(双目标 TAG_DW1000/TAG_DW3000)、
  `toolchain.cmake`、GCC 版 `startup_stm32f103xb.s`、`STM32F103XX_FLASH.ld`、
  `Core/Src/syscalls.c + sysmem.c` 全齐;缺口是 `APP_SRC_FILES` 还指向旧副本
  `instance_tag.c`,且 `dw_main.c` 首行还 include `instance.h`;
- `dw_instance_tag.c` 与 `instance_tag.c` 全文 diff 仅首行 include,无落后修复;
  `dw_instance.h` 已是 `MAX_AHCHOR_NUMBER=3`(`ONE_SLOT_TIME_MS_850K=12`,
  超帧 = 12ms × 50 = 600ms,与基站一致);
- 驱动树:CMake 用 `Core/Dw1000/`、`Core/Dw3000/`(均完整);顶层旧
  `Core/decadriver` + `Core/platform` 与 `Core/Dw1000` 差异只有 include 文件名,弃用待删;
- 睡眠:`dwt_configuresleep/entersleep` 全工程零调用;DW1000 唤醒函数
  `port_wakeup_IC_fast()` 在 `Core/Dw1000/platform/port_dw1000.c:300`(~2.2ms);
  DW3000 用 `wakeup_device_with_io()`(WAKEUP=PA1)+ `dwt_restoreconfig()`;
- 帧过滤:标签只需新增 `dwt_seteui()`(RNG_INIT 是长目的地址 DATA 帧,现有 DATA_EN 即可放行);
  RSVD_EN 是基站 A0 收 blink 用的,标签不需要;
- `STA_SEND_BLINK/STA_WAIT_INIT/STA_RECV_INIT` 枚举已预留未接线(`dw_instance.h:251-253`);
  旧宏 `BLINK_MSG_LEN=10 / INIT_MSG_LEN=12` 是占位,与真实帧长不符,需重定义;
- WFI 约束:SPI 事务同步完成(内部忙等 DMA 标志),UART 发后不管 DMA,MCU Sleep 不停
  DMA/外设时钟 → 无在途事务约束;**WFI 只能放 STA_IDLE**(SysTick 1ms 粒度),
  交换内 500us 级忙等(`while(tx_status==TX_WAIT)` 等)保持忙等;
- 报警链路(TIM2 4Hz 回调驱 LED/蜂鸣器,`Core/Src/main.c:196-237`)在 WFI 下不受影响。

---

## Phase 0:CMake 双目标构建打通 ⬜

改动文件:

1. `CMakeLists.txt`:`APP_SRC_FILES` 中 `Core/Application/instance_tag.c` →
   `Core/Application/dw_instance_tag.c`;
2. `Core/application/dw_main.c`:首行 `#include "instance.h"` → `#include "dw_instance.h"`;
3. `Core/application/dw_instance.h`:USE_DW1000/USE_DW3000 分支里的相对路径 include 改裸
   include(`#include "port_dw1000.h"` 等,CMake 已加对应 include 目录);恢复被注释的
   `#error "请定义 USE_DW1000 或 USE_DW3000"`(现 fallback 静默落 DW3000 是隐患);
4. 旧副本 `instance.h / instance_tag.c / instance_anchor.c`、旧树 `Core/decadriver`、
   `Core/platform` **不删**(留 Phase 4),从此不参与编译。

### 验证(用户执行)

- [ ] 双目标编译零错误,应用层 `-Wall -Wextra` 告警清零(GCC 比 Keil 严格);
- [ ] 烧录 TAG_DW1000:RTT 出 boot log,poll 周期性发出(此时仍旧时序,与新基站测不了距属预期)。

**风险**:GCC 启动文件/.ld 从未真机跑过 —— RTT 能出字、状态机能转即基本通过。

## Phase 1:TREK 时序适配(TWR_TIMING.md §5)⬜

1. **`Core/application/dw_instance.h`**:
   - 删 18 个旧速率宏(`FINAL_RX_TIMEOUT_* / RESP_RX_TIMEOUT_* / FIRST_RESP_SEND_* /
     DATA_INTERVAL_TIME_* / ANC_RESP_SEND_BACK_* / TAG_FINALE_SEND_BACK_*` 三档全部)及
     extern `inst_final_rx_timeout / inst_resp_rx_timeout / inst_init_rx_timeout /
     inst_poll2final_time / inst_data_interval`;
   - 照抄基站 `../Anchor_RTOS/01_Core/App/Inc/dw_instance.h:102-107,249-256`:
     `DW_RX_ON_DELAY=16`、`RX_RESPONSE_TURNAROUND=500`(**双端必须同值**,基站实测收紧时一起改)、
     `twrTimings_t`(fixedReplyDelayAnc32h / preambleDuration32h / pollTx2FinalTxDelay32h /
     fixedReplyDelay_sy / fwto4RespFrame_sy / fwto4FinalFrame_sy),文件级全局 `twr_timings`
     (标签不引入 instance_data_t 单例);
2. **`Core/application/dw_main.c`**:从基站 `../Anchor_RTOS/01_Core/App/Src/dw_main.c:326-449`
   **逐字移植** `calc_length_data()`(含 110K 分支 USE_DW1000 保护)、`plen_symbols()`、
   `sfd_length()`、`conv_us_to_devtime()`、`twr_set_replydelay()`;init 里 `dwt_configure()` 后调
   `twr_set_replydelay()`;删三档 if-else 中对已删全局的装填(保留 `inst_one_slot_time` 选择);
   boot log 打印 `TWR timing:` 与基站同格式对账(850K/PLEN1024/N=3 期望 replyDelay≈1801us、
   pollTx2Final≈7204us);
3. **`Core/application/dw_instance_tag.c`**:`tag_calc_resp_rx_time / tag_calc_final_tx_time`
   (现 :41-107 芯片×速率 6 分支)整体替换为芯片无关 TREK 公式(32h 时间基):
   - resp 槽 k 开窗 = `(poll_tx_ts>>8) + (k+1)*fixedReplyDelayAnc32h − preambleDuration32h`;
   - final TX = `(poll_tx_ts>>8) + pollTx2FinalTxDelay32h`(final_tx_time 改 uint32/32h,
     `(<<8)+ant_dly` 组装模式保留);
   - `STA_SEND_POLL` 的 `dwt_setrxtimeout` 改传 `twr_timings.fwto4RespFrame_sy`
     (symbol 单位,顺带修掉旧代码 us 当 symbol 传的 2.5% 偏差)。

### 验证(用户执行,基站刷 Phase B 固件)

- [ ] 双端 boot log `TWR timing:` 逐项一致(尤其 replyDelay 32h 值);
- [ ] 三基站 T2A:测距成功率/距离稳定性 ≥ 旧版,基站侧 A2A 不受影响;
- [ ] 30min 长跑无 `FINAL tx FAILED` 刷屏;
- [ ] SWO/RTT 实测回调耗时后,与基站同步收紧 `RX_RESPONSE_TURNAROUND`。

**风险**:公式手滑(±1 槽、忘减 preamble)表现为某基站永远超时 —— 靠 boot log 对账 +
基站 SWO 帧踪定位。Tag 主循环轮询(无 RTOS 队列),turnaround 余量比基站富裕,500us 应稳。

## Phase 2:睡眠唤醒(DW 深睡 + MCU WFI)⬜

1. **新文件 `Core/application/dw_power.c`**(声明入 `dw_instance.h`,加入 CMakeLists APP_SRC):
   - `tag_dw_sleep_config()`:init 时一次。
     DW1000:`dwt_configuresleep(DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV, DWT_WAKE_CS|DWT_SLP_EN)`;
     DW3000:`dwt_configuresleep(DWT_CONFIG|DWT_PGFCAL, DWT_PRES_SLEEP|DWT_WAKE_CSN|DWT_WAKE_WUP|DWT_SLP_EN)`;
   - `tag_dw_entersleep()`:DW1000 `dwt_entersleep()`;DW3000 `dwt_entersleep(idle_rc 按 SDK 注释)`;
   - `tag_dw_wakeup()`:DW1000 `port_wakeup_IC_fast()`;DW3000 `wakeup_device_with_io()` +
     `while(!dwt_checkidlerc())` + `dwt_restoreconfig()`;两者唤醒后统一重下:
     **tx/rx 天线延迟、`dwt_seteui`、TX 功率**(深睡不保留,TREK 参考 §7.3);
     首轮真机确认 `dwt_setlnapamode` 与中断掩码是否也需重下,若是纳入序列;
2. **`Core/application/dw_instance_tag.c`**:
   - 散落 6 处的 `next_period_time = range_time + ...` 收拢为 `tag_round_end()`:
     `next_period_time = range_time + inst_one_slot_time*inst_slot_number + tagSleepCorrection_ms
     + (Correction_flag?0:diff_time); tagSleepCorrection_ms=0; tag_dw_entersleep();
     dw_sleeping=1; state=STA_IDLE;`(即 TREK `nextPeriod = tagSleepRnd + tagSleepTime + tagSleepCorrection`);
   - `STA_IDLE`:到 `next_period_time − DW_WAKEUP_LEAD_MS`(约 4ms,唤醒 2.2ms + 重下配置余量)
     先 `tag_dw_wakeup()`,到点转 `STA_SEND_POLL`;否则 `__WFI()`(SysTick 1ms 唤醒再查;
     TIM2 报警/ADC/UART 中断均正常唤醒);
   - `STA_WAIT_RESP` 的 50ms 看门狗重启保留(唤醒失败兜底);交换内忙等一律不动。

### 验证(用户执行)

- [ ] 测距与 Phase 1 完全一致;报警 LED/蜂鸣器 4Hz 正常(WFI 不影响 TIM2);
- [ ] 电流计确认睡眠窗电流显著下降(DW1000 深睡 ~µA 级 + MCU Sleep);
- [ ] 长跑 correction 收敛、周期不漂;
- [ ] 唤醒后首轮测距成功且距离无系统性偏移(偏移数米 = 天线延迟没重下)。

**风险**:DW1000 唤醒与 IRQ 共享 EXTI0(PB0/PA0),`port_wakeup_IC_fast` 内开关 RSTn IRQ 后须
恢复 EXTI0 使能态(与 `dw1000_init` 末尾 `HAL_NVIC_EnableIRQ(EXTI0_IRQn)` 同一个坑);
DW3000 深睡路径本项目首次跑,单独安排长跑(基站有 ARFE 静默停收教训)。

## Phase 3:Discovery(标签侧)+ 失联回退 ⬜

**前置**:端到端联调需基站 Phase C;标签侧可先开发,超时自闭环冒烟。

1. **帧定义(`dw_instance.h`)**:删旧 `BLINK_MSG_LEN / INIT_MSG_LEN`,新增:
   - `BLINK_MSG_DATA_LEN 10`(0xC5 + seq + EUI64(8);+FCS = 空口 12B ISO blink);
   - `RNG_INIT_MSG_LEN 20`(fc2 + seq1 + pan2 + destEUI8 + src2 + fcode1 + corr2 + tagid2,
     +FCS = 22B;帧控 `0x41 0x8C`)、`RNG_INIT_FUNC_IDX 15`、`RNG_INIT_SLEEP_COR_IDX 16`
     (int16,字节序与 RESP sleepCorr 一致)、`RNG_INIT_TAG_ID_IDX 18`、`BLINK_PERIOD_MS 2000`;
   - `FUNC_CODE_BLINK 0x36` 弃用(真 blink 无 fcode);`FUNC_CODE_INIT 0x38` 沿用;
   - **索引落定后回写基站 Phase C 实现与 TWR_TIMING.md,双端同源**;
2. **EUI-64(`dw_main.c`)**:`tag_make_eui64()` 由 STM32 96bit UID(`UID_BASE 0x1FFFF7E8`)
   折叠 64bit,置 locally-administered 位;init 中 `dwt_seteui(eui64)`(唤醒后也重下);
   删 `#define TAG_ID 0x07`,上电 `tag_id=0xFF` 哨兵,`dwt_setaddress16` 延迟到注册成功;
   上电初始 `state = STA_SEND_BLINK`;
3. **状态机(`dw_instance_tag.c`,接线已预留的三个枚举)**:
   - `STA_SEND_BLINK`:组 12B blink,`dwt_starttx(IMMEDIATE|RESPONSE_EXPECTED)`,
     `dwt_setrxaftertxdelay(0)` + 宽接收窗(起步 ~20ms 等效 symbol,基站 rngInitTxDly 定型后
     收紧为 delayed RX)→ `STA_WAIT_INIT`;
   - `STA_WAIT_INIT`:与 `STA_WAIT_RESP` 同构(rx_status 轮询 + 看门狗),超时 →
     `tag_blink_end()`(`next_period_time = range_time + BLINK_PERIOD_MS + 伪随机抖动;
     入睡; STA_IDLE`);
   - `STA_RECV_INIT`:校验 `rx_buffer[RNG_INIT_FUNC_IDX]==0x38`(64bit 目的已被帧过滤校验),
     取 sleepCorrection 与 tag_id,`dwt_setaddress16(tag_id)`,`Correction_flag=1;
     discovery_mode=0`,按 corr 定下轮唤醒(基站 RNG_INIT 校正已含 +sfPeriod,不再叠加超帧
     —— **正负号/叠加规则以基站 Phase C 实现为准,联调双方 RTT 打印 corr 原值对账,
     这是最可能对不上的点**),入睡 → 正常 POLL 循环;
   - `STA_IDLE` 唤醒后按 `discovery_mode` 分派回 `STA_SEND_BLINK` 或 `STA_SEND_POLL`;
4. **失联回退(`dw_main.c` 主循环 :528 附近)**:`lost_flag` 置位(5s 无基站,黄灯告警保留)
   且 `!discovery_mode && state==STA_IDLE` 时:`discovery_mode=1; Correction_flag=0;`
   必要时先 `tag_dw_wakeup()`,`dwt_forcetrxoff(); state=STA_SEND_BLINK;` 并重置
   `last_range_ok_tick` 防重复触发;tag_id 保留旧值直至重新注册;
5. **帧过滤**:DW1000/DW3000 过滤模式均不变(DATA_EN 已够),仅靠 `dwt_seteui`;
   DW3000 真机确认非法帧走 rx_err 后接收机状态正常(ARFE 教训)。

### 验证(用户执行)

- [ ] 无基站:RTT 每 ~2s 一次 blink,超时入睡,电流正常;
- [ ] 与基站 Phase C 联调:blink → A0 注册 + RNG_INIT → 标签 `discovered: tag_id=N` →
      转正常 T2A;两台标签依次上电分到不同 slot;
- [ ] 拔全部基站 >5s:黄灯 + 回 discovery;恢复基站后重新注册并测距。

**风险**:基站 `rngInitTxDly` 未定,标签 RNG_INIT 接收窗先给宽(20ms),联调后收紧成
delayed RX(省电),此参数应进 TWR_TIMING.md 成为双端契约;RNG_INIT tag_id 字段 2B 的
字节序、slot 上限(MAX_TAG_NUMBER=50)双端一致性。

## Phase 4:清理与文档 ⬜

1. 删 `Core/decadriver/`、`Core/platform/`、`Core/Application/instance.h / instance_tag.c /
   instance_anchor.c`,确认无残留引用;
2. Keil 退役:`MDK-ARM/` 与 `keilkilll.bat` 删除前先打 git tag 留档(需用户确认删除时机);
3. 更新 `CLAUDE.md`:CMake 双目标构建、dw_* 文件结构、TREK 时序引用
   `../Anchor_RTOS/docs/TWR_TIMING.md`、discovery/睡眠机制、失联语义(5s 超时);
4. 回归:三基站 + 双标签 30min 长跑,报警/失联/discovery 三链路各验一遍。

---

## 关键文件

| 文件 | 角色 |
|---|---|
| `Core/Application/dw_instance_tag.c` | 状态机/时序公式/discovery/睡眠接入点(主战场) |
| `Core/Application/dw_instance.h` | 常量增删、twrTimings_t、blink/RNG_INIT 帧定义 |
| `Core/Application/dw_main.c` | twr_set_replydelay 移植、init/EUI/帧过滤、主循环失联回退 |
| `Core/Application/dw_power.c`(新) | DW 深睡/唤醒封装 |
| `CMakeLists.txt` | APP_SRC 切到 dw_instance_tag.c、加 dw_power.c |
| `../Anchor_RTOS/01_Core/App/Src/dw_main.c:326-449` | 时序计算移植源(逐字移植) |
| `../Anchor_RTOS/docs/TWR_TIMING.md` | 双端时序契约 |
| `../Anchor_RTOS/.claude/TREK1000_TWR_STATE_MACHINE_REFERENCE.md` | TREK 原版规格(睡眠 §7、discovery §3.2/§6) |

## 双端契约点(改一处必须两边同步)

1. `RX_RESPONSE_TURNAROUND`(现 500us)与 `twrTimings_t` 全套公式 —— 与基站二进制级同源;
2. RNG_INIT 帧字段偏移(`RNG_INIT_*_IDX`)、blink 帧长 —— 落定后回写基站 Phase C 与 TWR_TIMING.md;
3. 基站 `rngInitTxDly`(Phase C 定型)→ 标签 RNG_INIT 接收窗收紧;
4. RNG_INIT 的 sleepCorrection 正负号/是否含 +sfPeriod —— 以基站实现为准,RTT 对账;
5. `ONE_SLOT_TIME_MS_850K=12`、`MAX_AHCHOR_NUMBER=3`、`MAX_TAG_NUMBER=50`、RF 档位
   (ch5/PRF64/PLEN1024/850K)。
