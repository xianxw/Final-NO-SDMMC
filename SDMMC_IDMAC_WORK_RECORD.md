# SDMMC/IDMAC Debug Work Record

## 问题背景

在当前 `starry` 代码中，SD/MMC IDMAC DMA 传输在 VisionFive2 上运行失败，导致系统无法顺利进入终端。

### 发现的主要问题

1. `dma_irq_handler()` 对 SDMMC 中断处理不完整
   - 中断处理函数中清除了 `RINTSTS`，这会丢失命令完成/响应完成信号。
   - 由于 `send_cmd_idmac()` 依赖 `RINTSTS.command_done()` 判断响应，响应信号被提前清除后会导致任务等待超时。

2. `IDSTS` 清理语义不正确
   - `IDSTS` 为写一清除寄存器，之前的代码没有明确按照硬件语义清除旧状态。

3. 中断等待方式不稳定
   - 最初 `send_cmd_idmac()` 使用 `WaitQueue::wait_timeout_until()` 阻塞等待，而当前调度器环境下这个方式触发了 `axtask` 内部断言。
   - 因此出现了 `assertion failed: curr.can_preempt(2)` 的 panic。

4. 终端启动被阻塞
   - 虽然 SDMMC 初始化和 IDMAC 传输部分已经运行，但由于中断状态处理和响应等待问题，系统仍没有稳定进入终端。

## 已做的改动

1. 修正 `dma_irq_handler()`
   - 保留 `IDSTS` 的写一清除行为。
   - 移除对 `RINTSTS` 的中断阶段清除，避免消耗命令完成信号。
   - 只在检测到有效中断时设置 `IDMAC_DONE_FLAG` 并通知等待逻辑。

2. 修正 `send_cmd_idmac()` 的 DMA 等待逻辑
   - 将之前使用 `IDMAC_WAIT_QUEUE.wait_timeout_until()` 的方式改为安全的 `IDMAC_DONE_FLAG` 轮询 + `axtask::yield_now()`。
   - 这样避免在当前调度器情况下触发 `blocked_resched()` 断言。

3. 改进 `send_cmd_idmac()` 准备阶段的状态清理
   - 在传输开始前清理旧的 `RINTSTS`。
   - 在必要时清理旧的 `IDSTS` 状态。

## 当前状态

- SD/MMC 初始化已成功完成。
- IDMAC 读写传输已按日志显示成功完成多次 `cmd 17` / `cmd 24`。
- 系统已继续进入 `axfs_ng` 文件系统初始化，并挂载了 `devfs`、`tmpfs`、`proc`、`sys` 等文件系统。
- 目前日志末尾仍显示 SDMMC 传输和 IRQ 处理继续进行，尚未确认进入用户终端，但未再出现此前的 panic 或明显失败。

## 后续观察点

- 继续观察日志是否最终进入终端或出现 `Init process exited`。
- 若仍然卡住，重点检查是否存在 `RINTSTS`/`IDSTS` 未清理状态导致的重复 IRQ。
- 进一步确认 `send_cmd_idmac()` 是否在命令响应阶段仍有遗漏。
