# Final 版本 SD/MMC DMA 修改记录

日期：2026-07-20

## 修改范围

主要修改文件：

- `Final-NO-SDMMC/modules/simple-sdmmc-extended/src/sdmmc.rs`
- `Final-NO-SDMMC/modules/simple-sdmmc-extended/Cargo.lock`

`Cargo.lock` 已按 root 版本使用的依赖版本进行同步。DMA 驱动修改集中在
`sdmmc.rs`，没有修改 `regs.rs` 的寄存器位定义。

## DMA 驱动修改

1. 增加 `IDMAC_ERROR_FLAG: AtomicBool`。每次传输开始前清零；IRQ 检测到
   `IDSTS.AIS/CES/DU/FBE` 或 `RINTSTS` 控制器错误时置位。轮询代码通过该标志
   保存 ISR 已经观察到并清除的错误事实。
2. PIO 后备路径固定访问 `base + 0x200` 的 FIFO MMIO 地址，不再按照缓冲区
   offset 移动 FIFO 寄存器地址；同时检查剩余缓冲区至少有 8 字节，避免切片越界。
3. DMA 响应等待同时检查 `command_done`、`RINTSTS.error()` 和
   `IDMAC_ERROR_FLAG`，避免错误中断被 ISR 清除后等待到超时。
4. 新增统一 DMA 收尾函数。错误或超时时先 reset IDMAC，随后 W1C 清理
   `RINTSTS/IDSTS`、释放描述符，并恢复 BMOD、CTRL、IDINTEN 和 INTMASK。
5. 如果 IDMAC reset 自身超时，则清理状态但保留描述符。此时不能证明硬件已经
   停止访问描述符，强行释放会造成 DMA use-after-free。
6. `start_cmd` 增加 100 ms 超时。超时后立即执行统一恢复并返回错误，不再继续
   进入响应或数据阶段。
7. 数据等待阶段检查相对传输基线新出现的 `FBE/DU/CES/RINTSTS error`；数据超时、
   ISR 错误和轮询错误均进入同一清理路径。
8. 正常释放描述符前检查 OWN 位已经由硬件归还。OWN 仍为 1 时按错误路径 reset，
   避免释放 IDMAC 仍可能访问的内存。
9. IRQ 完成判定限制为 `IDSTS.RI/TI`、`RINTSTS.DTO` 或明确错误。
   `CMD_DONE`、`RXDR/TXDR` 和其他无关状态不会再误报 DMA 完成。
10. 增加 DMA 启动可见性：第一次成功执行启动流程使用 `warn!` 打印命令、FSM、
    OWN 和 DBADDR；后续启动降为 `debug!`，避免单块读取产生大量 warn 日志。

## 上板结果

2026-07-20 的 VisionFive 2 日志显示：

- SD 卡完成 CMD0、CMD8、ACMD41、CID、RCA、CSD、选卡和 SCR 初始化。
- 首次数据命令为 CMD17，启动日志为 `fsm=3`、`desc_own=true`，说明 IDMAC
  已进入 DESC_CHK 并接管描述符。
- CMD17 之后系统继续读取根文件系统并进入 `starry:~#` shell。
- `read_block()` 会对 `send_cmd_idmac()` 的结果执行 `unwrap()`；若 DMA 返回错误，
  系统会在进入 shell 前 panic。因此本次日志能够确认 DMA 传输成功，而不仅是
  描述符配置成功。
- 启动后出现过一次 RINTSTS/IDSTS 均为零的 stray IRQ。它没有伴随 DMA 错误、
  超时或 panic，不属于此次 CMD17 传输失败。

## 日志与注释精简

确认 DMA 上板传输成功后，对 `sdmmc.rs` 进行了不改变寄存器编程和错误处理的
降噪整理：

- 删除初始化阶段逐寄存器打印、时钟配置步骤流水账和正常命令成功日志。
- 删除每个块读写前后的寄存器快照，以及 IDMAC 的 DBADDR、CMDARG、CMD、
  PLDMND 等正常阶段快照。
- 删除重复的 BMOD、CTRL、IDINTEN、INTMASK expected/actual 成功打印；寄存器
  回读校验仍然执行，只有校验失败才输出完整告警。
- 删除注释掉的旧描述符字段、64 位地址设想、临时代码和重复叙述性注释。
- 将 RINTSTS/IDSTS 均为零的 stray IRQ 从两条 `warn!` 合并为一条 `debug!`。
- 保留所有错误、超时、异常寄存器状态、恢复失败和第一次 DMA 启动日志。
- 文件由整理前约 1878 行缩减到 1228 行。

## 验证

修改后使用完整 VisionFive 2 配置执行了：

```text
rustfmt --check modules/simple-sdmmc-extended/src/sdmmc.rs
git diff --check
AX_CONFIG_PATH=/tmp/final-vf2-axconfig.toml cargo check \
  --features vf2 \
  --target riscv64gc-unknown-none-elf \
  --offline \
  --target-dir /tmp/try_sdmmc-root-check
```

全工程检查通过。输出中的 dead-code 警告来自现有内核代码，与 SD/MMC 修改无关。
