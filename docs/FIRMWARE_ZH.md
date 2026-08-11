# 固件产物

[English](FIRMWARE.md)

CI 在每个成功的示例构建后生成可刷写压缩包。ESP-IDF 包由 `flasher_args.json` 生成；Arduino 包包含导出的二进制和必要启动文件。

每个包包含 `manifest.json`、`flash_args.txt`、平台刷写脚本和 `bin/` 下的固件。`Firmware/` 中的工厂/恢复镜像是独立交付边界，不是 examples CI 产物。
