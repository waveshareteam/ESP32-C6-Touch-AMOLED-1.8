# 贡献指南

[English](CONTRIBUTING.md)

感谢帮助改进 ESP32-C6-Touch-AMOLED-1.8 示例。

## 提交 PR 前

- 保持 ESP-IDF 示例与 CI 矩阵兼容。
- Arduino 修改保留在对应的 V1 或 V2 目录。
- 优先在 `main/idf_component.yml` 中使用受管 ESP-IDF 组件。
- 不提交生成的 `build/`、`managed_components/`、`dependencies.lock`、本地 `sdkconfig` 或缓存。
- 行为、硬件要求、menuconfig 选项或预期输出变化时更新示例 README。

## 示例风格

新的 ESP-IDF 示例应位于 `examples/esp-idf/<number>_<name>/`，并包含 `README.md`、`CMakeLists.txt`、`main/CMakeLists.txt` 和 `main/` 源码。仅在示例需要非默认 Kconfig 值时添加 `sdkconfig.defaults`。

请遵循[示例索引](examples/README_ZH.md)中的学习顺序。

## 文档与 CI

公开说明使用仓库相对路径，避免本机路径和私有信息。CI 路由说明见 [持续集成](docs/CI_ZH.md)。
