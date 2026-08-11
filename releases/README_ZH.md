# Release 工具

[English](README.md)

## 下载 CI 产物

使用 Python 3.10 或更新版本运行 `python3 releases/download_artifacts.py --run-id RUN_ID --clean`。省略 `--run-id` 会选择当前分支最近成功的工作流；可使用 `--artifact` 或 `--pattern` 精确选择。

下载文件会解压到 `releases/downloads`，并由 Git 忽略。

## 打包固件

GitHub Actions 在成功构建后调用 `package_firmware.py`。ESP-IDF 读取 `flasher_args.json`，Arduino 读取导出的二进制；输出归档包含 manifest、刷写参数、刷写辅助脚本和所需二进制。
