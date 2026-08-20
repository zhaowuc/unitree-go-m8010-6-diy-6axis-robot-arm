# DM-G6220 电机位置控制工程

本工程用于在 Windows 上通过达妙官方 USB-CAN/USB2FDCAN 模块控制 DM-G6220。

## 文件

- `dm_g6220_memory_gui.py`：精简位置控制 GUI，支持自动扫描、角度发送、位置记忆、复位、失能和串口上电日志诊断。
- `dm_g6220_run_100_cycle.py`：达妙 USB-CAN 通信、参数读取、反馈解析和位置控制后端。
- `dm_g6220_position_memory.json`：GUI 保存的位置记忆。
- `start_gui.bat`：双击启动 GUI。
- `requirements.txt`：Python 依赖。

## 使用

1. 电机使用独立合适电源供电。
2. CAN_H 对 CAN_H，CAN_L 对 CAN_L。
3. 如连接 3-pin 调试串口，应为 RX 对 TX、TX 对 RX、GND 对 GND。
4. 双击 `start_gui.bat`。
5. 等待状态显示已连接官方 USB-CAN，再输入角度并发送。

默认角度是电机坐标系中的绝对角度，单位为度。发送运动命令前应固定电机并清理旋转范围内的线缆。

## 环境

- Windows 10/11
- Python 3.10 或更高版本
- 达妙官方 DM-USB2FDCAN 驱动
- `pip install -r requirements.txt`

