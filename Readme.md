# 电机型号
CTN28-0601-100  电流0.5A，螺距1mm

# 电机接线
红A+
蓝A-
黑B+
绿B-

# DM420拨码
1 0 0 0 1 1 0 0 #1为向上，0为向下。

# DM420脉冲
PUL+  ---GPIO
DIR+  ---GPIO
PUL-,DIR-,ENA- 共GND
ENA+悬空

# 启动方式
- 真实硬件模式：
  - `python app.py`
- 模拟模式（实时模拟，不连接GPIO/电机）：
  - `python app.py --simulate`

# 说明
- 模拟模式会保留原有接口与控制流程（/move、/home、/cycle、/position），用于在无硬件时验证操作流程和状态变化。
