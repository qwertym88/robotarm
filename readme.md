## 机械臂模型

限制条件：

1. -90 <= j1 <= 90
2. 0 <= j2 <= 180
3. 0 <= j3 <= 180
4. 180 - j2 - j3 >= 20

## rs485通讯协议

### 语句结束标志/空语句

`0x3b` ';'

### 机械爪部分

**命令头**

`0x50 0x55 0x4d 0x50 0x20`  'PUMP '

**参数**

开启 `0x4f 0x4e`  'ON'

关闭 `0x4f 0x46 0x46`  'OFF'

查询 `0x3f` '?'

**结束标志**

`0x3b` ';'

**返回值**

开启状态

1 -> 开启

2-> 关闭

**完整示例**

开启吸盘/机械爪夹住 'PUNP ON;'  返回值：1

关闭吸盘/机械爪松开 'PUNP OFF;' 返回值：0

查询状态 'PUNP ？;' 返回值：0/1

### MOVE

**命令头**

`0x4d 0x4f 0x56 0x45 0x20`  'MOVE '

**参数**

(x,y,z) 坐标点 

V 速度

**结束标志**

`0x3b` ';'

**返回值**

能否到达。若不能到达，则不会运动，直接返回0。若能，到达时返回1。

**完整示例**

以 120 mm/s 速度移动到点(120,160,137.77777) 'MOVE 120 160 137.77777 160;'  返回值：0/1

### MOVP

**命令头**

`0x4d 0x4f 0x56 0x50 0x20`  'MOVP '

**参数**

(x,y,z) 坐标点 

V = 60 mm/s

**结束标志**

`0x3b` ';'

**返回值**

能否到达。若不能到达，则不会运动，直接返回0。若能，到达时返回1。

**完整示例**

以 120 mm/s 速度移动到点(120,160,137.77777) 'MOVP 120 160 137.77777;'  返回值：0/1

### MOVA

**命令头**

`0x4d 0x4f 0x56 0x41 0x20`  'MOVA '

**参数**

(a1,a2,a3) 关节角度

**结束标志**

`0x3b` ';'

**返回值**

能否到达。若不能到达，则不会运动，直接返回0。若能，到达时返回1。

**完整示例**

机械臂摆到初始状态('7'字形) 'MOVA 0 90 0;'  返回值：0/1