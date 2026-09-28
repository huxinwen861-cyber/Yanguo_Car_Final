# Camera Lane V1 Freeze

状态：**FROZEN / 真机验证通过**

硬件：MT9V03X

图像尺寸：188 × 120

---

## 1. 冻结对象

正式冻结以下模块：

| 文件 | 说明 |
|---|---|
| `project/user/camera_lane.c` | Camera Lane V1 正式实现 |
| `project/user/camera_lane.h` | Camera Lane V1 正式接口 |
| `project/user/main.c` | 调用 Camera Lane V1 的入口 |

只读参考快照（**不加入 Keil Target**）：

| 文件 | 说明 |
|---|---|
| `project/user/frozen/camera_lane_v1.c` | 冻结快照 |
| `project/user/frozen/camera_lane_v1.h` | 冻结快照 |

基线来源（**禁止修改**）：

`project/user/main_backup_before_servo_direction_test.c`

---

## 2. 完整处理链

```text
MT9V03X
→ image_copy
→ 固定阈值二值化
→ 左右边线
→ center_line
→ LOOKAHEAD
→ steering_error
```

---

## 3. 冻结参数

| 参数 | 值 |
|---|---|
| `IMAGE_THRESHOLD` | 128 |
| `EDGE_SEARCH_TOP` | 50 |
| `EDGE_SEARCH_RANGE` | 20 |
| `IMAGE_CENTER_X` | `MT9V03X_W / 2` |
| 当前 188 宽度下 `IMAGE_CENTER_X` | 94 |
| `LOOKAHEAD_ROW` | 90 |

---

## 4. 误差定义

```c
steering_error =
    (int16)center_line[LOOKAHEAD_ROW]
    - (int16)IMAGE_CENTER_X;
```

### 符号定义

| 条件 | 含义 |
|---|---|
| `error < 0` | LOOKAHEAD 中线位于图像中心左侧 |
| `error ≈ 0` | 中线接近图像中心 |
| `error > 0` | LOOKAHEAD 中线位于图像中心右侧 |

### 有效条件

只有 `center_valid[LOOKAHEAD_ROW]` 有效时，`steering_error` 才有效。

---

## 5. 保留功能

- 固定阈值二值化
- 左右边线搜索
- center_line
- center_valid
- LOOKAHEAD
- steering_error
- error_valid
- 左右边线可视化
- 中线可视化
- LOOKAHEAD 可视化
- firmware mark
- UART8 低频诊断输出

---

## 6. 冻结边界

Camera Lane V1 **不负责**：

- 前轮舵机控制
- steering P / PD / PID
- 后轮电机
- ESC
- 编码器
- 速度 PI
- 十字
- 环岛
- 坡道
- 动态前瞻
- Otsu / 自适应阈值
- 图像滤波
- 打靶

后续这些功能**必须通过接口使用 Camera Lane V1 输出**，
不得为了开发其他模块随意修改 Camera Lane V1。

---

## 7. 机械结构

本车：

- 前轮 = 舵机转向
- 后轮 = 电机驱动

如果文档或新增注释中出现 `Rear steering servo`，必须修正。
但**不要因此修改历史 backup 文件**。

---

## 8. 冻结规则

Camera Lane V1 从此视为**冻结模块**。

后续正常开发**禁止修改**：

- `camera_lane.c`
- `camera_lane.h`

如果确实发现 Camera Lane V1 本身存在 Bug：

1. 先复现问题
2. 保存证据
3. 创建单独修复任务
4. 修改后重新 Rebuild
5. 重新真机回归
6. 更新 Freeze 文档
7. 必要时升级为 V1.1

**禁止**因为开发舵机、PID、十字、坡道等功能，顺手修改 Camera Lane V1。

---

## 9. 验证记录

| 项目 | 结果 |
|---|---|
| Keil UV4 Build | 0 Error(s), 0 Warning(s) |
| 真机回归测试 | 用户已确认，功能正常 |
| 冻结日期 | 2026-09-28 |
