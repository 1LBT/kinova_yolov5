# 环境检测与测试工具集

本目录包含从原项目归档的测试脚本和环境诊断工具。这些脚本主要用于调试 TF 变换、手眼标定和机械臂控制，不参与核心业务逻辑。

## 脚本说明

### 核心备份
*   **`kinova_open_loop_grasp.py`**: 开环抓取脚本的备份。这是 `kinova_yolo_grasp.py` 的前身，不依赖 YOLO，仅用于测试基本的抓取动作序列（Home -> 移动 -> 抓取 -> 提起）。

### 环境诊断
*   **`diagnose_env.sh`**: Shell 脚本，用于快速检查当前 Shell 的环境变量（ROS_IP, ROS_MASTER_URI 等）和 Python 环境。
*   **`diagnose_tf_problem.py`**: 诊断 TF 树中是否存在断链或时间戳不同步的问题。
*   **`debug_tf_chain.py`**: 打印特定 TF 链的详细信息。

### 功能测试
*   **`test_handeye_calibration.py`**: 验证手眼标定结果。通常会让机械臂移动到几个点，检查末端和相机的相对位置是否符合预期。
*   **`test_correct_tf.py`**: 专门用于测试 TF 修正逻辑（如坐标系旋转）。
*   **`test_xy_control.py`**: 测试机械臂在 XY 平面上的位移控制精度，用于排查“移动不到位”或“方向反了”的问题。
*   **`test.py` / `test2.py`**: 临时的功能测试片段。

## 使用方法

由于这些脚本被移动到了子目录，如果它们内部有相对路径引用（如 `from helpers import ...`），直接运行可能会报错。

**建议的运行方式**：
在 `scripts` 目录下运行，并指定 PYTHONPATH，或者临时将脚本复制回 `scripts` 目录运行。

例如：
```bash
# 临时复制回上级目录运行
cp tests_and_utils/test_xy_control.py ../scripts/
cd ../scripts
python3 test_xy_control.py
rm test_xy_control.py
```
