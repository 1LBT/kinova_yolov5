#!/bin/bash
# 环境诊断脚本 - 检测 Conda 与 ROS 的冲突

echo "==============================================="
echo "        Conda + ROS 环境冲突诊断工具"
echo "==============================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 诊断函数
diagnose_environment() {
    echo -e "${BLUE}[1/8] 检查当前环境状态${NC}"
    echo "==============================================="
    
    # 检查 Conda 环境
    if [ -n "$CONDA_PREFIX" ]; then
        echo -e "${GREEN}✓ 当前在 Conda 环境中: $CONDA_PREFIX${NC}"
        echo "Conda 环境名称: $(basename $CONDA_PREFIX)"
    else
        echo -e "${YELLOW}⚠ 当前不在 Conda 环境中${NC}"
    fi
    
    # 检查 ROS 环境
    if [ -n "$ROS_DISTRO" ]; then
        echo -e "${GREEN}✓ ROS 环境已设置: $ROS_DISTRO${NC}"
    else
        echo -e "${RED}✗ ROS 环境未设置${NC}"
    fi
    
    echo ""
    
    # 检查 Python 解释器
    echo -e "${BLUE}[2/8] 检查 Python 环境${NC}"
    echo "==============================================="
    PYTHON_PATH=$(which python)
    PYTHON3_PATH=$(which python3)
    echo "Python 路径: $PYTHON_PATH"
    echo "Python3 路径: $PYTHON3_PATH"
    
    if [[ "$PYTHON_PATH" == *"conda"* ]]; then
        echo -e "${GREEN}✓ 使用 Conda 的 Python${NC}"
    else
        echo -e "${YELLOW}⚠ 使用系统 Python${NC}"
    fi
    
    # 检查 Python 版本
    python --version 2>/dev/null && echo -e "${GREEN}✓ Python 可执行${NC}" || echo -e "${RED}✗ Python 不可执行${NC}"
    echo ""
    
    # 检查关键 Python 包
    echo -e "${BLUE}[3/8] 检查关键 Python 包${NC}"
    echo "==============================================="
    
    check_package() {
        python -c "import $1" 2>/dev/null
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ $1 可导入${NC}"
            python -c "import $1; print('  版本:', $1.__version__); print('  路径:', $1.__file__)" 2>/dev/null
        else
            echo -e "${RED}✗ $1 不可导入${NC}"
        fi
        echo ""
    }
    
    packages=("rospkg" "torch" "scipy" "skimage" "numpy" "cv2" "defusedxml" "lxml")
    
    for pkg in "${packages[@]}"; do
        check_package $pkg
    done
    
    # 检查 PyTorch 特定问题
    echo -e "${BLUE}[4/8] 检查 PyTorch 库链接${NC}"
    echo "==============================================="
    
    if python -c "import torch" 2>/dev/null; then
        TORCH_PATH=$(python -c "import torch; print(torch.__file__)" 2>/dev/null)
        TORCH_LIB_DIR=$(dirname "$TORCH_PATH")/lib
        
        echo "PyTorch 路径: $TORCH_PATH"
        echo "PyTorch lib 目录: $TORCH_LIB_DIR"
        
        # 检查 libtorch_cpu.so 的依赖
        if [ -f "$TORCH_LIB_DIR/libtorch_cpu.so" ]; then
            echo -e "${GREEN}✓ 找到 libtorch_cpu.so${NC}"
            
            # 检查缺失的符号
            if ldd "$TORCH_LIB_DIR/libtorch_cpu.so" 2>/dev/null | grep -q "not found"; then
                echo -e "${RED}✗ 发现缺失的依赖库:${NC}"
                ldd "$TORCH_LIB_DIR/libtorch_cpu.so" | grep "not found"
            else
                echo -e "${GREEN}✓ 所有依赖库都找到${NC}"
            fi
            
            # 检查 iJIT_NotifyEvent 符号
            if nm -D "$TORCH_LIB_DIR/libtorch_cpu.so" 2>/dev/null | grep -q "iJIT_NotifyEvent"; then
                echo -e "${YELLOW}⚠ 找到 iJIT_NotifyEvent 符号（可能有问题）${NC}"
            fi
        else
            echo -e "${RED}✗ 未找到 libtorch_cpu.so${NC}"
        fi
    else
        echo -e "${RED}✗ PyTorch 不可用${NC}"
    fi
    echo ""
    
    # 检查环境变量冲突
    echo -e "${BLUE}[5/8] 检查环境变量冲突${NC}"
    echo "==============================================="
    
    echo "PYTHONPATH:"
    echo "$PYTHONPATH" | tr ':' '\n' | while read line; do
        if [[ "$line" == *"conda"* ]]; then
            echo -e "${YELLOW}  $line (Conda 路径)${NC}"
        elif [[ "$line" == *"ros"* ]]; then
            echo -e "${BLUE}  $line (ROS 路径)${NC}"
        else
            echo "  $line"
        fi
    done
    echo ""
    
    echo "LD_LIBRARY_PATH:"
    echo "$LD_LIBRARY_PATH" | tr ':' '\n' | while read line; do
        if [[ "$line" == *"conda"* ]]; then
            echo -e "${YELLOW}  $line (Conda 路径)${NC}"
        elif [[ "$line" == *"ros"* ]]; then
            echo -e "${BLUE}  $line (ROS 路径)${NC}"
        else
            echo "  $line"
        fi
    done
    echo ""
    
    # 检查包版本冲突
    echo -e "${BLUE}[6/8] 检查包版本冲突${NC}"
    echo "==============================================="
    
    check_version_conflict() {
        echo "检查 $1:"
        # 检查 Conda 安装的版本
        conda list $1 2>/dev/null | grep -E "^$1\s" && echo -e "${YELLOW}  ↑ Conda 版本${NC}"
        # 检查 Pip 安装的版本
        pip show $1 2>/dev/null | grep Version && echo -e "${BLUE}  ↑ Pip 版本${NC}"
        # 检查系统包版本
        dpkg -l $1 2>/dev/null | grep ii && echo -e "${GREEN}  ↑ 系统版本${NC}"
        echo ""
    }
    
    check_version_conflict "python3-scipy"
    check_version_conflict "python3-sklearn"
    check_version_conflict "python3-opencv"
    
    # 检查路径优先级冲突
    echo -e "${BLUE}[7/8] 检查路径优先级${NC}"
    echo "==============================================="
    
    python -c "
import sys
print('Python 路径搜索顺序:')
for i, path in enumerate(sys.path):
    if 'conda' in path:
        print(f'  {i}: {path} ← ${YELLOW}Conda 路径${NC}')
    elif 'ros' in path or 'opt' in path:
        print(f'  {i}: {path} ← ${BLUE}ROS/系统路径${NC}')
    else:
        print(f'  {i}: {path}')
" 2>/dev/null || echo "无法获取 Python 路径"
    echo ""
    
    # 生成诊断报告
    echo -e "${BLUE}[8/8] 生成诊断报告${NC}"
    echo "==============================================="
    
    # 检测常见问题模式
    PROBLEMS=()
    
    # 问题1: 同时存在 Conda 和系统 Python 路径
    if [[ "$PYTHONPATH" == *"conda"* && "$PYTHONPATH" == *"ros"* ]]; then
        PROBLEMS+=("Conda 和 ROS Python 路径混合，可能导致导入冲突")
    fi
    
    # 问题2: PyTorch 符号问题
    if python -c "import torch" 2>/dev/null; then
        if ldd "$(python -c "import torch; print(torch.__file__)" 2>/dev/null)/../lib/libtorch_cpu.so" 2>/dev/null | grep -q "not found"; then
            PROBLEMS+=("PyTorch 库有缺失的依赖")
        fi
    fi
    
    # 问题3: 关键包缺失
    MISSING_PKGS=()
    for pkg in "${packages[@]}"; do
        python -c "import $pkg" 2>/dev/null || MISSING_PKGS+=("$pkg")
    done
    
    if [ ${#MISSING_PKGS[@]} -gt 0 ]; then
        PROBLEMS+=("缺失关键包: ${MISSING_PKGS[*]}")
    fi
    
    # 输出诊断结果
    if [ ${#PROBLEMS[@]} -eq 0 ]; then
        echo -e "${GREEN}✓ 未发现明显环境冲突${NC}"
    else
        echo -e "${RED}✗ 发现以下问题:${NC}"
        for problem in "${PROBLEMS[@]}"; do
        echo "  • $problem"
    done
    
    echo ""
    echo -e "${YELLOW}建议解决方案:${NC}"
    
    if [[ " ${PROBLEMS[@]} " =~ "Conda 和 ROS Python 路径混合" ]]; then
        echo "  1. 清理 PYTHONPATH: unset PYTHONPATH"
        echo "  2. 重新设置环境: source /opt/ros/noetic/setup.bash"
    fi
    
    if [[ " ${PROBLEMS[@]} " =~ "PyTorch" ]]; then
        echo "  1. 重新安装 PyTorch: conda install pytorch cpuonly -c pytorch"
        echo "  2. 或设置环境变量: export LD_PRELOAD= MKL_THREADING_LAYER=GNU"
    fi
    
    if [[ " ${PROBLEMS[@]} " =~ "缺失关键包" ]]; then
        echo "  1. 安装缺失包: conda install ${MISSING_PKGS[*]}"
        echo "  2. 或使用 pip: pip install ${MISSING_PKGS[*]}"
    fi
    fi
    
    echo ""
    echo "==============================================="
    echo -e "${GREEN}诊断完成！${NC}"
    echo "==============================================="
}

# 保存原始环境
ORIGINAL_PYTHONPATH="$PYTHONPATH"
ORIGINAL_LD_LIBRARY_PATH="$LD_LIBRARY_PATH"

# 执行诊断
diagnose_environment

# 恢复原始环境（如果需要）
export PYTHONPATH="$ORIGINAL_PYTHONPATH"
export LD_LIBRARY_PATH="$ORIGINAL_LD_LIBRARY_PATH"