import torch
import torch.nn as nn
from collections import OrderedDict
import os
import sys
import torch.nn.functional as F
# 添加 GGCNN 仓库路径到系统路径
sys.path.append('/home/liu/kinova_ws/src/kinova-ros/ggcnn/ggcnn-master/models')

# 从外部文件导入 GGCNN2 模型定义
try:
    from ggcnn2 import GGCNN2
    print("成功从外部文件导入 GGCNN2 模型定义")
except ImportError as e:
    print(f"无法导入 GGCNN2 模型定义: {e}")
    print("使用备用定义")
    
def load_ggcnn2(model_path):
    """加载 GGCNN2 模型
    参数:
        model_path (str): 模型文件路径
    返回:
        model (nn.Module): 加载的模型
        device (torch.device): 使用的设备
    """
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"使用设备: {device}")
    # 检查文件是否存在
    if not os.path.exists(model_path):
        print(f"错误: 文件不存在: {model_path}")
        # 尝试可能的路径变体
        possible_paths = [
            '/' + model_path,  # 添加根目录
            os.path.expanduser('~') + '/' + model_path.lstrip('home/'),  # 扩展到用户目录
            os.path.join(os.path.dirname(__file__), model_path),  # 相对于脚本目录
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                model_path = path
                print(f"找到文件: {path}")
                break
        else:
            raise FileNotFoundError(f"无法找到模型文件: {model_path}")
    # 加载模型文件
    try:
        # 尝试使用 weights_only=True（更安全）
        state_dict = torch.load(model_path, map_location=device, weights_only=True)
        print("使用 weights_only=True 成功加载状态字典")
    except:
        print("使用 weights_only=True 失败，尝试 weights_only=False")
        state_dict = torch.load(model_path, map_location=device, weights_only=False)
    
    # 检查状态字典键名
    print("状态字典键名示例:")
    for i, key in enumerate(list(state_dict.keys())):
        if i < 5:  # 只显示前5个键
            print(f"  {key}: {state_dict[key].shape}")
    
    # 创建模型实例
    model = GGCNN2()
    
    # 加载状态字典到模型
    try:
        model.load_state_dict(state_dict)
        print("成功加载状态字典到模型")
    except Exception as e:
        print(f"加载状态字典失败: {e}")
        print("尝试检查键名不匹配...")
        
        # 检查键名不匹配
        model_keys = set(model.state_dict().keys())
        state_keys = set(state_dict.keys())
        
        print(f"模型键名: {len(model_keys)}")
        print(f"状态字典键名: {len(state_keys)}")
        
        missing_keys = model_keys - state_keys
        unexpected_keys = state_keys - model_keys
        
        if missing_keys:
            print(f"缺失的键: {missing_keys}")
        if unexpected_keys:
            print(f"意外的键: {unexpected_keys}")
        
        raise
    
    model.to(device)
    model.eval()
    
    return model, device

# 使用
model_path = '/home/liu/kinova_ws/src/kinova-ros/ggcnn/ggcnn2_weights_cornell/epoch_50_cornell_statedict.pt'
model, device = load_ggcnn2(model_path)

print("GGCNN2模型加载成功")
print(f"模型类型: {type(model)}")
print(f"模型参数数量: {sum(p.numel() for p in model.parameters())}")