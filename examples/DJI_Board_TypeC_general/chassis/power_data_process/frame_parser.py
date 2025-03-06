import struct

# 定义帧头
FRAME_HEADER = bytes([0xAA, 0xBB, 0xCC, 0xDD])
HEADER_SIZE = len(FRAME_HEADER)

# 定义数据字段的格式
# 格式: (名称, 格式化字符串, 字节大小)
# 格式化字符串参考Python struct模块文档
FIELD_DEFINITIONS = [
    ('angular_velocity', '<f', 4),  # 小端格式的float, 4字节
    ('raw_current', '<h', 2),       # 小端格式的int16, 2字节
    ('read_current', '<h', 2),      # 小端格式的int16, 2字节
]

# 计算帧总大小
FRAME_SIZE = HEADER_SIZE + sum(field[2] for field in FIELD_DEFINITIONS)

def parse_frame(data):
    """解析单个数据帧
    
    Args:
        data: 完整的数据帧
        
    Returns:
        字典，包含解析后的数据，如果解析失败返回None
    """
    if len(data) != FRAME_SIZE:
        return None
    
    # 检查帧头
    if data[0:HEADER_SIZE] != FRAME_HEADER:
        return None
    
    try:
        result = {}
        offset = HEADER_SIZE  # 从帧头后开始解析
        
        # 根据定义的字段格式解析数据
        for field_name, format_str, size in FIELD_DEFINITIONS:
            field_data = data[offset:offset+size]
            result[field_name] = struct.unpack(format_str, field_data)[0]
            offset += size
        
        return result
    except struct.error:
        return None

def find_and_parse_frames(buffer):
    """从数据缓冲区中查找并解析所有有效的数据帧
    
    Args:
        buffer: 字节数据缓冲区
        
    Returns:
        tuple: (list of parsed frames, remaining buffer)
    """
    frames = []
    start_idx = 0
    
    while start_idx <= len(buffer) - FRAME_SIZE:
        # 查找帧头
        if buffer[start_idx:start_idx+HEADER_SIZE] == FRAME_HEADER:
            # 提取完整帧
            frame_data = buffer[start_idx:start_idx+FRAME_SIZE]
            parsed_data = parse_frame(frame_data)
            if parsed_data:
                frames.append(parsed_data)
            start_idx += FRAME_SIZE
        else:
            start_idx += 1
            
    # 返回解析到的帧和剩余数据
    return frames, buffer[start_idx:]

def add_field(name, format_str, size):
    """添加新的数据字段到帧结构中
    
    Args:
        name: 字段名称
        format_str: 格式化字符串 (例如 '<f'、'<h')
        size: 字段大小（字节）
        
    Returns:
        None
    """
    global FRAME_SIZE
    FIELD_DEFINITIONS.append((name, format_str, size))
    FRAME_SIZE = HEADER_SIZE + sum(field[2] for field in FIELD_DEFINITIONS)
    
def remove_field(name):
    """从帧结构中移除数据字段
    
    Args:
        name: 要移除的字段名称
        
    Returns:
        bool: 移除成功返回True，字段不存在返回False
    """
    global FRAME_SIZE, FIELD_DEFINITIONS
    
    # 查找字段
    field_index = None
    for i, (field_name, _, _) in enumerate(FIELD_DEFINITIONS):
        if field_name == name:
            field_index = i
            break
    
    if field_index is None:
        return False
        
    # 移除字段
    FIELD_DEFINITIONS.pop(field_index)
    FRAME_SIZE = HEADER_SIZE + sum(field[2] for field in FIELD_DEFINITIONS)
    return True
