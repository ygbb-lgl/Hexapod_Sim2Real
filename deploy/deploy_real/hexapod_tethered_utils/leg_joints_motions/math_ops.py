"""math_ops.py

这是对 `math_ops.c/.h` 的 Python 版等价实现（尽量保持函数名/行为一致）。

用途：
- 提供打包/解包电机 CAN 协议时需要的数学/数值转换工具。
- 其中 `float_to_uint` / `uint_to_float` 用于把连续的浮点量映射到固定位宽整数（协议里常见）。

注意：
- Python 自带 `max/min`，这里保留 `fmaxf/fminf` 只是为了和原 C 代码对齐，方便你对照学习。
"""

from __future__ import annotations

import math
from typing import Tuple

PI: float = 3.14159265359


def fmaxf(x: float, y: float) -> float:
    """返回 x、y 中较大值（对应 C 的 fmaxf）。"""

    return x if x > y else y


def fminf(x: float, y: float) -> float:
    """返回 x、y 中较小值（对应 C 的 fminf）。"""

    return x if x < y else y


def fmaxf3(x: float, y: float, z: float) -> float:
    """返回 x、y、z 中最大值（对应 C 的 fmaxf3）。"""

    return max(x, y, z)


def fminf3(x: float, y: float, z: float) -> float:
    """返回 x、y、z 中最小值（对应 C 的 fminf3）。"""

    return min(x, y, z)


def limit_norm(x: float, y: float, limit: float) -> Tuple[float, float]:
    """将向量 (x, y) 的模长限制到 <= limit。

    C 版是就地修改指针参数；Python 版返回新的 (x, y)。
    """

    norm = math.sqrt(x * x + y * y)
    if norm > limit and norm > 0.0:
        scale = limit / norm
        return x * scale, y * scale
    return x, y


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """把浮点数映射为无符号整数。

    对应原 C 实现：
        span = x_max - x_min
        offset = x_min
        return (int)((x-offset) * ((1<<bits)-1) / span)

    备注：这里不做额外 clamp，clamp 逻辑在上层打包函数里处理（保持和原代码一致）。
    """

    span = x_max - x_min
    offset = x_min
    return int((x - offset) * float((1 << bits) - 1) / span)


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    """把无符号整数反映射为浮点数（协议解包用）。"""

    span = x_max - x_min
    offset = x_min
    return float(x_int) * span / float((1 << bits) - 1) + offset


def float32_to_float16(value: float) -> int:
    """把 float32 按原 C 算法转换成“float16 位模式”（返回 16-bit int）。

    说明：
    - 原 C 代码实现的是一种近似/特定格式的转换（不是完整 IEEE754 float16 处理）。
    - 这里为了“对照原逻辑”，尽量逐字节复刻其位操作。

    如果你只是想用标准 float16，建议用 numpy.float16；但这里不引入额外依赖。
    """

    import struct

    b = struct.pack("<f", float(value))  # little-endian: b[0] 为最低字节
    # 对应 C:
    # temp=(buf[3]&0x7F)<<1 | ((buf[2]&0x80)>>7);
    # temp-=112;
    # out=temp<<10 | (buf[2]&0x7F)<<3 | buf[1]>>5;
    temp = ((b[3] & 0x7F) << 1) | ((b[2] & 0x80) >> 7)
    temp = (temp - 112) & 0xFFFF
    out = ((temp << 10) & 0xFFFF) | ((b[2] & 0x7F) << 3) | (b[1] >> 5)
    # 符号位：*float16 |= ((f32.v_int & 0x80000000) >> 16);
    sign = (b[3] & 0x80) << 8
    return int((out | sign) & 0xFFFF)


def float16_to_float32(bits16: int) -> float:
    """把 16-bit 位模式按原 C 算法转换回 float32。"""

    import struct

    bits16 &= 0xFFFF
    temp2 = (((bits16 & 0x7C00) >> 10) + 112) & 0xFFFF

    b3 = (temp2 >> 1) & 0xFF
    b2 = (((temp2 & 0x01) << 7) | ((bits16 & 0x03FC) >> 3)) & 0xFF
    b1 = ((bits16 & 0x03) << 6) & 0xFF
    b0 = 0

    # 组回 little-endian 的 float32 字节
    # 同时加上符号位：f32.v_int |= ((*float16 & 0x8000) << 16);
    sign = 0x80 if (bits16 & 0x8000) else 0x00
    b3 = (b3 | sign) & 0xFF

    return struct.unpack("<f", bytes([b0, b1, b2, b3]))[0]
