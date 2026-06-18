import ctypes
class Test(ctypes.Structure):
    _pack_ = 1
    _fields_ = [('motor_num', ctypes.c_uint8), ('can_ide', ctypes.c_uint8), ('motor_data', ctypes.c_uint8 * 60)]
print(ctypes.sizeof(Test))
