import ctypes
class Test(ctypes.Structure):
    _pack_ = 1
    _fields_ = [('id', ctypes.c_uint16), ('data', ctypes.c_uint8 * 8)]
print(ctypes.sizeof(Test))
