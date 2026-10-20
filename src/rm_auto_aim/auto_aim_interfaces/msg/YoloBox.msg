# 单个 YOLO 检测框（OBB 旋转框，图像像素坐标，原图坐标系）
string class_name        # 原始类别名，如 "R_1" / "B_G"
string number            # 装甲板编号："1"~"5" / "guard" / "outpost" / "base"
string color             # "RED" / "BLUE"
float32 confidence       # 置信度 [0,1]
geometry_msgs/Point[4] corners  # 4 角点，顺序：左灯下、左灯上、右灯上、右灯下
