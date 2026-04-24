"""
网站工具函数
"""
#import ObjManager_RangeParam
#import ObjManager_Button
#import ObjManager_CheckBox
import Message_cvMat
import cv2
import time


def get_range_params_info():
    mutparams = []
    for name in ObjManager_RangeParam.names():
        mp = ObjManager_RangeParam.find(name)
        if mp is not None:
            mutparams.append((name, mp))
    return mutparams


def get_range_param(name):
    return ObjManager_RangeParam.find(name)


def get_buttons_info():
    buttons = []
    for name in ObjManager_Button.names():
        b = ObjManager_Button.find(name)
        if b is not None:
            buttons.append((name, b))
    return buttons


def get_button(name):
    return ObjManager_Button.find(name)


def get_checkboxes_info():
    checkbox = []
    for name in ObjManager_CheckBox.names():
        b = ObjManager_CheckBox.find(name)
        if b is not None:
            checkbox.append((name, b))
    return checkbox


def get_checkbox(name):
    return ObjManager_CheckBox.find(name)


def get_cvmat_names():
    return Message_cvMat.names()


def get_cvmat_jpegcode(name, print_fps=True):
    """
    获取视频流的JPEG编码，可选在控制台打印帧率
    
    参数:
        name: 视频流名称
        print_fps: 是否在控制台打印帧率，默认为True
    
    返回:
        生成器，每次生成一帧JPEG编码的图像
    """
    wi = Message_cvMat.Subscriber(name, 1)
    if wi is not None:
        # 用于计算FPS的变量
        frame_count = 0
        start_time = time.time()
        fps = 0
        last_fps_print = time.time()
        
        while True:
            # 获取原始图像并编码
            frame = wi.pop_for(2000).get_nparray()
            jpeg_code = cv2.imencode(".jpeg", frame)[1].tobytes()
            
            # # 计算FPS
            # frame_count += 1
            # current_time = time.time()
            # elapsed_time = current_time - start_time
            
            # # 每秒更新一次FPS计数
            # if elapsed_time > 1.0:
            #     fps = frame_count / elapsed_time
            #     frame_count = 0
            #     start_time = current_time
                
            #     # 打印FPS，但限制打印频率（例如每5秒打印一次）
            #     if print_fps and (current_time - last_fps_print) >= 5.0:
            #         print(f"Stream '{name}' FPS: {fps:.2f}")
            #         last_fps_print = current_time
            
            # 生成帧数据
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg_code + b'\r\n\r\n'