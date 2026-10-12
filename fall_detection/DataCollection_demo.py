#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import pdb
import sys
import time
import queue
import clr
import importlib
import threading
from System import Array, Byte


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DLL_FILE = os.path.join(BASE_DIR, "HifMsgDataCollectionLib.dll")

# 设备配置：默认使用 UART
USE_UART = False
SPI_INDEX = 0
SPI_FREQ_MODE = 0
UART_COM = "COM12"
UART_BAUD_RATE = "1000000"

CONFIG_FILE = "test.csv"

ERROR_CODE = {
    -8: "数据或文件为空",
    -7: "配置失败",
    -6: "启动雷达失败",
    -5: "停止雷达失败",
    -4: "唤醒失败",
    -3: "ack超时",
    -2: "ack错误(checksum)",
    -1: "未停止采数",
    0: "成功",
    1: "命令不支持",
    5: "参数非法",
    9: "雷达设备忙",
}


def print_result(name, result):
    """打印接口返回值。"""
    if isinstance(result, bool):
        print(f"{name}: {'成功' if result else '失败'}")
        return result

    code = int(result)
    print(f"{name}: {code}({ERROR_CODE.get(code, '未知错误')})")
    return result


class DllLoader:
    """加载 DLL 并获取命名空间。"""

    def __init__(self, dll_path: str):
        if not os.path.exists(dll_path):
            raise FileNotFoundError(f"DLL 文件不存在: {dll_path}")

        self.dll_path = os.path.abspath(dll_path)
        dll_dir = os.path.dirname(self.dll_path)
        dll_name = os.path.splitext(os.path.basename(self.dll_path))[0]

        if dll_dir not in sys.path:
            sys.path.append(dll_dir)

        clr.AddReference(dll_name)
        self.namespace = importlib.import_module(dll_name)


class DataCollection:
    """数据采集和接口封装。"""

    DATA_TYPE_NAME = {
        0: "psic_debug",
        1: "字符串",
        2: "datacube_0xC1",
        3: "datacube_0xC2",
        4: "point_cloud",
    }

    def __init__(self, data_collection_api):
        self.api = data_collection_api
        self.api.callback += self.data_callback
        self.data_queue = queue.Queue(maxsize=1000)
        self.running = True

        # DataCube 解析参数，按实际雷达配置修改
        self.rangeFftPointNum = 256
        self.dopFftPointNum = 32
        self.txAntNum = 2
        self.rxAntNum = 4

        # 后台线程处理回调数据
        self.worker = threading.Thread(target=self.process_data, daemon=True)
        self.worker.start()

    def data_callback(self, sender, receive_data):
        """接收回调中只入队，避免阻塞 DLL。"""
        try:
            self.data_queue.put_nowait(receive_data)
        except queue.Full:
            print("队列已满，丢弃数据")

    def process_data(self):
        """后台解析数据。"""
        while self.running:
            try:
                data = self.data_queue.get(timeout=1)
                self.parse_data(data)
                self.data_queue.task_done()
            except queue.Empty:
                continue

    def _has_api_method(self, name):
        """判断 DLL 是否支持某个接口。"""
        return hasattr(self.api, name)

    def _get_payload_bytes(self, receive_data):
        """优先取新接口 payloadData。"""
        if hasattr(receive_data, "payloadData") and receive_data.payloadData is not None:
            return receive_data.payloadData
        return receive_data.pointCloudRawDataOrStringData

    def _decode_text(self, raw_bytes):
        """把字节流转成字符串预览。"""
        if raw_bytes is None:
            return ""

        data = bytes(raw_bytes)
        for encoding in ("utf-8", "gbk", "ascii"):
            try:
                return data.decode(encoding).strip("\x00\r\n ")
            except UnicodeDecodeError:
                continue
        return data.decode("utf-8", errors="ignore").strip("\x00\r\n ")

    def _preview_text(self, text, max_len=80):
        """限制字符串打印长度。"""
        if len(text) <= max_len:
            return text
        return text[:max_len] + "..."

    def _format_first_point(self, point_cloud):
        """简略打印第一个点。"""
        values = []
        for field_name in ("x", "y", "z", "w", "u", "v"):
            field_value = getattr(point_cloud, field_name, None)
            if field_value is not None and len(field_value) > 0:
                values.append(f"{field_name}={field_value[0]:.3f}")

        if not values:
            return "无坐标"

        return "首点 " + ", ".join(values)

    def _print_point_cloud_summary(self, point_cloud):
        """简略打印点云摘要。"""
        if point_cloud is None:
            print("解析结果为空")
            return

        print(
            f"signal={point_cloud.signalName}, dim={int(point_cloud.dim)}, "
            f"pointNum={int(point_cloud.pointNum)}, {self._format_first_point(point_cloud)}"
        )

    def _print_datacube_summary(self, data_cube):
        """简略打印 datacube 摘要。"""
        if data_cube is None:
            print("DataCube 解析失败")
            return

        antenna_num = len(data_cube.antennaData) if data_cube.antennaData is not None else 0
        print(
            f"frame={int(data_cube.frameIndex)}, len={int(data_cube.frameLen)}, "
            f"rangeFft={int(data_cube.rangeFftPointNum)}, dopFft={int(data_cube.dopFftPointNum)}, "
            f"antennas={antenna_num}"
        )

        if antenna_num > 0:
            antenna = data_cube.antennaData[0]
            real_len = len(antenna.real) if antenna.real is not None else 0
            imag_len = len(antenna.imag) if antenna.imag is not None else 0
            real_preview = antenna.real[0] if real_len > 0 else "NA"
            imag_preview = antenna.imag[0] if imag_len > 0 else "NA"
            print(
                f"首天线 tx={antenna.txAntId}, rx={antenna.rxAntId}, "
                f"realLen={real_len}, imagLen={imag_len}, "
                f"real0={real_preview}, imag0={imag_preview}"
            )

    def _parse_psic_debug(self, receive_data):
        """解析 debug 点数据。"""
        payload = self._get_payload_bytes(receive_data)
        if payload is None:
            print("psic_debug 数据为空")
            return

        if self._has_api_method("PsicDebugDataConversion"):
            point_cloud = self.api.PsicDebugDataConversion(payload)
        else:
            point_cloud = self.api.PointCloudConversion(payload)

        self._print_point_cloud_summary(point_cloud)

    def _parse_string_data(self, receive_data):
        """解析字符串数据。"""
        payload = self._get_payload_bytes(receive_data)
        text = self._decode_text(payload)
        print(f"长度={len(text)}, 内容={self._preview_text(text)}")

    def _parse_datacube_c1(self, receive_data):
        """解析 0xC1 datacube。"""
        data_cube = self.api.DatacubeConversion(
            receive_data.datacubeRawData,
            receive_data.frameIndex,
            receive_data.frameLen,
            self.rangeFftPointNum,
            self.dopFftPointNum,
            self.txAntNum,
            self.rxAntNum,
        )
        self._print_datacube_summary(data_cube)

    def _parse_datacube_c2(self, receive_data):
        """解析 0xC2 datacube。"""
        payload = self._get_payload_bytes(receive_data)
        if payload is None:
            print("datacube_0xC2 数据为空")
            return

        if self._has_api_method("DatacubeConversion"):
            try:
                data_cube = self.api.DatacubeConversion(
                    payload,
                    receive_data.frameIndex,
                    receive_data.frameLen,
                )
                self._print_datacube_summary(data_cube)
                return
            except TypeError:
                pass

        print(f"当前 DLL 不支持 0xC2 解析，rawLen={len(payload)}")

    def _parse_point_cloud(self, receive_data):
        """解析新增点云数据。"""
        payload = self._get_payload_bytes(receive_data)
        if payload is None:
            print("point_cloud 数据为空")
            return

        if not self._has_api_method("ConvertPointCloudData"):
            print(f"当前 DLL 不支持 point_cloud 解析，rawLen={len(payload)}")
            return

        point_cloud_list = self.api.ConvertPointCloudData(
            payload,
            receive_data.frameIndex,
            receive_data.frameLen,
        )
        if point_cloud_list is None:
            print("point_cloud 解析结果为空")
            return

        print(f"pointCloudCount={len(point_cloud_list)}")
        for point_cloud in point_cloud_list:
            self._print_point_cloud_summary(point_cloud)

    def parse_data(self, receive_data):
        """按数据类型解析一帧数据。"""
        print("*" * 20)
        data_type = int(receive_data.dataType)
        type_name = self.DATA_TYPE_NAME.get(data_type, f"未知类型({data_type})")
        print(f"时间戳：{receive_data.timeStamp}")
        print(f"数据类型：{type_name}")

        if data_type == 0:
            self._parse_psic_debug(receive_data)
        elif data_type == 1:
            self._parse_string_data(receive_data)
        elif data_type == 2:
            self._parse_datacube_c1(receive_data)
        elif data_type == 3:
            self._parse_datacube_c2(receive_data)
        elif data_type == 4:
            self._parse_point_cloud(receive_data)
        else:
            payload = self._get_payload_bytes(receive_data)
            raw_len = len(payload) if payload is not None else 0
            print(f"未处理的数据类型，rawLen={raw_len}")

    def GetLibVersion(self):
        version = self.api.GetLibVersion()
        print(f"库版本：{version}")
        return version

    def OpenSpiDevice(self, index, freqMode=0):
        return print_result("OpenSpiDevice", self.api.OpenSpiDevice(index, freqMode))

    def CloseSpiDevice(self, index):
        return print_result("CloseSpiDevice", self.api.CloseSpiDevice(index))

    def OpenUartDevice(self, com, baudRate):
        return print_result("OpenUartDevice", self.api.OpenUartDevice(com, baudRate))

    def CloseUartDevice(self):
        return print_result("CloseUartDevice", self.api.CloseUartDevice())

    def StartCollectingData(self):
        return print_result("StartCollectingData", self.api.StartCollectingData())

    def StopCollectingData(self):
        return print_result("StopCollectingData", self.api.StopCollectingData())

    def WakeupDevice(self):
        return print_result("WakeupDevice", self.api.WakeupDevice())

    def StopRadar(self):
        return print_result("StopRadar", self.api.StopRadar())

    def StartRadar(self):
        return print_result("StartRadar", self.api.StartRadar())

    def RadarEnterSleep(self):
        return print_result("RadarEnterSleep", self.api.RadarEnterSleep())

    def SendData(self, data):
        data_bytes = Array[Byte](
            [int(x, 16) for x in data.split()]
        )
        return print_result("SendData", self.api.SendData(data_bytes))

    def SendConfigFile(self, config_file):
        return print_result("SendConfigFile", self.api.SendConfigFile(config_file))

    def Close(self):
        """退出脚本时关闭后台线程。"""
        self.running = False
        if self.worker.is_alive():
            self.worker.join(timeout=2)
        self.api.callback -= self.data_callback


def open_device(data_collection):
    """打开默认设备。"""
    if USE_UART:
        return data_collection.OpenUartDevice(UART_COM, UART_BAUD_RATE)
    return data_collection.OpenSpiDevice(SPI_INDEX, SPI_FREQ_MODE)


def close_device(data_collection):
    """关闭默认设备。"""
    if USE_UART:
        return data_collection.CloseUartDevice()
    return data_collection.CloseSpiDevice(SPI_INDEX)


def send_config_file(data_collection):
    """按配置文件一键下发。"""
    if not CONFIG_FILE:
        return
    data_collection.StopCollectingData()
    status = data_collection.SendConfigFile(CONFIG_FILE)
    return status

def send_data(data_collection, data):
    """ 自定义发送数据。"""
    data_collection.StopCollectingData()
    data_collection.WakeupDevice()
    data_collection.StopRadar()
    status = data_collection.SendData(data)
    data_collection.StartRadar()
    return status


if __name__ == "__main__":
    data_collection = None
    try:
        # DLL加载
        DllLoader(DLL_FILE)
        from HifMsgDataCollectionLib import HifMsgDataCollectionApi

        # DataCollection类初始化
        api = HifMsgDataCollectionApi()
        data_collection = DataCollection(api)
        data_collection.GetLibVersion()

        # 打开设备
        if not open_device(data_collection):
            sys.exit(1)

        # 获取版本号
        send_data(data_collection, "A5 1D 15 07 01 20 00 EA F8 FE DF")
        # 发送配置
        send_config_file(data_collection)
        # 开始采数
        data_collection.StartCollectingData()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("用户停止程序")
    except Exception as e:
        print(f"程序运行出错: {str(e)}")
    finally:
        if data_collection is not None:
            data_collection.StopCollectingData()
            close_device(data_collection)
            data_collection.Close()
