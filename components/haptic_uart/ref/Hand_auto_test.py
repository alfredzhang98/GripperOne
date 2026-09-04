import serial
import time
import serial.tools.list_ports
import threading
from typing import Optional, Dict, List, Tuple
from pynput import keyboard
import csv
import os
from datetime import datetime


class HighSpeedCommBoard:
    def __init__(self):
        # 基础属性
        self.ser = None
        self.auto_receive_running = False
        self.calibration_state = "未标定"
        self.distribution_points_cache = {}  # 各模组分布力点数（0x0030-0x0067）
        self.module_config = {}  # 模组组合配置（0x0010-0x0013）
        self.module_config_bits = {}  # 解析后的模组位配置
        self.data_type = 0  # 数据类型（0x0016）
        self.palm_sensor_limit = 9
        self.connected_sensors = []  # 存储实际启用的传感器

        # 传感器地址映射（0x0030-0x0067）
        self.distribution_points_addrs = {
            "大拇指近节": 0x0030, "大拇指中节": 0x0032, "大拇指指尖": 0x0034, "大拇指指甲": 0x0036,
            "食指近节": 0x0038, "食指中节": 0x003A, "食指指尖": 0x003C, "食指指甲": 0x003E,
            "中指近节": 0x0040, "中指中节": 0x0042, "中指指尖": 0x0044, "中指指甲": 0x0046,
            "无名指近节": 0x0048, "无名指中节": 0x004A, "无名指指尖": 0x004C, "无名指指甲": 0x004E,
            "小拇指近节": 0x0050, "小拇指中节": 0x0052, "小拇指指尖": 0x0054, "小拇指指甲": 0x0056,
            "掌心1": 0x0066, "掌心2": 0x0068, "掌心3": 0x006A, "掌心4": 0x006C,
            "掌心5": 0x006E, "掌心6": 0x0070, "掌心7": 0x0072, "掌心8": 0x0074
        }
        
        # 模组组合寄存器命名和地址（0x0010-0x0013）
        self.module_config_addrs = {
            "AUTO_POST_BACK_SENSOR_COMBINATION_0": 0x0010,  # 大拇指+食指
            "AUTO_POST_BACK_SENSOR_COMBINATION_1": 0x0011,  # 中指+无名指
            "AUTO_POST_BACK_SENSOR_COMBINATION_2": 0x0012,  # 小拇指+掌心1-4
            "AUTO_POST_BACK_SENSOR_COMBINATION_3": 0x0013   # 掌心5-8
        }
        
        # 寄存器位定义映射
        self.module_bit_defs = {
            0x0010: {
                "bit0": "大拇指近节", "bit1": "大拇指中节", "bit2": "大拇指指尖", "bit3": "大拇指指甲",
                "bit4": "食指近节", "bit5": "食指中节", "bit6": "食指指尖", "bit7": "食指指甲"
            },
            0x0011: {
                "bit0": "中指近节", "bit1": "中指中节", "bit2": "中指指尖", "bit3": "中指指甲",
                "bit4": "无名指近节", "bit5": "无名指中节", "bit6": "无名指指尖", "bit7": "无名指指甲"
            },
            0x0012: {
                "bit0": "小拇指近节", "bit1": "小拇指中节", "bit2": "小拇指指尖", "bit3": "小拇指指甲",
                "bit4": "掌心1", "bit5": "掌心2", "bit6": "掌心3", "bit7": "掌心4"
            },
            0x0013: {
                "bit0": "掌心5", "bit1": "掌心6", "bit2": "掌心7", "bit3": "掌心8",
                "bit4": None, "bit5": None, "bit6": None, "bit7": None  # 未定义位
            }
        }
        
        # 数据类型寄存器
        self.data_type_addr = 0x0016

    def log(self, message: str):
        """控制台日志输出"""
        print(f"[{time.strftime('%H:%M:%S')}] {message}")

    def scan_available_ports(self) -> List[Dict]:
        """扫描可用串口"""
        available_ports = []
        ports = serial.tools.list_ports.comports()
        
        self.log("===== 扫描可用串口设备 =====")
        if not ports:
            self.log("  未检测到任何可用串口")
            return available_ports
            
        for idx, port in enumerate(ports):
            port_info = {
                "index": idx + 1,
                "port": port.device,
                "description": port.description,
                "hwid": port.hwid
            }
            available_ports.append(port_info)
            self.log(f"  [{port_info['index']}] {port_info['port']} - {port_info['description']}")
        
        return available_ports

    def auto_connect(self, baudrate: int = 921600) -> bool:
        """自动连接串口"""
        ports = self.scan_available_ports()
        if not ports:
            return False
            
        first_port = ports[0]["port"]
        return self.connect(com_port=first_port, baudrate=baudrate)

    def connect(self, com_port: str, baudrate: int = 921600) -> bool:
        """优化串口连接参数，提升稳定性"""
        if self.ser and self.ser.is_open:
            self.log("已连接设备")
            return True
            
        try:
            self.ser = serial.Serial(
                port=com_port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,
                write_timeout=0.01,
                rtscts=False,
                dsrdtr=False
            )
            self.ser.flushInput()
            self.ser.flushOutput()
            
            if self.ser.is_open:
                self.log(f"成功连接到 {com_port}，波特率 {baudrate}")
                # 连接成功后读取配置参数（解析位配置+识别启用传感器）
                self.read_module_config()
                self.parse_module_config_bits()  # 解析寄存器位
                self.get_connected_sensors()     # 识别启用的传感器
                self.read_data_type()
                self.check_distribution_points()
                return True
        except Exception as e:
            self.log(f"连接{com_port}失败: {str(e)}")
        return False

    def disconnect(self):
        """断开连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.log("已断开连接")
        else:
            self.log("未连接设备")

    def calculate_lrc(self, data: bytes) -> int:
        """LRC校验计算：累加所有字节→取反→加1→保留低8位"""
        lrc = 0
        for byte in data:
            lrc = (lrc + byte) & 0xFF
        lrc = ((~lrc) + 1) & 0xFF
        return lrc

    def build_request_frame(self, func_code: int, reg_addr: int, data: bytes = b"") -> bytes:
        """构建请求帧（帧头55AA）"""
        head = b"\x55\xAA"
        reserved = b"\x00"
        reg_addr_bytes = reg_addr.to_bytes(2, byteorder='little', signed=False)
        data_len = len(data) if func_code == 0x10 else 2
        data_len_bytes = data_len.to_bytes(2, byteorder='little', signed=False)
        frame_body = head + reserved + bytes([func_code]) + reg_addr_bytes + data_len_bytes + data
        lrc = self.calculate_lrc(frame_body)
        return frame_body + bytes([lrc])

    def read_register(self, reg_addr: int) -> Optional[int]:
        """通用寄存器读取方法"""
        try:
            request_frame = self.build_request_frame(0x03, reg_addr, b"")
            self.ser.write(request_frame)
            time.sleep(0.005)
            response = self.ser.read(128)
            
            parsed = self._parse_response_frame(response)
            if parsed and parsed["func_code"] == 0x03 and parsed["data_len"] == 2:
                return int.from_bytes(parsed["data"], byteorder='little', signed=False)
            else:
                self.log(f"读取寄存器{reg_addr:04X}失败")
                return None
        except Exception as e:
            self.log(f"读取寄存器{reg_addr:04X}错误: {str(e)}")
            return None

    def read_module_config(self):
        """读取模组组合配置（0x0010-0x0013）"""
        self.log("===== 读取模组组合配置 =====")
        for config_name, reg_addr in self.module_config_addrs.items():
            value = self.read_register(reg_addr)
            self.module_config[config_name] = value if value is not None else 0
            self.log(f"  {config_name}(0x{reg_addr:04X}): 0x{self.module_config[config_name]:02X} ({self.module_config[config_name]})")

    def parse_module_config_bits(self):
        """解析模组配置寄存器的bit位的位定义"""
        self.log("\n===== 解析模组配置位定义 =====")
        self.module_config_bits.clear()
        
        for config_name, reg_addr in self.module_config_addrs.items():
            reg_value = self.module_config.get(config_name, 0)
            bit_info = {}
            
            # 解析当前寄存器的每一位
            for bit in range(8):
                bit_value = (reg_value >> bit) & 0x01
                sensor_name = self.module_bit_defs[reg_addr].get(f"bit{bit}")
                
                if sensor_name:
                    bit_info[f"bit{bit}"] = {
                        "sensor": sensor_name,
                        "enabled": bit_value == 1
                    }
                    self.log(f"  {config_name} bit{bit}: {sensor_name} - {'启用' if bit_value == 1 else '禁用'}")
                else:
                    bit_info[f"bit{bit}"] = {"sensor": None, "enabled": False}
            
            self.module_config_bits[config_name] = bit_info

    def get_connected_sensors(self):
        """根据位解析结果，获取实际启用的传感器列表"""
        self.connected_sensors.clear()
        self.log("\n===== 识别启用的传感器 =====")
        
        for config_bits in self.module_config_bits.values():
            for bit_info in config_bits.values():
                sensor = bit_info.get("sensor")
                enabled = bit_info.get("enabled")
                
                if sensor and enabled and sensor not in self.connected_sensors:
                    self.connected_sensors.append(sensor)
        
        if self.connected_sensors:
            self.log(f"  已启用传感器: {', '.join(self.connected_sensors)}")
        else:
            self.log("  未检测到任何启用的传感器")

    def read_data_type(self):
        """读取数据类型配置（0x0016）"""
        self.log("\n===== 读取数据类型配置 =====")
        value = self.read_register(self.data_type_addr)
        self.data_type = value if value is not None else 0
        self.log(f"  数据类型(0x{self.data_type_addr:04X}): {self.data_type}")

    def get_version(self) -> Optional[str]:
        """获取版本号"""
        if not self.ser or not self.ser.is_open:
            self.log("请先连接设备")
            return None
            
        try:
            version_frame = bytes.fromhex("55AA000300000F00EF")
            self.ser.write(version_frame)
            self.log(f"发送版本号请求: {version_frame.hex().upper()}")
            
            time.sleep(0.1)
            response = self.ser.read(128)
            
            if not response:
                self.log("未收到版本号响应")
                return None
                
            self.log(f"收到版本号响应: {response.hex().upper()}")
            
            parsed = self._parse_response_frame(response)
            if parsed and parsed["func_code"] == 0x03:
                version = self._parse_version_data(parsed["data"])
                self.log(f"设备版本号: {version}")
                time.sleep(2)
                return version
            else:
                self.log("版本号响应解析失败")
                return None
            
        except Exception as e:
            self.log(f"获取版本号错误: {str(e)}")
            return None

    def _parse_version_data(self, data: bytes) -> str:
        """解析版本数据"""
        try:
            return data.decode('ascii').strip()
        except UnicodeDecodeError:
            return f"无法解析的版本数据: {data.hex().upper()}"

    def _parse_response_frame(self, frame: bytes) -> Optional[Dict]:
        """解析响应帧（帧头AA55）"""
        if len(frame) < 8 or frame[:2] != b"\xaa\x55":
            self.log(f"帧头错误：长度{len(frame)}字节，前2字节{frame[:2].hex() if len(frame)>=2 else '空'}")
            return None
        
        frame_body = frame[:-1]
        lrc = frame[-1]
        calc_lrc = self.calculate_lrc(frame_body)
        
        if calc_lrc != lrc:
            self.log(f"LRC校验失败 - 帧体: {frame_body.hex()}, 计算值={calc_lrc:02X}, 实际值={lrc:02X}")
            return None
        
        return {
            "reserved": frame[2],
            "func_code": frame[3],
            "reg_addr": int.from_bytes(frame[4:6], byteorder='little'),
            "data_len": int.from_bytes(frame[6:8], byteorder='little'),
            "data": frame[8:-1]
        }

    def check_distribution_points(self):
        """仅读取启用传感器的分布力点数"""
        self.log("\n===== 预读取传感器分布力点数 =====")
        
        # 只处理启用的传感器，提升效率
        target_sensors = self.connected_sensors if self.connected_sensors else self.distribution_points_addrs.keys()
        
        for sensor_name in target_sensors:
            addr = self.distribution_points_addrs.get(sensor_name)
            if not addr:
                self.log(f"  {sensor_name}: 无对应地址，跳过")
                continue
                
            try:
                request_frame = self.build_request_frame(0x03, addr, b"")
                self.ser.write(request_frame)
                time.sleep(0.005)
                response = self.ser.read(128)
                
                parsed = self._parse_response_frame(response)
                if parsed and parsed["func_code"] == 0x03 and parsed["data_len"] == 2:
                    point_count = int.from_bytes(parsed["data"], byteorder='little', signed=False)
                    self.distribution_points_cache[sensor_name] = point_count
                    self.log(f"  {sensor_name}(0x{addr:04X}): {point_count}")
                else:
                    self.distribution_points_cache[sensor_name] = 0
                    
            except Exception as e:
                self.log(f"  {sensor_name} 点数读取错误: {str(e)}")
                self.distribution_points_cache[sensor_name] = 0

    def calculate_effective_data_length(self) -> int:
        """
        基于实际启用的传感器计算有效数据长度
        有效数据长度 = 固定头部分(6字节) + 各启用模组的数据长度
        固定头部分：帧头(2) + 预留(1) + 长度(2) + 错误码(1) = 6字节
        模组数据长度：根据启用传感器、数据类型、各模组点数计算
        """
        effective_len = 6  # 固定头部分
        
        # 仅遍历启用的传感器，避免无效计算
        for sensor_name in self.connected_sensors:
            point_count = self.distribution_points_cache.get(sensor_name, 0)
            if point_count > 0:
                effective_len += 6  # 合力6字节
                # 掌心传感器限制9个点，其他无限制
                if sensor_name.startswith("掌心"):
                    effective_len += min(point_count, self.palm_sensor_limit) * 3
                else:
                    effective_len += point_count * 3
        
        self.log(f"动态计算有效数据长度: {effective_len} 字节（基于{len(self.connected_sensors)}个启用传感器）")
        return effective_len

    def calibrate(self, calib_cmd_hex: str = "55AA00170200010001E6") -> bool:
        """标定"""
        if not self.ser or not self.ser.is_open:
            self.log("请先连接设备")
            return False
        
        self.calibration_state = "标定中"
        self.log("\n===== 开始标定流程 =====")
        
        if not calib_cmd_hex:
            self.log("标定命令帧不能为空")
            self.calibration_state = "失败"
            return False
        
        try:
            calib_frame = bytes.fromhex(calib_cmd_hex)
            self.ser.write(calib_frame)
            self.log(f"发送标定命令: {calib_cmd_hex}")
            
            time.sleep(1)
            response = self.ser.read(256)
            if not response:
                self.log("未收到标定响应，可能超时")
                self.calibration_state = "失败"
                return False
            
            parsed = self._parse_response_frame(response)
            if parsed and parsed["data"] == b"\x00":
                self.log("标定成功！")
                self.calibration_state = "成功"
                return True
            else:
                self.log(f"标定失败，响应码: {parsed['data'].hex() if parsed else response.hex()}")
                self.calibration_state = "失败"
                return False
                
        except Exception as e:
            self.log(f"标定操作错误: {str(e)}")
            self.calibration_state = "失败"
            return False

    def start_auto_receive(self):
        """开启自动回传"""
        if self.auto_receive_running:
            self.log("自动回传已开启")
            return
        
        if not self.ser or not self.ser.is_open:
            self.log("请先连接设备")
            return
        
        self._write_register(0x0017, b"\x01")
        self.auto_receive_running = True
        self.log("开始自动回传接收...")
        # 使用线程运行回传循环，避免阻塞主线程
        receive_thread = threading.Thread(target=self._auto_receive_loop, daemon=True)
        receive_thread.start()

    def stop_auto_receive(self):
        """停止自动回传"""
        if not self.auto_receive_running:
            self.log("自动回传未开启，无需停止")
            return
        
        self.auto_receive_running = False
        if self.ser and self.ser.is_open:
            self.ser.flushInput()
            self.ser.flushOutput()
            self._write_register(0x0017, b"\x00")
            self.log("已发送停止自动回传指令，清空串口缓冲区")
        self.log("自动回传已停止")

    def _write_register(self, reg_addr: int, data: bytes):
        """写寄存器"""
        try:
            request_frame = self.build_request_frame(0x10, reg_addr, data)
            self.ser.write(request_frame)
            time.sleep(0.005)
            response = self.ser.read(16)
            parsed = self._parse_response_frame(response)
            if parsed and parsed["func_code"] == 0x10:
                self.log(f"写寄存器{reg_addr:04X}成功")
            else:
                self.log(f"写寄存器{reg_addr:04X}失败")
        except Exception as e:
            self.log(f"写寄存器错误: {str(e)}")

    def _auto_receive_loop(self):
        """自动回传数据解析（帧头AA56，动态计算有效帧长度）- 线程运行"""
        self.log("自动回传解析线程已启动")
        while self.auto_receive_running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    response = self.ser.read(self.ser.in_waiting)
                    self.log(f"\n收到自动回传原始数据: 长度={len(response)}字节 | 十六进制: {response.hex()[:250]}...")
                    
                    remaining_data = response
                    while len(remaining_data) >= 8 and self.auto_receive_running:
                        # 1. 查找帧头AA56（严格匹配协议）
                        frame_start = remaining_data.find(b"\xaa\x56")
                        if frame_start == -1:
                            self.log(f"未找到帧头AA56，剩余数据长度: {len(remaining_data)}字节")
                            break
                        
                        # 2. 截取帧头后的数据
                        frame_data = remaining_data[frame_start:]
                        
                        # 3. 动态计算有效数据长度和有效帧长度
                        effective_data_len = self.calculate_effective_data_length()
                        valid_frame_len = effective_data_len + 1  # 有效帧长度=有效数据长度+1（LRC）
                        
                        self.log(f"解析帧结构: 有效数据长度={effective_data_len} | 有效帧长度={valid_frame_len} | 当前可用长度={len(frame_data)}")
                        
                        # 4. 检查帧长度是否足够
                        if len(frame_data) < valid_frame_len:
                            self.log(f"帧不完整，等待后续数据（需{valid_frame_len}字节，仅{len(frame_data)}字节）")
                            break
                        
                        # 5. 提取完整帧
                        full_frame = frame_data[:valid_frame_len]
                        
                        # 6. LRC校验（校验前面所有字节）
                        lrc = full_frame[-1]  # 最后1字节是LRC
                        check_data = full_frame[:-1]  # 校验范围：除LRC外的所有字节
                        calc_lrc = self.calculate_lrc(check_data)
                        
                        self.log(f"LRC校验: 计算值={calc_lrc:02X} | 帧中值={lrc:02X}")
                        
                        if calc_lrc != lrc:
                            self.log(f"LRC校验失败，跳过该帧 | 计算值={calc_lrc:02X} vs 实际值={lrc:02X}")
                            remaining_data = frame_data[valid_frame_len:]
                            continue
                        
                        # 7. 解析有效数据段（跳过固定头6字节）
                        self.log(f"解析到有效自动回传帧: 长度={valid_frame_len}字节（LRC校验通过）")
                        valid_data_segment = check_data[6:]  # 跳过帧头(2)+预留(1)+长度(2)+错误码(1)=6字节
                        self._parse_auto_receive_force_data(valid_data_segment)
                        
                        # 8. 处理剩余数据
                        remaining_data = frame_data[valid_frame_len:]
            except Exception as e:
                self.log(f"自动回传解析错误: {str(e)} | 异常类型: {type(e).__name__}")
            time.sleep(0.015)
        self.log("自动回传解析线程已退出")

    def _parse_auto_receive_frame(self, frame: bytes) -> Optional[Dict]:
        """解析自动回传帧（帧头校验）"""
        if len(frame) < 7 or frame[:2] != b"\xaa\x56":
            self.log(f"自动回传帧头错误: 前2字节={frame[:2].hex() if len(frame)>=2 else '空'}（预期AA56）")
            return None
        
        data_segment_len = int.from_bytes(frame[3:5], byteorder='little')
        return {
            "reserved": frame[2],
            "data_segment_len": data_segment_len,
            "error_code": frame[5],
            "data": frame[6:-1]
        }

    def _init_dynamic_csv(self, sensor_info, file_path="force_sensor_dynamic_data.csv"):
        """动态生成CSV表头：根据启用的传感器和点数自动创建"""
        # 默认保存到桌面（可自定义路径）
        desktop_path = os.path.join(os.path.expanduser("~"), "Desktop")
        full_path = os.path.join(desktop_path, file_path)
        self.log(f"动态传感器数据将保存到：{full_path}")
    
        # 仅在文件不存在时生成表头
        if not os.path.exists(full_path):
            header = ["时间戳(毫秒)"]  # 基础列
        
            # 遍历所有启用的传感器，动态生成表头
            for sensor_name, point_count in sensor_info.items():
                # 合力列
                header.extend([
                    f"{sensor_name}-Fx(N)",
                    f"{sensor_name}-Fy(N)",
                    f"{sensor_name}-Fz(N)"
                ])
                # 分布力列（按实际点数生成）
                for i in range(1, point_count + 1):
                    header.extend([
                        f"{sensor_name}-分布力-{i}点-X(N)",
                        f"{sensor_name}-分布力-{i}点-Y(N)",
                        f"{sensor_name}-分布力-{i}点-Z(N)"
                    ])
        
            # 写入动态表头
            with open(full_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(header)
    
        return full_path

    def _save_dynamic_force_data(self, sensor_info, sensor_data, file_path="force_sensor_dynamic_data.csv"):
        """动态存储数据：按启用传感器和实际点数匹配存储"""
        full_path = self._init_dynamic_csv(sensor_info, file_path)
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    
        row = [timestamp]  # 首列：时间戳
    
        # 遍历所有启用的传感器，按表头顺序填充数据
        for sensor_name, point_count in sensor_info.items():
            # 填充合力数据（无数据填0）
            force_data = sensor_data.get(sensor_name, {}).get("force", {})
            row.extend([
                force_data.get("Fx", 0.0),
                force_data.get("Fy", 0.0),
                force_data.get("Fz", 0.0)
            ])
        
            # 填充分布力数据（不足点数补0）
            dist_points = sensor_data.get(sensor_name, {}).get("dist", [])
            for i in range(point_count):
                if i < len(dist_points):
                    x, y, z = dist_points[i]
                else:
                    x, y, z = 0.0, 0.0, 0.0
                row.extend([x, y, z])
    
        # 追加写入CSV
        with open(full_path, "a+", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(row)
        self.log(f"动态数据已保存 | 时间戳：{timestamp} | 启用传感器：{list(sensor_info.keys())}")


    def _parse_auto_receive_force_data(self, data: bytes):
        """按实际启用传感器顺序解析"""
        current_offset = 0

        # 存储启用传感器的关键信息：{传感器名: 点数}
        enabled_sensor_info = {}
        # 存储解析后的数据：{传感器名: {"force": 合力字典, "dist": 分布力点列表}}
        parsed_sensor_data = {}

        self.log("\n===== 自动回传传感器数据解析（启用传感器顺序） =====")

        # 严格按照硬件回传的真实顺序：
        parse_sensor_list = [
            "大拇指近节", "大拇指中节", "大拇指指尖", "大拇指指甲",
            "食指近节", "食指中节", "食指指尖", "食指指甲",
            "中指近节", "中指中节", "中指指尖", "中指指甲",
            "无名指近节", "无名指中节", "无名指指尖", "无名指指甲",
            "小拇指近节", "小拇指中节", "小拇指指尖", "小拇指指甲",
                "掌心1", "掌心2", "掌心3", "掌心4",
            "掌心5", "掌心6", "掌心7", "掌心8"
        ]

        # 收集启用传感器的信息（名称+点数）
        for sensor_name in parse_sensor_list:
            if sensor_name not in self.connected_sensors:
                continue
            point_count = self.distribution_points_cache.get(sensor_name, 0)
            if point_count > 0:
                enabled_sensor_info[sensor_name] = point_count

        # 解析每个启用传感器的合力和分布力
        for sensor_name in parse_sensor_list:
            if sensor_name not in enabled_sensor_info:
                continue

            point_count = enabled_sensor_info[sensor_name]

            # 解析合力
            if current_offset + 6 > len(data):
                self.log(f"  {sensor_name}: 合力数据不足，终止解析")
                return
            force_data = data[current_offset:current_offset+6]
            current_offset += 6
            # 存储合力数据
            force_result = self._parse_single_sensor_total_force(force_data, sensor_name)
            parsed_sensor_data[sensor_name] = {"force": force_result, "dist": []}

            # 解析分布力（动态点数）
            if sensor_name.startswith("掌心"):
                actual_points = min(point_count, self.palm_sensor_limit)
            else:
                actual_points = point_count

            dist_len = actual_points * 3
            if current_offset + dist_len > len(data):
                self.log(f"  {sensor_name}: 分布力数据不足，终止解析")
                return

            distribution_data = data[current_offset:current_offset+dist_len]
            current_offset += dist_len
            # 存储所有分布力点
            dist_points = self._parse_all_distribution_points(distribution_data)
            parsed_sensor_data[sensor_name]["dist"] = dist_points

        self.log(f"解析完成，总偏移：{current_offset}/{len(data)} 字节")
        # 动态保存数据（根据启用传感器自动适配）
        self._save_dynamic_force_data(enabled_sensor_info, parsed_sensor_data)

    def _parse_single_sensor_total_force(self, force_data: bytes, sensor_name: str):
        """解析合力        
        合力：6字节 小端
        Fx(2) Fy(2) Fz(2)
        只解析 低字节 算力值
        Fx/Fy：有符号
        Fz：无符号
        """
        # 只取低字节
        fx_byte = force_data[0]
        fy_byte = force_data[2]
        fz_byte = force_data[4]

        fx = (int.from_bytes(bytes([fx_byte]), byteorder='little', signed=True)) / 10.0
        fy = (int.from_bytes(bytes([fy_byte]), byteorder='little', signed=True)) / 10.0
        fz = (int.from_bytes(bytes([fz_byte]), byteorder='little', signed=False)) / 10.0
    
        self.log(f"  {sensor_name}合力: Fx= {fx:4.1f}N, Fy= {fy:4.1f}N, Fz= {fz:4.1f}N")
        return {"Fx": round(fx, 1), "Fy": round(fy, 1), "Fz": round(fz, 1)}

    def _parse_all_distribution_points(self, dist_data: bytes):
        """解析所有分布力点（X/Y有符号，Z无符号），返回点列表"""
        dist_points = []
        for i in range(0, len(dist_data), 3):
            # X/Y：1字节有符号，Z：1字节无符号，均÷10转换为牛顿
            x = (int.from_bytes(dist_data[i:i+1], byteorder='little', signed=True)) / 10.0
            y = (int.from_bytes(dist_data[i+1:i+2], byteorder='little', signed=True)) / 10.0
            z = (int.from_bytes(dist_data[i+2:i+3], byteorder='little', signed=False)) / 10.0
        
            dist_points.append((round(x, 1), round(y, 1), round(z, 1)))
        return dist_points

# ===================== 按键控制核心逻辑 =====================
class KeyController:
    def __init__(self, board: HighSpeedCommBoard, exit_flag: threading.Event):
        self.board = board
        self.exit_flag = exit_flag
        self.listener = None
        
    def on_press(self, key):
        """按键按下事件处理"""
        try:
            if key.char == 's' or key.char == 'S':
                print("\n>>> 检测到[S]键按下，停止自动回传 <<<")
                self.board.stop_auto_receive()
            elif key.char == 'r' or key.char == 'R':
                print("\n>>> 检测到[R]键按下，重新开启自动回传 <<<")
                self.board.start_auto_receive()
            elif key.char == 'e' or key.char == 'E':
                print("\n>>> 检测到[E]键按下，退出程序 <<<")
                self.board.stop_auto_receive()
                time.sleep(2)
                self.board.disconnect()
                self.exit_flag.set()
                if self.listener:
                    self.listener.stop()
        except AttributeError:
            pass
        
    def start_listen(self):
        """启动按键监听"""
        print("\n===== 按键控制指令 =====")
        print("按 [S] 键   - 停止自动回传")
        print("按 [R] 键   - 重新开启自动回传")
        print("按 [E] 键   - 停止回传并退出程序")
        print("========================\n")
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.daemon = True
        self.listener.start()

# ===================== 测试示例 =====================
if __name__ == "__main__":
    try:
        from pynput import keyboard
    except ImportError:
        print("请先安装按键监听库：pip install pynput")
        exit(1)
    
    board = HighSpeedCommBoard()
    exit_flag = threading.Event()
    
    # 扫描并连接串口
    available_ports = board.scan_available_ports()
    if not available_ports:
        print("未检测到任何可用串口，程序退出")
        exit(1)
    
    print("\n请选择要连接的串口编号:")
    for port in available_ports:
        print(f"  [{port['index']}] {port['port']} - {port['description']}")
    
    try:
        selected_idx = int(input("\n输入编号: ")) - 1
        if selected_idx < 0 or selected_idx >= len(available_ports):
            print("无效编号")
            exit(1)
        
        selected_port = available_ports[selected_idx]["port"]
        connect_success = board.connect(com_port=selected_port, baudrate=921600)
        
    except ValueError:
        print("输入无效，请输入数字")
        exit(1)
    
    if not connect_success:
        print("串口连接失败，程序退出")
        exit(1)
    
    # 初始化配置
    time.sleep(2)  # 缩短不必要的等待时间
    board.get_version()
    time.sleep(1)
    board.calibrate()
    time.sleep(1)
    
    # 启动按键控制
    key_controller = KeyController(board, exit_flag)
    key_controller.start_listen()
    
    # 开启自动回传
    board.start_auto_receive()
    
    # 主循环
    try:
        while not exit_flag.is_set():
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n>>> 检测到Ctrl+C，执行退出流程 <<<")
        board.stop_auto_receive()
        time.sleep(1)
        board.disconnect()
        exit_flag.set()
        print("程序已退出")





