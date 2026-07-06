import serial
import time
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from typing import Optional, Dict, List, Tuple

class HighSpeedCommBoard:
    def __init__(self, root):
        self.root = root
        self.root.title("高速通信集成板数据采集系统")
        self.root.geometry("1300x900")
        
        self.ser = None
        self.auto_receive_running = False
        self.cycle_read_running = False
        self.module_cycle_read_running = False
        self.calibration_state = "未标定"
        self.calib_state_label = None
        self.distribution_points_cache = {}  # 存储传感器分布力点数的缓存

        # 自动回传数据长度配置
        self.force_data_bytes = 6  # 合力数据长度（字节）
        self.distribution_data_bytes = 204  # 分布力数据长度（字节）
        self.total_data_bytes = self.force_data_bytes + self.distribution_data_bytes  # 总有效数据长度
        
        # 传感器模组配置 (地址0500-05A7)
        self.module_count = 28  # 28个传感器模组
        self.module_force_bytes = 6  # 每个模组合力数据: 3轴×2字节
        self.module_total_bytes = self.module_count * self.module_force_bytes  # 168字节
        
        # 掌心传感器特殊配置 - 只解析前9个分布力点
        self.palm_sensor_limit = 9  # 掌心传感器最多解析9个点
        
        self.connected_sensors = [] # 存储连接成功的传感器名称

        # 传感器模组顺序
        self.module_names = [
            # 大拇指
            "大拇指近节", "大拇指中节", "大拇指指尖", "大拇指指甲",
            # 食指
            "食指近节", "食指中节", "食指指尖", "食指指甲",
            # 中指
            "中指近节", "中指中节", "中指指尖", "中指指甲",
            # 无名指
            "无名指近节", "无名指中节", "无名指指尖", "无名指指甲",
            # 小拇指
            "小拇指近节", "小拇指中节", "小拇指指尖", "小拇指指甲",
            # 掌心
            "掌心1", "掌心2", "掌心3", "掌心4",
            "掌心5", "掌心6", "掌心7", "掌心8"
        ]
        
        # 传感器分布力地址区间映射
        self.distribution_addr_ranges = {
            # 大拇指
            (0x1000, 0x11FF): "大拇指近节",
            (0x1200, 0x13FF): "大拇指中节",
            (0x1400, 0x15FF): "大拇指指尖",
            (0x1600, 0x17FF): "大拇指指甲",
            # 食指
            (0x1800, 0x19FF): "食指近节",
            (0x1A00, 0x1BFF): "食指中节",
            (0x1C00, 0x1DFF): "食指指尖",
            (0x1E00, 0x1FFF): "食指指甲",
            # 中指
            (0x2000, 0x21FF): "中指近节",
            (0x2200, 0x23FF): "中指中节",
            (0x2400, 0x25FF): "中指指尖",
            (0x2600, 0x27FF): "中指指甲",
            # 无名指
            (0x2800, 0x29FF): "无名指近节",
            (0x2A00, 0x2BFF): "无名指中节",
            (0x2C00, 0x2DFF): "无名指指尖",
            (0x2E00, 0x2FFF): "无名指指甲",
            # 小拇指
            (0x3000, 0x31FF): "小拇指近节",
            (0x3200, 0x33FF): "小拇指中节",
            (0x3400, 0x35FF): "小拇指指尖",
            (0x3600, 0x37FF): "小拇指指甲",
            # 掌心
            (0x3800, 0x38FF): "掌心1",
            (0x3900, 0x39FF): "掌心2",
            (0x3A00, 0x3AFF): "掌心3",
            (0x3B00, 0x3BFF): "掌心4",
            (0x3C00, 0x3CFF): "掌心5",
            (0x3D00, 0x3DFF): "掌心6",
            (0x3E00, 0x3EFF): "掌心7",
            (0x3F00, 0x3FFF): "掌心8"
        }
        
        # 传感器模组分布力点数地址映射
        self.distribution_points_addrs = {
            # 大拇指
            "大拇指近节": 0x0030,
            "大拇指中节": 0x0032,
            "大拇指指尖": 0x0034,
            "大拇指指甲": 0x0036,
            # 食指
            "食指近节": 0x0038,
            "食指中节": 0x003A,
            "食指指尖": 0x003C,
            "食指指甲": 0x003E,
            # 中指
            "中指近节": 0x0040,
            "中指中节": 0x0042,
            "中指指尖": 0x0044,
            "中指指甲": 0x0046,
            # 无名指
            "无名指近节": 0x0048,
            "无名指中节": 0x004A,
            "无名指指尖": 0x004C,
            "无名指指甲": 0x004E,
            # 小拇指
            "小拇指近节": 0x0050,
            "小拇指中节": 0x0052,
            "小拇指指尖": 0x0054,
            "小拇指指甲": 0x0056,
            # 掌心
            "掌心8": 0x0058,
            "掌心7": 0x005A,
            "掌心6": 0x005C,
            "掌心5": 0x005E,
            "掌心4": 0x0060,
            "掌心3": 0x0062,
            "掌心2": 0x0064,
            "掌心1": 0x0066
        }
        
        # 传感器状态地址定义
        self.sensor_status_addrs = {
            0x0010: "大拇指和食指传感器",
            0x0011: "中指和无名指传感器",
            0x0012: "小拇指和掌心1-4传感器",
            0x0013: "掌心5-8传感器"
        }
        
        # 系统控制地址
        self.SYSTEM_RESET_ADDR = 0x0022  # 高速通信集成板重启控制地址
        
        # 计算完整帧长度
        self.full_frame_length = 2 + 1 + 2 + 1 + self.total_data_bytes + 1

        self.init_ui()
        
    def init_ui(self):
        # 通信配置区域
        config_frame = ttk.LabelFrame(self.root, text="通信配置")
        config_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(config_frame, text="COM口:").grid(row=0, column=0, padx=5, pady=5)
        self.com_var = tk.StringVar()
        self.com_combo = ttk.Combobox(config_frame, textvariable=self.com_var, width=10)
        self.com_combo.grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(config_frame, text="刷新COM", command=self.refresh_com_ports).grid(row=0, column=2, padx=5, pady=5)
        
        ttk.Label(config_frame, text="波特率:").grid(row=0, column=3, padx=5, pady=5)
        self.baud_var = tk.StringVar(value="921600")
        self.baud_combo = ttk.Combobox(config_frame, textvariable=self.baud_var, width=10)
        self.baud_combo['values'] = ["9600", "115200", "921600"]
        self.baud_combo.grid(row=0, column=4, padx=5, pady=5)
        
        ttk.Button(config_frame, text="连接设备", command=self.connect).grid(row=0, column=5, padx=5, pady=5)
        ttk.Button(config_frame, text="断开连接", command=self.disconnect).grid(row=0, column=6, padx=5, pady=5)
        
        # 传感器状态检查按钮
        ttk.Button(config_frame, text="检查传感器连接状态", command=self.check_sensor_status).grid(row=0, column=7, padx=5, pady=5)
        
        # 检查分布力点数按钮
        ttk.Button(config_frame, text="检查分布力点数", command=self.check_distribution_points).grid(row=0, column=8, padx=5, pady=5)
        
        # 版本号获取按钮
        ttk.Button(config_frame, text="获取高速通信集成板版本号", command=self.get_version).grid(row=0, column=9, padx=5, pady=5)

        # 清屏按钮
        ttk.Button(config_frame, text="清空数据日志", command=self.clear_log).grid(row=0, column=10, padx=5, pady=5)

        # 读取已连接传感器分布力按钮
        ttk.Button(config_frame, text="读取已连接传感器分布力", command=self.read_connected_sensors).grid(row=0, column=11, padx=5, pady=5)

        # 信息配置区域
        auto_recv_config_frame = ttk.LabelFrame(self.root, text="信息")
        auto_recv_config_frame.pack(fill=tk.X, padx=10, pady=5, ipady=5)  # 增加内部垂直间距

        # 使用文本框展示格式说明，支持滚动和选择
        frame_info = """
        自动回传帧结构:以单个M2324模组为例
        - 帧头: 0xAA 0x56 (2字节)
        - 预留: 0x00 (1字节)
        - 有效帧长度: 2字节(小端)
        - 总错误码(0x0023): 1字节
        - 有效数据(M): {self.total_data_bytes}字节
        - 传感器数据区: 按以下顺序回传
            1. 大拇指: 近节 → 中节 → 指尖 → 指甲
            2. 食指: 近节 → 中节 → 指尖 → 指甲
            3. 中指: 近节 → 中节 → 指尖 → 指甲
            4. 无名指: 近节 → 中节 → 指尖 → 指甲
            5. 小拇指: 近节 → 中节 → 指尖 → 指甲
            6. 掌心: 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8
        - 单个传感器数据格式: 合力(6字节) + 分布力(点数×3字节)
        - LRC校验: 1字节
        - 完整帧总长度: {self.full_frame_length}字节

        传感器模组合力(地址0500-05A7):
        - {self.module_count}个模组, 每个{self.module_force_bytes}字节, 共{self.module_total_bytes}字节
        - 小端，仅解析低字节用于力值计算

        分布力点数地址: 0030-0067 (uint16, 分布力字节数 = 点数 × 3)
        """.format(self=self)

        # 使用Text组件替代Label，支持换行和滚动（当内容过长时）
        info_text = tk.Text(auto_recv_config_frame, height=10, wrap=tk.WORD, state=tk.DISABLED, relief=tk.FLAT, bg=self.root.cget("bg"))
        info_text.pack(fill=tk.X, padx=5, pady=5)
        info_text.config(state=tk.NORMAL)
        info_text.insert(tk.END, frame_info)
        info_text.config(state=tk.DISABLED, selectbackground="#a6a6a6")  # 允许选中复制

        # 数据操作区域
        op_frame = ttk.LabelFrame(self.root, text="数据操作")
        op_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 读操作
        ttk.Label(op_frame, text="读寄存器:").grid(row=0, column=0, padx=5, pady=5)
        self.read_addr_var = tk.StringVar(value="1A00")
        ttk.Entry(op_frame, textvariable=self.read_addr_var, width=8).grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(op_frame, text="长度:").grid(row=0, column=2, padx=5, pady=5)
        self.read_len_var = tk.StringVar(value="204")
        ttk.Entry(op_frame, textvariable=self.read_len_var, width=5).grid(row=0, column=3, padx=5, pady=5)
        ttk.Button(op_frame, text="单次读", command=self.read_registers).grid(row=0, column=4, padx=5, pady=5)
        ttk.Button(op_frame, text="开始循环读", command=self.start_cycle_read).grid(row=0, column=5, padx=5, pady=5)
        ttk.Button(op_frame, text="停止循环读", command=self.stop_cycle_read).grid(row=0, column=6, padx=5, pady=5)
        
        # 读取传感器模组合力按钮
        ttk.Label(op_frame, text="传感器模组合力:").grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(op_frame, text="单次读模组合力", command=self.read_module_forces).grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(op_frame, text="开始循环读模组合力", command=self.start_module_cycle_read).grid(row=1, column=2, padx=5, pady=5)
        ttk.Button(op_frame, text="停止循环读模组合力", command=self.stop_module_cycle_read).grid(row=1, column=3, padx=5, pady=5)

        # 写操作
        ttk.Label(op_frame, text="写寄存器:").grid(row=2, column=0, padx=5, pady=5)
        self.write_addr_var = tk.StringVar(value="0017")
        ttk.Entry(op_frame, textvariable=self.write_addr_var, width=8).grid(row=2, column=1, padx=5, pady=5)
        ttk.Label(op_frame, text="数据:").grid(row=2, column=2, padx=5, pady=5)
        self.write_data_var = tk.StringVar(value="01")
        ttk.Entry(op_frame, textvariable=self.write_data_var, width=20).grid(row=2, column=3, padx=5, pady=5)
        ttk.Button(op_frame, text="执行写操作", command=self.write_registers).grid(row=2, column=4, padx=5, pady=5)
        
        # 自动回传控制
        ttk.Button(op_frame, text="开启自动回传", command=self.start_auto_receive).grid(row=2, column=5, padx=5, pady=5)
        ttk.Button(op_frame, text="停止自动回传", command=self.stop_auto_receive).grid(row=2, column=6, padx=5, pady=5)
        
        # 系统控制区域
        system_frame = ttk.LabelFrame(self.root, text="系统控制")
        system_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(system_frame, text="高速通信集成板控制:").grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(
            system_frame, 
            text="重启高速通信集成板", 
            command=self.reset_communication_board,
            style="Accent.TButton"
        ).grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(
            system_frame, 
            text="(地址0022: 写入1触发重启，设备将断开连接并重新启动)"
        ).grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        
        # 标定设置区域
        calib_frame = ttk.LabelFrame(self.root, text="标定设置")
        calib_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(calib_frame, text="标定命令帧:").grid(row=0, column=0, padx=5, pady=5)
        self.calib_cmd_var = tk.StringVar(value="55AA00170200010001E6")
        ttk.Entry(calib_frame, textvariable=self.calib_cmd_var, width=30).grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(calib_frame, text="当前状态:").grid(row=0, column=2, padx=5, pady=5)
        self.calib_state_var = tk.StringVar(value=self.calibration_state)
        self.calib_state_label = ttk.Label(
            calib_frame, 
            textvariable=self.calib_state_var, 
            foreground="red"
        )
        self.calib_state_label.grid(row=0, column=3, padx=5, pady=5)
        
        ttk.Button(calib_frame, text="开始标定", command=self.start_calibration).grid(row=0, column=4, padx=5, pady=5)
        
        # 日志区域
        log_frame = ttk.LabelFrame(self.root, text="数据日志")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        self.log_text = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 设置按钮样式
        self.style = ttk.Style()
        self.style.configure("Accent.TButton", foreground="red")
        
        self.refresh_com_ports()

    def clear_log(self):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)
        self.log("日志已清空")

    def log(self, message: str):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def refresh_com_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.com_combo['values'] = ports
        if ports:
            self.com_var.set(ports[0])

    def connect(self):
        if self.ser and self.ser.is_open:
            self.log("已连接设备")
            return
        com_port = self.com_var.get()
        baudrate = int(self.baud_var.get())
        try:
            self.ser = serial.Serial(
                port=com_port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.1
            )
            if self.ser.is_open:
                self.log(f"成功连接到 {com_port}，波特率 {baudrate}")
        except Exception as e:
            self.log(f"连接失败: {str(e)}")

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.log("已断开连接")
        else:
            self.log("未连接设备")

    def calculate_lrc(self, data: bytes) -> int:
        lrc = 0
        for byte in data:
            lrc = (lrc + byte) & 0xFF
        lrc = ((~lrc) + 1) & 0xFF
        return lrc

    def build_request_frame(self, func_code: int, reg_addr: int, data: bytes = b"") -> bytes:
        head = b"\x55\xAA"
        reserved = b"\x00"
        reg_addr_bytes = reg_addr.to_bytes(2, byteorder='little', signed=False)
        data_len = len(data) if func_code == 0x10 else int(self.read_len_var.get())
        data_len_bytes = data_len.to_bytes(2, byteorder='little', signed=False)
        frame_body = head + reserved + bytes([func_code]) + reg_addr_bytes + data_len_bytes + data
        lrc = self.calculate_lrc(frame_body)
        return frame_body + bytes([lrc])

    def parse_response_frame(self, frame: bytes) -> Optional[Dict]:
        if len(frame) < 8:
            self.log("响应帧长度不足")
            return None
        
        """处理读写操作的响应帧（aa55头）"""
        if len(frame) < 8 or frame[:2] != b"\xAA\x55":
            return None
        
        frame_body = frame[:-1]
        lrc = frame[-1]
        if self.calculate_lrc(frame_body) != lrc:
            print(f"LRC校验失败: 计算值={self.calculate_lrc(frame_body):02X}, 实际值={lrc:02X}")
            return None
        return {
            "reserved": frame[2],
            "func_code": frame[3],
            "reg_addr": int.from_bytes(frame[4:6], byteorder='little'),
            "data_len": int.from_bytes(frame[6:8], byteorder='little'),
            "data": frame[8:-1]
        }
    
    def parse_version_data(self, data: bytes) -> str:
        try:
            return data.decode('ascii').strip()
        except UnicodeDecodeError:
            return f"无法解析的版本数据: {data.hex().upper()}"

    def get_version(self):
        if not self.ser or not self.ser.is_open:
            self.log("请先连接设备")
            return
            
        try:
            version_frame = bytes.fromhex("55AA000300000F00EF")
            self.ser.write(version_frame)
            self.log(f"发送版本号请求: {version_frame.hex().upper()}")
            
            time.sleep(0.1)
            response = self.ser.read(128)
            
            if not response:
                self.log("未收到版本号响应")
                return
                
            self.log(f"收到版本号响应: {response.hex().upper()}")
            
            parsed = self.parse_response_frame(response)
            if parsed and parsed["func_code"] == 0x03:
                version = self.parse_version_data(parsed["data"])
                self.log(f"设备版本号: {version}")
            else:
                self.log("版本号响应解析失败")
                
        except Exception as e:
            self.log(f"获取版本号错误: {str(e)}")

    # 检查传感器连接状态
    def check_sensor_status(self):
        # 清空之前的连接状态
        self.connected_sensors = []
        
        if not self.ser or not self.ser.is_open:
            self.log("请先连接设备")
            return
            
        self.log("===== 开始检查传感器连接状态 =====")
        
        # 读取所有状态地址
        for addr in self.sensor_status_addrs:
            try:
                # 设置读取参数
                self.read_addr_var.set(f"{addr:04X}")
                self.read_len_var.set("1")  # 每个地址读取1字节
                
                reg_addr = addr
                read_len = 1
                
                request_frame = self.build_request_frame(0x03, reg_addr, b"")
                self.ser.write(request_frame)
                self.log(f"发送状态请求: 地址={reg_addr:04X} ({self.sensor_status_addrs[addr]}), 帧={request_frame.hex()}")
                
                time.sleep(0.05)
                response = self.ser.read(128)
                
                if not response:
                    self.log(f"未收到地址{reg_addr:04X}的响应")
                    continue
                    
                parsed = self.parse_response_frame(response)
                if parsed and parsed["func_code"] == 0x03 and parsed["data_len"] == 1:
                    status_byte = parsed["data"][0]
                    self.log(f"地址{reg_addr:04X}状态字节: 0x{status_byte:02X} ({bin(status_byte)[2:].zfill(8)})")
                    
                    # 根据不同地址解析位状态
                    if addr == 0x0010:
                        self._parse_addr_0010(status_byte)
                    elif addr == 0x0011:
                        self._parse_addr_0011(status_byte)
                    elif addr == 0x0012:
                        self._parse_addr_0012(status_byte)
                    elif addr == 0x0013:
                        self._parse_addr_0013(status_byte)
                else:
                    self.log(f"地址{reg_addr:04X}状态读取失败")
                    
            except Exception as e:
                self.log(f"读取地址{reg_addr:04X}错误: {str(e)}")
        
        self.log("===== 传感器连接状态检查完成 =====")
        # 显示连接成功的传感器数量
        self.log(f"共检测到 {len(self.connected_sensors)} 个连接成功的传感器")

    '''
    # 检查分布力点数
    def check_distribution_points(self):
        if not self.ser or not self.ser.is_open:
            self.log("请先连接设备")
            return

        self.log("===== 开始检查分布力点数 =====")
        
        # 按类别分组读取和显示
        groups = {
            "大拇指": ["大拇指近节", "大拇指中节", "大拇指指尖", "大拇指指甲"],
            "食指": ["食指近节", "食指中节", "食指指尖", "食指指甲"],
            "中指": ["中指近节", "中指中节", "中指指尖", "中指指甲"],
            "无名指": ["无名指近节", "无名指中节", "无名指指尖", "无名指指甲"],
            "小拇指": ["小拇指近节", "小拇指中节", "小拇指指尖", "小拇指指甲"],
            "掌心": ["掌心1", "掌心2", "掌心3", "掌心4", "掌心5", "掌心6", "掌心7", "掌心8"]
        }
        
        for group_name, modules in groups.items():
            self.log(f"\n===== {group_name}传感器分布力点数 =====")
            for module in modules:
                if module not in self.distribution_points_addrs:
                    self.log(f"  {module}: 无对应地址信息")
                    continue
                    
                addr = self.distribution_points_addrs[module]
                try:
                    # 读取2字节（uint16）
                    self.read_addr_var.set(f"{addr:04X}")
                    self.read_len_var.set("2")
                    
                    reg_addr = addr
                    read_len = 2
                    
                    request_frame = self.build_request_frame(0x03, reg_addr, b"")
                    self.ser.write(request_frame)
                    
                    time.sleep(0.05)
                    response = self.ser.read(128)
                    
                    if not response:
                        self.log(f"  {module} (地址{reg_addr:04X}): 未收到响应")
                        continue
                        
                    parsed = self.parse_response_frame(response)
                    if parsed and parsed["func_code"] == 0x03 and parsed["data_len"] == 2:
                        # 解析uint16值（小端）
                        point_count = int.from_bytes(parsed["data"], byteorder='little', signed=False)
                        byte_count = point_count * 3  # 计算字节数
                        
                        # 掌心传感器显示特殊提示
                        if module.startswith("掌心"):
                            actual_points = min(point_count, self.palm_sensor_limit)
                            status = f"已连接 (将只解析前{actual_points}个点)" if point_count > 0 else "未连接"
                        else:
                            status = "已连接" if point_count > 0 else "未连接"
                            
                        self.log(
                            f"  {module} (地址{reg_addr:04X}): "
                            f"点数={point_count}, 字节数={byte_count}, 状态={status}"
                        )
                    else:
                        self.log(f"  {module} (地址{reg_addr:04X}): 读取失败")
                        
                except Exception as e:
                    self.log(f"  {module} (地址{reg_addr:04X}) 错误: {str(e)}")
        
        self.log("\n===== 分布力点数检查完成 =====")
    '''

    def check_distribution_points(self):
        if not self.ser or not self.ser.is_open:
            self.log("请先连接设备")
            return
        
        # 清空缓存
        self.distribution_points_cache.clear()
    
        self.log("===== 开始检查分布力点数 =====")
    
        # 按类别分组读取和显示
        groups = {
            "大拇指": ["大拇指近节", "大拇指中节", "大拇指指尖", "大拇指指甲"],
            "食指": ["食指近节", "食指中节", "食指指尖", "食指指甲"],
            "中指": ["中指近节", "中指中节", "中指指尖", "中指指甲"],
            "无名指": ["无名指近节", "无名指中节", "无名指指尖", "无名指指甲"],
            "小拇指": ["小拇指近节", "小拇指中节", "小拇指指尖", "小拇指指甲"],
            "掌心": ["掌心1", "掌心2", "掌心3", "掌心4", "掌心5", "掌心6", "掌心7", "掌心8"]
        }
    
        for group_name, modules in groups.items():
            self.log(f"\n===== {group_name}传感器分布力点数 =====")
            for module in modules:
                if module not in self.distribution_points_addrs:
                    self.log(f"  {module}: 无对应地址信息")
                    continue
                
                addr = self.distribution_points_addrs[module]
                try:
                    # 读取2字节（uint16）
                    self.read_addr_var.set(f"{addr:04X}")
                    self.read_len_var.set("2")
                
                    reg_addr = addr
                    read_len = 2
                
                    request_frame = self.build_request_frame(0x03, reg_addr, b"")
                    self.ser.write(request_frame)
                
                    time.sleep(0.05)
                    response = self.ser.read(128)
                
                    if not response:
                        self.log(f"  {module} (地址{reg_addr:04X}): 未收到响应")
                        self.distribution_points_cache[module] = 0  # 缓存为0表示未收到响应
                        continue
                    
                    parsed = self.parse_response_frame(response)
                    if parsed and parsed["func_code"] == 0x03 and parsed["data_len"] == 2:
                        # 解析uint16值（小端）
                        point_count = int.from_bytes(parsed["data"], byteorder='little', signed=False)
                        self.distribution_points_cache[module] = point_count  # 缓存点数
                    
                        byte_count = point_count * 3  # 计算字节数
                    
                        # 掌心传感器显示特殊提示
                        if module.startswith("掌心"):
                            actual_points = min(point_count, self.palm_sensor_limit)
                            status = f"已连接 (将只解析前{actual_points}个点)" if point_count > 0 else "未连接"
                        else:
                            status = "已连接" if point_count > 0 else "未连接"
                        
                        self.log(
                            f"  {module} (地址{reg_addr:04X}): "
                            f"点数={point_count}, 字节数={byte_count}, 状态={status}"
                        )
                    else:
                        self.log(f"  {module} (地址{reg_addr:04X}): 读取失败")
                        self.distribution_points_cache[module] = 0  # 缓存为0表示读取失败
                    
                except Exception as e:
                    self.log(f"  {module} (地址{reg_addr:04X}) 错误: {str(e)}")
                    self.distribution_points_cache[module] = 0  # 缓存为0表示发生错误
        
            self.log("\n===== 分布力点数检查完成 =====")

    # 解析地址0010的状态（大拇指和食指）
    def _parse_addr_0010(self, status_byte):
        # bit0-bit3: 大拇指近节、中节、指尖、指甲
        # bit4-bit7: 食指近节、中节、指尖、指甲
        thumb_sensors = [
            "大拇指近节", "大拇指中节", "大拇指指尖", "大拇指指甲"
        ]
        index_sensors = [
            "食指近节", "食指中节", "食指指尖", "食指指甲"
        ]
        
        self.log("  大拇指传感器状态:")
        for i in range(4):
            status = "连接成功" if (status_byte & (1 << i)) else "未连接"
            self.log(f"    {thumb_sensors[i]}: {status}")
            if status == "连接成功":
                self.connected_sensors.append(thumb_sensors[i])
            
        self.log("  食指传感器状态:")
        for i in range(4):
            status = "连接成功" if (status_byte & (1 << (i + 4))) else "未连接"
            self.log(f"    {index_sensors[i]}: {status}")
            if status == "连接成功":
                self.connected_sensors.append(index_sensors[i])

    # 解析地址0011的状态（中指和无名指）
    def _parse_addr_0011(self, status_byte):
        # bit0-bit3: 中指近节、中节、指尖、指甲
        # bit4-bit7: 无名指近节、中节、指尖、指甲
        middle_sensors = [
            "中指近节", "中指中节", "中指指尖", "中指指甲"
        ]
        ring_sensors = [
            "无名指近节", "无名指中节", "无名指指尖", "无名指指甲"
        ]
        
        self.log("  中指传感器状态:")
        for i in range(4):
            status = "连接成功" if (status_byte & (1 << i)) else "未连接"
            self.log(f"    {middle_sensors[i]}: {status}")
            if status == "连接成功":
                self.connected_sensors.append(middle_sensors[i])
            
        self.log("  无名指传感器状态:")
        for i in range(4):
            status = "连接成功" if (status_byte & (1 << (i + 4))) else "未连接"
            self.log(f"    {ring_sensors[i]}: {status}")
            if status == "连接成功":
                self.connected_sensors.append(ring_sensors[i])

    # 解析地址0012的状态（小拇指和掌心1-4）
    def _parse_addr_0012(self, status_byte):
        # bit0-bit3: 小拇指近节、中节、指尖、指甲
        # bit4-bit7: 掌心1、掌心2、掌心3、掌心4
        pinky_sensors = [
            "小拇指近节", "小拇指中节", "小拇指指尖", "小拇指指甲"
        ]
        palm_sensors = [
            "掌心1", "掌心2", "掌心3", "掌心4"
        ]
        
        self.log("  小拇指传感器状态:")
        for i in range(4):
            status = "连接成功" if (status_byte & (1 << i)) else "未连接"
            self.log(f"    {pinky_sensors[i]}: {status}")
            if status == "连接成功":
                self.connected_sensors.append(pinky_sensors[i])
            
        self.log("  掌心传感器(1-4)状态:")
        for i in range(4):
            status = "连接成功" if (status_byte & (1 << (i + 4))) else "未连接"
            self.log(f"    {palm_sensors[i]}: {status}")
            if status == "连接成功":
                self.connected_sensors.append(palm_sensors[i])

    # 解析地址0013的状态（掌心5-8）
    def _parse_addr_0013(self, status_byte):
        # bit0-bit3: 掌心5、掌心6、掌心7、掌心8
        palm_sensors = [
            "掌心5", "掌心6", "掌心7", "掌心8"
        ]
        
        self.log("  掌心传感器(5-8)状态:")
        for i in range(4):
            status = "连接成功" if (status_byte & (1 << i)) else "未连接"
            self.log(f"    {palm_sensors[i]}: {status}")
            if status == "连接成功":
                self.connected_sensors.append(palm_sensors[i])

    def read_connected_sensors(self):
        """读取所有连接成功的传感器分布力数据"""
        if not self.ser or not self.ser.is_open:
            self.log("请先连接设备")
            return
        
        # 先检查传感器连接状态，确保数据最新
        self.log("===== 正在更新传感器连接状态 =====")
        self.check_sensor_status()
            
        if not self.connected_sensors:
            self.log("没有检测到连接成功的传感器，无法读取分布力数据")
            return
            
        self.log("\n===== 开始读取所有连接成功的传感器分布力 =====")
        self.log(f"共检测到 {len(self.connected_sensors)} 个连接成功的传感器")
        
        # 按类别分组读取，提高可读性
        groups = {
            "大拇指": [], "食指": [], "中指": [], 
            "无名指": [], "小拇指": [], "掌心": []
        }
        
        for sensor in self.connected_sensors:
            if sensor.startswith("大拇指"):
                groups["大拇指"].append(sensor)
            elif sensor.startswith("食指"):
                groups["食指"].append(sensor)
            elif sensor.startswith("中指"):
                groups["中指"].append(sensor)
            elif sensor.startswith("无名指"):
                groups["无名指"].append(sensor)
            elif sensor.startswith("小拇指"):
                groups["小拇指"].append(sensor)
            elif sensor.startswith("掌心"):
                groups["掌心"].append(sensor)
        
        # 逐个读取传感器数据
        success_count = 0
        fail_count = 0
        
        for group_name, sensors in groups.items():
            if sensors:
                self.log(f"\n----- {group_name}传感器 -----")
                for idx, sensor in enumerate(sensors):
                    # 显示进度
                    self.log(f"\n[{idx+1}/{len(sensors)}] 处理 {sensor}...")
                    
                    # 获取传感器地址范围
                    start_addr, end_addr = self.get_address_by_sensor_name(sensor)
                    if not start_addr or not end_addr:
                        self.log(f"  {sensor}: 未找到对应的地址范围")
                        fail_count += 1
                        continue
                        
                    # 从分布力点数地址获取该传感器的点数
                    points_addr = self.distribution_points_addrs.get(sensor)
                    if not points_addr:
                        self.log(f"  {sensor}: 未找到点数地址信息")
                        fail_count += 1
                        continue
                        
                    # 先读取该传感器的分布力点数
                    try:
                        self.read_addr_var.set(f"{points_addr:04X}")
                        self.read_len_var.set("2")  # 点数是2字节(uint16)
                        
                        request_frame = self.build_request_frame(0x03, points_addr, b"")
                        self.ser.write(request_frame)
                        
                        time.sleep(0.05)
                        response = self.ser.read(128)
                        
                        if not response:
                            self.log(f"  {sensor}: 未收到点数响应")
                            fail_count += 1
                            continue
                            
                        parsed = self.parse_response_frame(response)
                        if parsed and parsed["func_code"] == 0x03 and parsed["data_len"] == 2:
                            point_count = int.from_bytes(parsed["data"], byteorder='little', signed=False)
                            
                            # 掌心传感器特殊处理 - 只取前9个点
                            if sensor.startswith("掌心"):
                                actual_points = min(point_count, self.palm_sensor_limit)
                                self.log(f"  {sensor}: 原始点数={point_count}, 将只解析前{actual_points}个点")
                                data_len = actual_points * 3  # 每个点3字节数据
                            else:
                                data_len = point_count * 3  # 每个点3字节数据
                                
                            if data_len <= 0:
                                self.log(f"  {sensor}: 无效的点数({point_count})")
                                fail_count += 1
                                continue
                                
                            # 验证数据长度是否在地址范围内
                            max_possible_len = end_addr - start_addr + 1
                            if data_len > max_possible_len:
                                self.log(f"  {sensor}: 数据长度({data_len})超过地址范围最大长度({max_possible_len})，将截断读取")
                                data_len = max_possible_len
                                
                            # 读取分布力数据
                            self.read_addr_var.set(f"{start_addr:04X}")
                            self.read_len_var.set(str(data_len))
                            
                            request_frame = self.build_request_frame(0x03, start_addr, b"")
                            self.ser.write(request_frame)
                            
                            time.sleep(0.05)
                            data_response = self.ser.read(1024)
                            
                            if not data_response:
                                self.log(f"  {sensor}: 未收到分布力数据")
                                fail_count += 1
                                continue
                                
                            data_parsed = self.parse_response_frame(data_response)
                            if data_parsed and data_parsed["func_code"] == 0x03:
                                self.log(f"  成功读取 {sensor} 分布力数据 (地址:0x{start_addr:04X}, 长度:{data_len}字节)")
                                self.parse_normal_force_data(data_parsed["data"], start_addr, source=f"[{sensor}] ")
                                success_count += 1
                            else:
                                self.log(f"  {sensor}: 分布力数据解析失败")
                                fail_count += 1
                        else:
                            self.log(f"  {sensor}: 点数读取失败")
                            fail_count += 1
                            
                    except Exception as e:
                        self.log(f"  {sensor} 读取错误: {str(e)}")
                        fail_count += 1
        
        self.log("\n===== 所有连接成功的传感器分布力读取完成 =====")
        self.log(f"读取结果: 成功 {success_count} 个, 失败 {fail_count} 个")

    # 重启高速通信集成板
    def reset_communication_board(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("警告", "请先连接设备")
            return
            
        # 确认对话框
        confirm = messagebox.askyesno(
            "确认重启", 
            "确定要重启高速通信集成板吗？\n重启后设备将断开连接，需要重新连接。"
        )
        if not confirm:
            return
            
        try:
            # 停止所有正在进行的操作
            self.auto_receive_running = False
            self.cycle_read_running = False
            self.module_cycle_read_running = False
            
            self.log("===== 准备重启高速通信集成板 =====")
            
            # 向地址0022写入1触发重启
            reg_addr = self.SYSTEM_RESET_ADDR
            data = bytes.fromhex("01")  # 1表示触发重启
            
            request_frame = self.build_request_frame(0x10, reg_addr, data)
            self.ser.write(request_frame)
            self.log(f"发送重启命令: 地址={reg_addr:04X}, 数据=01, 帧={request_frame.hex()}")
            
            # 短暂延迟确保命令发送成功
            time.sleep(0.5)
            
            # 断开连接
            self.disconnect()
            
            self.log("重启命令已发送，高速通信集成板正在重启...")
            self.log("请等待片刻后重新连接设备")
            
        except Exception as e:
            self.log(f"重启操作错误: {str(e)}")

    def get_address_by_sensor_name(self, sensor_name: str) -> Tuple[Optional[int], Optional[int]]:
        """根据传感器名称获取对应的地址范围"""
        for (start, end), name in self.distribution_addr_ranges.items():
            if name == sensor_name:
                return (start, end)
        return (None, None)

    def parse_normal_force_data(self, data: bytes, addr: int, source: str = "") -> List[Dict]:
        """解析分布力数据并按传感器分组显示"""
        # 根据地址获取传感器名称
        sensor_name = None
        for (start, end), name in self.distribution_addr_ranges.items():
            if start <= addr <= end:
                sensor_name = name
                break
        
        if not sensor_name:
            sensor_name = f"未知传感器(0x{addr:04X})"
            
        parsed = []
        total_groups = len(data) // 3
        remainder = len(data) % 3
        
        if remainder != 0:
            self.log(f"{source}{sensor_name}分布力数据长度不是3的倍数，剩余{remainder}字节将被忽略")
        
        # 掌心传感器特殊处理 - 只解析前9个点
        if sensor_name.startswith("掌心"):
            total_groups = min(total_groups, self.palm_sensor_limit)
            print(f"{source}{sensor_name}特殊处理: 只解析前{total_groups}个分布力点")
        
        for i in range(total_groups):
            offset = i * 3
            b1, b2, b3 = data[offset], data[offset+1], data[offset+2]
            
            val1 = b1 if b1 <= 127 else b1 - 256
            val2 = b2 if b2 <= 127 else b2 - 256
            val3 = b3
            
            scaled1 = round(val1 * 0.1, 1)
            scaled2 = round(val2 * 0.1, 1)
            scaled3 = round(val3 * 0.1, 1)
            
            parsed.append({
                "index": i,
                "raw_bytes": (b1, b2, b3),
                "raw_hex": (f"0x{b1:02X}", f"0x{b2:02X}", f"0x{b3:02X}"),
                "converted": (val1, val2, val3),
                "scaled": (scaled1, scaled2, scaled3)
            })
        
        if parsed:
            self.log(f"\n===== {source}{sensor_name}分布力数据 =====")
            self.log(f"共{len(parsed)}组数据（每组×0.1N）")

            # 计算每组物理值的统计信息
            max_x = max(p["scaled"][0] for p in parsed)
            max_y = max(p["scaled"][1] for p in parsed)
            max_z = max(p["scaled"][2] for p in parsed)
            
            max_x_idx = next(i for i, p in enumerate(parsed) if p["scaled"][0] == max_x)
            max_y_idx = next(i for i, p in enumerate(parsed) if p["scaled"][1] == max_y)
            max_z_idx = next(i for i, p in enumerate(parsed) if p["scaled"][2] == max_z)
            
            self.log(f"X轴最大值: {max_x}N (组{max_x_idx})")
            self.log(f"Y轴最大值: {max_y}N (组{max_y_idx})")
            self.log(f"Z轴最大值: {max_z}N (组{max_z_idx})")
            
            # 显示所有数据
            print(f"{source}{sensor_name}")
            for group in parsed:
                print(
                    f"  组{group['index']:02d} | 原始: {group['raw_hex']} "
                    f"| 转换值: ({group['converted'][0]:3d}, {group['converted'][1]:3d}, {group['converted'][2]:3d}) "
                    f"| 物理值: X={group['scaled'][0]:5.1f}N, Y={group['scaled'][1]:5.1f}N, Z={group['scaled'][2]:5.1f}N"
                )
        else:
            self.log(f"{source}{sensor_name}无有效分布力数据")
        
        return parsed
    
    '''
    #仅实现单模组自动回传数据解析，未实现多模组自动回传数据解析
    def parse_auto_receive_force_data(self, data: bytes) -> List[Dict]:
        parsed = []
        
        if len(data) != self.distribution_data_bytes:
            self.log(f"警告: 自动回传分布力数据长度不符 - 预期: {self.distribution_data_bytes}字节, 实际: {len(data)}字节")
        
        total_groups = len(data) // 3
        remainder = len(data) % 3
        
        if remainder != 0:
            self.log(f"自动回传分布力数据长度不是3的倍数，剩余{remainder}字节将被忽略")
        
        for i in range(total_groups):
            offset = i * 3
            b1, b2, b3 = data[offset], data[offset+1], data[offset+2]
            
            val1 = b1 if b1 <= 127 else b1 - 256
            val2 = b2 if b2 <= 127 else b2 - 256
            val3 = b3
            
            scaled1 = round(val1 * 0.1, 1)
            scaled2 = round(val2 * 0.1, 1)
            scaled3 = round(val3 * 0.1, 1)
            
            parsed.append({
                "index": i,
                "raw_bytes": (b1, b2, b3),
                "raw_hex": (f"0x{b1:02X}", f"0x{b2:02X}", f"0x{b3:02X}"),
                "converted": (val1, val2, val3),
                "scaled": (scaled1, scaled2, scaled3)
            })
        
        if parsed:
            self.log(f"自动回传分布力数据: {len(parsed)}组（每组×0.1N）")

            # 只显示前10组数据，避免日志过长
            display_limit = 10
            if len(parsed) <= display_limit:
                for group in parsed:
                    self.log(
                        f"  组{group['index']:02d} "
                        f"| 转换值: ({group['converted'][0]:3d}, {group['converted'][1]:3d}, {group['converted'][2]:3d}) "
                        f"| 物理值: X={group['scaled'][0]:5.1f}N, Y={group['scaled'][1]:5.1f}N, Z={group['scaled'][2]:5.1f}N"
                    )
            else:
                for group in parsed[:display_limit]:
                    self.log(
                        f"  组{group['index']:02d} "
                        f"| 转换值: ({group['converted'][0]:3d}, {group['converted'][1]:3d}, {group['converted'][2]:3d}) "
                        f"| 物理值: X={group['scaled'][0]:5.1f}N, Y={group['scaled'][1]:5.1f}N, Z={group['scaled'][2]:5.1f}N"
                    )
                self.log(f"  ... 省略后续{len(parsed) - display_limit}组数据 ...")
        else:
            self.log("自动回传无有效分布力数据")
        
        return parsed
    '''
    '''
    def parse_total_force(self, data: bytes) -> Optional[Dict]:
        if len(data) < 6:
            self.log("合力数据不足6字节，无法解析")
            return None
        
        fx_low_byte = data[0]
        fy_low_byte = data[2]
        fz_low_byte = data[4]
        
        fx_bytes = data[0:2]
        fy_bytes = data[2:4]
        fz_bytes = data[4:6]
        
        fx_raw = fx_low_byte if fx_low_byte <= 127 else fx_low_byte - 256
        fy_raw = fy_low_byte if fy_low_byte <= 127 else fy_low_byte - 256
        fz_raw = fz_low_byte 
        
        fx_scaled = round(fx_raw * 0.1, 1)
        fy_scaled = round(fy_raw * 0.1, 1)
        fz_scaled = round(fz_raw * 0.1, 1)
        
        return {
            "raw_hex": (
                fx_bytes.hex().upper(),
                fy_bytes.hex().upper(),
                fz_bytes.hex().upper()
            ),
            "used_byte": (
                f"0x{fx_low_byte:02X}",
                f"0x{fy_low_byte:02X}",
                f"0x{fz_low_byte:02X}"
            ),
            "converted": (fx_raw, fy_raw, fz_raw),
            "scaled": (fx_scaled, fy_scaled, fz_scaled)
        }

    def auto_receive_loop(self):
        if not self.auto_receive_running or not self.ser or not self.ser.is_open:
            return
        try:
            if self.ser.in_waiting > 0:
                response = self.ser.read(self.ser.in_waiting)
               
                # 寻找帧头位置
                frame_start = response.find(b"\xAA\x56")
                if frame_start == -1:
                    self.log("未找到自动回传帧头")
                    self.root.after(10, self.auto_receive_loop)
                    return
                parsed_frame = self.parse_auto_receive_frame(response[frame_start:])
                if parsed_frame:
                    self.log(f"\n自动回传: 长度={parsed_frame['frame_len']}, 错误码={parsed_frame['error_code']}")
                   
                    total_force = self.parse_total_force(parsed_frame["data"])
                    if total_force:
                        self.log(
                            f"  合力数据:\n"
                            f"  Fx : 原始={total_force['raw_hex'][0]} | 使用字节={total_force['used_byte'][0]} | 转换值={total_force['converted'][0]:2d} | 物理值={total_force['scaled'][0]:5.1f}N\n"
                            f"  Fy : 原始={total_force['raw_hex'][1]} | 使用字节={total_force['used_byte'][1]} | 转换值={total_force['converted'][1]:2d} | 物理值={total_force['scaled'][1]:5.1f}N\n"
                            f"  Fz : 原始={total_force['raw_hex'][2]} | 使用字节={total_force['used_byte'][2]} | 转换值={total_force['converted'][2]:2d} | 物理值={total_force['scaled'][2]:5.1f}N"
                        )
                   
                    force_data = parsed_frame["data"][6:]
                    if force_data:
                        self.parse_auto_receive_force_data(force_data)
                    else:
                        self.log("自动回传无分布力数据")
        except Exception as e:
            self.log(f"自动回传处理错误: {str(e)}")
        self.root.after(10, self.auto_receive_loop)
    '''

    def parse_module_forces(self, data: bytes) -> List[Dict]:
        parsed = []
        
        if len(data) != self.module_total_bytes:
            self.log(f"警告: 传感器模组合力数据长度不符 - 预期: {self.module_total_bytes}字节, 实际: {len(data)}字节")
        
        valid_modules = min(len(data) // self.module_force_bytes, self.module_count)
        self.log(f"传感器模组合力数据: 共{valid_modules}/{self.module_count}个有效模组数据（仅解析低字节）")
        
        for i in range(valid_modules):
            offset = i * self.module_force_bytes
            
            if offset + self.module_force_bytes > len(data):
                self.log(f"警告: 数据长度不足，终止解析")
                break
                
            # 仅提取每个轴的低字节（忽略高字节）
            fx_low_byte = data[offset]       # Fx低字节（第1字节）
            fy_low_byte = data[offset + 2]   # Fy低字节（第3字节）
            fz_low_byte = data[offset + 4]   # Fz低字节（第5字节）
            
            # 提取完整2字节用于显示（原始数据）
            fx_bytes = data[offset:offset+2]
            fy_bytes = data[offset+2:offset+4]
            fz_bytes = data[offset+4:offset+6]
            
            # Fx、Fy按有符号处理，Fz按无符号处理
            fx_raw = fx_low_byte if fx_low_byte <= 127 else fx_low_byte - 256
            fy_raw = fy_low_byte if fy_low_byte <= 127 else fy_low_byte - 256
            fz_raw = fz_low_byte  # 无符号
            
            # 转换为物理值（×0.1N）
            fx_scaled = round(fx_raw * 0.1, 1)
            fy_scaled = round(fy_raw * 0.1, 1)
            fz_scaled = round(fz_raw * 0.1, 1)
            
            parsed.append({
                "index": i,
                "name": self.module_names[i] if i < len(self.module_names) else f"未知模组{i}",
                "raw_hex": (          # 完整2字节的原始数据
                    fx_bytes.hex().upper(),
                    fy_bytes.hex().upper(),
                    fz_bytes.hex().upper()
                ),
                "used_byte": (        # 实际使用的低字节
                    f"0x{fx_low_byte:02X}",
                    f"0x{fy_low_byte:02X}",
                    f"0x{fz_low_byte:02X}"
                ),
                "converted": (fx_raw, fy_raw, fz_raw),
                "scaled": (fx_scaled, fy_scaled, fz_scaled)
            })
        
        if parsed:
            # 按类别分组显示
            self.log("\n===== 大拇指传感器 =====")
            for module in parsed[0:4]:
                self.log_module_force(module)
                
            self.log("\n===== 食指传感器 =====")
            for module in parsed[4:8]:
                self.log_module_force(module)
                
            self.log("\n===== 中指传感器 =====")
            for module in parsed[8:12]:
                self.log_module_force(module)
                
            self.log("\n===== 无名指传感器 =====")
            for module in parsed[12:16]:
                self.log_module_force(module)
                
            self.log("\n===== 小拇指传感器 =====")
            for module in parsed[16:20]:
                self.log_module_force(module)
                
            self.log("\n===== 掌心传感器 =====")
            for module in parsed[20:28]:
                self.log_module_force(module)
        else:
            self.log("无有效传感器模组合力数据")
        
        return parsed
    
    def log_module_force(self, module: Dict):
        self.log(f"{module['name']:8s} | "
            f"Fx={module['scaled'][0]:5.1f}N "
            f"Fy={module['scaled'][1]:5.1f}N "
            f"Fz={module['scaled'][2]:5.1f}N "
        )

    def read_module_forces(self, cycle_mode=False):
        if not self.ser or not self.ser.is_open:
            self.log("请先连接设备")
            if cycle_mode:
                self.root.after(50, self.read_module_forces, True)
            return
            
        try:
            self.read_addr_var.set("0500")
            self.read_len_var.set(str(self.module_total_bytes))
            
            reg_addr = int(self.read_addr_var.get(), 16)
            read_len = int(self.read_len_var.get())
            
            request_frame = self.build_request_frame(0x03, reg_addr, b"")
            self.ser.write(request_frame)
            
            log_prefix = "[循环读模组]" if cycle_mode else ""
            self.log(f"{log_prefix}发送传感器模组合力请求: 地址={reg_addr:04X}, 长度={read_len}, 帧={request_frame.hex()}")
            
            time.sleep(0.01)
            response = self.ser.read(1024)
            
            if not response:
                self.log(f"{log_prefix}未收到传感器模组合力响应")
                if cycle_mode:
                    self.root.after(50, self.read_module_forces, True)
                return
                
            parsed = self.parse_response_frame(response)
            if parsed and parsed["func_code"] == 0x03:
                self.log(f"{log_prefix}传感器模组合力读取成功: 地址={parsed['reg_addr']:04X}, 长度={parsed['data_len']}")
                self.parse_module_forces(parsed["data"])
            else:
                self.log(f"{log_prefix}传感器模组合力读取失败")
                
        except Exception as e:
            self.log(f"{log_prefix}传感器模组合力读取错误: {str(e)}")
        
        if cycle_mode and self.module_cycle_read_running:
            self.root.after(50, self.read_module_forces, True)

    def start_module_cycle_read(self):
        if self.module_cycle_read_running:
            self.log("已在循环读取传感器模组数据中")
            return
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("警告", "请先连接设备")
            return
        self.module_cycle_read_running = True
        self.log("开始循环读取传感器模组数据...")
        self.read_module_forces(cycle_mode=True)

    def stop_module_cycle_read(self):
        self.module_cycle_read_running = False
        self.log("已停止循环读取传感器模组数据")

    def read_registers(self, cycle_mode=False):
        if not self.ser or not self.ser.is_open:
            self.log("请先连接设备")
            if cycle_mode:
                self.root.after(50, self.read_registers, True)
            return
        try:
            reg_addr = int(self.read_addr_var.get(), 16)
            read_len = int(self.read_len_var.get())
            if read_len > 512:
                self.log("最大读取长度为512字节")
                if cycle_mode:
                    self.root.after(50, self.read_registers, True)
                return
            request_frame = self.build_request_frame(0x03, reg_addr, b"")
            self.ser.write(request_frame)
            log_prefix = "[循环读]" if cycle_mode else ""
            self.log(f"{log_prefix}发送读请求: 地址={reg_addr:04X}, 长度={read_len}, 帧={request_frame.hex()}")
            time.sleep(0.01)
            response = self.ser.read(1024)
            if not response:
                self.log(f"{log_prefix}未收到响应")
                if cycle_mode:
                    self.root.after(50, self.read_registers, True)
                return
            parsed = self.parse_response_frame(response)
            if parsed and parsed["func_code"] == 0x03:
                self.log(f"{log_prefix}读成功: 地址={parsed['reg_addr']:04X}, 长度={parsed['data_len']}")
                # 传入地址参数，用于识别传感器
                self.parse_normal_force_data(parsed["data"], reg_addr, source=log_prefix)
            else:
                self.log(f"{log_prefix}读失败")
        except Exception as e:
            self.log(f"{log_prefix}读操作错误: {str(e)}")
        if cycle_mode and self.cycle_read_running:
            self.root.after(50, self.read_registers, True)

    def start_cycle_read(self):
        if self.cycle_read_running:
            self.log("已在循环读取中")
            return
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("警告", "请先连接设备")
            return
        self.cycle_read_running = True
        self.log("开始循环读取...")
        self.read_registers(cycle_mode=True)

    def stop_cycle_read(self):
        self.cycle_read_running = False
        self.log("已停止循环读取")

    def write_registers(self):
        if not self.ser or not self.ser.is_open:
            self.log("请先连接设备")
            return
        try:
            # 保存当前自动回传状态
            auto_recv_state = self.auto_receive_running
            if auto_recv_state:
                self.stop_auto_receive()  # 写操作前先停止自动回传
        
            # 执行写操作
            reg_addr = int(self.write_addr_var.get(), 16)
            data_hex = self.write_data_var.get().replace(" ", "")
            data = bytes.fromhex(data_hex)

            request_frame = self.build_request_frame(0x10, reg_addr, data)
            self.ser.write(request_frame)
            self.log(f"发送写请求: 地址={reg_addr:04X}, 数据={data.hex()}, 帧={request_frame.hex()}")
        
            # 只读取指定长度的响应（避免混入自动回传数据）
            time.sleep(3)
            response = self.ser.read(16)  # 限制读取长度
            if response:
                parsed = self.parse_response_frame(response)
                if parsed and parsed["func_code"] == 0x10:
                    status = parsed["data"][0] if parsed["data"] else 0
                    self.log(f"写结果: {'成功' if status == 0 else f'失败（错误码{status}）'}")
                else:
                    self.log(f"写失败: {response.hex()}")
            else:
                self.log("未收到响应")
            
            # 恢复自动回传状态
            if auto_recv_state and self.write_data_var.get() == "01":
                self.start_auto_receive()
            
        except Exception as e:
            self.log(f"写操作错误: {str(e)}")

    def start_calibration(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("警告", "请先连接设备")
            return
        
        self._update_calibration_state("标定中")
        self.log("===== 开始标定流程 =====")
        
        calib_cmd_hex = self.calib_cmd_var.get().strip()
        if not calib_cmd_hex:
            self.log("标定命令帧不能为空")
            self._update_calibration_state("失败")
            return
        
        try:
            calib_frame = bytes.fromhex(calib_cmd_hex)
            self.ser.write(calib_frame)
            self.log(f"发送标定命令: {calib_cmd_hex}")
            
            time.sleep(1)
            response = self.ser.read(256)
            if not response:
                self.log("未收到标定响应，可能超时")
                self._update_calibration_state("失败")
                return
            
            parsed = self.parse_response_frame(response)
            if parsed:
                self.log(f"标定响应: 地址={parsed['reg_addr']:04X}, 数据={parsed['data'].hex()}")
                if parsed["data"] == b"\x00":
                    self.log("标定成功！")
                    self._update_calibration_state("成功")
                    self.read_registers()
                else:
                    self.log(f"标定失败，响应码: {parsed['data'].hex()}")
                    self._update_calibration_state("失败")
            else:
                self.log(f"标定响应解析失败: {response.hex()}")
                self._update_calibration_state("失败")
                
        except Exception as e:
            self.log(f"标定操作错误: {str(e)}")
            self._update_calibration_state("失败")

    def _update_calibration_state(self, state: str):
        self.calibration_state = state
        self.calib_state_var.set(state)
        if state == "成功":
            self.calib_state_label.configure(foreground="green")
        elif state == "失败":
            self.calib_state_label.configure(foreground="red")
        elif state == "标定中":
            self.calib_state_label.configure(foreground="orange")
        else:
            self.calib_state_label.configure(foreground="red")

    def parse_auto_receive_frame(self, frame: bytes) -> Optional[Dict]:
        if len(frame) < 5:
            return None
        
        """处理自动回传帧（aa56头）"""
        if len(frame) < 8 or frame[:2] != b"\xAA\x56":
            return None
        
        frame_body = frame[:-1]
        lrc = frame[-1]
        if self.calculate_lrc(frame_body) != lrc:
            self.log(f"自动回传LRC校验失败")
            return None
        return {
            "reserved": frame[2],
            "frame_len": int.from_bytes(frame[3:5], byteorder='little'),
            "error_code": frame[5],
            "data": frame[6:-1]
        }
    def parse_auto_receive_force_data(self, data: bytes) -> List[Dict]:
        """
        解析自动回传的分布力数据，严格按照以下顺序处理：
        大拇指(近节→中节→指尖→指甲) → 食指 → 中指 → 无名指 → 小拇指 → 掌心(1→8)
        每个传感器数据包含：合力(6字节) + 分布力(点数×3字节)
        """
        parsed = []
        current_offset = 0
    
        # 定义完整解析顺序
        parse_order = [
            # 大拇指：近节→中节→指尖→指甲
            ["大拇指近节", "大拇指中节", "大拇指指尖", "大拇指指甲"],
            # 食指：近节→中节→指尖→指甲
            ["食指近节", "食指中节", "食指指尖", "食指指甲"],
            # 中指：近节→中节→指尖→指甲
            ["中指近节", "中指中节", "中指指尖", "中指指甲"],
            # 无名指：近节→中节→指尖→指甲
            ["无名指近节", "无名指中节", "无名指指尖", "无名指指甲"],
            # 小拇指：近节→中节→指尖→指甲
            ["小拇指近节", "小拇指中节", "小拇指指尖", "小拇指指甲"],
            # 掌心：1→2→3→4→5→6→7→8
            ["掌心1", "掌心2", "掌心3", "掌心4", "掌心5", "掌心6", "掌心7", "掌心8"]
        ]
    
        self.log("\n===== 自动回传传感器数据解析 =====")
        # print(f"总接收数据长度: {len(data)}字节")
        print(f"解析顺序: 大拇指 → 食指 → 中指 → 无名指 → 小拇指 → 掌心")
    
        # 按组解析（每组对应一个手指/掌心区域）
        for group in parse_order:
            group_name = group[0].split()[0]  # 提取组名（如"大拇指"）
            print(f"\n----- {group_name}传感器组 -----")
        
            # 遍历组内传感器
            for sensor_name in group:
                # 跳过未连接的传感器
                if sensor_name not in self.connected_sensors:
                    print(f"  {sensor_name}: 未连接，跳过解析")
                    continue
            
                # 1. 解析传感器合力数据（6字节：Fx(2字节)+Fy(2字节)+Fz(2字节)）
                if current_offset + 6 > len(data):
                    print(f"  {sensor_name}: 合力数据不足6字节，终止解析")
                    return parsed
            
                # 提取合力数据
                force_data = data[current_offset:current_offset+6]
                current_offset += 6
            
                # 解析合力值（使用现有parse_total_force逻辑）
                sensor_force = self.parse_single_sensor_total_force(force_data, sensor_name)
                if not sensor_force:
                    print(f"  {sensor_name}: 合力数据解析失败")
                    continue
            
                # 2. 获取分布力点数并解析分布力数据
                point_count = self._get_cached_point_count(sensor_name)
                if point_count <= 0:
                    print(f"  {sensor_name}: 无效分布力点数({point_count})，跳过分布力解析")
                    parsed.append({
                        "sensor": sensor_name,
                        "total_force": sensor_force,
                        "distribution_force": []
                    })
                    continue
            
                # 掌心传感器特殊处理（限制最大点数）
                if sensor_name.startswith("掌心"):
                    actual_points = min(point_count, self.palm_sensor_limit)
                    distribution_data_len = actual_points * 3
                    print(f"  {sensor_name}: 原始点数={point_count}, 实际解析{actual_points}点, 分布力数据长度={distribution_data_len}字节")
                else:
                    distribution_data_len = point_count * 3
                    print(f"  {sensor_name}: 分布力点数={point_count}, 数据长度={distribution_data_len}字节")
            
                # 检查分布力数据是否足够
                if current_offset + distribution_data_len > len(data):
                    self.log(f"  {sensor_name}: 分布力数据不足(需要{distribution_data_len}字节, 剩余{len(data)-current_offset}字节)，终止解析")
                    return parsed
            
                # 提取并解析分布力数据
                distribution_data = data[current_offset:current_offset+distribution_data_len]
                current_offset += distribution_data_len
                distribution_parsed = self._parse_single_sensor_force(distribution_data, sensor_name)
            
                # 3. 汇总该传感器数据
                parsed.append({
                    "sensor": sensor_name,
                    "total_force": sensor_force,
                    "distribution_force": distribution_parsed,
                    "total_points": len(distribution_parsed)
                })
    
        # 解析完成后检查数据完整性
        # if current_offset < len(data):
            # print(f"\n解析完成，剩余未解析数据: {len(data)-current_offset}字节")
        # else:
            # print(f"\n解析完成，所有数据已处理完毕")
    
        return parsed
    
    def _get_cached_point_count(self, sensor_name: str) -> int:
        """获取缓存的传感器分布力点数"""
        # 从缓存中获取，如果不存在或缓存值为0，尝试返回默认值
        if sensor_name in self.distribution_points_cache:
            cached_value = self.distribution_points_cache[sensor_name]
            if cached_value > 0:
                return cached_value
    
        # 如果没有缓存或缓存值无效，尝试返回合理的默认值
        print(f"  警告: {sensor_name} 没有有效缓存的分布力点数，使用默认值")
        return 16  # 默认值，可根据实际情况调整

    def _parse_single_sensor_force(self, data: bytes, sensor_name: str) -> List[Dict]:
        """解析单个传感器的分布力数据"""
        parsed = []
        total_groups = len(data) // 3
        remainder = len(data) % 3
    
        if remainder != 0:
            print(f"  {sensor_name}: 数据长度不是3的倍数，剩余{remainder}字节将被忽略")
    
        for i in range(total_groups):
            offset = i * 3
            b1, b2, b3 = data[offset], data[offset+1], data[offset+2]
        
            # 转换为有符号值
            val1 = b1 if b1 <= 127 else b1 - 256
            val2 = b2 if b2 <= 127 else b2 - 256
            val3 = b3
        
            # 转换为物理值（×0.1N）
            scaled1 = round(val1 * 0.1, 1)
            scaled2 = round(val2 * 0.1, 1)
            scaled3 = round(val3 * 0.1, 1)
        
            parsed.append({
                "sensor": sensor_name,
                "index": i,
                "raw_bytes": (b1, b2, b3),
                "raw_hex": (f"0x{b1:02X}", f"0x{b2:02X}", f"0x{b3:02X}"),
                "converted": (val1, val2, val3),
                "scaled": (scaled1, scaled2, scaled3)
            })
    
        # 显示解析结果
        if parsed:
            # 计算最大值
            max_x = max(p["scaled"][0] for p in parsed)
            max_y = max(p["scaled"][1] for p in parsed)
            max_z = max(p["scaled"][2] for p in parsed)
        
            max_x_idx = next(i for i, p in enumerate(parsed) if p["scaled"][0] == max_x)
            max_y_idx = next(i for i, p in enumerate(parsed) if p["scaled"][1] == max_y)
            max_z_idx = next(i for i, p in enumerate(parsed) if p["scaled"][2] == max_z)
        
            self.log(f"  最大值: X={max_x}N(点{max_x_idx}), Y={max_y}N(点{max_y_idx}), Z={max_z}N(点{max_z_idx})")
        
            # 只显示前5个点的数据，避免日志过长
            point_count = self._get_cached_point_count(sensor_name)
            display_limit = point_count
            for i, group in enumerate(parsed[:display_limit]):
                print(
                f"  点{i:02d}: X={group['scaled'][0]:5.1f}N, Y={group['scaled'][1]:5.1f}N, Z={group['scaled'][2]:5.1f}N"
                )
        
            # if len(parsed) > display_limit:
                # print(f"  ... 省略后续{len(parsed)-display_limit}个点数据 ...")
    
        return parsed

    def parse_single_sensor_total_force(self, data: bytes, sensor_name: str) -> Optional[Dict]:
        """解析单个传感器的合力数据（6字节）"""
        if len(data) != 6:
            return None
    
        # 解析Fx/Fy/Fz（每轴2字节，使用低字节计算）
        fx_low = data[0]
        fy_low = data[2]
        fz_low = data[4]
    
        # 转换为有符号值
        fx_raw = fx_low if fx_low <= 127 else fx_low - 256
        fy_raw = fy_low if fy_low <= 127 else fy_low - 256
        fz_raw = fz_low  # Fz按无符号处理
    
        # 转换为物理值（×0.1N）
        fx_scaled = round(fx_raw * 0.1, 1)
        fy_scaled = round(fy_raw * 0.1, 1)
        fz_scaled = round(fz_raw * 0.1, 1)
    
        # 记录合力日志
        self.log(f"  {sensor_name}合力:")
        self.log(f"    Fx: {fx_scaled:5.1f}N (原始字节:0x{fx_low:02X})")
        self.log(f"    Fy: {fy_scaled:5.1f}N (原始字节:0x{fy_low:02X})")
        self.log(f"    Fz: {fz_scaled:5.1f}N (原始字节:0x{fz_low:02X})")
    
        return {
            "raw_bytes": data.hex().upper(),
            "converted": (fx_raw, fy_raw, fz_raw),
            "scaled": (fx_scaled, fy_scaled, fz_scaled)
        }
    
    def auto_receive_loop(self):
        """优化自动回传主循环，适配新的数据格式"""
        if not self.auto_receive_running or not self.ser or not self.ser.is_open:
            return
        try:
            if self.ser.in_waiting > 0:
                response = self.ser.read(self.ser.in_waiting)
                self.log(f"\n===== 收到自动回传数据 =====")
                self.log(f"原始数据: {response.hex().upper()[:64]}... (共{len(response)}字节)")
            
                # 查找帧头(AA56)
                frame_start = response.find(b"\xAA\x56")
                if frame_start == -1:
                    self.log("未找到有效帧头(AA56)")
                    self.root.after(10, self.auto_receive_loop)
                    return
            
                # 解析帧结构
                parsed_frame = self.parse_auto_receive_frame(response[frame_start:])
                if parsed_frame:
                    self.log(f"自动回传帧信息: 长度={parsed_frame['frame_len']}, 错误码=0x{parsed_frame['error_code']:02X}")
                
                    # 解析传感器数据（包含合力+分布力）
                    if parsed_frame["data"]:
                        self.parse_auto_receive_force_data(parsed_frame["data"])
                    else:
                        self.log("自动回传帧无有效数据")
        except Exception as e:
            self.log(f"自动回传处理错误: {str(e)}")
    
        # 继续循环
        self.root.after(5, self.auto_receive_loop)

    def start_auto_receive(self):
        if self.auto_receive_running:
            self.log("自动回传已开启")
            return
        self.write_addr_var.set("0017")
        self.write_data_var.set("01")
        self.write_registers()
        self.auto_receive_running = True
        self.log("开始自动回传接收...")
        self.auto_receive_loop()

    def stop_auto_receive(self):
        self.auto_receive_running = False
        if self.ser and self.ser.is_open:
            self.ser.flushInput()
            self.ser.flushOutput()
        self.write_addr_var.set("0017")
        self.write_data_var.set("00")
        self.write_registers()
        self.log("已停止自动回传")

if __name__ == "__main__":
    root = tk.Tk()
    app = HighSpeedCommBoard(root)
    root.mainloop()


