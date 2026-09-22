"""
实验名称：YOLO11 物品检测(dacong专用单类模型) + 中心坐标提取 + 串口发送(XH-1.25 UART/I2C 接口 TX2/RX2)
          ->  STM32F407 物体跟随  —— 语音助手版(2026-09-05)
实验平台：01Studio CanMV K230 / CanMV K230 mini
模型说明：本版本加载的是 dacong 专用单类模型(yolo11s, 640输入, int16量化,
          295正样本+150负样本训练, 含误检抑制), 只输出 dacong 一个类别(模型类别号0)。
          (如需6类模型: 用 main_6cls.py + yolo11s_6cls_640.kmodel; 如需pingzi单类:
          用 main_pingzi_备份.py + yolo11s_det_640_pingzi旧模型.kmodel)

【语音助手版相对原版新增的功能】
  0. 原视觉功能 100% 保留(检测/跟踪/坐标下发 C: 帧, 协议不变)。
  1. 板载麦克风持续做语音活动检测(VAD): 检测到一句人声(约≤2.8s)自动截取。
  2. 通过板载 2.4G WiFi 把 PCM 音频 POST 到百度短语音识别API(免费额度), 返回文字。
  3. 识别文字命中词表 -> 通过同一根 UART2 下发 V:START/V:STOP/V:ACC/V:DEC
     给 STM32F407, 与物理按键 K0(启停)/K1(调速) 作用等效:
         "启动/开始..."  -> V:START  (等效 K0 按下: 启动自动跟随)
         "停车/停止..."  -> V:STOP   (等效 K0 按下: 停车)
         "加速/快点..."  -> V:ACC    (等效 K1 按一次: 档位+1, 20/40/60/80 RPM)
         "减速/慢点..."  -> V:DEC    (等效 K1 按一次: 档位-1)
  4. 语音逻辑运行在独立线程(_thread), 识别/联网期间视觉照常跑, 不会丢帧停车。
  5. STM32 侧需要配套升级固件(新增 V: 指令解析, 详见交付目录 README 与 stm32_patch)。

【部署】
  1. 本文件即工作版 main.py: 备份板上 /sdcard/main.py 后, 把本文件放入 /sdcard/ 上电自运行;
     也可用 CanMV IDE 直接运行调试。
  2. 必须先在文件顶部"语音助手配置"填入: WIFI_SSID / WIFI_PWD / BAIDU_TOKEN
     (百度 access_token 申请方法见交付目录 README.md)。
  3. 无网络/未配置时自动只跑视觉, 不影响原功能。

【已知注意点】
  * 语音触发靠音量门限, 电机/环境噪声大时会变迟钝或误触发, 阈值调法见 README。
  * 目标丢失后 STM32 处于搜索态, 此时喊"启动"会被 STM32 忽略(防无目标乱跑),
    需目标重新出现在画面后再喊。
  * 本文件与 01Studio v1.8 固件 + 配套 libs(PipeLine/YOLO11)配套。
"""

from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11
from libs.Utils import *
from media.sensor import *
from machine import UART, FPIOA
import os, sys, gc
import time

# ================= 语音助手: 模块可用性预检(缺线程/数组时仅提示) =================
_thread_ok = False
try:
    import _thread                       # CanMV K230 固件自带 _thread
    _thread_ok = True
except Exception:
    pass
try:
    import struct                       # 16bit PCM 字节解析(部分固件 array 无 frombytes)
except Exception:
    struct = None

kmodel_path = "/sdcard/yolo11s_det_640.kmodel"
# ================= 模型配置: dacong 专用单类模型 =================
labels = {0: 'dacong'}   # 本模型只输出 dacong 一个类别
# 画框颜色(与 labels 序号一一对应)
BOX_COLORS = [(0, 255, 0)]
PROTO_CLASS_ID = 4       # 上报给 STM32 的类别号(数值沿用6类协议中的4; 本模型实为dacong, STM32如按类别区分请自行调整)
model_input_size = [640, 640]
# 显示模式，可以选择"hdmi"、"lcd3_5"(3.5寸mipi屏)和"lcd2_4"(2.4寸mipi屏)
display = "lcd3_5"

# ================= 串口与协议配置 =================
UART_ID        = 2            # 串口2（UART.UART2），XH-1.25-4P UART/I2C 接口
UART_BAUD      = 115200       # 波特率，需与 STM32F407 USART2 一致
UART_PIN_TX    = 11           # TX2 (引脚11, GPIO11) -> UART2_TXD -> STM32 PA3
UART_PIN_RX    = 12           # RX2 (引脚12, GPIO12) -> UART2_RXD <- STM32 PA2(可选)
PROTOCOL       = "ASCII"      # "ASCII" 或 "BIN"
SEND_MODE      = "ABS"        # "ABS"=发绝对中心坐标(默认); "ERR"=直接发偏差(err_x=cx-400, err_y=240-cy)，STM32省一步换算
SEND_AREA_RATIO = True        # 附带发送 A=目标框面积占画面比例(千分比0~1000)，STM32做远近/俯仰控制；False=不带A(格式与旧版一致)
MAX_JUMP       = 200          # 目标中心单帧跳变超过该像素数=误检尖峰并丢弃该帧；0=关闭
TRACK_LOCK_DIST = 250         # 目标延续锁定：新帧优先匹配距上一帧目标中心250px内的检测(防多目标串跟)；0=关闭
SEND_EVERY_N   = 1            # 每 N 帧发送一次目标坐标；串口发不过来时可调大(如2)
LOST_SEND_EVERY_N = 20        # 目标丢失时，每 N 帧补发一次丢失帧(保持心跳)
LOST_GRACE       = 3          # 目标连续丢失 N 帧后才发 C:-1(防单帧闪烁导致电机急停)
OSD_TEXT         = False      # LCD上叠加坐标/目标名文字(v1.8固件文字接口已弃用会刷屏，默认关；需要屏显可设True)
TRACK_LARGEST  = True         # True: 跟踪面积最大的目标; False: 跟踪置信度最高
DRAW_TRACKED_ONLY = True      # 屏幕只画跟踪目标一个框(画面干净)；False=画所有检测到的框
TRACK_CLASS    = 0            # dacong 单类模型: 只跟踪 dacong(类别号恒为0)。改 -1 无意义(仅一类)
MIN_AREA       = 0            # 过滤面积小于该值(像素)的误检，0 表示不过滤
SMOOTH_ENABLE  = True         # 中心坐标平滑滤波，减少抖动，电机控制更稳
SMOOTH_ALPHA   = 0.4          # 平滑系数 0~1，越大越跟手，越小越稳
DEBUG_PRINT    = False        # 终端打印发送内容；调试时需要观察可改回 True(生产建议False提性能)
FPS_PRINT_EVERY = 20          # 每 N 帧打印一次 FPS，减少刷屏
GC_EVERY_N     = 10           # 每 N 帧执行一次内存回收(每帧回收反而增加卡顿)
# =================================================

# ================= 语音助手配置(新增: 云端识别+等效K0/K1) =================
VOICE_ENABLE    = True        # 总开关; False 则本文件行为与原 main.py 完全一致
WIFI_SSID       = "your-2.4g-ssid"          # ★ 2.4G WiFi 账号(必填, 板载WiFi不支持5G/混合)
WIFI_PWD        = "your-wifi-password"          # ★ 2.4G WiFi 密码(必填)
BAIDU_TOKEN     = "your-baidu-access-token"  # ★ 百度"短语音识别"access_token(30天有效, 2026-09-05 填入, 过期重换)
BAIDU_CUID      = "k230_car"  # 设备标识(任意)
BAIDU_HOST      = "vop.baidu.com"
BAIDU_DEV_PID   = 1537        # 1537=普通话(支持16k/8k pcm)
VOICE_DEBUG     = True        # True: 打印 音量/识别文本/下发指令 便于调试; 稳定后改 False

# --- VAD 语音活动检测参数(语音不灵时先调这里, 详见 README) ---
VAD_START_MULT  = 3.0         # 人声能量 >= 底噪估计*MULT 判定"开始说话"(越大越不易误触发)
VAD_SIL_MULT    = 2.0         # 能量低于 底噪*SIL_MULT 视为停顿(判断一句话是否说完)
VAD_START_ABS   = 1200        # 绝对启动下限(高于远处人声~800; 靠近30cm说话即可触发)
VAD_SIL_ABS     = 300         # 绝对静音判定下限
VAD_ABORT_MS    = 200         # 触发后若连续这么久没有真正语音 => 判定为误触发丢弃
VAD_TAIL_MS     = 500         # 句尾静音达该时长 => 一句话结束, 送识别
VAD_MAX_MS      = 1800        # 单句最长录音(超出强切; 指令词都很短, 不必等太长)
VAD_MIN_MS      = 120         # 有效语音太短(噪声毛刺)则丢弃
VOICE_COOLDOWN_S = 1.2        # 下发指令后冷却(秒), 防止回声/余音连发
VOICE_CHUNK_DIV  = 25         # 采样块时长=1000/该值 ms: 25=40ms/块(默认), 10=100ms/块
                              # 语音开启时视觉FPS低, 可切换 25<->10 对比哪个视觉更快

# --- 触发模式: auto=全自动VAD监听(随时喊) / key=按键对讲(推荐答辩演示用) ---
VOICE_TRIGGER   = "key"       # "key": 按一下开始录音, 再按一下结束识别(平时麦克风关闭, 视觉不掉帧)
KEY_PIN         = 21          # 板载按键 GPIO(01Studio K230: KEY=GPIO21, 按下为低电平)
VOICE_KEY_MAX_MS = 2000      # 按键模式单次录音上限(ms), 超时自动结束送识别
VOICE_KEY_MIN_MS = 200        # 短于该时长视为误触, 不识别

# --- F103 蜂鸣器提示(经 F407 转发; 时长整秒协议, 0.3s 用"响1秒+0.3s后停"实现) ---
VOICE_BEEP      = True        # 按键对讲时响蜂鸣器: 开始前响(准备说话), 结束后响(开始识别)
BEEP_START_MS   = 300         # 开始提示音时长(ms): 哔声响完才开始收音, 听到"哔"即可开口
BEEP_END_MS     = 300         # 结束提示音时长(ms): 录音结束(按键或2s自动超时)后响, 提示开始识别

# --- 抗噪决策参数(答辩等嘈杂环境建议 VOICE_CONFIRM_2X=True) ---
VOICE_MAX_TEXT     = 10       # 识别文本去标点后超过该字数(闲聊长句)一律忽略
VOICE_MIN_VOICED_PCT = 20     # 一句录音里"真语音"时长占比(%)低于此值判定为噪声, 丢弃
VOICE_SNR_MIN      = 4        # 句内平均语音能量须 >= 底噪*该倍数, 否则视为远距离/混响丢弃
VOICE_SNR_ABS      = 700     # 平均语音能量的绝对下限(低于=远处人声/含糊声, 不送识别)
VOICE_CONFIRM_2X   = True     # True: 同一指令连续说两次(时间窗内)才下发, 可大幅防环境误触发(答辩演示推荐)
VOICE_CONFIRM_WIN_S = 8       # 两次确认的最大间隔(秒)

# --- 语音指令词表: 识别文本"包含"任一关键词即命中; 按 CMD_PRIORITY 顺序匹配 ---
# 注意: 每项必须写成元组 (带逗号!), 如 ("加速",) 或 ("加速","快点","提速");
#       写 ( "加速" ) 不带逗号 = 字符串, 会按单个字匹配(导致"速"同时命中加速/减速)
CMD_PRIORITY = ["STOP", "START", "DEC", "ACC"]   # 冲突时 停 > 启 > 减 > 加
VOICE_WORDS = {
    "START": ("开始",),          # 想加同音/同义说法: ("开始","启动","出发","开跑")
    "STOP":  ("刹车",),          # ("刹车","停车","停止","停下","急停")
    "ACC":   ("加速",),          # ("加速","快点","快一点","提速","升档")
    "DEC":   ("减速",),          # ("减速","慢点","慢一点","降速","降档")
}
VOICE_CMD_UART = {"START": "V:START\n", "STOP": "V:STOP\n",
                  "ACC": "V:ACC\n", "DEC": "V:DEC\n"}
# =================================================

if display == "hdmi":
    display_mode = "hdmi"
    display_size = [1920, 1080]
elif display == "lcd3_5":
    display_mode = "st7701"
    display_size = [800, 480]
elif display == "lcd2_4":
    display_mode = "st7701"
    display_size = [640, 480]

rgb888p_size = [800, 480]   # 图像处理分辨率(与开发板一致, 如 800x480)

pl = PipeLine(
    rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode
)

if display == "lcd2_4":
    pl.create(sensor=Sensor(id=2, width=1280, height=960))
else:
    pl.create(sensor=Sensor(id=2, width=1920, height=1080))

display_size = pl.get_display_size()

confidence_threshold = 0.4  # 置信度(实测目标常在0.5附近抖动，降到0.4更稳；误检多可调回0.5)
yolo = YOLO11(
    task_type="detect",
    mode="video",
    kmodel_path=kmodel_path,
    labels=labels,
    rgb888p_size=rgb888p_size,
    model_input_size=model_input_size,
    display_size=display_size,
    conf_thresh=confidence_threshold,
    debug_mode=0,
)
yolo.config_preprocess()


# -------------------- 串口初始化（与 01Studio 串口实验写法完全一致） --------------------
def uart_init(baudrate=UART_BAUD):
    """FPIOA 引脚复用 + 初始化 UART2（TX2/RX2），返回 UART 对象"""
    fpioa = FPIOA()
    fpioa.set_function(UART_PIN_TX, FPIOA.UART2_TXD)   # 设置引脚11(TX2)为UART2的TXD
    fpioa.set_function(UART_PIN_RX, FPIOA.UART2_RXD)   # 设置引脚12(RX2)为UART2的RXD
    uart = UART(UART.UART2, baudrate)                  # 初始化UART2
    return uart


uart = None
uart_lock = None      # 语音线程与视觉线程共用 UART 的写锁(语音开启后赋值)
try:
    uart = uart_init()
    uart.write("K")   # 握手测试数据：STM32 串口助手应能收到 'K'
    print("UART%d init OK (TX=GPIO%d, RX=GPIO%d), baud=%d" %
          (UART_ID, UART_PIN_TX, UART_PIN_RX, UART_BAUD))
except Exception as e:
    uart = None
    print("UART init failed:", e, "-> 摄像头画面仍可显示，但不发送数据")

# ============================================================================
#  语音助手: 云端识别 + 等效 K0/K1 指令(全部新增, 运行于独立线程, 不影响视觉帧率)
# ============================================================================

def voice_beep_ms(ms):
    """控制 F103 蜂鸣器响 ms 毫秒(F103 指令为整秒, 用"响1秒+到时发停"实现短哔)

    B:1 = 响1秒(最坏兜底只多响约0.3s); 到 ms 后发 B:0 提前停。调用期间阻塞 ms。"""
    if not VOICE_BEEP or ms <= 0:
        return
    voice_uart_write("B:1\n")          # F407 -> F103: 蜂鸣器响(1秒上限)
    time.sleep_ms(ms)
    voice_uart_write("B:0\n")          # 提前停 => 实际响约 ms 毫秒


def voice_uart_write(line):
    """带锁的串口发送(与视觉 send_frame 互斥, 避免 C: 与 V: 帧字节交错)"""
    if uart is None:
        print("[VOICE] uart not ready, drop:", line.strip())
        return
    if uart_lock is not None:
        uart_lock.acquire()
    try:
        uart.write(line)
    except Exception as e:
        print("[VOICE] uart write err:", e)
    finally:
        if uart_lock is not None:
            try:
                uart_lock.release()
            except Exception:
                pass


def voice_wifi_connect(timeout_s=20):
    """连接 2.4G WiFi(STA), 成功返回 wlan 对象, 失败返回 None"""
    try:
        import network
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        if wlan.isconnected():
            return wlan
        print("[VOICE] WiFi connecting: %s ..." % WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PWD)          # 官方例程: 约15s超时
        t0 = time.time()
        while not wlan.isconnected() and time.time() - t0 < timeout_s:
            time.sleep_ms(200)
        if wlan.isconnected():
            print("[VOICE] WiFi connected:", wlan.ifconfig())
            return wlan
        print("[VOICE] WiFi connect timeout")
        return None
    except Exception as e:
        print("[VOICE] WiFi err:", e)
        return None


def voice_dechunk(buf):
    """把 HTTP chunked 分块传输的响应体还原为纯数据(bytes)。

    实测百度的 server_api 响应带 chunked 尾帧(<hex>\\r\\n<data>\\r\\n ... 0\\r\\n\\r\\n),
    直接 decode 会在 JSON 后面残留垃圾导致 json 解析失败。"""
    out = bytearray()
    i = 0
    n = len(buf)
    while i < n:
        j = buf.find(b"\r\n", i)
        if j < 0:
            break
        line = buf[i:j]
        semi = line.find(b";")            # 可能有扩展参数 "1a;ext=..."
        if semi >= 0:
            line = line[:semi]
        try:
            size = int(line.strip(), 16)
        except Exception:
            break
        i = j + 2
        if size == 0:                     # 结束块
            break
        if i + size > n:
            break
        out += buf[i:i + size]
        i += size
        if buf[i:i + 2] == b"\r\n":
            i += 2
    return bytes(out)


def voice_http_post(host, path, body, extra_headers="", timeout=12):
    """极简 HTTP/1.1 POST(纯 socket, 无TLS), 返回响应体 str; 失败返回 None

    处理两种响应: Content-Length 收满 / chunked 分块解码; 该固件 socket recv
    偶发"提前返回/超时", 收不满就重试补齐, 避免 JSON 被截断。"""
    try:
        import socket
        ai = socket.getaddrinfo(host, 80)
    except Exception as e:
        print("[VOICE] DNS fail:", e)
        return None
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect(ai[0][4])
        req = ("POST %s HTTP/1.1\r\nHost: %s\r\n%sContent-Length: %d\r\n"
               "Connection: close\r\n\r\n" % (path, host, extra_headers, len(body)))
        sock.sendall(req.encode("utf-8") + body)

        # ---- 收头部(至 \r\n\r\n) ----
        buf = b""
        while b"\r\n\r\n" not in buf:
            if len(buf) > 8192:
                print("[VOICE] HTTP header too big")
                return None
            try:
                b = sock.recv(512)
            except Exception:
                break                     # 没等到头也退出, 下面统一判
            if not b:
                break
            buf += b
        idx = buf.find(b"\r\n\r\n")
        if idx < 0:
            print("[VOICE] HTTP: no header terminator, got %d bytes" % len(buf))
            return None
        head = buf[:idx]
        payload = buf[idx + 4:]           # 头之后可能已带部分/全部响应体

        code = 0
        try:
            code = int(head.split(b"\r\n")[0].split(b" ")[1])
        except Exception:
            pass
        if code != 200:
            print("[VOICE] HTTP code:", code)
            return None

        # 解析头: Content-Length / Transfer-Encoding
        clen = None
        chunked = False
        for ln in head.split(b"\r\n"):
            low = ln.lower()
            if low.startswith(b"content-length:"):
                try:
                    clen = int(ln.split(b":", 1)[1].strip())
                except Exception:
                    clen = None
            elif low.startswith(b"transfer-encoding:") and b"chunked" in low:
                chunked = True

        # ---- 收响应体 ----
        if not chunked and clen is not None:
            retry = 30
            while len(payload) < clen and retry > 0:
                need = clen - len(payload)
                try:
                    b = sock.recv(need if need < 1024 else 1024)
                except Exception:
                    retry -= 1
                    time.sleep_ms(15)     # 等剩余数据到达后再收
                    continue
                if not b:
                    break
                payload += b
                retry = 30                # 有进展就重置重试计数
        else:
            while True:
                try:
                    b = sock.recv(1024)
                except Exception:
                    break
                if not b:
                    break
                payload += b

        # ---- 还原响应体: chunked 解码 / Content-Length 截断 ----
        if chunked:
            payload = voice_dechunk(payload)
        elif clen is not None:
            if len(payload) < clen:
                print("[VOICE] HTTP body incomplete %d/%d" % (len(payload), clen))
            payload = payload[:clen]

        if VOICE_DEBUG:
            print("[VOICE] HTTP resp len=%d chunked=%s head=%s" %
                  (len(payload), chunked, head.split(b"\r\n")[0]))
        try:
            return payload.decode("utf-8")
        except Exception:
            return None
    except Exception as e:
        print("[VOICE] HTTP fail:", e)
        return None
    finally:
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass


def voice_baidu_recognize(pcm_bytes, rate):
    """把 16bit PCM(rate=16000 或 8000) 送百度短语音识别, 返回文字; 失败返回 ''"""
    if not BAIDU_TOKEN or not pcm_bytes:
        return ""
    path = "/server_api?dev_pid=%d&cuid=%s&token=%s" % (BAIDU_DEV_PID, BAIDU_CUID, BAIDU_TOKEN)
    body = pcm_bytes
    headers = "Content-Type: audio/pcm;rate=%d\r\n" % rate
    resp = voice_http_post(BAIDU_HOST, path, body, headers, timeout=12)
    if not resp:
        return ""
    try:
        import json
        try:
            obj = json.loads(resp)
        except Exception:
            # 兜底: 某些代理会带 chunked 尾帧/多余换行, 裁到最后一个 '}' 再解析一次
            i = resp.rfind("}")
            if i <= 0:
                raise
            obj = json.loads(resp[:i + 1])
    except Exception as e:
        print("[VOICE] resp parse err:", e, resp[:100])
        print("[VOICE] resp tail:", repr(resp[-30:]))   # 排错: 看尾部是否有多余字节
        return ""
    err_no = obj.get("err_no")
    if err_no not in (0, None):
        print("[VOICE] baidu err_no=%s err_msg=%s" % (err_no, obj.get("err_msg", "")))
        return ""
    try:
        res = obj.get("result") or []
        return res[0] if res else ""
    except Exception:
        return ""


def voice_clean_text(text):
    """只保留中英文/数字, 去掉标点与空格(百度结果常带句号逗号问号)"""
    out = []
    for ch in text:
        o = ord(ch)
        if (0x4e00 <= o <= 0x9fff) or (0x61 <= o <= 0x7a) or \
           (0x41 <= o <= 0x5a) or (0x30 <= o <= 0x39):
            out.append(ch)
    return "".join(out)


def voice_match_cmd(text):
    """识别文本 -> 指令标签(START/STOP/ACC/DEC), 未命中或可疑返回 None

    抗噪策略:
      1) 去标点后总字数 > VOICE_MAX_TEXT 的闲聊长句不理会(指令词都是短词);
      2) 一句里同时出现两个不同指令词(如"加速停车")判定为可疑, 不动作。"""
    if not text:
        return None
    t = voice_clean_text(text)
    if not t:
        return None
    if len(t) > VOICE_MAX_TEXT:
        if VOICE_DEBUG:
            print("[VOICE] 文本过长(%d字)按闲聊忽略: %s" % (len(t), t))
        return None
    tags = []
    for tag in CMD_PRIORITY:
        ws = VOICE_WORDS[tag]
        if isinstance(ws, str):       # 容错: 误写成 ("加速") 无逗号时当整词处理
            ws = (ws,)
        for w in ws:
            if len(w) < 2:            # 跳过单字, 防止"速"之类误命中多组
                continue
            if w in t:
                tags.append(tag)
                break
    if not tags:
        return None
    first = tags[0]
    for x in tags:
        if x != first:
            if VOICE_DEBUG:
                print("[VOICE] 一句含多个指令词, 判定可疑忽略: %s" % t)
            return None
    return first


_VAD_STRIDE = 4          # 能量抽样步长: 右声道每4个采样取1个, 解析开销降~4倍(只影响VAD估算)


def voice_chunk_energy(data):
    """双声道 int16 小端 -> 右声道平均幅度(抽样估算, 仅VAD用, 快)"""
    usable = (len(data) // 4) * 4
    n = usable // 2                               # 总 int16 采样数
    if n < 8:
        return 0
    vals = struct.unpack("<%dh" % n, data[:usable])
    s = 0
    c = 0
    i = 1
    while i < n:
        v = vals[i]
        s += -v if v < 0 else v
        c += 1
        i += 2 * _VAD_STRIDE                      # 右声道索引1,3,5.. 每隔_STRIDE个取1
    return s // c if c else 0


def voice_extract_right_bytes(stereo):
    """从原始双声道 bytes 抽右声道 16bit PCM(板载麦克风在右路)。仅收句后调用一次"""
    usable = (len(stereo) // 4) * 4
    n = usable // 2
    if n < 2:
        return b""
    vals = struct.unpack("<%dh" % n, stereo[:usable])
    # 该固件不支持步进切片(vals[1::2] 会报错), 用 range 循环;
    # 且单次 pack 实参不能超 65535, 长录音分段打包
    out = bytearray()
    seg = []
    for i in range(1, n, 2):
        seg.append(vals[i])
        if len(seg) >= 8000:
            out += struct.pack("<%dh" % len(seg), *seg)
            seg = []
    if seg:
        out += struct.pack("<%dh" % len(seg), *seg)
    return bytes(out)


def voice_decimate16k_bytes(pcm_in, rate):
    """把 rate Hz 的 int16 小端 PCM bytes 线性抽取为 16kHz PCM bytes(44.1k 兜底用)"""
    n = len(pcm_in) // 2
    if n == 0:
        return b""
    vals = struct.unpack("<%dh" % n, pcm_in)
    stepf = float(rate) / 16000.0
    out = bytearray()
    acc = []
    pos = 0.0
    m = len(vals)
    while int(pos) < m:
        acc.append(vals[int(pos)])
        if len(acc) >= 800:                       # 分片 pack, 避免单次实参过多
            out += struct.pack("<%dh" % len(acc), *acc)
            acc = []
        pos += stepf
    if acc:
        out += struct.pack("<%dh" % len(acc), *acc)
    return bytes(out)


_media_attempted = False

def voice_ensure_media():
    """MediaManager 可能已被视觉管线初始化, 重复 init 报错则忽略(绝不在语音线程 deinit)"""
    global _media_attempted
    if _media_attempted:
        return
    _media_attempted = True
    try:
        from media.media import MediaManager
        MediaManager.init()
    except Exception:
        pass                     # 已初始化过, 忽略


def voice_open_stream(rate):
    """按 rate 打开输入流(channels=2, 麦克风右声道)。成功返回 (p, stream), 失败 (None, None)"""
    from media.pyaudio import PyAudio, paInt16, RIGHT, AUDIO_3A_ENABLE_ANS
    chunk = rate // VOICE_CHUNK_DIV          # 每块采样帧数(默认25=40ms/块)
    p = PyAudio()
    voice_ensure_media()
    try:
        p.initialize(chunk)      # 部分固件(v1.8等)无此方法, 无则跳过, 不影响 open
    except Exception:
        pass
    stream = p.open(format=paInt16, channels=2, rate=rate,
                    input=True, frames_per_buffer=chunk)
    try:
        stream.volume(85, RIGHT)             # 板载麦克风在右声道
    except Exception:
        pass
    try:
        stream.enable_audio3a(AUDIO_3A_ENABLE_ANS)   # 自动噪声抑制
    except Exception:
        pass
    return p, stream


def voice_key_debounce(key, want):
    """消抖: 引脚电平为 want(0=按下) 且 ~12ms 后仍为 want 才算有效"""
    if key.value() == want:
        time.sleep_ms(12)
        return key.value() == want
    return False


def voice_close_audio(p, stream):
    try:
        if stream is not None:
            stream.stop_stream()
            stream.close()
    except Exception:
        pass
    try:
        if p is not None:
            p.terminate()
    except Exception:
        pass


def voice_utterance_to_text(chunks_raw, act_rate):
    """按键录音(原始双声道块) -> 抽右声道 -> (降采样) -> 云端识别, 返回文本"""
    try:
        stereo = b"".join(chunks_raw)
        pcm = voice_extract_right_bytes(stereo)          # 右声道(板载麦在右路)
        if act_rate == 16000 or act_rate == 8000:
            send_rate = act_rate
        else:
            pcm = voice_decimate16k_bytes(pcm, act_rate)
            send_rate = 16000
        text = voice_baidu_recognize(pcm, send_rate)
        if VOICE_DEBUG:
            print("[VOICE] 识别结果: %r" % text)
        return text
    except Exception as e:
        print("[VOICE] 识别流程异常:", e)
        return ""


def voice_engine_key():
    """按键对讲模式(推荐答辩): 按一下开始录音, 再按一下结束并识别。

    平时不开麦克风 -> 不影响视觉帧率; 录音内容=按键期间说的话, 无误触发;
    识别到指令直接下发, 不需要两次确认。"""
    print("[VOICE] 按键对讲模式: 按 KEY(GPIO%d)一下开始说话, 再按一下结束识别" % KEY_PIN)
    if not BAIDU_TOKEN:
        print("[VOICE] BAIDU_TOKEN 未配置 -> 语音助手停用(视觉照常)。申请方法见 README")
        return
    if not WIFI_SSID:
        print("[VOICE] WIFI_SSID 未配置 -> 语音助手停用(视觉照常)")
        return
    try:
        from machine import Pin, FPIOA
        fpioa = FPIOA()
        fpioa.set_function(KEY_PIN, getattr(FPIOA, "GPIO%d" % KEY_PIN))
        key = Pin(KEY_PIN, Pin.IN, Pin.PULL_UP)
    except Exception as e:
        print("[VOICE] KEY 初始化失败, 按键模式不可用:", e)
        return

    open_fail = 0
    while True:
        p = None
        stream = None
        try:
            # ---- 待机: 等按下 KEY(几乎不耗CPU) ----
            wlan = voice_wifi_connect(20)
            if wlan is None:
                time.sleep_ms(8000)
                continue
            while True:
                if voice_key_debounce(key, 0):       # 第一次按下 => 开始
                    break
                time.sleep_ms(20)

            # ---- 开始提示音: 哔(0.3s)响完才开始收音(此时麦克风还没开, 不会录到蜂鸣声) ----
            voice_beep_ms(BEEP_START_MS)
            print("[VOICE] == 开始收音: 请说话, 说完再按一下KEY ==")

            # ---- 打开麦克风 + 实测真实采样率 ----
            rate = 0
            last_err = "unknown"
            for cand in (44100, 16000, 8000):
                try:
                    p, stream = voice_open_stream(cand)
                    if stream is not None:
                        rate = cand
                        break
                except Exception as e:
                    last_err = repr(e)
                    p = None
                    stream = None
            if stream is None:
                open_fail += 1
                print("[VOICE] 麦克风打开失败(%s) 第%d次" % (last_err, open_fail))
                if open_fail >= 3:
                    print("[VOICE] 提示: 请给板子断电重启后再运行")
                    open_fail = 0
                time.sleep_ms(3000)
                continue
            open_fail = 0
            chunk_frames = rate // VOICE_CHUNK_DIV
            # 实测结论: 请求44.1k时设备真实即44.1k(多轮验证); 计时反推会被线程调度干扰
            # (曾误测成25491Hz导致变调), 故 44.1k/16k 直接采信请求值, 其余才计时反推。
            if rate == 44100:
                act_rate = 44100
            elif rate == 16000:
                act_rate = 16000
            else:
                t0 = time.ticks_ms()
                for _ in range(20):
                    stream.read()
                el_ms = time.ticks_diff(time.ticks_ms(), t0)
                if el_ms > 0:
                    act_rate = int(round(20 * chunk_frames * 1000.0 / el_ms))
                else:
                    act_rate = rate
                for std in (8000, 16000, 22050, 32000, 44100, 48000):
                    if abs(act_rate - std) < std * 0.08:
                        act_rate = std
                        break
            if VOICE_DEBUG:
                print("[VOICE] mic on: 请求%dHz 使用%dHz" % (rate, act_rate))
            print("[VOICE] == 可以说话了, 说完再按一下KEY结束 ==")

            # ---- 录音: 松开后再按下 = 结束; 或超时自动结束 ----
            chunks = []
            key_down = (key.value() == 0)            # 启动瞬间按键可能仍按着
            t_start = time.ticks_ms()
            ended = False
            while not ended:
                cur = (key.value() == 0)
                if not cur:
                    key_down = False                 # 已松开
                elif cur and not key_down:
                    time.sleep_ms(15)                # 二次消抖
                    if key.value() == 0:
                        ended = True                 # 第二次按下 => 结束
                        break
                try:
                    data = stream.read()
                except Exception:
                    ended = True
                    break
                if data and len(data) >= 8:
                    chunks.append(data)
                if time.ticks_diff(time.ticks_ms(), t_start) >= VOICE_KEY_MAX_MS:
                    ended = True                     # 超时保护
                    break
            dur_ms = time.ticks_diff(time.ticks_ms(), t_start)
            voice_close_audio(p, stream)
            p = None
            stream = None
            if VOICE_DEBUG:
                print("[VOICE] == 录音结束 %dms, %d块, 送识别 ==" % (dur_ms, len(chunks)))
            if dur_ms < VOICE_KEY_MIN_MS or not chunks:
                print("[VOICE] 录音太短, 忽略(误触?)")
                time.sleep_ms(600)
                continue

            # ---- 结束提示音(按键结束 或 2s自动超时结束 都会走到这里): 哔(0.3s)提示开始识别 ----
            voice_beep_ms(BEEP_END_MS)

            # ---- 识别 + 匹配 + 下发(按键=有意说话, 直接执行) ----
            text = voice_utterance_to_text(chunks, act_rate)
            tag = voice_match_cmd(text)
            if tag is not None:
                voice_uart_write(VOICE_CMD_UART[tag])
                print("[VOICE] 下发指令: %s" % VOICE_CMD_UART[tag].strip())
            else:
                print("[VOICE] 未命中指令词(可调 VOICE_WORDS): %s" % text)
            time.sleep_ms(800)                       # 间隔, 防误触连发
        except Exception as e:
            print("[VOICE] key-engine error:", e)
            voice_close_audio(p, stream)
            p = None
            stream = None
            time.sleep_ms(3000)


def voice_engine():
    """语音助手线程主循环: VAD 采集 -> 百度识别 -> V:指令下发。

    语音活动检测为纯能量门限+自适应底噪:
      * 平时不断估计底噪 floor(静止/电机声);
      * 块能量 > max(floor*VAD_START_MULT, VAD_START_ABS) 判定开口;
      * 尾静音 VAD_TAIL_MS 或超 VAD_MAX_MS 判定一句话结束。
    """
    print("[VOICE] voice engine thread start")
    if not BAIDU_TOKEN:
        print("[VOICE] BAIDU_TOKEN 未配置 -> 语音助手停用(视觉照常)。申请方法见 README")
        return
    if not WIFI_SSID:
        print("[VOICE] WIFI_SSID 未配置 -> 语音助手停用(视觉照常)。填入板载2.4G WiFi 账号")
        return

    # 按键对讲模式: 走独立引擎(平时不耗CPU/不影响视觉), 否则走下方全自动VAD
    if VOICE_TRIGGER == "key":
        voice_engine_key()
        return

    open_fail = 0                    # 连续麦克风打开失败计数(用于提示断电重启)
    last_tag = None                  # 两次确认模式: 上一次命中的指令
    last_tag_ms = 0                  # 上一次命中的时间

    while True:
        stream = None
        p = None
        try:
            # ---------- 1. WiFi ----------
            wlan = voice_wifi_connect(20)
            if wlan is None:
                time.sleep_ms(8000)
                continue
            # ---------- 2. 打开麦克风(官方例程用44.1k; 本固件可能忽略请求率, 见2.5校准) ----------
            rate = 0
            last_err = "unknown"
            for cand in (44100, 16000, 8000):
                try:
                    p, stream = voice_open_stream(cand)
                    if stream is not None:
                        rate = cand
                        break
                except Exception as e:
                    last_err = repr(e)
                    p = None
                    stream = None
            if stream is None:
                open_fail += 1
                print("[VOICE] 麦克风打开失败(%s), 8s后重试(第%d次)" % (last_err, open_fail))
                if open_fail >= 3:
                    print("[VOICE] 提示: 音频通道可能被占用(常见于IDE中断/软重启后),"
                          " 请给板子断电重启后再运行")
                    open_fail = 0
                time.sleep_ms(8000)
                continue
            open_fail = 0

            # ---------- 2.5 确定真实采样率 ----------
            # 实测结论: 请求44.1k时设备真实即44.1k(多轮验证); 请求16k时设备反而按44.1k采
            # (曾导致录音快2.7倍)。计时反推会被线程调度干扰(曾误测25491Hz), 故:
            #   请求44.1k -> 采信44.1k; 请求16k -> 仍按44.1k处理; 其余才计时反推。
            chunk_frames = rate // VOICE_CHUNK_DIV   # 与 voice_open_stream 的块大小保持一致
            if rate == 44100:
                act_rate = 44100
            elif rate == 16000:
                act_rate = 44100        # 该固件请求16k时真实仍是44.1k(历史实测)
            else:
                cal_n = 30
                t0 = time.ticks_ms()
                for _ in range(cal_n):
                    stream.read()
                el_ms = time.ticks_diff(time.ticks_ms(), t0)
                if el_ms > 0:
                    act_rate = int(round(cal_n * chunk_frames * 1000.0 / el_ms))
                else:
                    act_rate = rate
                for std in (8000, 16000, 22050, 32000, 44100, 48000):
                    if abs(act_rate - std) < std * 0.08:   # 靠到常见标准率
                        act_rate = std
                        break
            chunk_ms = max(5, int(round(chunk_frames * 1000.0 / act_rate)))
            need_dec = act_rate not in (8000, 16000)   # 真实率不是16k/8k => 降采样
            if VOICE_DEBUG:
                print("[VOICE] mic on: 请求%dHz 使用%dHz, 每块~%dms%s" %
                      (rate, act_rate, chunk_ms,
                       " -> 降采样到16k" if need_dec else ""))

            # ---------- 3. 逐块监听 + VAD ----------
            sil_abs = VAD_SIL_ABS
            floor = VAD_START_ABS           # 底噪初值
            chunks_raw = []                 # 原始双声道块列表(收句后统一抽右声道, 省CPU)
            state = 0                       # 0=待机 1=收音
            voiced_ch = 0
            voice_sum_e = 0                 # 语音块能量累加(算句内平均, 做信噪比门限)
            idle_ch = 0
            total_ch = 0
            last_cmd_ms = time.ticks_ms()
            cooldown_ms = int(VOICE_COOLDOWN_S * 1000)

            while True:
                try:
                    data = stream.read()
                except Exception as e:
                    print("[VOICE] mic read err:", e)
                    break                   # 重新初始化音频
                if not data or len(data) < 8:
                    continue
                e = voice_chunk_energy(data)          # 右声道平均幅度(抽样, 快)
                if e == 0:
                    continue

                if state == 0:
                    # 底噪自适应(只在安静时缓慢跟随; 有持续人声时基本不动)
                    if e < sil_abs:
                        floor = int(0.90 * floor + 0.10 * e)
                    elif e < int(floor * VAD_SIL_MULT):
                        floor = int(0.97 * floor + 0.03 * e)
                    # 冷却期不触发
                    if time.ticks_diff(time.ticks_ms(), last_cmd_ms) < cooldown_ms:
                        continue
                    start_thr = max(int(floor * VAD_START_MULT), VAD_START_ABS)
                    if e >= start_thr:
                        state = 1
                        voiced_ch = 0
                        voice_sum_e = 0
                        idle_ch = 0
                        total_ch = 0
                        chunks_raw = []
                        chunks_raw.append(data)
                        if VOICE_DEBUG:
                            print("[VOICE] >> 开始收音 e=%d floor=%d" % (e, floor))
                else:
                    chunks_raw.append(data)
                    total_ch += 1
                    sil_thr = max(int(floor * VAD_SIL_MULT), sil_abs)
                    if e >= sil_thr:
                        voiced_ch += 1
                        voice_sum_e += e
                        idle_ch = 0
                    else:
                        idle_ch += 1
                    # 触发后前 VAD_ABORT_MS 全是静音 => 误触发(关门声/碰撞声)
                    if voiced_ch == 0 and total_ch * chunk_ms >= VAD_ABORT_MS:
                        state = 0
                        chunks_raw = []
                        if VOICE_DEBUG:
                            print("[VOICE] << 误触发丢弃")
                        continue
                    # 句尾静音足够或超长 => 收句
                    done = (idle_ch * chunk_ms >= VAD_TAIL_MS) or \
                           (total_ch * chunk_ms >= VAD_MAX_MS)
                    if not done:
                        continue
                    state = 0
                    dur_ms = total_ch * chunk_ms
                    if voiced_ch * chunk_ms < VAD_MIN_MS or dur_ms < VAD_MIN_MS:
                        if VOICE_DEBUG:
                            print("[VOICE] << 过短丢弃 voiced=%dms" % (voiced_ch * chunk_ms))
                        continue
                    # 真语音占比过低 => 主要是噪声/远处说话, 丢弃
                    if total_ch > 0 and voiced_ch * 100 // total_ch < VOICE_MIN_VOICED_PCT:
                        if VOICE_DEBUG:
                            print("[VOICE] << 语音占比过低丢弃 voiced=%d/%dms" %
                                  (voiced_ch * chunk_ms, dur_ms))
                        continue
                    # 信噪比门限: 句内平均语音能量不够强 => 远距离/混响声, 不送识别
                    if voiced_ch > 0:
                        mean_e = voice_sum_e // voiced_ch
                        if mean_e < max(int(floor * VOICE_SNR_MIN), VOICE_SNR_ABS):
                            if VOICE_DEBUG:
                                print("[VOICE] << 信噪比过低丢弃 mean_e=%d floor=%d" %
                                      (mean_e, floor))
                            continue
                    if VOICE_DEBUG:
                        print("[VOICE] << 收句 %dms(voiced %dms), floor=%d, 送识别..." %
                              (dur_ms, voiced_ch * chunk_ms, floor))

                    # ---------- 4. 送云端识别 ----------
                    try:
                        stereo = b"".join(chunks_raw)     # 原始双声道
                        pcm = voice_extract_right_bytes(stereo)  # 抽右声道(板载麦在右路)
                        if act_rate == 16000 or act_rate == 8000:
                            send_rate = act_rate          # 真实率已满足, 直接上传
                        else:
                            # 实测率 44.1k/48k 等 => 降采样到 16k 再上传
                            pcm = voice_decimate16k_bytes(pcm, act_rate)
                            send_rate = 16000
                        text = voice_baidu_recognize(pcm, send_rate)
                        if VOICE_DEBUG:
                            print("[VOICE] 识别结果: %r" % text)
                        tag = voice_match_cmd(text)
                        if tag is not None:
                            if VOICE_CONFIRM_2X:
                                # 两次确认模式: 同一指令在时间窗内连续命中两次才下发
                                if tag == last_tag and \
                                   time.ticks_diff(time.ticks_ms(), last_tag_ms) < \
                                   int(VOICE_CONFIRM_WIN_S * 1000):
                                    voice_uart_write(VOICE_CMD_UART[tag])
                                    last_cmd_ms = time.ticks_ms()
                                    last_tag = None
                                    print("[VOICE] 两次确认一致, 下发指令: %s" %
                                          VOICE_CMD_UART[tag].strip())
                                else:
                                    last_tag = tag
                                    last_tag_ms = time.ticks_ms()
                                    print("[VOICE] 第一次识别到 %s, 请再说一次确认" % tag)
                            else:
                                voice_uart_write(VOICE_CMD_UART[tag])
                                last_cmd_ms = time.ticks_ms()
                                print("[VOICE] 下发指令: %s" % VOICE_CMD_UART[tag].strip())
                        else:
                            print("[VOICE] 未命中指令词(可调 VOICE_WORDS)")
                    except Exception as ex:
                        print("[VOICE] 识别流程异常:", ex)
                    finally:
                        chunks_raw = []
        except Exception as e:
            print("[VOICE] engine error:", e)
            try:
                if stream is not None:
                    stream.stop_stream()
                    stream.close()
            except Exception:
                pass
            try:
                if p is not None:
                    p.terminate()
            except Exception:
                pass
            time.sleep_ms(5000)


# 启动语音线程(要求: 开关开 + 线程可用 + struct 模块可用 + 配置完整)
if VOICE_ENABLE and _thread_ok and struct is not None:
    if WIFI_SSID and BAIDU_TOKEN:
        try:
            import _thread as _t
            uart_lock = _t.allocate_lock()
            _t.start_new_thread(voice_engine, ())
            print("[VOICE] 语音助手线程已启动 (词表: 启动/停车/加速/减速 -> V:START/STOP/ACC/DEC)")
        except Exception as e:
            print("[VOICE] 线程启动失败(仅视觉):", e)
    else:
        print("[VOICE] 提示: 语音助手需要先在文件顶部填 WIFI_SSID/WIFI_PWD/BAIDU_TOKEN (见 README)")
elif not VOICE_ENABLE:
    print("[VOICE] 语音助手已关闭 (VOICE_ENABLE=False), 运行纯视觉模式")
elif not _thread_ok:
    print("[VOICE] 固件无 _thread 模块, 语音助手不可用, 运行纯视觉模式")

# ============================================================================
#  以下为原视觉代码(仅 send_frame 增加串口写锁)
# ============================================================================

# OSD 绘制坐标换算系数：检测框坐标基于 rgb888p_size，OSD 图层基于 display_size
scale_x = display_size[0] / rgb888p_size[0]
scale_y = display_size[1] / rgb888p_size[1]
frame_area = rgb888p_size[0] * rgb888p_size[1]   # 画面面积(算面积占比A用)


def iter_boxes(res):
    """解析 yolo.run() 结果，逐框输出 [x, y, w, h, score, class_id]

    01Studio v1.8 固件的返回格式是三个并行列表(实测):
      res[0] = [每个目标的 [x,y,w,h] 数组, ...]    例: [array([439,299,75,179],dtype=int16)]
      res[1] = [每个目标的类别, ...]               例: [4]
      res[2] = [每个目标的置信度, ...]             例: [0.8486]
    """
    if res is None:
        return
    if isinstance(res, (list, tuple)) and len(res) >= 3:
        boxes_xywh = res[0]
        classes = res[1]
        scores = res[2]
        n = min(len(boxes_xywh), len(classes), len(scores))
        for i in range(n):
            xywh = boxes_xywh[i]
            if xywh is None or len(xywh) < 4:
                continue
            yield [xywh[0], xywh[1], xywh[2], xywh[3], scores[i], classes[i]]
        return
    # ---------- 以下为其他固件格式的兼容处理 ----------
    if hasattr(res, "shape"):                    # res 是 (N,6) 数组
        for i in range(res.shape[0]):
            yield res[i]
        return
    for item in res:                             # res 是 list / tuple
        if item is None:
            continue
        if isinstance(item, (list, tuple)):
            if len(item) > 0 and not isinstance(item[0], (list, tuple)):
                yield item                       # item 本身就是一个框
            else:
                for box in item:                 # item 是一层(多个框)
                    if box is not None:
                        yield box
        elif hasattr(item, "shape"):             # item 是 numpy 输出层
            if len(item.shape) == 1:
                yield item                       # 一维数组: 单个框
            else:
                for i in range(item.shape[0]):
                    yield item[i]
        else:
            yield item


def send_frame(cls_id, cx, cy, w, h, conf, area=0):
    """发送一帧目标信息；cls_id < 0 表示目标丢失；area=框面积占画面比例(千分比0~1000)
    单类(dacong)模型: 上报类别号固定为 PROTO_CLASS_ID(数值4, 协议兼容), 与6类协议一致"""
    if uart is None:
        return
    try:
        rep_cls = cls_id if cls_id < 0 else PROTO_CLASS_ID   # 丢失=-1, 命中=协议类别号
        if PROTOCOL == "BIN":
            d = bytearray(16)
            d[0] = 0xAA
            d[1] = 0x55
            d[2] = 0x01                                  # 帧类型: 目标信息
            d[3] = rep_cls & 0xFF                        # 类别(协议号, 0xFF=丢失)
            d[4] = cx & 0xFF
            d[5] = (cx >> 8) & 0xFF                      # 中心X 小端
            d[6] = cy & 0xFF
            d[7] = (cy >> 8) & 0xFF                      # 中心Y 小端
            d[8] = w & 0xFF
            d[9] = (w >> 8) & 0xFF                       # 宽 小端
            d[10] = h & 0xFF
            d[11] = (h >> 8) & 0xFF                      # 高 小端
            d[12] = int(conf * 100) & 0xFF               # 置信度百分比
            d[13] = area & 0xFF
            d[14] = (area >> 8) & 0xFF                   # 面积占比千分比 小端
            s = 0
            for i in range(2, 15):
                s += d[i]
            d[15] = s & 0xFF                             # 校验和
            if uart_lock is not None:
                uart_lock.acquire()
            try:
                uart.write(d)
            finally:
                if uart_lock is not None:
                    try:
                        uart_lock.release()
                    except Exception:
                        pass
        else:
            if SEND_AREA_RATIO:
                line = "C:%d,X:%d,Y:%d,W:%d,H:%d,S:%d,A:%d\n" % (
                    rep_cls, cx, cy, w, h, int(conf * 100), area)
            else:
                line = "C:%d,X:%d,Y:%d,W:%d,H:%d,S:%d\n" % (
                    rep_cls, cx, cy, w, h, int(conf * 100))
            if uart_lock is not None:
                uart_lock.acquire()
            try:
                uart.write(line)
            finally:
                if uart_lock is not None:
                    try:
                        uart_lock.release()
                    except Exception:
                        pass
            if DEBUG_PRINT:
                print("UART%d ->" % UART_ID, line.rstrip("\n"))
    except Exception as e:
        print("UART send error:", e)


def handle_stm32_cmd(data):
    """解析 STM32 下行指令：S0~S5 锁定跟踪类别 / A 恢复任意 / ? 查询
    dacong 单类模型: 任何 S 指令均视为锁定当前唯一类别(dacong)"""
    global track_class
    if not data:
        return
    try:
        s = data.decode().strip().upper()
    except Exception:
        return                       # 非文本噪声(RX2 悬空读到), 忽略不打印
    if not s:
        return
    if len(s) >= 2 and s[0] == "S" and s[1].isdigit():
        c = int(s[1])
        if c == PROTO_CLASS_ID:
            track_class = 0
            print("CMD: 锁定跟踪 dacong(S%d)" % c)
        else:
            print("CMD: 本模型只有 dacong(S%d), 忽略 S%d" % (PROTO_CLASS_ID, c))
    elif s == "A":
        track_class = -1
        print("CMD: 恢复任意类别(本模型只有dacong)")
    elif s == "?":
        print("CMD: 当前跟踪类别 = %d" % track_class)
    else:
        # 只打印可打印 ASCII 的短文本(过滤悬空 RX2 的噪声字节)
        ok = True
        for ch in s:
            o = ord(ch)
            if o < 32 or o > 126:
                ok = False
                break
        if ok and len(s) <= 40:
            print("STM32 ->", s)


def recv_from_stm32():
    """非阻塞读取 STM32 回传数据（指令/回执），无数据时直接返回"""
    if uart is None:
        return
    try:
        data = uart.read(128)
        if data:
            handle_stm32_cmd(data)
    except Exception:
        pass


def draw_text(img, x, y, text, color, scale=1.5):
    """在 OSD 上绘制文字(仅 OSD_TEXT=True 时调用)：依次尝试 draw_string_advanced 的
    几种常见签名，全部失败才退回旧版 draw_string(会打印弃用提示)"""
    try:
        img.draw_string_advanced(x, y, text, color, (0, 0, 0), scale)  # 带背景色6参
        return
    except Exception:
        pass
    try:
        img.draw_string_advanced(x, y, text, color, scale)             # 位置参数
        return
    except Exception:
        pass
    try:
        img.draw_string_advanced(x, y, text, color=color, scale=scale) # 关键字参数
        return
    except Exception:
        pass
    try:
        img.draw_string(x, y, text, color=color, scale=scale)          # 旧版(有弃用提示)
    except Exception as e:
        print("osd text error:", e)


def clear_osd(img):
    """每帧清空 OSD 图层，防止上一帧的框/十字残留在屏幕上"""
    try:
        img.clear()
    except Exception:
        try:
            img.draw_rectangle(0, 0, img.width(), img.height(),
                               color=(0, 0, 0), fill=True)
        except Exception:
            pass


def dump_res(res):
    """打印 yolo.run() 返回结果的真实结构(排错用)"""
    try:
        print("== res type:", type(res).__name__, "| len:", len(res))
        for i in range(min(3, len(res))):
            it = res[i]
            try:
                print("   res[%d] type=%s len=%s head=%s" %
                      (i, type(it).__name__, len(it), repr(it[:3])[:120]))
            except Exception:
                print("   res[%d] type=%s repr=%s" %
                      (i, type(it).__name__, repr(it)[:120]))
    except Exception as e:
        print("dump_res error:", e)


clock = time.clock()
track_class = TRACK_CLASS    # 当前跟踪类别(可被 STM32 下行指令 S0~S5/A 修改)
frame_cnt = 0
sx, sy = 0, 0          # 平滑后的中心坐标
have_last = False      # 上一帧是否有目标(平滑用)
lost_cnt = 0           # 连续丢失帧计数
err_cnt = 0            # 检测结果解析失败计数(限流打印用)

while True:
    clock.tick()
    img = pl.get_frame()
    clear_osd(pl.osd_img)      # 每帧清空OSD，避免上一帧的框/十字残留
    res = yolo.run(img)
    if res is None:
        res = []
    if DEBUG_PRINT and frame_cnt == 0:
        dump_res(res)          # 第一帧打印结果结构(排错)
    # ---------- 解析所有检测框(一次解析，画框和选目标共用) ----------
    boxes = []          # 有效检测框列表: (cls_id, x, y, w, h, score)
    try:
        for box in iter_boxes(res):
            x = int(box[0])
            y = int(box[1])
            w = int(box[2])
            h = int(box[3])
            score = float(box[4])
            cls_id = int(box[5])
            if score < confidence_threshold:
                continue
            if w * h < MIN_AREA:
                continue
            if track_class >= 0 and cls_id != track_class:
                continue
            boxes.append((cls_id, x, y, w, h, score))
    except Exception as e:
        # 解析失败不影响主循环：限流打印并输出一次真实结构，方便定位固件返回格式
        err_cnt += 1
        if err_cnt == 1 or err_cnt % 50 == 0:
            print("解析检测结果失败(%d次):" % err_cnt, e)
            dump_res(res)

    # ---------- 选出跟踪目标(延续锁定: 优先跟上一次同一目标, 防多目标串跟) ----------
    best = None         # (cls_id, x, y, w, h, score)
    best_key = -1
    if TRACK_LOCK_DIST > 0 and have_last:
        # 画面里有多个同类目标时，优先匹配"距上一帧目标中心最近"的那个(保持跟同一个)
        best_near = None
        best_d = TRACK_LOCK_DIST
        for (cls_id, x, y, w, h, score) in boxes:
            d = abs((x + w // 2) - sx) + abs((y + h // 2) - sy)
            if d < best_d:
                best_d = d
                best_near = (cls_id, x, y, w, h, score)
        if best_near is not None:
            best = best_near
    if best is None:
        # 无延续锁定命中(首帧/刚恢复/目标已移动远)时：按面积或置信度选
        for (cls_id, x, y, w, h, score) in boxes:
            key = w * h if TRACK_LARGEST else score
            if key > best_key:
                best_key = key
                best = (cls_id, x, y, w, h, score)

    # ---------- 画框：默认只画跟踪目标一个框(画面干净)；DRAW_TRACKED_ONLY=False 画全部 ----------
    if DRAW_TRACKED_ONLY:
        draw_list = [best] if best is not None else []
    else:
        draw_list = boxes
    for (cls_id, x, y, w, h, score) in draw_list:
        try:
            color = BOX_COLORS[cls_id % len(BOX_COLORS)]   # 每类一种颜色
            pl.osd_img.draw_rectangle(int(x * scale_x), int(y * scale_y),
                                      int(w * scale_x), int(h * scale_y),
                                      color=color, thickness=2)
        except Exception as e:
            print("osd box error:", e)
            break

    # ---------- 跟踪目标：中心计算 + 平滑 + 跳变滤除 ----------
    no_target = best is None
    send_target = False
    cx = cy = 0
    if not no_target:
        cls_id, x, y, w, h, score = best
        cx0 = x + w // 2          # 中心坐标 = 左上角 + 半宽高
        cy0 = y + h // 2
        if SMOOTH_ENABLE and have_last:
            # 一阶低通平滑，避免坐标跳变导致电机抖动
            cx = int(SMOOTH_ALPHA * cx0 + (1 - SMOOTH_ALPHA) * sx)
            cy = int(SMOOTH_ALPHA * cy0 + (1 - SMOOTH_ALPHA) * sy)
        else:
            cx, cy = cx0, cy0
        if MAX_JUMP > 0 and have_last and \
                (abs(cx - sx) > MAX_JUMP or abs(cy - sy) > MAX_JUMP):
            # 单帧跳变过大：视为误检尖峰，静默丢弃本帧(不更新/不发送/不计丢失)
            have_last = False
        else:
            sx, sy = cx, cy
            have_last = True
            lost_cnt = 0
            send_target = True

    if send_target:
        # OSD 上画中心十字(+可选文字)，换算到显示分辨率
        try:
            dx = int(cx * scale_x)
            dy = int(cy * scale_y)
            pl.osd_img.draw_cross(dx, dy, color=(255, 0, 0), size=12, thickness=2)
            if OSD_TEXT:
                draw_text(pl.osd_img, dx + 12, dy - 30,
                          "C:%d X:%d Y:%d" % (cls_id, cx, cy), (255, 255, 0))
                draw_text(pl.osd_img, 10, 10,
                          "TARGET:%s" % labels[cls_id], (0, 255, 0))
        except Exception as e:
            print("osd draw error:", e)

        # 发送坐标给 STM32F407: ABS=绝对坐标 / ERR=相对画面中心偏差；附带面积占比A
        if frame_cnt % SEND_EVERY_N == 0:
            if SEND_AREA_RATIO:
                area = w * h * 1000 // frame_area   # 框面积占画面比例(千分比 0~1000)
            else:
                area = 0
            if SEND_MODE == "ERR":
                send_frame(cls_id, cx - rgb888p_size[0] // 2,
                           rgb888p_size[1] // 2 - cy, w, h, score, area)
            else:
                send_frame(cls_id, cx, cy, w, h, score, area)

    elif no_target:
        # 没有检测到目标：连续丢失 LOST_GRACE 帧后才发 C:-1(防一帧闪烁就停车)
        have_last = False
        lost_cnt += 1
        try:
            if OSD_TEXT:
                draw_text(pl.osd_img, 10, 10, "NO TARGET!", (255, 0, 0))
        except Exception as e:
            print("osd draw error:", e)
        if lost_cnt == LOST_GRACE or lost_cnt % LOST_SEND_EVERY_N == 0:
            send_frame(-1, 0, 0, 0, 0, 0.0)

    recv_from_stm32()          # 读取 STM32 回传数据（非阻塞）

    pl.show_image()
    if frame_cnt % GC_EVERY_N == 0:
        gc.collect()          # 每 GC_EVERY_N 帧回收一次，避免每帧GC造成卡顿
    frame_cnt += 1
    if frame_cnt % FPS_PRINT_EVERY == 0:
        print("FPS:", clock.fps())
