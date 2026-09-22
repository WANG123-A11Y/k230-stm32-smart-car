"""WiFi 诊断脚本(01Studio CanMV K230)
作用: 1) 扫描周围可见的 2.4G AP; 2) 尝试连接指定热点; 3) 打印详细结果。
用法: CanMV IDE 打开运行, 或改名 main.py 临时放 /sdcard 运行(注意备份原 main.py)。
"""
import network
import time

SSID = "your-2.4g-ssid"   # <<< 改成你的热点名
PWD  = "your-wifi-password"        # <<< 改成你的热点密码

wlan = network.WLAN(network.STA_IF)
try:
    print("active ->", wlan.active(True))
except Exception as e:
    print("active exc:", e)
time.sleep_ms(800)

print("---- 扫描可见 AP(只看得到 2.4G) ----")
try:
    nets = wlan.scan()
    print("scan count:", len(nets))
    for n in nets:
        try:
            name = n[0].decode() if isinstance(n[0], bytes) else str(n[0])
        except Exception:
            name = "?"
        # 各固件元组含义: (ssid, bssid, channel, rssi, security, hidden?) 顺序可能不同
        print("  - %r  ch=%s  rssi=%s  sec=%s  rest=%s" % (name, n[2] if len(n) > 2 else "?", n[3] if len(n) > 3 else "?", n[4] if len(n) > 4 else "?", n[5:] if len(n) > 5 else ""))
except Exception as e:
    print("scan exc:", e)

print("---- 尝试连接 %r ----" % SSID)
if wlan.isconnected():
    print("已连接:", wlan.ifconfig())
else:
    try:
        r = wlan.connect(SSID, PWD)
        print("connect() return:", r)
    except Exception as e:
        print("connect() exc:", e)
    t0 = time.time()
    while time.time() - t0 < 25 and not wlan.isconnected():
        time.sleep_ms(300)
    print("最终连接状态:", wlan.isconnected())
    if wlan.isconnected():
        print("ifconfig:", wlan.ifconfig())
    else:
        print("连接失败 -> 请检查: 热点是否2.4G频段/是否WPA2/名字密码是否正确/是否能被扫描到")
