import pigpio

pi = pigpio.pi()
if not pi.connected:
    print("連接失敗")
    exit()
else:
    print("連接成功")
