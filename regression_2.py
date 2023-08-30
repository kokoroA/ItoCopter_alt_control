BINARY_VISIBLE = True  # 二値化画像を表示するかどうかを制御するフラグ

import sensor, image, time, pyb,struct,ustruct  # カメラとイメージ処理、時間、UART通信に関するライブラリをインポート

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
clock = time.clock()

THRESHOLD = (80, 255)
sensor.set_auto_exposure(False, exposure_us=13000)
sensor.set_auto_gain(False, gain_db=5)

uart = pyb.UART(1, 115200, timeout=1000)
uart.init(115200, bits=8, parity=None, stop=1)

LINE_GAP_THRESHOLD = 5  #検出しない区間の閾値-数字が低いほど、検出しない
line_gap_count = 0  # 検出しない直線の連続数
gap_number = 0  # 検出しない区間の数

while True:
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD]) if BINARY_VISIBLE else sensor.snapshot()

    line = img.get_regression([(255, 255) if BINARY_VISIBLE else THRESHOLD])

    if line:
        if line_gap_count > LINE_GAP_THRESHOLD:
            gap_number += 1
            print("検出しない区間の数: %d" % gap_number)

        line_gap_count = 0  # 直線が検出されたので連続カウントをリセット
        img.draw_line(line.line(), color=127)

        line_center_x = (line.x1() + line.x2()) // 2
        x_zure = line_center_x - img.width() // 2

        if line.theta() > 90:
            theta_left = line.theta() - 180
        else:
            theta_left = line.theta()

        #x_byte = struct.pack('i',x_zure)

        #angle_byte = struct.pack('i',theta_left)
        #data_send = [x_zure,theta_left]
        #data_bytes = struct.pack('ii',*data_send)
        uart.write(str((x_zure, theta_left)).encode() + b'\n')  # データを文字列として送信
        #uart.write(ustruct.pack('B',x_zure))

        print("FPS %f, mag = %s, ずれ = %s, angle = %s" % (clock.fps(), str(line.magnitude()), x_zure, theta_left))
    else:
        line_gap_count += 1
