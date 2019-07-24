import sensor, image, time, math, utime
from pyb import UART

uart = UART(3, 9600)
uart.init(9600, bits=8, parity=None, stop=1) # init with given parameters

# Tracks a black line. Use [(128, 255)] for a tracking a white line.
red_threshold_01 = (16, 100, -3, 4, -3, 2)
black_threshold_01 = (0, 64)
#设置阈值，如果是黑线，GRAYSCALE_THRESHOLD = [(0, 64)]；
#如果是白线，GRAYSCALE_THRESHOLD = [(128，255)]


ROIS = [ # [ROI, weight]
        (0, 100, 160, 20, 0.7), # You'll need to tweak the weights for you app
        (0, 050, 160, 20, 0.3), # depending on how your robot is setup.
        (0, 000, 160, 20, 0.1)
       ]
#roi代表三个取样区域，（x,y,w,h,weight）,代表左上顶点（x,y）宽高分别为w和h的矩形，
#weight为当前矩形的权值。注意本例程采用的QQVGA图像大小为160x120，roi即把图像横分成三个矩形。
#三个矩形的阈值要根据实际情况进行调整，离机器人视野最近的矩形权值要最大，
#如上图的最下方的矩形，即(0, 100, 160, 20, 0.7)

# Compute the weight divisor (we're computing this so you don't have to make weights add to 1).
weight_sum = 0 #权值和初始化
for r in ROIS: weight_sum += r[4] # r[4] is the roi weight.
#计算权值和。遍历上面的三个矩形，r[4]即每个矩形的权值。

# Camera setup...
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # use grayscale.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(30) # Let new settings take affect.
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
#关闭白平衡
clock = time.clock() # Tracks FPS.

'''
def MoveServo(servoId, pos, time):
    global uart
    buf = bytearray(b'\x55\x55\x08\x03\x01')
    buf.append(0xff & actNum)
    buf.append(0xff & time)
    buf.append(0xff & (time >> 8))
    buf.append((0xff & servoId))
    buf.append(0xff & pos)
    buf.append(0xff & (pos >> 8))
    uart.write(buf)
'''

def RunActionGroup(actNum, num):
    global uart
    buf = bytearray(b'\x55\x55\x05\x06') #帧头， 长度， 命令 6
    buf.append(0xff & actNum)  #追加动作组号数
    buf.append(0xff & num) #运行次数
    buf.append(0xff & (num >> 8))
    uart.write(buf)

def StopActionGroup():
    uart.write(b'\x55\x55\x02\x07')  #发送停止指令

'''
def WaitForFinish(timeout):
    buf = bytearray()
    timeout = utime.time() + timeout #计算超时时间到达后的系统时间
    while True:
        if utime.time() > timeout:  #如果系统时间达到就返回False,即动作组没在指定时间内停止
            return False
        try:
            rcv = uart.readline()  #接收数据
            if rcv is not None:
                buf = rcv
                while True:
                    try:
                        index = buf.index(b'\x55\x55')  #查找帧头
                        if len(buf) >= buf[index + 2]:
                            buf = buf[index:]  #将帧头前面不符合的部分剔除
                            if (buf[2] + 2) <= len(buf): #缓存中的数据数据是否完整
                                cmd = buf[0: (buf[2]+2)] #将命令取出 print("OK")
                                buf = buf[buf[2]+2:]
                                print(cmd[3])
                                if cmd[3] == 0x08 or cmd[3] == 0x07:  #接受到的数据的命令号为8或7就是动作已停止
                                    return True
                    except:
                        break
        except:
            continue

MoveServo(19, 1500, 1000)
MoveServo(20, 1500, 1000)
'''

while(True):

    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.
    centroid_sum = 0
    #利用颜色识别分别寻找三个矩形区域内的线段

    for r in ROIS:
        blobs1 = img.find_blobs([black_threshold_01], roi=r[0:4], merge=True)
        blobs = img.find_blobs([red_threshold_01], roi=r[0:4], merge=True)
        # r[0:4] is roi tuple.
        #找到视野中的线,merge=true,将找到的图像区域合并成一个

        #目标区域找到红线
        if blobs:
            # Find the index of the blob with the most pixels.
            most_pixels = 0
            largest_blob = 0
            for i in range(len(blobs)):
            #目标区域找到的颜色块（线段块）可能不止一个，找到最大的一个，作为本区域内的目标直线
                if blobs[i].pixels() > most_pixels:
                    most_pixels = blobs[i].pixels()
                    #merged_blobs[i][4]是这个颜色块的像素总数，如果此颜色块像素总数大于most_pixels，则把本区域作为像素总数最大的颜色块。更新most_pixels和largest_blob
                    largest_blob = i

            # Draw a rect around the blob.
            #img.draw_rectangle(blobs[largest_blob].rect())
            #img.draw_rectangle((0,0,30, 30))
            #将此区域的像素数最大的颜色块画矩形和十字形标记出来
            #img.draw_cross(blobs[largest_blob].cx(),
                           #blobs[largest_blob].cy())

            centroid_sum += blobs[largest_blob].cx() * r[4] # r[4] is the roi weight.
            #计算centroid_sum，centroid_sum等于每个区域的最大颜色块的中心点的x坐标值乘本区域的权值

        #目标区域找到黑线
        if blobs1:
            # Find the index of the blob with the most pixels.
            most_pixels = 0
            largest_blob = 0
            for i in range(len(blobs1)):
            #目标区域找到的颜色块（线段块）可能不止一个，找到最大的一个，作为本区域内的目标直线
                if blobs1[i].pixels() > most_pixels:
                    most_pixels = blobs1[i].pixels()
                    #merged_blobs[i][4]是这个颜色块的像素总数，如果此颜色块像素总数大于most_pixels，则把本区域作为像素总数最大的颜色块。更新most_pixels和largest_blob
                    largest_blob = i

            # Draw a rect around the blob.
            #img.draw_rectangle(blobs1[largest_blob].rect())
            #img.draw_rectangle((0,0,30, 30))
            #将此区域的像素数最大的颜色块画矩形和十字形标记出来
            #img.draw_cross(blobs1[largest_blob].cx(),
                           #blobs1[largest_blob].cy())

            #centroid_sum += blobs[largest_blob].cx() * r[4] # r[4] is the roi weight.
            #计算centroid_sum，centroid_sum等于每个区域的最大颜色块的中心点的x坐标值乘本区域的权值

    center_pos = (centroid_sum / weight_sum) # Determine center of line.
    #中间公式

    deflection_angle = 0
    #机器人应该转的角度

    deflection_angle = -math.atan((center_pos-80)/60)
    #角度计算.80 60 分别为图像宽和高的一半，图像大小为QQVGA 160x120.
    #注意计算得到的是弧度值
    deflection_angle = math.degrees(deflection_angle)
    #print(deflection_angle)
    #将计算结果的弧度值转化为角度值

    if deflection_angle > 15 and deflection_angle < 90:#向左转
        RunActionGroup(54,1)
    elif deflection_angle <= 15 and deflection_angle >= -15:#直走
        RunActionGroup(53,1)
    elif deflection_angle < -15 and deflection_angle > -90:#向右转
        RunActionGroup(55,1)
    time.sleep(450).ino
