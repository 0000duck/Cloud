import cv2
import os
import shutil
from PIL import ImageGrab
from time import sleep
import numpy as np
import pyautogui as pgui
import random

import scsho
import matching




def image_recognition(target, IMG_DIR):
    TARGET_FILE = target
    IMG_SIZE = (2735, 1741)

    target_img_path = TARGET_FILE
    target_img = cv2.imread(target_img_path, cv2.IMREAD_GRAYSCALE)   #画像をグレースケール化


    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    detector = cv2.ORB_create()
    (target_kp, target_des) = detector.detectAndCompute(target_img, None)   #target_imgのキーポイントと特徴量を抽出

    #print('TARGET_FILE: %s' % (TARGET_FILE))

    files = os.listdir(IMG_DIR)   #比較する画像の集合（比較元の画像も込み）
    files.sort()
    #print("files_lengs:" + str(files.sort()))
    state_pattern_num = len(files) - 1
    state_pattern = np.zeros((state_pattern_num +1,2))
    i = 0
    for file in files:
        if file == '.DS_Store' or file == TARGET_FILE:
            continue

        comparing_img_path = IMG_DIR + file
        #print('COMPARE_FILE: %s' % (comparing_img_path))
        if not (comparing_img_path == target_img_path):
            try:
               #print("try_start")
               comparing_img = cv2.imread(comparing_img_path, cv2.IMREAD_GRAYSCALE)
               #print("matches")
               #comparing_img = cv2.resize(comparing_img, IMG_SIZE)
               #print("matches") #出力されず
               (comparing_kp, comparing_des) = detector.detectAndCompute(comparing_img, None)    #comparing_imgのキーポイントと特徴量を抽出
               matches = bf.match(target_des, comparing_des)   #特徴点のマッチング
               #print("matches:" , matches)
               #IMG3_path = "images_surabaku\\match.png"
               #img3 = cv2.drawMatches(IMG3_path, target_kp, comparing_img, comparing_kp, matches[:10], None, flags=2)
               dist = [m.distance for m in matches]   #distance：距離行列
               if len(dist)!=0:
                   ret = sum(dist) / len(dist)
               else:
                   ret = 100000
            except cv2.error:
               ret = 100000

            #print(comparing_img_path, ret, i)
            state_pattern[i] = (ret,i+1) 
            i += 1
            #print(state_pattern)

    now_state = state_pattern[state_pattern[:,0].argsort(), :] #2次元配列を並び替え（画像が一致している順）
    now_state_num = round(now_state[0][1])
    #print(now_state)

    if now_state[0][0] > 15:  #全ての画像との一致度が90以上なら，stateを存在しない値"0"にする
        now_state_num = 0

    return now_state_num





if __name__ == "__main__":

  
    IMAGE = "images\\"
    IMAGE2 = "..\\picture\\"
    IMAGE3 = "atom\\"

    count = 1000000
    for i in range(count):
        # sleep(5)
        delete=1
        str = scsho.scsho(IMAGE3)
        now_state_num = image_recognition(str, IMAGE)
        #print("select:",now_state_num)
        
        if now_state_num != 0:
            #画像をトリミングしてから比較
            """
            image = "icon\\134_shimei"
            match, w, h = matching.pattern_matching_one(str, image)
            if match != 0:
                im_crop = im.crop((w-20, h-20, 2735, 1741))
                im_crop.save(str, quality=95)
                match = matching.pattern_matching(str, IMAGE)
                if match == 0:
                    delete=0
                    print("Success")
            """
            str2 = os.path.basename(str)
            shutil.move(str, IMAGE2 + str2)
            delete=0

        if delete==1:
            os.remove(str) #ファイルの削除
            #print("同一画像の発見")

        #sleep(random.uniform(1,2))
        #print("\n\n\n")



