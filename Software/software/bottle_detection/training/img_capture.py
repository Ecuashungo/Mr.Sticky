# -*- coding: utf-8 -*-
##This file is an automated process of creating positive images once run
##it takes "num" images with a rate of "rate" frames per second. They are
##stored in the folder with the relative path pos_before. For using them
##in a haar training, they need to be treated with the matlabscript main.m
##it is important that each image only contains one bottle.

import cv2
import os


def foldercheck(mode):
    if mode == 'neg':
        if not os.path.exists('negatives'):
            os.makedirs('negatives')
            print('directory \'negatives\' made')
    if mode == 'pos':
        if not os.path.exists('positives_to_process'):
            os.makedirs('positives_to_process')
            print('directory \'positives_to_process\' made')


def create_neg(run):
    if os.path.exists('neg.txt'):
        os.remove('neg.txt')
        print('neg.txt has been removed')
    for img in os.listdir('negatives/'):
        try:
            if not img.endswith('.jpg'):
                print('Not a .jpg file and therefore skipped')
            elif img.endswith('.jpg'):
                line = 'negatives/' + str(img) + '\n'
                with open('neg.txt', 'a') as f:
                    f.write(line)
        except Exception as e:
            print(str(e))
    print('The file neg.txt has been written')


def grab(mode, num, rate, run):
    ## mode: pos neg
    ## num: number of images wanted
    ## rate: every 'rate' seconds a image is saved
    cap = cv2.VideoCapture(0)
    fps = cap.get(5) #5th option is frames per seconds

    rate = rate / 10  # 10ths of a second
    counter = 1
    name_start = 0
    if run == "continue" and mode == 'pos':
        img_folder_path = 'positives_to_process/'
        name = int(len([f for f in os.listdir(img_folder_path) if f.endswith('.jpg') and os.path.isfile(os.path.join(img_folder_path, f))]))
        name_start = name
    elif run == "continue" and mode == 'neg':
        img_folder_path = 'negatives/'
        name = int(len([f for f in os.listdir(img_folder_path) if f.endswith('.jpg') and os.path.isfile(os.path.join(img_folder_path, f))]))
        name_start = name
    elif run == "start":
        name = 0
    else:
        return "Pleas add arguments in this sequence: python3 img_capture.py pos/neg num rate continue/start"

    while True:
        
        ret, img = cap.read()
        counter = counter+1

        if mode == 'pos':

            if counter % int((fps) * rate) == 0:
                name = name + 1
                cv2.imwrite('positives_to_process/'+str(name)+'.jpg',img)
                print('positives_to_process/'+str(name)+'.jpg saved')
        elif mode == 'neg':
            if (counter % int((fps) * rate) == 0):
                name = name + 1
                cv2.imwrite('negatives/' + str(name) + '.jpg', img)
                print('negatives/' + str(name) + '.jpg saved')
        if ((name - name_start) == num):
            print('Process terminated: ' + str(num) + ' images saved')
            break
            
    cap.release()
    cv2.destroyAllWindows()

## for commandline use: python3 pos_img_capture.py pos 5 5 start
if __name__ == "__main__":
    import sys
    foldercheck(str(sys.argv[1]))
    grab(str(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), str(sys.argv[4]))
    if str(sys.argv[1])=='neg':
        create_neg(str(sys.argv[4]))

