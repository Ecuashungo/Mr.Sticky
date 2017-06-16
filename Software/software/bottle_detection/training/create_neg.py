import cv2
import os

def create_neg():
    for img in os.listdir('negatives/'):
        try:
            line = 'negatives/'+str(img)+'\n'
            with open('neg.txt','a') as f:
                f.write(line)
        except Exception as e:
            print(str(e))
                
#create_neg()

## for commandline use: python3 create_neg.py
if __name__ == "__main__":
    create_neg()
