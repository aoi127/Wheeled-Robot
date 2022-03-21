import numpy as np
import cv2

def rotate90(src):
    r_Img = cv2.rotate(src, cv2.ROTATE_90_CLOCKWISE)
    return r_Img

def rotate180(src):
    r_Img = cv2.rotate(src, cv2.ROTATE_180)
    return r_Img

def resize(src, scale_percent):
    width = int(src.shape[1] * scale_percent / 100)
    height = int(src.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    resized = cv2.resize(src, dim, interpolation = cv2.INTER_AREA)
    return resized

def calcGrayHist(src):
    rows, cols = src.shape
    grayHist = np.zeros([256], np.uint64)
    for r in xrange(rows):
        for c in xrange(cols):
            grayHist[src[r][c]] += 1
    return grayHist

def linear_trans(src, alpha):
    r_Img = alpha * src
    r_Img[r_Img>255] = 255
    r_Img = np.round(r_Img)
    r_Img = r_Img.astype(np.uint8)
    return r_Img

def normalize(src):
    r_Img = cv2.normalize(src, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)

def gamma_correct(src):
    fI = src/255.0
    gamma = 0.5
    r_Img = np.power(fI, gamma)
    return r_Img

def create_clahe(src):
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    dst = clahe.apply(src)
    return dst

def threshold(src, the, maxval=255):
    if the == 0:
        otsuThe = 0
        otsuThe, dst = cv2.threshold(src, otsuThe, maxval, cv2.THRESH_OTSU)
    else:
        dst = cv2.threshold(src, the, maxval, cv2.THRESH_BINARY)
    return dst

def adaptiveThresh(src, winSize, ratio=0.15):
    I_mean = cv2.boxFilter(src, cv2.CV_32FC1, winSize)
    out = src - (1.0-ratio)*I_mean
    out[out>=0] = 255
    out[out<0] = 0
    out = out.astype(np.uint8)
    return out

def morphologyEx(src, r, i, type):
    if type == 0:
        s = cv2.getStructuringElement(cv2.MORPH_RECT, (2*r+1, 2*r+1))
        d = cv2.morphologyEx(src, cv2.MORPH_OPEN, s, iterations=i)
    elif type==1:
        s = cv2.getStructuringElement(cv2.MORPH_RECT, (2*r+1, 2*r+1))
        d = cv2.morphologyEx(src, cv2.MORPH_CLOSE, s, iterations=i)
    return d




