import numpy as np
import cv2 as cv

def blur_edge(img, d=150):
    h, w  = img.shape[:2]
    img_pad = cv.copyMakeBorder(img, d, d, d, d, cv.BORDER_WRAP)
    img_blur = cv.GaussianBlur(img_pad, (2*d+1, 2*d+1), -1)[d:-d,d:-d]
    y, x = np.indices((h, w))
    dist = np.dstack([x, w-x-1, y, h-y-1]).min(-1)
    w = np.minimum(np.float32(dist)/d, 1.0)
    return img*w + img_blur*(1-w)

def motion_kernel(angle, d, sz=65):
    kern = np.ones((1, d), np.float32)
    c, s = np.cos(angle), np.sin(angle)
    A = np.float32([[c, -s, 0], [s, c, 0]])
    sz2 = sz // 2
    A[:,2] = (sz2, sz2) - np.dot(A[:,:2], ((d-1)*0.5, 0))
    kern = cv.warpAffine(kern, A, (sz, sz), flags=cv.INTER_CUBIC)
    return kern




img = cv.imread('eyantra.jpg', cv.IMREAD_GRAYSCALE)
cv.imshow('input', img)

img = np.float32(img)/255.0
img = blur_edge(img) 

IMG = cv.dft(img, flags=cv.DFT_COMPLEX_OUTPUT)

ang = np.deg2rad( 90 ) #90
d = 20                 #20
noise = 10**(-0.1*25)  #25

psf = motion_kernel(ang, d)
cv.imshow('psf', psf)

psf /= psf.sum()
psf_pad = np.zeros_like(img)
kh, kw = psf.shape
psf_pad[:kh, :kw] = psf
PSF = cv.dft(psf_pad, flags=cv.DFT_COMPLEX_OUTPUT, nonzeroRows = kh)
PSF2 = (PSF**2).sum(-1)
iPSF = PSF / (PSF2 + noise)[...,np.newaxis]
RES = cv.mulSpectrums(IMG, iPSF, 0)
res = cv.idft(RES, flags=cv.DFT_SCALE | cv.DFT_REAL_OUTPUT )
res = np.roll(res, -kh//2, 0)
res = np.roll(res, -kw//2, 1)
cv.imshow('output',res)


