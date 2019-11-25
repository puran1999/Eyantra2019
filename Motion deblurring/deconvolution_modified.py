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




img = cv.imread('eyantra.jpg', cv.IMREAD_COLOR)
cv.imshow('input', img)
channels = []
results = []
channels = cv.split(img)
i=0
for channel in channels:
	i=i+1
	channel = np.float32(channel)/255.0
	cv.imwrite(str(i)+'before_blur_edge_forchannel.jpg',channel * 255)
	channel = blur_edge(channel) 
	cv.imwrite(str(i)+'blur_edge_for_channel.jpg',channel * 255)
	IMG = cv.dft(channel, flags=cv.DFT_COMPLEX_OUTPUT)
	ang = np.deg2rad( 90 ) #90
	d = 20                 #20
	noise = 10**(-0.1*25)  #25
	psf = motion_kernel(ang, d)
	print(psf.shape)
	cv.imshow('psf', psf)
	psf /= psf.sum()
	print(psf.shape)
	psf_pad = np.zeros_like(channel)
	kh, kw = psf.shape
	psf_pad[:kh, :kw] = psf
	PSF = cv.dft(psf_pad, flags=cv.DFT_COMPLEX_OUTPUT, nonzeroRows = kh)
	PSF2 = (PSF**2).sum(-1)
	iPSF = PSF / (PSF2 + noise)[...,np.newaxis]
	RES = cv.mulSpectrums(IMG, iPSF, 0)
	res = cv.idft(RES, flags=cv.DFT_SCALE | cv.DFT_REAL_OUTPUT )
	res = np.roll(res, -kh//2, 0)
	res = np.roll(res, -kw//2, 1)
	res = res*255.0
	cv.imshow('output',res)
	results.append(res)
	cv.imwrite(str(i) + 'deconvuled_eyantra_perchannel.jpg',res)

result = cv.merge(results)
cv.imwrite('deconvuled_eyantra_allchannel.jpg',result)

