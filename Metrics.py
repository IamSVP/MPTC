import numpy as np
import cv2
import os
import math

def compute_SSIM(imagename1,imagename2):
	img1 = cv2.imread(imagename1)
	img2 = cv2.imread(imagename2)
	print imagename2,imagename1
	img1 = cv2.cvtColor(img1, cv2.COLOR_RGB2GRAY)
	img2 = cv2.cvtColor(img2, cv2.COLOR_RGB2GRAY)
	height,widht = img1.shape
	img1 = np.float64(img1)
	img2 = np.float64(img2)
	
	window = cv2.getGaussianKernel(11,2.0,cv2.CV_64F)

	
	C1 = 6.5025
	C2 = 58.5525

	mu1 = cv2.filter2D(img1,cv2.CV_64F,window)
	mu2 = cv2.filter2D(img2,cv2.CV_64F,window)
	mu1_sq = cv2.multiply(mu1,mu1)
	mu2_sq = cv2.multiply(mu2,mu2)
	mu1_mu2 = cv2.multiply(mu1,mu2)

	img1_sq = cv2.multiply(img1,img1)
	img2_sq = cv2.multiply(img2,img2)
	img1_img2 = cv2.multiply(img1,img2)

	sigma1_sq = cv2.filter2D(img1_sq,cv2.CV_64F,window)
	sigma1_sq = cv2.subtract(sigma1_sq,mu1_sq)

	sigma2_sq = cv2.filter2D(img2_sq,cv2.CV_64F,window)
	sigma2_sq = cv2.subtract(sigma2_sq,mu2_sq)	


	sigma12 = cv2.filter2D(img1_img2,cv2.CV_64F,window)
	sigma12 = cv2.subtract(sigma12,mu1_mu2)

	t1 = 2*mu1_mu2 + C1
	t2 = 2*sigma12 + C2
	t3 = cv2.multiply(t1,t2)

	t1 = mu1_sq + mu2_sq + C1
	t2 = sigma2_sq + sigma1_sq + C2
	t1 = cv2.multiply(t1,t2)

	ssim_map = cv2.divide(t3,t1)
	mssim = cv2.mean(ssim_map)
	return "{0:.5f}".format(mssim[0]),height,widht

def compute_PSNR(imagename1,imagename2):
	img1 = cv2.imread(imagename1)
	img2 = cv2.imread(imagename2)
	height, widht = img1.shape[:2]

	s1 = cv2.absdiff(img1,img2)
	s1 = np.float32(s1)
	s1 = cv2.multiply(s1,s1)
	S = cv2.sumElems(s1)
	sse = S[0] + S[1] + S[2]
	mse = sse /(3*height*widht);
	psnr = 10.0*math.log10((255*255)/mse);
	return "{0:.4f}".format(psnr)

def compute_DELTAE(imagename1,imagename2):
	img1 = cv2.imread(imagename1)
	img2 = cv2.imread(imagename2)

	
	

	img1 = cv2.cvtColor(img1, cv2.COLOR_RGB2LAB)
	img2 = cv2.cvtColor(img2, cv2.COLOR_RGB2LAB)
	#cv2.imwrite(imagename2+".png",img1)

	# s1 = cv2.absdiff(img1,img2)
	# s1 = np.float32(s1)
	# s1 = cv2.multiply(s1,s1)
	# s1 = cv2.sqrt(s1)

	L1,a1,b1 = cv2.split(img1)
	L2,a2,b2 = cv2.split(img2)
	
	dL = L1 - L2
	da = a1-a2
	db = b1-b2
	# cv2.imwrite(imagename2+".png",dL)
	
	# dL_2 = cv2.multiply(dL,dL)
	# da_2 = cv2.multiply(da,da)
	# db_2 = cv2.multiply(db,db)
	# dL_2 = np.float32(dL_2)
	# da_2 = np.float32(da_2)
	# db_2 = np.float32(db_2)
	# dE = cv2.sqrt( (dL_2) + (da_2) + (db_2))
	# mde = cv2.mean(dE)
	# print mde


	c1 = np.sqrt(cv2.multiply(a1,a1) + cv2.multiply(b1,b1))
	c2 = np.sqrt(cv2.multiply(a2,a2) + cv2.multiply(b2,b2))
	dCab = c1-c2
	dH = np.sqrt(cv2.multiply(da,da) + cv2.multiply(db,db)- cv2.multiply(db,db))
	sL = 1
	K1 = 0.045 #can be changed
	K2 = 0.015 #can be changed
	sC = 1+K1*c1
	sH = 1+K2 *c1
	kL = 1 #can be changed

	t1 = cv2.divide(dL,kL*sL)
	t2 = cv2.divide(dCab,sC)
	t3 = cv2.divide(dH,sH)
	t1 = cv2.multiply(t1,t1)
	t2 = cv2.multiply(t2,t2)
	t3 = cv2.multiply(t3,t3)
	t1 = np.float32(t1)
	t2 = np.float32(t2)
	t3 = np.float32(t3)
	dE = cv2.sqrt(t1+t2+t3)
	mde = cv2.mean(dE)
	return "{0:.4f}".format(mde[0])
 

if __name__ == "__main__":    
	imagename1 = "one.png"
	imagename2 = "out.png"
	#print compute_SSIM(imagename1,imagename2)
	print compute_PSNR(imagename1,imagename2)
	#print compute_DELTAE(imagename1,imagename2)
	print "\n"