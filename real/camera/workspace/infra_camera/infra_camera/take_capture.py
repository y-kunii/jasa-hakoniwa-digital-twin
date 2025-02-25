#--- OpenCV
import cv2

camera_cap = cv2.VideoCapture(0)
#camera_cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
#camera_cap.set(cv2.CAP_PROP_EXPOSURE, -6)
ret, target_pic = camera_cap.read()
cv2.imwrite('/home/jasa/work/template/test_1.png', target_pic)


while True:
    ret, target_pic = camera_cap.read()
    cv2.imshow('image', target_pic)



