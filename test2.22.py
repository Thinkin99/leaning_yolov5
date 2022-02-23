import cv2
import tuoyuan

cap = cv2.VideoCapture(1)
tuoyuan.circle_detect_init()
while 1:
    ret, img = cap.read()
    # img = cv2.imread("1.jpg")
    tuoyuan.circle_dectect(img)
    # tuoyuan.ell_detect(img)
    key = cv2.waitKey(20)
    if key & 0xFF == ord('q') or key == 27:
        break
cap.release()
cv2.destroyAllWindows()
