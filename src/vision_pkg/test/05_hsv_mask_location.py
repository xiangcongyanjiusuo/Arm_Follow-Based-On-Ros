import cv2
import numpy as np

def nothing(x):
    pass

cv2.namedWindow('Controls')
cv2.createTrackbar('H Min', 'Controls', 89, 179, nothing)
cv2.createTrackbar('H Max', 'Controls', 117, 179, nothing)
cv2.createTrackbar('S Min', 'Controls', 146, 255, nothing)
cv2.createTrackbar('S Max', 'Controls', 255, 255, nothing)
cv2.createTrackbar('V Min', 'Controls', 73, 255, nothing)
cv2.createTrackbar('V Max', 'Controls', 255, 255, nothing)
cv2.createTrackbar('Erode', 'Controls', 3, 10, nothing)
cv2.createTrackbar('Dilate', 'Controls', 5, 10, nothing)

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    h_min = cv2.getTrackbarPos('H Min', 'Controls')
    h_max = cv2.getTrackbarPos('H Max', 'Controls')
    s_min = cv2.getTrackbarPos('S Min', 'Controls')
    s_max = cv2.getTrackbarPos('S Max', 'Controls')
    v_min = cv2.getTrackbarPos('V Min', 'Controls')
    v_max = cv2.getTrackbarPos('V Max', 'Controls')
    erode_iter = cv2.getTrackbarPos('Erode', 'Controls')
    dilate_iter = cv2.getTrackbarPos('Dilate', 'Controls')
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower, upper)
    
    kernel = np.ones((5, 5), np.uint8)
    if erode_iter > 0:
        mask = cv2.erode(mask, kernel, iterations=erode_iter)
    if dilate_iter > 0:
        mask = cv2.dilate(mask, kernel, iterations=dilate_iter)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    result = frame.copy()
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cx = x + w // 2
            cy = y + h // 2
            cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(result, f'({cx},{cy})', (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
