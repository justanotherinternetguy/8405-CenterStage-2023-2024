import numpy as np
import cv2

cam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
object_center = (1, 1)

while True:
    check, frame = cam.read()
    lower_val = (0, 100, 100)
    upper_val = (10, 255, 255)

    lower_val2 = (160, 100, 100)
    upper_val2 = (190, 255, 255)
    temp = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(temp, lower_val, upper_val)
    mask2 = cv2.inRange(temp, lower_val2, upper_val2)
    final_mask = cv2.bitwise_or(mask, mask2)
    # print(temp.shape[0]) # 640 x 480
    
    kernel = np.ones((6,6), np.uint8)
    final_mask = cv2.erode(final_mask, kernel, iterations=1)
    final_mask = cv2.dilate(final_mask, kernel, iterations=1)
    res = cv2.bitwise_and(frame, frame, mask=final_mask)

    contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize the object area to zero
    object_area = 0

    # If there are contours, calculate the area of the largest one
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        object_area = cv2.contourArea(largest_contour)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            object_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        cv2.circle(res, object_center, 5, (0, 0, 255), -1)  # Mark the center with a red circle

    # Display the object area
    cv2.putText(res, f'Object Area: {object_area:.2f} pixels', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('video', res)
    print(object_center)

    third = temp.shape[1]/3

    if object_center[0] < third:
        print('left')
    if object_center[0] > 2 * third:
        print('right')
    if (object_center[0] > third) and (object_center[0] < 2 * third):
        print('center yay')
    

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()