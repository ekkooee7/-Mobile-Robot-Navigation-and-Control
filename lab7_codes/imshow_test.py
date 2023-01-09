import cv2
import numpy

img = cv2.imread("jr.jpeg")


cv2.imshow("jr", img)

# img = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)

cv2.waitKey(0)


