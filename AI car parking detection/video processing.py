import cv2
import pickle
import cvzone
import numpy as np

# Video feed
cap = cv2.VideoCapture('carParkingInput.mp4')

# Load the ROI positions from the parkingSlotPosition file
with open('parkingSlotPosition', 'rb') as file:
    position_list = pickle.load(file)

# Define width and height
width, height = 107, 48

def checkParkingSpace(imgPro):
    spaceCounter = 0
    for pos in position_list:
        x, y = pos  # Unpack the position tuple
        # Crop the image based on ROI
        imgCrop = imgPro[y:y+height, x:x+width]
        # Counting the pixel value from cropped image
        count = cv2.countNonZero(imgCrop)
        if count < 900:
            color = (0, 255, 0)
            thickness = 2
            spaceCounter += 1
        else:
            color = (0, 0, 255)
            thickness = 2
        # Draw the rectangle based on the condition defined above

        cv2.rectangle(img, pos, (pos[0] + width, pos[1] + height), color,thickness)
    # Display the available parking slot count / total parking slot count
    cvzone.putTextRect(img, f'Free: {spaceCounter}/{len(position_list)}', (100, 50), scale=3, thickness=5, offset=20,colorR=(0, 200, 0))
while True:
    # Looping video
    if cap.get(cv2.CAP_PROP_POS_FRAMES) == cap.get(cv2.CAP_PROP_FRAME_COUNT):
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    # Reading frame by frame from video
    success, img = cap.read()
    if not success:
        break
    # Converting to grayscale image
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray, (3, 3), 1)  # Applying blur to image
    # Applying threshold to the image
    imgThreshold = cv2.adaptiveThreshold(imgBlur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 25, 16)
    imgMedian = cv2.medianBlur(imgThreshold, 5)  # Applying blur to image
    kernel = np.ones((3, 3), np.uint8)
    imgDilate = cv2.dilate(imgMedian, kernel, iterations=1)
    # Passing dilate image to the function
    checkParkingSpace(imgDilate)
    cv2.imshow("Image", img)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
