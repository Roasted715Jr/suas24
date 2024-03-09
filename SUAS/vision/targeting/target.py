import cv2
import numpy as np
from matplotlib import pyplot as plt
import pytesseract

CAMERA_RES = "4056x3040"

pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'

# # image path
#path = "SUASTestImage.png"
path = "TargetTestImage7.png"
# # Reading an image in default mode:
inputImage = cv2.imread(path)

# # Deep copy for results:
# inputImageCopy = inputImage.copy()

# # # Convert RGB to grayscale:
grayscaleImage = cv2.cvtColor(inputImage, cv2.COLOR_BGR2GRAY)

edges = cv2.Canny(inputImage, 100, 200)

contours, hierarchy = cv2.findContours(edges, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
image_copy = inputImage.copy()

for i in range(len(contours)):
    if hierarchy[0][i][3] == -1:
        # Contour with a parent index of -1 is an external contour
        letter_contour = contours[i]
        break

cv2.drawContours(image_copy, [letter_contour], -1, (0, 255, 0), 2)


x, y, w, h = cv2.boundingRect(letter_contour)

padding = 10
x -= padding
y -= padding
w += 2 * padding
h += 2 * padding
# Ensure that the bounding box stays within image bounds
x = max(0, x)
y = max(0, y)
w = min(inputImage.shape[1] - x, w)
h = min(inputImage.shape[0] - y, h)
# Extract the region of interest (ROI) using the bounding box
letter_roi = inputImage[y:y+h, x:x+w]

# Convert the ROI to grayscale
gray_letter = cv2.cvtColor(letter_roi, cv2.COLOR_BGR2GRAY)
# Example: Apply Gaussian blur for smoothing
gray_letter = cv2.GaussianBlur(gray_letter, (3, 3), 0)

# Apply thresholding if needed
_, binary_image = cv2.threshold(gray_letter, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

# Example: Apply Gaussian blur for denoising
blurred_image = cv2.GaussianBlur(binary_image, (3, 3), 0)

# Use pytesseract to perform OCR on the binary image
recognized_text = pytesseract.image_to_string(blurred_image, config='--psm 6 --oem 3 -l eng')
print("Recognized text:", recognized_text)


#letter_roi = inputImage[y:y+h, x:x+w]



# Convert the region to grayscale
#gray_letter = cv2.cvtColor(letter_roi, cv2.COLOR_BGR2GRAY)

# Apply thresholding if needed
#_, binary_image = cv2.threshold(gray_letter, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

# Use pytesseract to perform OCR on the binary image
#recognized_text = pytesseract.image_to_string(binary_image)
#print("Recognized text:", recognized_text)



print(len(contours), "objects were found in this image.")


# Save the image to a file
cv2.imwrite('test_image.jpg', cv2.cvtColor(image_copy, cv2.COLOR_RGB2BGR))
cv2.imwrite('gray.jpg', cv2.cvtColor(blurred_image, cv2.COLOR_RGB2BGR))
