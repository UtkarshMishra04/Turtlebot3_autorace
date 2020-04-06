import numpy as np
import cv2
import math

def convert_hls(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)

def convert_hsv(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


def convert_gray_scale(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def apply_smoothing(image, kernel_size=3):
    return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

def detect_edges(image, low_threshold=20, high_threshold=1500):
    return cv2.Canny(image, low_threshold, high_threshold)

def hough_lines(image):

    return cv2.HoughLinesP(image, rho=1, theta=np.pi/180, threshold=20, minLineLength=1, maxLineGap=1000)

def filter_region(image, vertices):
    
    mask = np.zeros_like(image)
    if len(mask.shape)==2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255,)*mask.shape[2])        
    return cv2.bitwise_and(image, mask)

    
def select_region(image):
 
    rows, cols = image.shape[:2]
    bottom_left  = [cols*0.05, rows*0.95]
    top_left     = [cols*0.2, rows*0.4]
    bottom_right = [cols*0.95, rows*0.95]
    top_right    = [cols*0.8, rows*0.4] 
    
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    return filter_region(image, vertices)

def select_white_yellow(image):
    converted = convert_hsv(image)
    # white color mask
    lower = np.uint8([  0, 200,   0])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(converted, lower, upper)
    # yellow color mask
    lower = np.uint8([ 10,   0, 100])
    upper = np.uint8([ 40, 255, 255])
    yellow_mask = cv2.inRange(converted, lower, upper)
    # combine the mask
    mask = cv2.bitwise_or(white_mask, yellow_mask)
    return cv2.bitwise_and(image, image, mask= mask)

def mask_image(image):
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	# set lower and upper color limits
	low_val = (0,0,0)
	high_val = (179,45,96)
	# Threshold the HSV image 
	mask = cv2.inRange(hsv, low_val,high_val)
	# remove noise
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((8,8),dtype=np.uint8))
	# close mask
	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel=np.ones((20,20),dtype=np.uint8))

	# improve mask by drawing the convexhull 
	ret, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	for cnt in contours:
	    hull = cv2.convexHull(cnt)
	    cv2.drawContours(mask,[hull],0,(255), -1)
	# erode mask a bit to migitate mask bleed of convexhull
	mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel=np.ones((5,5),dtype=np.uint8))

	road_hsv = cv2.bitwise_and(hsv, hsv,mask=mask)
	# set lower and upper color limits
	low_val = (0,0,102)
	high_val = (179,255,255)
	# Threshold the HSV image 
	mask2 = cv2.inRange(road_hsv, low_val,high_val)
	# apply mask to original image
	
	return cv2.bitwise_and(image, image,mask=mask2)



def average_slope_intercept(lines):
    left_lines    = [] # (slope, intercept)
    left_weights  = [] # (length,)
    right_lines   = [] # (slope, intercept)
    right_weights = [] # (length,)
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            if x2==x1:
                continue # ignore a vertical line
            slope = (y2-y1)/(x2-x1)
            intercept = y1 - slope*x1
            length = np.sqrt((y2-y1)**2+(x2-x1)**2)
            if slope < 0: # y is reversed in image
                left_lines.append((slope, intercept))
                left_weights.append((length))
            else:
                right_lines.append((slope, intercept))
                right_weights.append((length))
    
    # add more weight to longer lines    
    left_lane  = np.dot(left_weights,  left_lines) /np.sum(left_weights)  if len(left_weights) >0 else None
    right_lane = np.dot(right_weights, right_lines)/np.sum(right_weights) if len(right_weights)>0 else None
    
    return left_lane, right_lane # (slope, intercept), (slope, intercept)

def make_line_points(y1, y2, line):
    """
    Convert a line represented in slope and intercept into pixel points
    """
    if line is None:
        return None
    
    slope, intercept = line
    
    # make sure everything is integer as cv2.line requires it
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    y1 = int(y1)
    y2 = int(y2)
    
    return ((x1, y1), (x2, y2))

def lane_lines(image, lines):
    left_lane, right_lane = average_slope_intercept(lines)
    
    y1 = image.shape[0] # bottom of the image
    y2 = y1*0.6         # slightly lower than the middle

    left_line  = make_line_points(y1, y2, left_lane)
    right_line = make_line_points(y1, y2, right_lane)
    
    return left_line, right_line


    
def draw_lane_lines(image, lines, color=[255, 0, 0], thickness=20):
    # make a separate image to draw lines and combine with the orignal later
    line_image = np.zeros_like(image)
    for line in lines:
        if line is not None:
            cv2.line(line_image, line[0],line[1],  color, thickness)
    return cv2.addWeighted(image, 1.0, line_image, 0.95, 0.0)



def draw_lines(image, lines, color=[255, 0, 0], thickness=1, make_copy=True):
    # the lines returned by cv2.HoughLinesP has the shape (-1, 1, 4)
    if make_copy:
        image = np.copy(image) # don't want to modify the original
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(image, (x1, y1), (x2, y2), color, thickness)
    return image


def main():


	image = cv2.imread('./assets/imgs/img1.png')

	#image = convert_gray_scale(image)
	#image = select_white_yellow(image)
	masked_image = mask_image(image)
	#image = apply_smoothing(image)
	edges_image = detect_edges(masked_image)
	region_image = select_region(edges_image)
	lines = hough_lines(edges_image)

	line_image = draw_lines(image,lines)

	line_lane = lane_lines(image,lines)

	print(lines.shape)

	final_image = draw_lane_lines(image,line_lane)


	cv2.imshow('image1',region_image)

	cv2.waitKey(0)

	cv2.destroyAllWindows()

	cv2.waitKey(1)


if __name__ == "__main__":
	main()