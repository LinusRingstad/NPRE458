import cv2
import numpy as np


#calibration likely has to be re-done
# from INI: TrimPoint1 (pixel 436) = 435.8nm, TrimPoint2 (pixel 546) = 546.1nm
# Slope (m) = (546.1 - 435.8) / (546 - 436) = 1.103 nm/pixel
# Intercept (b) = 435.8 - (1.103 * 436) = -45.108
SLOPE = 1.103
INTERCEPT = -45.108

#region of interest
START_X, END_X = 329, 968
START_Y, SIZE_Y = 371, 141

cap = cv2.VideoCapture(0) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

def draw_graph(intensity, width=640, height=400):
    """Creates an OpenCV image representing the spectrum graph."""
    graph = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Normalize intensity to fit graph height (0 to 255 -> 0 to height)
    points = len(intensity)
    if points == 0: return graph
    
    # Scaling factors
    x_scale = width / points
    y_scale = height / 255
    
    for i in range(1, points):
        # Calculate coordinates
        pt1 = (int((i-1) * x_scale), height - int(intensity[i-1] * y_scale))
        pt2 = (int(i * x_scale), height - int(intensity[i] * y_scale))
        
        # Draw line (Green for the plot)
        cv2.line(graph, pt1, pt2, (0, 255, 0), 1)
        
    # Add simple labels for start/end wavelength
    start_nm = (START_X * SLOPE) + INTERCEPT
    end_nm = (END_X * SLOPE) + INTERCEPT
    cv2.putText(graph, f"{start_nm:.1f}nm", (10, height-10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(graph, f"{end_nm:.1f}nm", (width-80, height-10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
    return graph

while True:
    ret, frame = cap.read()
    if not ret: break

    roi = frame[START_Y:START_Y+SIZE_Y, START_X:END_X]
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
    # Average the rows 
    intensity_profile = np.mean(gray_roi, axis=0)

    spectrum_graph = draw_graph(intensity_profile, width=(END_X - START_X), height=300)

    cv2.imshow('1. Live Spectrum Image', roi)
    cv2.imshow('2. Intensity Graph', spectrum_graph)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        peak_pixel_local = np.argmax(intensity_profile)
        peak_pixel_global = START_X + peak_pixel_local
        peak_nm = (peak_pixel_global * SLOPE) + INTERCEPT
        print(f"Strongest Peak detected at: {peak_nm:.2f} nm")
        break

cap.release()
cv2.destroyAllWindows()