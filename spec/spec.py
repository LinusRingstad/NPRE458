import cv2
import numpy as np

# --- Calibration Constants ---
SLOPE = 1.103
INTERCEPT = -45.108

# --- Region of Interest ---
START_X, END_X = 329, 968
START_Y, SIZE_Y = 371, 141

# --- Stacking Settings ---
# ALPHA determines how much "weight" the new frame has.
# 0.1 = heavy stacking (very smooth, but slow to react)
# 0.5 = light stacking (faster reaction, more noise)
ALPHA = 0.1

cap = cv2.VideoCapture(1) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# Adjust these based on your specific UVC driver response
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # Manual mode
cap.set(cv2.CAP_PROP_EXPOSURE, 1)     # Larger values = brighter 2S to 2P0

def draw_graph(intensity, width=640, height=300):
    """Creates an OpenCV image representing the spectrum graph."""
    graph = np.zeros((height, width, 3), dtype=np.uint8)
    
    points = len(intensity)
    if points == 0: return graph
    
    x_scale = width / points
    # Since we normalize the data to 255, y_scale remains consistent
    y_scale = height / 255
    
    for i in range(1, points):
        pt1 = (int((i-1) * x_scale), height - int(intensity[i-1] * y_scale))
        pt2 = (int(i * x_scale), height - int(intensity[i] * y_scale))
        cv2.line(graph, pt1, pt2, (0, 255, 0), 1)
        
    start_nm = (START_X * SLOPE) + INTERCEPT
    end_nm = (END_X * SLOPE) + INTERCEPT
    cv2.putText(graph, f"{start_nm:.1f}nm", (10, height-10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(graph, f"{end_nm:.1f}nm", (width-80, height-10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
    return graph

# Initialize the stacked profile as None
stacked_profile = None

print("Press 'q' to quit and see peak detection.")

while True:
    ret, frame = cap.read()
    if not ret: break

    # 1. Extract ROI and convert to grayscale
    roi = frame[START_Y:START_Y+SIZE_Y, START_X:END_X]
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
    # 2. Average the rows to get the current frame profile
    current_profile = np.mean(gray_roi, axis=0)

    # 3. Frame Stacking (Running Average)
    if stacked_profile is None:
        stacked_profile = current_profile.copy()
    else:
        # Formula: (1 - alpha) * old_data + alpha * new_data
        stacked_profile = cv2.addWeighted(stacked_profile, 1 - ALPHA, current_profile, ALPHA, 0)

    # 4. Rescaling (Normalization)
    # This stretches the profile so the highest peak is always 255
    rescaled_profile = cv2.normalize(stacked_profile, None, 0, 255, cv2.NORM_MINMAX)

    # 5. Visualization
    spectrum_graph = draw_graph(rescaled_profile, width=(END_X - START_X), height=300)

    cv2.imshow('1. Live ROI (Raw)', roi)
    cv2.imshow('2. Stacked & Rescaled Spectrum', spectrum_graph)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        peak_pixel_local = np.argmax(stacked_profile)
        peak_pixel_global = START_X + peak_pixel_local
        peak_nm = (peak_pixel_global * SLOPE) + INTERCEPT
        print(f"\n--- Results ---")
        print(f"Strongest Peak detected at: {peak_nm:.2f} nm")
        break

cap.release()
cv2.destroyAllWindows()