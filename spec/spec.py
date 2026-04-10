import cv2
import numpy as np
import time

# --- Calibration Constants ---
SLOPE = 1.103
INTERCEPT = -45.108
CALIBRATION_TIME = 15  # Seconds to collect background

# --- Region of Interest ---
START_X, END_X = 329, 968
START_Y, SIZE_Y = 371, 141

# --- Stacking Settings ---
ALPHA = 0.05

cap = cv2.VideoCapture(1) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) 
cap.set(cv2.CAP_PROP_EXPOSURE, 5)     
cap.set(cv2.CAP_PROP_GAIN, 255)

def draw_graph(intensity, width=640, height=300, label="Spectrum"):
    """Creates an OpenCV image representing the spectrum graph."""
    graph = np.zeros((height, width, 3), dtype=np.uint8)
    points = len(intensity)
    if points == 0: return graph
    
    x_scale = width / points
    y_scale = height / 255
    
    for i in range(1, points):
        pt1 = (int((i-1) * x_scale), height - int(intensity[i-1] * y_scale))
        pt2 = (int(i * x_scale), height - int(intensity[i] * y_scale))
        cv2.line(graph, pt1, pt2, (0, 255, 0), 1)
        
    start_nm = (START_X * SLOPE) + INTERCEPT
    end_nm = (END_X * SLOPE) + INTERCEPT
    cv2.putText(graph, f"{label} | {start_nm:.1f}nm", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(graph, f"{end_nm:.1f}nm", (width-80, height-10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return graph

# Initialize profiles
stacked_profile = None
background_profile = None
start_time = time.time()

print(f"--- CALIBRATING: Keep sensor covered or light constant for {CALIBRATION_TIME}s ---")

while True:
    ret, frame = cap.read()
    if not ret: break

    elapsed = time.time() - start_time
    is_calibrating = elapsed < CALIBRATION_TIME

    # 1. Extract ROI and convert to grayscale
    roi = frame[START_Y:START_Y+SIZE_Y, START_X:END_X]
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    current_profile = np.mean(gray_roi, axis=0)

    if is_calibrating:
        # Build the background profile
        if background_profile is None:
            background_profile = current_profile.copy()
        else:
            background_profile = cv2.addWeighted(background_profile, 1 - ALPHA, current_profile, ALPHA, 0)
        
        # FIX: Flatten the array here
        display_profile = cv2.normalize(background_profile, None, 0, 255, cv2.NORM_MINMAX).flatten()
        graph_label = "Capturing Background..."
    
    else:
        subtracted = np.maximum(current_profile - background_profile, 0)

        if stacked_profile is None:
            stacked_profile = subtracted.copy()
        else:
            stacked_profile = cv2.addWeighted(stacked_profile, 1 - ALPHA, subtracted, ALPHA, 0)

        # FIX: Flatten the array here
        display_profile = cv2.normalize(stacked_profile, None, 0, 255, cv2.NORM_MINMAX).flatten()
        graph_label = "Background Subtracted"

    # 5. Visualization
    spectrum_graph = draw_graph(display_profile, width=(END_X - START_X), height=300, label=graph_label)

    cv2.imshow('1. Live ROI', roi)
    cv2.imshow('2. Spectrum', spectrum_graph)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        if not is_calibrating:
            peak_pixel_local = np.argmax(stacked_profile)
            peak_pixel_global = START_X + peak_pixel_local
            peak_nm = (peak_pixel_global * SLOPE) + INTERCEPT
            print(f"\n--- Results ---")
            print(f"Strongest Peak (Subtracted) detected at: {peak_nm:.2f} nm")
        break

cap.release()
cv2.destroyAllWindows()