import cv2
import numpy as np

# --- Calibration Constants ---
SLOPE = 1.103
INTERCEPT = -45.108

# --- Region of Interest ---
START_X, END_X = 329, 968
START_Y, SIZE_Y = 371, 141
ALPHA = 0.2  # Increased for faster reaction

cap = cv2.VideoCapture(1) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

def draw_graph(intensity, width=640, height=300, label="Spectrum"):
    # Ensure 1D and float
    intensity = np.array(intensity, dtype=np.float32).flatten()
    graph = np.zeros((height, width, 3), dtype=np.uint8)
    
    if intensity.size == 0: return graph

    # Normalization for display
    min_val = np.min(intensity)
    max_val = np.max(intensity)
    
    if max_val - min_val > 0:
        norm_intens = ((intensity - min_val) / (max_val - min_val)) * 255
    else:
        norm_intens = intensity * 0

    x_scale = width / len(norm_intens)
    y_scale = height / 255
    
    for i in range(1, len(norm_intens)):
        pt1 = (int((i-1) * x_scale), height - int(norm_intens[i-1] * y_scale))
        pt2 = (int(i * x_scale), height - int(norm_intens[i] * y_scale))
        cv2.line(graph, pt1, pt2, (0, 255, 0), 1)
        
    cv2.putText(graph, label, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    return graph

# Initialization
background_profile = None
stacked_profile = None

print("CONTROLS:")
print("Press 'b' to CAPTURE background (Zero the sensor)")
print("Press 'r' to RESET/Clear background")
print("Press 'q' to QUIT")

while True:
    ret, frame = cap.read()
    if not ret: break

    # 1. Extract ROI and Process Signal
    roi = frame[START_Y:START_Y+SIZE_Y, START_X:END_X]
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    current_raw = np.mean(gray_roi, axis=0, dtype=np.float32)

    # 2. Apply Stacking (Smoothes the raw signal)
    if stacked_profile is None:
        stacked_profile = current_raw
    else:
        stacked_profile = (1 - ALPHA) * stacked_profile + ALPHA * current_raw

    # 3. Handle Subtraction Logic
    if background_profile is not None:
        # Show cleaned signal
        display_data = np.maximum(stacked_profile - background_profile, 0)
        status_text = "SUBTRACTING BACKGROUND (Active)"
        text_color = (0, 255, 0) # Green
    else:
        # Show raw signal
        display_data = stacked_profile
        status_text = "RAW SPECTRUM (Press 'b' to zero)"
        text_color = (0, 0, 255) # Red

    # 4. Visualization
    spectrum_graph = draw_graph(display_data, width=roi.shape[1], label=status_text)
    
    # Overlay status on the ROI window
    cv2.putText(roi, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)

    cv2.imshow('1. Live ROI', roi)
    cv2.imshow('2. Spectrum Graph', spectrum_graph)

    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('b'):
        # Freeze the current stacked profile as the background
        background_profile = np.copy(stacked_profile)
        print("Background Captured!")
        
    elif key == ord('r'):
        background_profile = None
        print("Background Reset to Raw Mode.")
        
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()