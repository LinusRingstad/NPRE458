import cv2
import numpy as np
import time

# --- Calibration Constants ---
SLOPE = 1.103
INTERCEPT = -45.108

# --- Region of Interest ---
START_X, END_X = 329, 968
START_Y, SIZE_Y = 371, 141

# --- Stacking Settings ---
ALPHA = 0.05
DURATION = 90  # seconds

cap = cv2.VideoCapture(1) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_EXPOSURE, 5)
cap.set(cv2.CAP_PROP_GAIN, 255)

def draw_graph(intensity, width=640, height=300, timer_text=None):
    graph = np.zeros((height, width, 3), dtype=np.uint8)
    points = len(intensity)
    if points == 0: return graph
    
    x_scale = width / points
    y_scale = height / 255
    
    for i in range(1, points):
        pt1 = (int((i-1) * x_scale), height - int(intensity[i-1] * y_scale))
        pt2 = (int(i * x_scale), height - int(intensity[i] * y_scale))
        cv2.line(graph, pt1, pt2, (0, 255, 0), 1)
        
    # Draw labels
    start_nm = (START_X * SLOPE) + INTERCEPT
    end_nm = (END_X * SLOPE) + INTERCEPT
    cv2.putText(graph, f"{start_nm:.1f}nm", (10, height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(graph, f"{end_nm:.1f}nm", (width-80, height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Draw timer/status
    if timer_text:
        cv2.putText(graph, timer_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
    return graph

stacked_profile = None
start_time = time.time()
recording = True

print(f"Recording for {DURATION} seconds...")

while True:
    ret, frame = cap.read()
    if not ret: break

    elapsed = time.time() - start_time
    remaining = DURATION - elapsed
    
    # 1. Processing (Stacking)
    roi = frame[START_Y:START_Y+SIZE_Y, START_X:END_X]
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    current_profile = np.mean(gray_roi, axis=0)

    if stacked_profile is None:
        stacked_profile = current_profile.copy()
    else:
        stacked_profile = cv2.addWeighted(stacked_profile, 1 - ALPHA, current_profile, ALPHA, 0)

    # 2. Display Logic
    if remaining > 0:
        timer_str = f"Recording: {int(remaining)}s left"
    else:
        timer_str = "Recording Finished. Press 'q' to save."
        recording = False

    rescaled_profile = cv2.normalize(stacked_profile, None, 0, 255, cv2.NORM_MINMAX)
    spectrum_graph = draw_graph(rescaled_profile, width=(END_X - START_X), height=300, timer_text=timer_str)

    cv2.imshow('1. Live ROI (Raw)', roi)
    cv2.imshow('2. Stacked Spectrum', spectrum_graph)

    # Exit condition: Press 'q' to stop early, or auto-stop after 60s
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if not recording and key == ord('q'):
        break

# --- Post-Processing & CSV Export ---
# --- Post-Processing & CSV Export ---
if stacked_profile is not None:
    print("Saving data to spectrum_data.csv...")

    # Create Wavelength Array
    pixel_bins = np.arange(len(stacked_profile)) + START_X
    wavelengths = (pixel_bins * SLOPE) + INTERCEPT

    # Combine columns
    data_to_save = np.column_stack((wavelengths, stacked_profile))

    # Save
    np.savetxt("spectrum_data.csv", data_to_save, delimiter=",", header="Wavelength_nm,Intensity", comments="")
    print("Export complete.")
else:
    print("Error: No frames were captured. Please check your camera connection and try again.")

cap.release()
cv2.destroyAllWindows()