import cv2
import numpy as np
import time  # Used for timing updates

# Target brightness level
TARGET_BRIGHTNESS = 75  

# Update interval (in seconds)
UPDATE_INTERVAL = 3  

def calculate_brightness(frame):
    """
    Compute the average brightness of the image.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
    return np.mean(gray)  # Compute the mean brightness

def determine_brightness_adjustment(current_brightness):
    """
    Determine how much to adjust the brightness to reach TARGET_BRIGHTNESS.
    Prints both the current brightness and the required adjustment.
    """
    brightness_diff = TARGET_BRIGHTNESS - current_brightness

    # Scale adjustment (proportional control)
    adjustment = brightness_diff * 0.5  # Change factor (can be fine-tuned)

    print(f"\nCurrent Brightness: {current_brightness:.2f}")

    if brightness_diff > 0:
        print(f"→ Increase brightness by: {adjustment:.2f}")
    elif brightness_diff < 0:
        print(f"→ Decrease brightness by: {-adjustment:.2f}")
    else:
        print("→ Brightness is optimal.")

def main():
    cap = cv2.VideoCapture(0)  # Open the default camera

    if not cap.isOpened():
        print("Error: Cannot access the camera.")
        return

    last_update_time = time.time()  # Store the last update time

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame.")
            break

        brightness = calculate_brightness(frame)

        # Check if UPDATE_INTERVAL seconds have passed since last update
        current_time = time.time()
        if current_time - last_update_time >= UPDATE_INTERVAL:
            determine_brightness_adjustment(brightness)
            last_update_time = current_time  # Reset the timer

        # Show the video feed
        cv2.imshow("Camera Feed", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
