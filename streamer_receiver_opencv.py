import cv2
import numpy as np

# Define the GStreamer pipeline
gst_pipeline = (
    "udpsrc port=5000 ! "
    "application/x-rtp,media=video,encoding-name=JPEG,payload=26 ! "
    "rtpjpegdepay ! "
    "jpegdec ! "
    "videoconvert ! "
    "appsink"
)

print("GStreamer pipeline:", gst_pipeline)

# Initialize the GStreamer pipeline with OpenCV
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: Unable to open GStreamer pipeline.")
    print("Is the pipeline syntax correct?")
    print("Do you have the necessary GStreamer plugins installed?")
    print("Does your OpenCV support GStreamer?")
    exit()

print("GStreamer pipeline opened successfully.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Unable to read from GStreamer pipeline.")
        break

    # Perform edge detection
    edges = cv2.Canny(frame, 100, 200)

    # Display the resulting frame
    cv2.imshow('Edges', edges)
    cv2.imshow('Original', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close OpenCV windows
cap.release()
cv2.destroyAllWindows()
