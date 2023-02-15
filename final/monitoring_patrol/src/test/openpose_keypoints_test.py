import cv2
import numpy as np
import openpose as op

params = dict()
params["model_folder"] = "path/to/openpose/models/"

opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()


// capture the video frame
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    datum = op.Datum()
    datum.cvInputData = frame
    opWrapper.emplaceAndPop([datum])

    poseKeypoints = datum.poseKeypoints


// check the body parts position keypoints


// display Outputs

    cv2.imshow("OpenPose 1.6.0 - Tutorial (python)", datum.cvOutputData)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
