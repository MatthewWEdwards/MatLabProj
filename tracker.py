import cv2

cap = cv2.VideoCapture(0)
cap.set(3, 640) #WIDTH
cap.set(4, 480) #HEIGHT

face_cascade = cv2.CascadeClassifier('/usr/local/Cellar/opencv/3.3.1_1/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('/usr/local/Cellar/opencv/3.3.1_1/share/OpenCV/haarcascades/haarcascade_eye.xml')

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    print(len(faces))
    # Display the resulting frame
    for (x,y,w,h) in faces:
         cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
         roi_gray = gray[y:y+h, x:x+w]
         roi_color = frame[y:y+h, x:x+w]
         eyes = eye_cascade.detectMultiScale(roi_gray)
         for (ex,ey,ew,eh) in eyes:
             cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)

    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

#!/usr/bin/python
# The contents of this file are in the public domain. See LICENSE_FOR_EXAMPLE_PROGRAMS.txt
#
#   This example program shows how to find frontal human faces in an image and
#   estimate their pose.  The pose takes the form of 68 landmarks.  These are
#   points on the face such as the corners of the mouth, along the eyebrows, on
#   the eyes, and so forth.
#
#   The face detector we use is made using the classic Histogram of Oriented
#   Gradients (HOG) feature combined with a linear classifier, an image pyramid,
#   and sliding window detection scheme.  The pose estimator was created by
#   using dlib's implementation of the paper:
#      One Millisecond Face Alignment with an Ensemble of Regression Trees by
#      Vahid Kazemi and Josephine Sullivan, CVPR 2014
#   and was trained on the iBUG 300-W face landmark dataset (see
#   https://ibug.doc.ic.ac.uk/resources/facial-point-annotations/):  
#      C. Sagonas, E. Antonakos, G, Tzimiropoulos, S. Zafeiriou, M. Pantic. 
#      300 faces In-the-wild challenge: Database and results. 
#      Image and Vision Computing (IMAVIS), Special Issue on Facial Landmark Localisation "In-The-Wild". 2016.
#   You can get the trained model file from:
#   http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2.
#   Note that the license for the iBUG 300-W dataset excludes commercial use.
#   So you should contact Imperial College London to find out if it's OK for
#   you to use this model file in a commercial product.
#
#
#   Also, note that you can train your own models using dlib's machine learning
#   tools. See train_shape_predictor.py to see an example.
#
#
# COMPILING/INSTALLING THE DLIB PYTHON INTERFACE
#   You can install dlib using the command:
#       pip install dlib
#
#   Alternatively, if you want to compile dlib yourself then go into the dlib
#   root folder and run:
#       python setup.py install
#   or
#       python setup.py install --yes USE_AVX_INSTRUCTIONS
#   if you have a CPU that supports AVX instructions, since this makes some
#   things run faster.  
#
#   Compiling dlib should work on any operating system so long as you have
#   CMake and boost-python installed.  On Ubuntu, this can be done easily by
#   running the command:
#       sudo apt-get install libboost-python-dev cmake
#
#   Also note that this example requires scikit-image which can be installed
#   via the command:
#       pip install scikit-image
#   Or downloaded from http://scikit-image.org/download.html. 

##import cv2
##import sys
##sys.path.append('/Applications/anaconda/lib/python2.7/site-packages')
##import os
##import dlib
##import glob
##from skimage import io
##
##
###if len(sys.argv) != 3:
###    print(
###        "Give the path to the trained shape predictor model as the first "
###        "argument and then the directory containing the facial images.\n"
###        "For example, if you are in the python_examples folder then "
###        "execute this program by running:\n"
###        "    ./face_landmark_detection.py shape_predictor_68_face_landmarks.dat ../examples/faces\n"
###        "You can download a trained facial shape predictor from:\n"
###        "    http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2")
###    exit()
##
##predictor_path = 'shape_predictor_68_face_landmarks.dat'#sys.argv[1]
###faces_folder_path = sys.argv[2]
##
##detector = dlib.get_frontal_face_detector()
##predictor = dlib.shape_predictor(predictor_path)
##win = dlib.image_window()
##cap = cv2.VideoCapture(0)
##cap.set(3, 640) #WIDTH
##cap.set(4, 480) #HEIGHT
##
##while(True):
##    # Capture frame-by-frame
##    ret, frame = cap.read()
##
##    win.clear_overlay()
##    win.set_image(frame)
##
##    # Ask the detector to find the bounding boxes of each face. The 1 in the
##    # second argument indicates that we should upsample the image 1 time. This
##    # will make everything bigger and allow us to detect more faces.
##    dets = detector(frame, 1)
##    print("Number of faces detected: {}".format(len(dets)))
##    for k, d in enumerate(dets):
##        print("Detection {}: Left: {} Top: {} Right: {} Bottom: {}".format(
##            k, d.left(), d.top(), d.right(), d.bottom()))
##        # Get the landmarks/parts for the face in box d.
##        shape = predictor(img, d)
##        print("Part 0: {}, Part 1: {} ...".format(shape.part(0),
##                                                  shape.part(1)))
##        # Draw the face landmarks on the screen.
##        win.add_overlay(shape)
##
##    win.add_overlay(dets)
##    cv2.imshow('frame',win)
## #   dlib.hit_enter_to_continue()
