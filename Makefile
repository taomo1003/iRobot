<<<<<<< HEAD
all:
	g++ -std=c++11 -o robotest robotest.cc robovision.cc open_CV_image.cc nav.cc safety.cc irobot-create.cc -pthread -L/opt/vc/lib -lserial -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_xfeatures2d -lopencv_calib3d -lopencv_features2d 

clean:
	rm robotest
=======
all:
	g++ -std=c++11 -o robotest robotest.cc nav.cc safety.cc irobot-create.cc -pthread -L/opt/vc/lib -lserial -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc

clean:
	rm robotest
>>>>>>> 1f4a68c02e75637d0d2df24e2ef66e7300c440b0
