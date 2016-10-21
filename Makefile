all:
	robomain

robomain: robovision.o robotest.o
	g++ -std=c++11 -o robovision.o robotest.o

robovision.o: robovision.cc
	g++ -std=c++11 -c robovision.cc -L/opt/vc/lib -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_xfeatures2d -lopencv_imgcodecs

robotest.o: robotest.cc
	g++ -std=c++11 -c robotest.cc irobot-create.cc -pthread -L/opt/vc/lib -lserial -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc

clean:
	rm -f robomain *.o
