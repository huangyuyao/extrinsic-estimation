all:
	g++ prog.cc -I/usr/local/include -L/usr/local/lib -lopencv_calib3d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -larmadillo -std=c++11 -o ee
clean:
	rm ee
