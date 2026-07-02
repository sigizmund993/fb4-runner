init:
	git submodule update --init --recursive
	cd ssl_packet_package && make init && make build
	python3 -m venv venv --system-site-packages
	./venv/bin/pip install -r ./requirements.txt
	sudo dpkg -i packages/hailort-pcie-driver_4.24.0_all.deb
	sudo dpkg -i packages/hailort_4.24.0_arm64.deb
	./venv/bin/pip install packages/hailort-4.24.0-cp31X-cp31X-linux_aarch64.whl

compile:
	g++ -shared -fPIC -O3 \
		-I/usr/include/hailo \
		-I/usr/include/hailo/tappas \
		yolo26_post.cpp -o libyolo26_post.so -lpthread
	g++ -std=c++17 -O2 \
		$$(pkg-config --cflags opencv4) \
		main.cpp -o ball_detector_cpp \
		$$(pkg-config --libs opencv4) \
		-L/usr/lib/aarch64-linux-gnu -lhailort -lpthread

run:
	LD_LIBRARY_PATH=/usr/lib:/usr/lib/aarch64-linux-gnu ./ball_detector_cpp

run-detect:
	./venv/bin/python3 detect_coords.py

run-gstreamer:
	./venv/bin/python3 detect_coords.py

.PHONY: init compile run run-detect run-gstreamer
