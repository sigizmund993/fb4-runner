SHELL := /bin/bash

HEF     ?= ./10_hours_brainrot.hef
SO      ?= ./libyolo26_post.so
MEDIAMTX ?= /root/mediamtx
MEDIAMTX_CFG ?= /root/mediamtx.yml

install-deps:
	# GStreamer + libcamera
	sudo apt install -y \
		gstreamer1.0-libcamera \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-ugly \
		gstreamer1.0-tools \
		gstreamer1.0-rtsp \
		python3-gi \
		python3-picamera2
	# Hailo TAPPAS (GStreamer плагины hailonet, hailofilter, hailooverlay)
	sudo apt install -y hailo-tappas-core python3-hailo-tappas

install-driver:
	# Kernel headers для сборки PCIe драйвера
	sudo apt install -y \
		linux-headers-$$(uname -r)-common-rpi \
		linux-headers-$$(uname -r)-rpi-2712 2>/dev/null || \
	sudo apt install -y \
		linux-headers-$$(uname -r | sed 's/+rpt.*//')+rpt-common-rpi \
		linux-headers-$$(uname -r)
	# Сборка и установка драйвера Hailo PCIe
	cd /usr/src/hailort-pcie-driver/hailort/drivers/linux/pcie && \
		sudo make -C /lib/modules/$$(uname -r)/build M=$$(pwd) modules && \
		sudo make -C /lib/modules/$$(uname -r)/build M=$$(pwd) modules_install && \
		sudo depmod -a && \
		sudo cp 51-hailo-udev.rules /etc/udev/rules.d/ && \
		sudo udevadm control --reload-rules
	sudo modprobe hailo_pci
	# Откат драйвера до версии библиотеки (4.23.0) если нужно
	# sudo apt install hailort-pcie-driver=4.23.0

install-mediamtx:
	cd /root && \
	LATEST=$$(curl -s https://github.com | grep -oP '"tag_name": "\K[^"]+') && \
	wget -q "https://github.com{LATEST}/mediamtx_$${LATEST}_linux_arm64.tar.gz" && \
	tar -xzf mediamtx_$${LATEST}_linux_arm64.tar.gz && \
	rm mediamtx_$${LATEST}_linux_arm64.tar.gz


install: install-deps install-driver install-mediamtx

compile-so:
	# Только .so (быстро, без opencv)
	g++ -shared -fPIC -O3 \
		-I/usr/include/hailo \
		-I/usr/include/hailo/tappas \
		yolo26_post.cpp -o libyolo26_post.so -lpthread


run:
	@trap 'kill 0' SIGINT; \
	./run.sh --no-stream > /dev/null& \
	sleep 1;\
	./venv/bin/python main.py & \
	wait
run-stream:
	echo "running in STREAM mode. run ffplay -rtsp_transport tcp -fflags nobuffer -flags low_delay rtsp://<RPI_IP>:8554/ball to watch stream"
	@trap 'kill 0' SIGINT; \
	./run.sh  > /dev/null& \
	sleep 3;\
	./venv/bin/python main.py & \
	wait
.PHONY: install install-deps install-driver install-mediamtx \
        compile-so \
        run run-stream 