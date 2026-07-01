init:
	git submodule update --init --recursive
	cd ssl_packet_package && make init && make build
	python3 -m venv venv --system-site-packages
	./venv/bin/pip install -r ./requirements.txt
	sudo dpkg -i packages/hailort-pcie-driver_4.24.0_all.deb
	sudo dpkg -i packages/hailort_4.24.0_arm64.deb
	./venv/bin/pip install packages/hailort-4.24.0-cp31X-cp31X-linux_aarch64.whl
run:
	./venv/bin/python3 main.py
.PHONY: init run