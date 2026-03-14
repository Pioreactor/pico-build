BUILD_DIR ?= build
CMAKE_BUILD_TYPE ?= Release
PICO_SDK_FETCH_FROM_GIT_TAG ?= 2.2.0

.PHONY: build clean

build:
	@command -v cmake >/dev/null 2>&1 || { echo "cmake is required. Install with: brew install cmake"; exit 1; }
	@command -v arm-none-eabi-gcc >/dev/null 2>&1 || { echo "arm-none-eabi-gcc is required. Install with: brew install arm-none-eabi-gcc"; exit 1; }
	cmake -S . -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) -DPICO_SDK_FETCH_FROM_GIT=ON -DPICO_SDK_FETCH_FROM_GIT_TAG=$(PICO_SDK_FETCH_FROM_GIT_TAG) -DPICO_NO_PICOTOOL=ON
	cmake --build $(BUILD_DIR) -j 2

clean:
	rm -rf $(BUILD_DIR)
