
ROOT_DIR:=$(PWD)
SRC_ROOT:=$(ROOT_DIR)/src

BUILD_TYPE:="Debug"
BUILD_DIR=$(ROOT_DIR)/build
CONAN_MARKER=$(BUILD_DIR)/conanbuildinfo.cmake
TEST_EXE=${BUILD_DIR}/bin/testall
CLI_EXE=${BUILD_DIR}/bin/tracker

#-------------------------------------------------------------------
#  Part 2: Invoke the call to make in the build directory
#-------------------------------------------------------------------
.PHONY: build
build: debug 

.PHONY: debug
debug: cmake-debug
	cd $(BUILD_DIR) && ninja

.PHONY: release
release: cmake-release
	cd $(BUILD_DIR) && ninja

${BUILD_DIR}:
	mkdir ${BUILD_DIR}

.PHONY: conan
conan: $(CONAN_MARKER)

$(CONAN_MARKER): ${BUILD_DIR}
	@ echo "did not find conan-build-info file... rebuilding: "
	cd $(BUILD_DIR) && conan install --build=missing ..

cmake-debug: $(CONAN_MARKER)
	cd $(BUILD_DIR) && \
		cmake .. -DCMAKE_BUILD_TYPE=Debug -GNinja

cmake-release: $(CONAN_MARKER)
	cd $(BUILD_DIR) && \
		cmake .. -DCMAKE_BUILD_TYPE=Release -GNinja

clean: 
	rm -rf build/*

format:
	@echo clang-tidy src

.PHONY: run
run: debug
	@ echo Running CLI:
	${CLI_EXE} -vv

.PHONY: help
help: 
	@ echo "Notes:"
	@ echo "....nyi"

.PHONY: test testgrid
test: test-all

test-all: build
	clear
	$(TEST_EXE)

