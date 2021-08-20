DIST=Norbert-drivers
PACKAGE_NAME=Norbert-drivers
LOCAL_UPLOAD_CONF= conf/upload.conf
OBJ=Norbert-drivers.hex
LOCAL_OBJ_DIR= build-nano328
LOCAL_SCRIPT_DIR= scripts
LOCAL_ARDUINO_MAKEFILE= Arduino-Makefile
DIST_INSTALL_PATH= /etc/norbert-drivers
DIST_OBJ_DIR=$(DIST_INSTALL_PATH)/obj
DIST_SCRIPT_DIR=$(DIST_INSTALL_PATH)/scripts

DIST_MACHINE=ubuntu@192.168.1.50
DEB_PATH=/tmp

PACKAGE_NAME=norbert-drivers
BUILD_CONF_FILE = conf/build.conf

TARGET_HEX= build-nano328/Norbert-drivers.hex
# Generated by ard-make version 1.2

include $(BUILD_CONF_FILE)
ARDMK_DIR = Arduino-Makefile

include $(ARDMK_DIR)/Arduino.mk

build: $(TARGET_HEX)


prepare_package: verify_size
	mkdir -p $(DIST)/$(DIST_OBJ_DIR)
	mkdir -p $(DIST)/$(DIST_SCRIPT_DIR)

	cp $(LOCAL_OBJ_DIR)/$(OBJ) $(DIST)/$(DIST_OBJ_DIR)
	cp $(LOCAL_SCRIPT_DIR)/* $(DIST)/$(DIST_SCRIPT_DIR)
	cp $(LOCAL_ARDUINO_MAKEFILE)/bin/ard-reset-arduino $(DIST)/$(DIST_SCRIPT_DIR)
	dpkg-deb --build $(DIST) $(DIST)/$(PACKAGE_NAME).deb

deploy:
	scp $(DIST)/$(PACKAGE_NAME).deb $(DIST_MACHINE):$(DEB_PATH)
	ssh -t $(DIST_MACHINE) 'sudo dpkg -i $(DEB_PATH)/$(PACKAGE_NAME).deb'

clean_package: clean
	find ./$(DIST) -mindepth 1 ! -regex '^./$(DIST)/DEBIAN\(/.*\)?' -delete