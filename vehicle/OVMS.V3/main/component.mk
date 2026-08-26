#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_DEPENDS := mongoose
COMPONENT_ADD_INCLUDEDIRS := .
COMPONENT_ADD_LDFLAGS = -Wl,--whole-archive -l$(COMPONENT_NAME) -Wl,--no-whole-archive -T main/ovms_boot.ld

ifdef CONFIG_SPIRAM_CACHE_WORKAROUND
COMPONENT_ADD_LDFLAGS += -mfix-esp32-psram-cache-issue
endif

# Resolve from the component source tree. The main component is often built with
# cwd under BUILD_DIR_BASE (outside the git repo), so bare `git describe` yields
# an empty OVMS_VERSION and a stale/empty ovms_version.cfg.
OVMS_GIT_ROOT := $(shell git -C "$(COMPONENT_PATH)" rev-parse --show-toplevel 2>/dev/null)
OVMS_VERSION := $(shell git -C "$(OVMS_GIT_ROOT)" describe --always --tags --dirty 2>/dev/null)
ifeq ($(strip $(OVMS_VERSION)),)
$(error OVMS_VERSION empty: git describe failed for COMPONENT_PATH=$(COMPONENT_PATH))
endif
CPPFLAGS := -D OVMS_VERSION=\"$(OVMS_VERSION)\" $(CPPFLAGS)

# update OVMS_VERSION dependency file:
ifneq '$(shell cat ovms_version.cfg 2>/dev/null)' '$(OVMS_VERSION)'
.PHONY: ovms_version.cfg
ovms_version.cfg:
	@echo '$(OVMS_VERSION)' >ovms_version.cfg
	@echo 'Build version is $(OVMS_VERSION)'
endif

ovms_version.o: ovms_version.cfg
