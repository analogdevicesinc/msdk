IPATH += $(LVGL_DIR)
IPATH += $(LVGL_DIR)/$(LVGL_DIR_NAME)
IPATH += $(LVGL_DIR)/$(LVGL_DIR_NAME)/src

## LVGL

ifeq ($(LVGL_DEMOS), 1)
include $(LVGL_DIR)/$(LVGL_DIR_NAME)/demos/lv_demos.mk
endif

ifeq ($(LVGL_EXAMPLES), 1)
include $(LVGL_DIR)/$(LVGL_DIR_NAME)/examples/lv_examples.mk
endif

ifeq ($(LVGL_LIBS), 1)
include $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/core/lv_core.mk
include $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/draw/lv_draw.mk
include $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/extra/lv_extra.mk
include $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/font/lv_font.mk
include $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/hal/lv_hal.mk
include $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/misc/lv_misc.mk
include $(LVGL_DIR)/$(LVGL_DIR_NAME)/src/widgets/lv_widgets.mk
endif

SRCS += $(CSRCS)

VPATH += $(shell find -L $(LVGL_DIR)/$(LVGL_DIR_NAME)/src) -type d)