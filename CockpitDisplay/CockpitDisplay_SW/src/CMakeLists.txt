SET(SOURCES screens/ui_ScreenTow.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_windpfeil_2_png.c)

add_library(ui ${SOURCES})
