// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: RoundDisplayTest

#include "../ui.h"

void ui_Screen2_screen_init(void)
{
ui_Screen2 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Screen2, lv_color_hex(0x00FF13), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Screen2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label2 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_Label2, 100);
lv_obj_set_height( ui_Label2, 100);
lv_obj_set_align( ui_Label2, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label2,"Text page one");
lv_obj_set_style_text_align(ui_Label2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label2, &lv_font_montserrat_26, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Arc2 = lv_arc_create(ui_Screen2);
lv_obj_set_width( ui_Arc2, 240);
lv_obj_set_height( ui_Arc2, 240);
lv_obj_set_align( ui_Arc2, LV_ALIGN_CENTER );

lv_obj_set_style_bg_color(ui_Arc2, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Arc2, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_color(ui_Arc2, lv_color_hex(0x72FF40), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_Arc2, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);


}
