// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: RoundDisplayTest

#include "../ui.h"

void ui_Screen3_screen_init(void)
{
ui_Screen3 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen3, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Screen3, lv_color_hex(0x0500FF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Screen3, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label3 = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_Label3, 100);
lv_obj_set_height( ui_Label3, 100);
lv_obj_set_align( ui_Label3, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label3,"Text page two");
lv_obj_set_style_text_align(ui_Label3, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label3, &lv_font_montserrat_26, LV_PART_MAIN| LV_STATE_DEFAULT);

}