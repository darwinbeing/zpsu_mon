// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.4
// Project name: SquareLine_Project

#include "../ui.h"

void ui_Screen1_screen_init(void)
{
ui_Screen1 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Screen1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Screen1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Screen1, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_LabelCurrent = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_LabelCurrent, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_LabelCurrent, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_LabelCurrent, 64 );
lv_obj_set_y( ui_LabelCurrent, -28 );
lv_obj_set_align( ui_LabelCurrent, LV_ALIGN_CENTER );
lv_label_set_text(ui_LabelCurrent,"00.00");
lv_obj_set_style_text_color(ui_LabelCurrent, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_LabelCurrent, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_LabelCurrent, &ui_font_DSEG7ClassicBold36, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_LabelVoltage = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_LabelVoltage, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_LabelVoltage, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_LabelVoltage, -64 );
lv_obj_set_y( ui_LabelVoltage, -28 );
lv_obj_set_align( ui_LabelVoltage, LV_ALIGN_CENTER );
lv_label_set_text(ui_LabelVoltage,"00.00");
lv_obj_set_style_text_color(ui_LabelVoltage, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_LabelVoltage, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_LabelVoltage, &ui_font_DSEG7ClassicBold36, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_LabelEnergy = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_LabelEnergy, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_LabelEnergy, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_LabelEnergy, 64 );
lv_obj_set_y( ui_LabelEnergy, 29 );
lv_obj_set_align( ui_LabelEnergy, LV_ALIGN_CENTER );
lv_label_set_text(ui_LabelEnergy,"00.00");
lv_obj_set_style_text_color(ui_LabelEnergy, lv_color_hex(0x00FFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_LabelEnergy, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_LabelEnergy, &ui_font_DSEG7ClassicBold36, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_LabelPower = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_LabelPower, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_LabelPower, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_LabelPower, -64 );
lv_obj_set_y( ui_LabelPower, 29 );
lv_obj_set_align( ui_LabelPower, LV_ALIGN_CENTER );
lv_label_set_text(ui_LabelPower,"00.00");
lv_obj_set_style_text_color(ui_LabelPower, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_LabelPower, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_LabelPower, &ui_font_DSEG7ClassicBold36, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label6 = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_Label6, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label6, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label6, 109 );
lv_obj_set_y( ui_Label6, -1 );
lv_obj_set_align( ui_Label6, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label6,"A");
lv_obj_set_style_text_color(ui_Label6, lv_color_hex(0xFFFF06), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label6, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label6, &lv_font_montserrat_16, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label5 = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_Label5, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label5, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label5, -20 );
lv_obj_set_y( ui_Label5, -1 );
lv_obj_set_align( ui_Label5, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label5,"V");
lv_obj_set_style_text_color(ui_Label5, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label5, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label5, &lv_font_montserrat_16, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label8 = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_Label8, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label8, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label8, 102 );
lv_obj_set_y( ui_Label8, 56 );
lv_obj_set_align( ui_Label8, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label8,"Wh");
lv_obj_set_style_text_color(ui_Label8, lv_color_hex(0x00FFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label8, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_Label8, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label8, &lv_font_montserrat_16, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label7 = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_Label7, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label7, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label7, -20 );
lv_obj_set_y( ui_Label7, 56 );
lv_obj_set_align( ui_Label7, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label7,"W");
lv_obj_set_style_text_color(ui_Label7, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label7, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label7, &lv_font_montserrat_16, LV_PART_MAIN| LV_STATE_DEFAULT);

}
