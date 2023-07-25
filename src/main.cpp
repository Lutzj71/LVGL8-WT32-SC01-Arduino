/*
    Simple Demo with WT32-SC01 + LovyanGFX + LVGL8.x
*/
#define LGFX_AUTODETECT // Autodetect board
#define LGFX_USE_V1     // set to use new version of library
// #define LV_CONF_INCLUDE_SIMPLE
#include <LovyanGFX.hpp> // main library
#include <lvgl.h>
#include "lv_conf.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#define ESP8266
#include <AHT10.h>

#include "profont.h"
#include "adobex11font.h"
#include <ArduinoOTA.h>

#define POLLING_PERIOD (30000/5)
uint32_t polling_counter = POLLING_PERIOD;
//#include "D:\Users\LukaszJ\Documents\PlatformIO\Projects\LVGL8-WT32-SC01-Arduino\.pio\libdeps\esp32dev\lvgl\src\core\lv_obj_style.h"
static LGFX lcd;         // declare display variable
static const lgfx::U8g2font helvB24 ( u8g2_font_helvB24_tr );

String ssid = "ILMMJ-TRI";
String password = "majaimichal";
String mqtt_server = "192.168.1.17";
String clientId = "temperatury/Bathroom";
#define TOPIC_TEMPERATURE "temperatury/wielicka"
WiFiClient espClient;
PubSubClient client(espClient);

lv_obj_t *label_temp_hum;

uint8_t readStatus = 0;

AHT10 myAHT15(AHT10_ADDRESS_0X38,AHT15_SENSOR);

/*** Setup screen resolution for LVGL ***/
static const uint16_t screenWidth = 320;
static const uint16_t screenHeight = 480;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];
boolean state1 = false;
boolean state2 = false;

/*** Function declaration ***/
void display_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
void lv_button_demo(void);

static lv_obj_t *label_slider;
static lv_obj_t *label_tmp;

void setup_wifi()
{

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      if (client.subscribe(TOPIC_TEMPERATURE))
        Serial.println("Subscribed to " TOPIC_TEMPERATURE);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  char buffer[10]={0};
  memcpy(buffer,payload,length);
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  lv_label_set_text(label_tmp, buffer);

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1')
  {
    //  digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  }
  else
  {
    //  digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }
}

/*** Display callback to flush the buffer to screen ***/
void display_flush(lv_disp_drv_t * disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  lcd.startWrite();
  lcd.setAddrWindow(area->x1, area->y1, w, h);
  lcd.pushColors((uint16_t *)&color_p->full, w * h, true);
  lcd.endWrite();

  lv_disp_flush_ready(disp);
}

/*** Touchpad callback to read the touchpad ***/
void touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data)
{
  uint16_t touchX, touchY;
  bool touched = lcd.getTouch(&touchX, &touchY);

  if (!touched)
  {
    data->state = LV_INDEV_STATE_REL;
  }
  else
  {
    data->state = LV_INDEV_STATE_PR;

    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;

    // Serial.printf("Touch (x,y): (%03d,%03d)\n",touchX,touchY );
  }
}

/* Counter button event handler */
static void counter_event_handler(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *btn = lv_event_get_target(e);
  if (code == LV_EVENT_CLICKED)
  {
    static uint8_t cnt = 0;
    cnt++;

    /*Get the first child of the button which is the label and change its text*/
    lv_obj_t *label = lv_obj_get_child(btn, 0);
    lv_label_set_text_fmt(label, "Button: %d", cnt);
    LV_LOG_USER("Clicked");
    Serial.println("Clicked");
  }
}

/* Toggle button event handler */
static void toggle_event_handler_2(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    LV_LOG_USER("Toggled");
    Serial.println("Toggled");
    if((state1 = !state1))
      client.publish("temperatury/Bathroom/Button1", "1");
    else
      client.publish("temperatury/Bathroom/Button1", "0");
  }
}

/* Toggle button event handler */
static void toggle_event_handler_3(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    LV_LOG_USER("Toggled");
    Serial.println("Toggled");
    if((state2 = !state2))
      client.publish("temperatury/Bathroom/Button2", "1");
    else
      client.publish("temperatury/Bathroom/Button2", "0");
  }
}  

static void checkbox_event_handler(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    LV_LOG_USER("Check Toggled");
    Serial.println("check Toggled");
  }
}

static void switch_event_handler(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    LV_LOG_USER("Switch toggled");
    Serial.println("Switch toggled");
  }
}

static void slider_event_handler(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    lv_obj_t *slider = lv_event_get_target(e);
    lv_label_set_text_fmt(label_slider, "%" LV_PRId32, lv_slider_get_value(slider));
    lv_obj_align_to(label_slider, slider, LV_ALIGN_OUT_TOP_MID, 0, -15); /*Align top of the slider*/
  }
}

void lv_button_demo(void)
{
  lv_obj_t *label_b1,*label_b2,*label_b3;

  // Button with counter
  lv_obj_t *btn1 = lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn1, counter_event_handler, LV_EVENT_ALL, NULL);

  lv_obj_set_pos(btn1, 20, 20);   /*Set its position*/
  lv_obj_set_size(btn1, 250, 50); /*Set its size*/

  label_b1 = lv_label_create(btn1);
  lv_label_set_text(label_b1, "Button");
  lv_obj_center(label_b1);

  // Toggle button
  lv_obj_t *btn2 = lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn2, toggle_event_handler_2, LV_EVENT_ALL, NULL);
  lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_pos(btn2, 20, 80);   /*Set its position*/
  lv_obj_set_size(btn2, 250, 50); /*Set its size*/

  label_b2 = lv_label_create(btn2);
  lv_label_set_text(label_b2, "Toggle Button");
  lv_obj_center(label_b2);

  // Toggle button
  lv_obj_t *btn3 = lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn3, toggle_event_handler_3, LV_EVENT_ALL, NULL);
  lv_obj_add_flag(btn3, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_pos(btn3, 20, 140);   /*Set its position*/
  lv_obj_set_size(btn3, 250, 50); /*Set its size*/

  label_b3 = lv_label_create(btn3);
  lv_label_set_text(label_b3, "Toggle Button");
  lv_obj_center(label_b3);

  lv_obj_t *cb1 = lv_checkbox_create(lv_scr_act());
  lv_obj_add_event_cb(cb1, checkbox_event_handler, LV_EVENT_ALL, NULL);
  lv_obj_add_flag(cb1, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_pos(cb1, 250, 210); /*Set its position*/
  lv_obj_set_size(cb1, 120, 50); /*Set its size*/
  lv_checkbox_set_text_static(cb1, "Fan ON");

  lv_obj_t *sw1 = lv_switch_create(lv_scr_act());
  lv_obj_add_event_cb(sw1, switch_event_handler, LV_EVENT_ALL, NULL);
  lv_obj_add_flag(sw1, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_pos(sw1, 50, 210); /*Set its position*/

  lv_obj_t *sl1 = lv_slider_create(lv_scr_act());
  lv_obj_add_event_cb(sl1, slider_event_handler, LV_EVENT_ALL, NULL);
  lv_slider_set_range(sl1, 0, 100);
  lv_obj_set_pos(sl1, 20, 280); /*Set its position*/

  label_slider = lv_label_create(sl1);
  lv_label_set_text(label_slider, "0");                           // set label text
  lv_obj_align_to(label_slider, sl1, LV_ALIGN_TOP_RIGHT, 0, -15); /*Align top of the slider*/
}


void setup(void)
{
  Serial.begin(115200); /* prepare for possible serial debug */

  lcd.init(); // Initialize LovyanGFX
  lv_init();  // Initialize lvgl

  // Setting display to landscape
  // if (lcd.width() < lcd.height())
  //    lcd.setRotation(lcd.getRotation() ^ 1);
  lcd.setRotation(0);

  /* LVGL : Setting up buffer to use for display */
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);

  /*** LVGL : Setup & Initialize the display device driver ***/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = display_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*** LVGL : Setup & Initialize the input device driver ***/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touchpad_read;
  lv_indev_drv_register(&indev_drv);

  /*** Create simple label and show LVGL version ***/
  String LVGL_Arduino = "WT32-SC01 with LVGL ";
  LVGL_Arduino += String('v') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  lv_obj_t *label = lv_label_create(lv_scr_act()); // full screen as the parent
  lv_label_set_text(label, LVGL_Arduino.c_str());  // set label text
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 5, 320);  // Center but 20 from the top

  String hum_tem_str = "t = 0 ; h = 0%";
  label_temp_hum = lv_label_create(lv_scr_act()); // full screen as the parent
  lv_label_set_text(label_temp_hum, hum_tem_str.c_str());  // set label text
  lv_obj_align(label_temp_hum, LV_ALIGN_TOP_LEFT, 5, 340);  // Center but 20 from the top

  // static lv_style_t style;
  // lv_style_init(&style);

  // lv_style_set_radius(&style, 5);
  // lv_style_set_bg_opa(&style, LV_OPA_COVER);
  // lv_style_set_bg_color(&style, lv_palette_lighten(LV_PALETTE_GREY, 2));
  // lv_style_set_border_width(&style, 2);
  // lv_style_set_border_color(&style, lv_palette_main(LV_PALETTE_BLUE));
  // lv_style_set_pad_all(&style, 10);

  // lv_style_set_text_color(&style, lv_palette_main(LV_PALETTE_BLUE));
  // lv_style_set_text_letter_space(&style, 5);
  // lv_style_set_text_line_space(&style, 20);
  // lv_style_set_text_decor(&style, LV_TEXT_DECOR_UNDERLINE);

  String temp_str = "t = 0 ";
  label_tmp = lv_label_create(lv_scr_act()); // full screen as the parent
  // obj_add_style(label_tmp, &style, 0);
  lv_label_set_text(label_tmp, temp_str.c_str());  // set label text
  lv_obj_align(label_tmp, LV_ALIGN_TOP_LEFT, 5, 360);  // Center but 60 from the top

  setup_wifi();
  client.setServer(mqtt_server.c_str(), 1883);
  client.setCallback(callback);

  while (myAHT15.begin(32,33) != true)
  {
    Serial.println(F("AHT15 not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free
    delay(5000);
  }
  Serial.println(F("AHT15 OK"));
  reconnect();

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();


  lv_button_demo();
}

void loop()
{
  ArduinoOTA.handle();  
  lv_timer_handler(); /* let the GUI do its work */
  delay(5);
  if (!client.loop())
  {
    //reconnect();
  }

  if(--polling_counter==0)
  {
    // Serial.print(F("Temperature: ")); Serial.print(myAHT15.readTemperature()); Serial.println(F(" +-0.3C")); //by default "AHT10_FORCE_READ_DATA"
    // Serial.print(F("Humidity...: ")); Serial.print(myAHT15.readHumidity());    Serial.println(F(" +-2%"));   //by default "AHT10_FORCE_READ_DATA"
    String hum_tem_str = "t=";
    hum_tem_str += myAHT15.readTemperature();
    hum_tem_str += "C h=";
    hum_tem_str += myAHT15.readHumidity();
    hum_tem_str += "%";
    lv_label_set_text(label_temp_hum, hum_tem_str.c_str());  // set label text
    lv_obj_align(label_temp_hum, LV_ALIGN_TOP_LEFT, 5, 340);  // Center but 20 from the top

    lcd.setCursor(5,135);
    lcd.setFont(&helvB24); lcd.print(hum_tem_str.c_str());
    polling_counter = POLLING_PERIOD;
  }
     

#ifdef DRAW_ON_SCREEN
  /*** Draw on screen with touch ***/
  if (lcd.getTouch(&x, &y))
  {
    lcd.fillRect(x - 2, y - 2, 5, 5, TFT_RED);
    lcd.setCursor(380, 0);
    lcd.printf("Touch:(%03d,%03d)", x, y);
    // }
#endif
  }

  /*** Display callback to flush the buffer to screen ***/
  void display_flush(lv_disp_drv_t * disp, const lv_area_t *area, lv_color_t *color_p)
  {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    lcd.startWrite();
    lcd.setAddrWindow(area->x1, area->y1, w, h);
    lcd.pushColors((uint16_t *)&color_p->full, w * h, true);
    lcd.endWrite();

    lv_disp_flush_ready(disp);
  }

  /*** Touchpad callback to read the touchpad ***/
  void touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data)
  {
    uint16_t touchX, touchY;
    bool touched = lcd.getTouch(&touchX, &touchY);

    if (!touched)
    {
      data->state = LV_INDEV_STATE_REL;
    }
    else
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touchX;
      data->point.y = touchY;

      // Serial.printf("Touch (x,y): (%03d,%03d)\n",touchX,touchY );
    }
  }

  /* Counter button event handler */
  static void counter_event_handler(lv_event_t * e)
  {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *btn = lv_event_get_target(e);
    if (code == LV_EVENT_CLICKED)
    {
      static uint8_t cnt = 0;
      cnt++;

      /*Get the first child of the button which is the label and change its text*/
      lv_obj_t *label = lv_obj_get_child(btn, 0);
      lv_label_set_text_fmt(label, "Button: %d", cnt);
      LV_LOG_USER("Clicked");
      Serial.println("Clicked");
    }
  }

  /* Toggle button event handler */
  static void toggle_event_handler(lv_event_t * e)
  {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED)
    {
      LV_LOG_USER("Toggled");
      Serial.println("Toggled");
      if(state1 = !state1)
        client.publish("temperatury/Bathroom/Button1", "1");
      else
        client.publish("temperatury/Bathroom/Button1", "0");
    }
  }

  static void checkbox_event_handler(lv_event_t * e)
  {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED)
    {
      LV_LOG_USER("Check Toggled");
      Serial.println("check Toggled");
      if(state2 = !state2)
        client.publish("temperatury/Bathroom/Button2", "1");
      else
        client.publish("temperatury/Bathroom/Button2", "0");
    }
  }

  static void switch_event_handler(lv_event_t * e)
  {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED)
    {
      LV_LOG_USER("Switch toggled");
      Serial.println("Switch toggled");
    }
  }

  static void slider_event_handler(lv_event_t * e)
  {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED)
    {
      lv_obj_t *slider = lv_event_get_target(e);
      lv_label_set_text_fmt(label_slider, "%" LV_PRId32, lv_slider_get_value(slider));
      lv_obj_align_to(label_slider, slider, LV_ALIGN_OUT_TOP_MID, 0, -15); /*Align top of the slider*/
    }
  }

  void lv_button_demo(void)
  {
    lv_obj_t *label, *label2;

    // Button with counter
    lv_obj_t *btn1 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn1, counter_event_handler, LV_EVENT_ALL, NULL);

    lv_obj_set_pos(btn1, 20, 20);   /*Set its position*/
    lv_obj_set_size(btn1, 280, 50); /*Set its size*/

    label = lv_label_create(btn1);
    lv_label_set_text(label, "Button");
    lv_obj_center(label);

    // Toggle button
    lv_obj_t *btn2 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn2, toggle_event_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_pos(btn2, 20, 80);   /*Set its position*/
    lv_obj_set_size(btn2, 280, 50); /*Set its size*/

    label2 = lv_label_create(btn2);
    lv_label_set_text(label2, LV_SYMBOL_POWER " Toggle Button");
    lv_obj_center(label2);

    lv_obj_t *cb1 = lv_checkbox_create(lv_scr_act());
    lv_obj_add_event_cb(cb1, checkbox_event_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(cb1, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_pos(cb1, 20, 180); /*Set its position*/
    lv_obj_set_size(cb1, 120, 50); /*Set its size*/
    lv_checkbox_set_text_static(cb1, "Fan ON");

    lv_obj_t *sw1 = lv_switch_create(lv_scr_act());
    lv_obj_add_event_cb(sw1, switch_event_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(sw1, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_pos(sw1, 20, 220); /*Set its position*/

    lv_obj_t *sl1 = lv_slider_create(lv_scr_act());
    lv_obj_add_event_cb(sl1, slider_event_handler, LV_EVENT_ALL, NULL);
    lv_slider_set_range(sl1, 0, 100);
    lv_obj_set_pos(sl1, 20, 280); /*Set its position*/

    label_slider = lv_label_create(sl1);
    lv_label_set_text(label_slider, "0");                           // set label text
    lv_obj_align_to(label_slider, sl1, LV_ALIGN_TOP_RIGHT, 0, -15); /*Align top of the slider*/
  }
