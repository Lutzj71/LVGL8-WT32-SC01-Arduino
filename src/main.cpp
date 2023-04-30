/*
    Simple Demo with WT32-SC01 + LovyanGFX + LVGL8.x
*/
#define LGFX_AUTODETECT // Autodetect board
#define LGFX_USE_V1     // set to use new version of library
//#define LV_CONF_INCLUDE_SIMPLE

/* Uncomment below line to draw on screen with touch */
//#define DRAW_ON_SCREEN

#include <LovyanGFX.hpp> // main library
static LGFX lcd; // declare display variable

#include <WiFi.h>
#include <PubSubClient.h>
String ssid =     "ILMMJ-TRI";
String password = "majaimichal";
String mqtt_server = "192.168.1.17";
String clientId = "Bathroom";
WiFiClient espClient;
PubSubClient client(espClient);

#include <lvgl.h>
#include "lv_conf.h"
/*** Setup screen resolution for LVGL ***/
static const uint16_t screenWidth = 480;
static const uint16_t screenHeight = 320;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];

// Variables for touch x,y
#ifdef DRAW_ON_SCREEN
static int32_t x, y;
#endif

/*** Function declaration ***/
void display_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
void lv_button_demo(void);

static lv_obj_t *label_slider;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  while (WiFi.status() != WL_CONNECTED) {
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
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
  //  digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
  //  digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void setup(void)
{
  Serial.begin(115200); /* prepare for possible serial debug */

  lcd.init(); // Initialize LovyanGFX
  lv_init();  // Initialize lvgl

  // Setting display to landscape
  if (lcd.width() < lcd.height())
    lcd.setRotation(lcd.getRotation() ^ 1);

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
  lv_obj_align(label, LV_ALIGN_TOP_RIGHT, 0, 20);      // Center but 20 from the top

  setup_wifi();
  client.setServer(mqtt_server.c_str(), 1883);
  client.setCallback(callback);
  reconnect() ;
  lv_button_demo();
}

void loop()
{
  lv_timer_handler(); /* let the GUI do its work */
  delay(5);
  if (!client.connected()) 
  {
    reconnect();
  }
  client.loop();

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
      client.publish("Button", "toggled");
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
      lv_label_set_text_fmt(label_slider, "%"LV_PRId32, lv_slider_get_value(slider));
      lv_obj_align_to(label_slider, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);    /*Align top of the slider*/
    }
  }

  void lv_button_demo(void)
  {
    lv_obj_t *label;

    // Button with counter
    lv_obj_t *btn1 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn1, counter_event_handler, LV_EVENT_ALL, NULL);

    lv_obj_set_pos(btn1, 20, 20);   /*Set its position*/
    lv_obj_set_size(btn1, 250, 50);   /*Set its size*/

    

    label = lv_label_create(btn1);
    lv_label_set_text(label, "Button");
    lv_obj_center(label);

    // Toggle button
    lv_obj_t *btn2 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn2, toggle_event_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_pos(btn2, 20, 80);   /*Set its position*/
    lv_obj_set_size(btn2, 250, 50);   /*Set its size*/

    lv_label_set_text(label, "Toggle Button");
    lv_obj_center(label);

    lv_obj_t *cb1 = lv_checkbox_create(lv_scr_act());
    lv_obj_add_event_cb(cb1, checkbox_event_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(cb1, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_pos(cb1, 250, 170);   /*Set its position*/
    lv_obj_set_size(cb1, 120, 50);   /*Set its size*/
    lv_checkbox_set_text_static(cb1, "Fan ON");

    lv_obj_t *sw1 = lv_switch_create(lv_scr_act());
    lv_obj_add_event_cb(sw1, switch_event_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(sw1, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_pos(sw1, 50, 210);   /*Set its position*/
    
    lv_obj_t *sl1 = lv_slider_create(lv_scr_act());
    lv_obj_add_event_cb(sl1, slider_event_handler, LV_EVENT_ALL, NULL);
    lv_slider_set_range(sl1, 0, 100);
    lv_obj_set_pos(sl1, 20, 280);   /*Set its position*/

    label_slider = lv_label_create(sl1);
    lv_label_set_text(label_slider, "0");  // set label text
    lv_obj_align_to(label_slider, sl1, LV_ALIGN_TOP_RIGHT, 0, -15);    /*Align top of the slider*/
  }
