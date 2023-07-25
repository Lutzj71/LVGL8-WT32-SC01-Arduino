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

#define TRUNK                             "temperatury/"
#define NODE_NAME                         "BathroomDisplay"
#define TOPIC_TEMP_AIR    TRUNK NODE_NAME "/air_parameters"
#define TOPIC_BUTTON_1    TRUNK NODE_NAME "/Button1"
#define TOPIC_BUTTON_2    TRUNK NODE_NAME "/Button2"
#define TOPIC_TEMPERATURE "temperatury/wielicka"

#define POLLING_PERIOD (30000/5)
uint32_t polling_counter = POLLING_PERIOD;
//#include "D:\Users\LukaszJ\Documents\PlatformIO\Projects\LVGL8-WT32-SC01-Arduino\.pio\libdeps\esp32dev\lvgl\src\core\lv_obj_style.h"
static LGFX lcd;         // declare display variable
static const lgfx::U8g2font helvB24 ( u8g2_font_helvB24_tr );

String ssid = "ILMMJ-TRI";
String password = "majaimichal";
String mqtt_server = "192.168.1.17";
String clientId = TRUNK NODE_NAME;

WiFiClient espClient;
PubSubClient client(espClient);
float t_outside=0.0;

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
  sscanf(buffer,"%f",&t_outside);

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
  polling_counter = 2;
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
      client.publish(TOPIC_BUTTON_1, "1");
    else
      client.publish(TOPIC_BUTTON_1, "0");
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
      client.publish(TOPIC_BUTTON_2, "1");
    else
      client.publish(TOPIC_BUTTON_2, "0");
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
  lv_obj_add_event_cb(btn2, toggle_event_handler_3, LV_EVENT_ALL, NULL);
  lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_pos(btn2, 20, 80);   /*Set its position*/
  lv_obj_set_size(btn2, 280, 100); /*Set its size*/

  label2 = lv_label_create(btn2);
  lv_label_set_text(label2, LV_SYMBOL_POWER " Wentylator");
  lv_obj_center(label2);

  lv_obj_t *sl1 = lv_slider_create(lv_scr_act());
  lv_obj_add_event_cb(sl1, slider_event_handler, LV_EVENT_ALL, NULL);
  lv_slider_set_range(sl1, 0, 100);
  lv_obj_set_pos(sl1, 20, 450); /*Set its position*/

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
  polling_counter = 2;
}

void loop()
{
  ArduinoOTA.handle();  
  lv_timer_handler(); /* let the GUI do its work */
  delay(5);
  if (!client.loop())
  {
    reconnect();
  }

  if(--polling_counter==0)
  {
    char data[128];
    // Serial.print(F("Temperature: ")); Serial.print(myAHT15.readTemperature()); Serial.println(F(" +-0.3C")); //by default "AHT10_FORCE_READ_DATA"
    // Serial.print(F("Humidity...: ")); Serial.print(myAHT15.readHumidity());    Serial.println(F(" +-2%"));   //by default "AHT10_FORCE_READ_DATA"
    float t_air,h_air;
    t_air = myAHT15.readTemperature();
    h_air = myAHT15.readHumidity();
    sprintf(data,"temperature,humidity\n%f,%f",t_air,h_air);
      if(client.publish(TOPIC_TEMP_AIR, data)){}
    sprintf(data,"Temperatura: %.1fC",t_air);
    lcd.setCursor(7,187);
    lcd.setFont(&helvB24); lcd.print(data);
    sprintf(data,"Wilgotnosc: %.1f%%",h_air);
    lcd.setCursor(7,227);
    lcd.setFont(&helvB24); lcd.print(data);
    lcd.setCursor(7,267);
    sprintf(data,"Za oknem: %.1fC",t_outside);
    lcd.setFont(&helvB24); lcd.print(data);
    polling_counter = POLLING_PERIOD;
  }
}


