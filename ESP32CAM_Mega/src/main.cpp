#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <esp32-hal-ledc.h>      //用於控制伺服馬達
#include "soc/soc.h"             //用於電源不穩不重開機 
#include "soc/rtc_cntl_reg.h"    //用於電源不穩不重開機

//官方函式庫
#include "esp_camera.h"          //視訊函式庫
#include "esp_http_server.h"     //HTTP Server函式庫
#include "img_converters.h"      //影像格式轉換函式庫
#include "esp_timer.h"
#include "Arduino.h"
#include "fb_gfx.h"

WebServer server(80);
int timer;

// Replace with your network credentials
const char* ssid = "Huawei P30";
const char* password = "ys771u9o";

#define PART_BOUNDARY "123456789000000000000987654321"

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>
  <head>
    <meta charset="utf-8">
    <title>ROV underwater view</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial; text-align: center; margin:0px auto; padding-top: 30px;}
      table { margin-left: auto; margin-right: auto; }
      td { padding: 8 px; }
      img {  
        transform: rotate(90deg);
        width: 380px ;
        max-width: 100% ;
        height: 320px ;  
      }
     .btn-group1 .buttonFrwBkw {
        background-color: #669999;
        border: 1px solid white;
        color: white;
        padding: 15px 32px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 16px;
        cursor: pointer;
        float: left;
        transition-duration: 0.4s;
        width: 100%;
      }      
      .btn-group3 .buttonRL {
        background-color: #669999;
        border: 1px solid white;
        color: white;
        padding: 15px 30px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 16px;
        cursor: pointer;
        float: left;
        width: 33.3%;
        transition-duration: 0.4s;
      }
      .btn-group3 .buttonStp {
        background-color: LightSlateGrey;
        border: 1px solid white;
        color: white;
        padding: 15px 30px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 16px;
        cursor: pointer;
        float: left;
        width: 33.3%;
        transition-duration: 0.4s;
      }
      .btn-group2 .buttonRsDv {
        background-color: #7c9ccd;
        border: 1px solid white;
        color: white;
        padding: 15px 32px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 16px;
        cursor: pointer;
        float: left;
        transition-duration: 0.4s;
        width: 50%;
      }
      .btn-group2 .buttonHdAm {
        background-color: #d4aa5c;
        border: 1px solid white;
        color: white;
        padding: 15px 32px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 16px;
        cursor: pointer;
        float: left;
        transition-duration: 0.4s;
        width: 50%;
      }
      .buttonStb {
        display: inline-block;
        background-color: DarkSalmon;
        border: none;
        color: #FFFFFF;
        text-align: center;
        font-size: 16px;
        padding: 10px;
        width: 100%;
        transition: all 0.5s;
        cursor: pointer;
        margin: 10px 0 0 0;
      }
      .card{
        max-width: auto;
        min-height: auto;
        background: #6175a1;
        text-align: center;
        margin: 20px auto;
        padding: 10px;
        box-sizing: border-box;
        color: #FFF;
        box-shadow: 0px 2px 18px -4px rgba(0,0,0,0.75);
      }
      ul {
        list-style-type: none;
        margin: 0;
        padding: 0 25px;
        overflow: hidden;
        background-color: #333;
        position: -webkit-sticky; 
        position: sticky;
        top: 0;
      }
      li {
        float: left;
      }
      li a {
        display: block;
        color: white;
        text-align: center;
        padding: 14px 16px;
        text-decoration: none;
      }
    </style>
    <h1 style="color:steelblue">水下觀察機</h1>
    <h3 id="clock"></h3>
    <ul>
      <li><a href="#photo">水下影像</a></li>
      <li><a href="#control">控制</a></li>
      <li><a href="#feedback">數值</a></li>
    </ul>
    <img src="" id="photo" alt="no signal"  style="transform:rotate(90deg);">
    <div class="btn-group1">
      <button id="control" class="buttonFrwBkw" onclick="toggleCheckbox('forward');">加速</button>
    </div>
    <div class="btn-group3">
      <button class="buttonRL" onclick="toggleCheckbox('left');">Left</button>
    </div>
    <div class="btn-group3">
      <button class="buttonStp" onclick="toggleCheckbox('stop');">Stop</button>
    </div>
    <div class="btn-group3">
      <button class="buttonRL" onclick="toggleCheckbox('right');">Right</button>
    </div>
    <div class="btn-group1">
      <button class="buttonFrwBkw" onclick="toggleCheckbox('backward');">減速</button>
    </div>
       <div class="btn-group2">
      <button class="buttonRsDv" onclick="toggleCheckbox('rise');">rise</button>
      <button class="buttonRsDv" onclick="toggleCheckbox('dive');">dive</button>
    </div>
    <div class="btn-group2">
      <button class="buttonHdAm" onclick="toggleCheckbox('hand');">hand</button>
      <button class="buttonHdAm" onclick="toggleCheckbox('arm');">arm</button>
    </div>
    <button class="buttonStb" style="vertical-align:middle" onclick="toggleCheckbox('stable');"><span>stable</span></button>
    <p style="clear:both"><br></p>
    <div class="card">
      <h2 id="feedback">Feedback Data</h2><br>
      <h4>Temperature : <span id="ADCValue">0</span> (degree C) </h4><br>
      <h4>Pressure : <span id="pressureValue">0</span> (Pa) </h4><br>
    </div>
   <script>
   function toggleCheckbox(x) {
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/action?go=" + x, true);
     xhr.send();
   }
   window.onload = document.getElementById("photo").src = window.location.href.slice(0, -1) + ":81/stream";

   setInterval(function() {
  // Call a function repetatively with 2 Second interval
    getTemper();
    getRSSI();
  }, 500); //500ms update rate
    
    var wifi;
    function getTemper() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (xhttp.readyState == 4 && xhttp.status == 200) {
          document.getElementById("ADCValue").innerHTML = xhttp.responseText;
        }
      };
      xhttp.open("GET", "readADC", true);
      xhttp.send();
    }
    
    function getRSSI() {
      var rssihttp = new XMLHttpRequest();
      rssihttp.onreadystatechange = function() {
        if (rssihttp.readyState == 4 && rssihttp.status == 200) {
          wifi = parseInt(rssihttp.responseText);
          console.log(wifi);  //主控台顯示wifi值
          if(wifi == 1)
          {
            alert("警告!訊號過低");
          }
        }
      };
      rssihttp.open("GET", "readRSSI", true);
      rssihttp.send();
    }

    function Clock() {
      var date = new Date();
      this.year = date.getFullYear();
      this.month = date.getMonth() + 1;
      this.date = date.getDate();
      this.day = new Array("星期日", "星期一", "星期二", "星期三", "星期四", "星期五", "星期六")[date.getDay()];
      this.hour = date.getHours() 
      this.minute = date.getMinutes() 
      this.second = date.getSeconds() 
      this.toString = function() {
      return this.year + "年" + this.month + "月" + this.date + "日 " + this.hour + ":" + this.minute + ":" + this.second + " " + this.day;
      };
      this.toSimpleDate = function() {
      return this.year + "-" + this.month + "-" + this.date;
      };
      this.toDetailDate = function() {
      return this.year + "-" + this.month + "-" + this.date + " " + this.hour + ":" + this.minute + ":" + this.second;
      };
      this.display = function(ele) {
      var clock = new Clock();
      ele.innerHTML = clock.toString();
      window.setTimeout(function() {clock.display(ele);}, 1000);
      };
    }
    var clock = new Clock();
    clock.display(document.getElementById("clock"));
    </script>
  </body> 
</html>
)rawliteral";

static esp_err_t index_handler(httpd_req_t *req){
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  return res;
}

static esp_err_t cmd_handler(httpd_req_t *req){
  char*  buf;
  size_t buf_len;
  char variable[32] = {0,};
  
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if(!buf){
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  sensor_t * s = esp_camera_sensor_get();  //設置攝影的參數
  //s->set_vflip(s, 1);  // Vertical flip： 0 = disable , 1 = enable

  int res = 0;
  
  if(!strcmp(variable, "forward")) {
    Serial.println("f");
  }
  else if(!strcmp(variable, "left")) {
    Serial.println("l");
  }
  else if(!strcmp(variable, "right")) {
    Serial.println("r");
  }
  else if(!strcmp(variable, "backward")) {
    Serial.println("b");
  }
  else if(!strcmp(variable, "stop")) {
    Serial.println("t");
  }
  else if(!strcmp(variable, "stable")) {
    Serial.println("s");
  }
  else if(!strcmp(variable, "hand")) {
    Serial.println("h");
  }
  else if(!strcmp(variable, "arm")) {
    Serial.println("a");
  }
  else if(!strcmp(variable, "dive")) {
    Serial.println("d");
  }
  else if(!strcmp(variable, "rise")) {
    Serial.println("e");
  }
  else {
    res = -1;
  }

  if(res){
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri       = "/action",
    .method    = HTTP_GET,
    .handler   = cmd_handler,
    .user_ctx  = NULL
  };
  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
  }
  config.server_port += 1;
  config.ctrl_port += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void handleRoot() {
 String s = INDEX_HTML; //Read HTML contents
 server.send(200, "text/html", s); //Send web page
}

String c; 
void handleADC() { 
  String adcValue = c;

  //int a = 666;
  // String adcValue = String(a);
 
  server.send(200, "text/plane", adcValue); //Send ADC value only to client ajax request
}

String w;
void handleRSSI(){
  String RSSI = w;
  server.send(200, "text/plane", RSSI); //Send RSSI only to client ajax request
}

int WIFIreminder;

String temperature = "";

void setup() {  
  WIFIreminder = 0;
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  
  Serial.begin(38400);
  Serial.setDebugOutput(false);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    // Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  // Wi-Fi connection
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    // Serial.print(".");
  }
  // Serial.println("");
  // Serial.println("WiFi connected");
  
  // Serial.print("Camera Stream Ready! Go to: http://");
  // Serial.println(WiFi.localIP());

  server.on("/", handleRoot);      //This is display page
  server.on("/readADC", handleADC);//To get update of ADC Value only
  server.on("/readRSSI", handleRSSI);//To get update of RSSI only
  
  // Start streaming web server
  startCameraServer();
  server.begin();
}

void loop(){    
  server.handleClient();
  delay(1);

  String Temper;
  Temper = char(Serial.read());
  if (Temper.length() > 0)
  {
    temperature = temperature + Temper;  //字串相加
    if(temperature.length() == 2)  //檢查字串是否達到 2 位數
    {
      c = temperature;    
      temperature = "";     //清空字串
    }
  }

  //Serial.print("WiFi RSSI:");
  //Serial.println(WiFi.RSSI()); //讀取WiFi強度

  if(WiFi.RSSI()<(-200))     //RSSI小於-60跳警告
  {
    if(WIFIreminder == 0) 
    {
      w = 1;
      WIFIreminder++;
    }
    else
    {
      WIFIreminder++;
      if(WIFIreminder == 6) WIFIreminder = 0;  //迴圈執行6次 最少3秒跳一次警告
      w = 2;
    }
    
    //Serial.println("signal too weak");
  }
  else 
  {
    w = 2;
  }
  delay(1);
}