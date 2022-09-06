#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <esp32-hal-ledc.h>      //用於控制伺服馬達
#include "soc/soc.h"             //用於電源不穩不重開機 
#include "soc/rtc_cntl_reg.h"    //用於電源不穩不重開機

//官方函式庫
#include "esp_http_server.h"     //HTTP Server函式庫
#include "esp_timer.h"
#include "Arduino.h"
#include "fb_gfx.h"

#include "MS5837.h"

WebServer server(80);

// Replace with your network credentials
const char* ssid = "Huawei P30";
const char* password = "ys771u9o";

#define I2C_SDA 21  //to sensor white
#define I2C_SCL 22  //to sensor green

#define ESP32_CAM_SCK 14   //to Mega pin 52
#define ESP32_CAM_MISO 12  //to Mega pin 50
#define ESP32_CAM_MOSI 13  //to Mega pin 51
#define ESP32_CAM_SS 2     //to Mega pin 53

httpd_handle_t camera_httpd = NULL;

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
        font-family:Microsoft JhengHei;
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
        font-family:Microsoft JhengHei;
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
        font-family:Microsoft JhengHei;
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
        font-family:Microsoft JhengHei;
      }
      .btn-group2 .buttonHdAm {
        background-color: #d4aa5c;
        border: 1px solid white;
        color: white;
        padding: 15px 15px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 16px;
        cursor: pointer;
        float: left;
        transition-duration: 0.4s;
        width: 33.333%;
        font-family:Microsoft JhengHei;
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
        font-family:Microsoft JhengHei;
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
        font-family:Microsoft JhengHei;
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
    <h1 style="color:steelblue; font-family:Microsoft JhengHei">水下觀察機</h1>
    <h3 id="clock" style="font-family:Microsoft JhengHei;"></h3>
    <ul>
      <li><a href="#photo">水下影像</a></li>
      <li><a href="#control">控制</a></li>
      <li><a href="#feedback">數值</a></li>
    </ul>
    <p id="photo"></p>
    <iframe
      noresize="noresize"
      style="position: middle; background: transparent; width: 100%; height:320px;"
      src="http://192.168.43.22/">
    </iframe>
    <div class="btn-group1">
      <button id="control" class="buttonFrwBkw" onclick="toggleCheckbox('forward');">加速</button>
    </div>
    <div class="btn-group3">
      <button class="buttonRL" onclick="toggleCheckbox('left');">左轉</button>
    </div>
    <div class="btn-group3">
      <button class="buttonStp" onclick="toggleCheckbox('stop');">停止</button>
    </div>
    <div class="btn-group3">
      <button class="buttonRL" onclick="toggleCheckbox('right');">右轉</button>
    </div>
    <div class="btn-group1">
      <button class="buttonFrwBkw" onclick="toggleCheckbox('backward');">減速</button>
    </div>
       <div class="btn-group2">
      <button class="buttonRsDv" onclick="toggleCheckbox('rise');">上升</button>
      <button class="buttonRsDv" onclick="toggleCheckbox('dive');">下潛</button>
    </div>
    <div class="btn-group2">
      <button class="buttonHdAm" onclick="toggleCheckbox('hand');">夾爪</button>
      <button class="buttonHdAm" onclick="toggleCheckbox('armup');">手臂抬升</button>
      <button class="buttonHdAm" onclick="toggleCheckbox('armdown');">手臂下降</button>
    </div>
    <button class="buttonStb" style="vertical-align:middle" onclick="toggleCheckbox('stable');"><span>穩定機身</span></button>
    <p style="clear:both"><br></p>
    <div class="card">
      <h2 id="feedback">感測器數值</h2><br>
      <h4>Temperature : <span id="temperValue"></span> (degree C) </h4><br>
      <h4>Pressure : <span id="pressureValue"></span> (mbar) </h4><br>
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
    getPressure();
  }, 50); //50ms update rate 太短可能導致按鈕失靈(經測試50不錯)
  setInterval(function() {
    getWarn();
  }, 3000);
   
    var warn;
    function getTemper() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (xhttp.readyState == 4 && xhttp.status == 200) {
          document.getElementById("temperValue").innerHTML = xhttp.responseText;
        }
      };
      xhttp.open("GET", "readTemper", true);
      xhttp.send();
    }
    function getPressure() {
      var phttp = new XMLHttpRequest();
      phttp.onreadystatechange = function() {
        if (phttp.readyState == 4 && phttp.status == 200) {
          document.getElementById("pressureValue").innerHTML = phttp.responseText;
        }
      };
      phttp.open("GET", "readPressure", true);
      phttp.send();
    }
    
    function getWarn() {
      var warnhttp = new XMLHttpRequest();
      warnhttp.onreadystatechange = function() {
        if (warnhttp.readyState == 4 && warnhttp.status == 200) {
          warn = parseInt(warnhttp.responseText);
          console.log(warn);  //主控台顯示warn值
          if(warn == 1 )
          {
            alert("警告!訊號過低");
          }
          if(warn == 3)
          {
            alert("警告!電池電壓過低");
          }
        }
      };      
      warnhttp.open("GET", "readWarn", true);
      warnhttp.send();
    }

    function Clock() {      //顯示時間的函數
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

char m_send;

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

  int res = 0;

  if(!strcmp(variable, "forward")) {
    Serial.println("f");
    m_send = 'f';
  }
  else if(!strcmp(variable, "left")) {
    Serial.println("l");
    m_send = 'l';
  }
  else if(!strcmp(variable, "right")) {
    Serial.println("r");
    m_send = 'r';
  }
  else if(!strcmp(variable, "backward")) {
    Serial.println("b");
    m_send = 'b';
  }
  else if(!strcmp(variable, "stop")) {
    Serial.println("t");
    m_send = 't';
  }
  else if(!strcmp(variable, "stable")) {
    Serial.println("s");
    m_send = 's';
  }
  else if(!strcmp(variable, "hand")) {
    Serial.println("h");
    m_send = 'h';
  }
  else if(!strcmp(variable, "armup")) {
    Serial.println("a");
    m_send = 'a';
  }
  else if(!strcmp(variable, "armdown")) {
    Serial.println("q");
    m_send = 'q';
  }
  else if(!strcmp(variable, "dive")) {
    Serial.println("d");
    m_send = 'd';
  }
  else if(!strcmp(variable, "rise")) {
    Serial.println("e");
    m_send = 'e';
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
  ESP_LOGI(TAG, "Starting web server on port: '%d'", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
  }
  config.server_port += 1;
  config.ctrl_port += 1;
}

void handleRoot() {
 String s = INDEX_HTML; //Read HTML contents
 server.send(200, "text/html", s); //Send web page
}

String temperature, pressure;

void handleTemper() { 
  String temperValue = temperature;
  //int a = 666;
  // String adcValue = String(a); 
  server.send(200, "text/plane", temperValue); //Send ADC value only to client ajax request
}

void handlePressure() { 
  String pressureValue = pressure;
  server.send(200, "text/plane", pressureValue); //Send ADC value only to client ajax request
}

String w;
void handleWarn(){
  String Warn = w;
  server.send(200, "text/plane", Warn); //Send RSSI only to client ajax request
}

int WIFIreminder;
MS5837 sensor;

void setup () {
  Serial.begin(115200); //set baud rate to 115200 for usart
  digitalWrite(SS, HIGH); // disable Slave Select
  SPI.begin(ESP32_CAM_SCK,  ESP32_CAM_MISO, ESP32_CAM_MOSI, ESP32_CAM_SS);
  SPI.setClockDivider(SPI_CLOCK_DIV16);//divide the clock by 8
  WIFIreminder = 0;
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  
  Serial.setDebugOutput(false);
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Wi-Fi connection
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    // Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  
  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);      //This is display page
  server.on("/readTemper", handleTemper);//To get update of ADC Value only
  server.on("/readPressure", handlePressure);
  server.on("/readWarn", handleWarn);//To get update of RSSI only
  
  // Start streaming web server
  startCameraServer();
  server.begin();

  sensor.init();
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

float tempera, pressu;
int sensernum = 0;
int SPIsendnum = 0;
byte m_receive;

void loop () {
  server.handleClient();
  delay(1);
  // Serial.print("WiFi RSSI:");
  // Serial.println(WiFi.RSSI()); //讀取WiFi強度

  if(WiFi.RSSI()<(-70))     //RSSI小於-200跳警告
  {
    w = 1;
  }
  else
  {
    w = 2;
  }

  sensernum++;
  if(sensernum >= 110)
  {
    sensernum = 0;
    sensor.read();   //函式裡有 delay 40 ms
    // Serial.print("Temperature: "); 
    // Serial.print(sensor.temperature()); 
    // Serial.println(" deg C");
    tempera = sensor.temperature();
    pressu = sensor.pressure();
    // Serial.println(tempera);
    temperature = String(tempera,2);
    pressure = String(pressu,2);
    //Serial.println(WiFi.RSSI());
  }

  SPIsendnum++;
  if(SPIsendnum >= 100)
  {
    SPIsendnum = 0;
    char c = m_send;
    digitalWrite(SS, LOW); // enable Slave Select
    m_receive = SPI.transfer(c);
    Serial.println(m_receive);
    m_send = 0;
  }
}