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

#define LED_PIN 27

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
      .btn-group1 .buttonpreres {
        background-color: hsl(197, 53%, 42%);
        border: 1px solid white;
        color: white;
        padding: 15px 32px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 16px;
        cursor: pointer;
        float: left;
        width: 100%;
        font-family:Microsoft JhengHei;
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
        padding: 15px 1px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 16px;
        cursor: pointer;
        float: left;
        transition-duration: 0.4s;
        width: 25%;
        font-family:Microsoft JhengHei;
      }
      .btn-group4 .buttonStb {
        background-color: DarkSalmon;
        border: 1px solid white;
        color: white;
        padding: 10px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 16px;
        cursor: pointer;
        float: left;
        transition-duration: 0.4s;
        width: 33.33%;
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
        line-height:2px
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
      
      body {
        font-family: Arial;
      }
                
      .progress {
        position: absolute;
        top: 830px;
        width: 0%;
        height: 10px;
        background-color: #2183DD;
        transition: width 0.2s;
      }
      .progress.progress--1 {
        width: calc(14.3%);
      }
      .progress.progress--2 {
        width: calc(28.6%);
      }
      .progress.progress--3 {
        width: calc(42.9%);
      }
      .progress.progress--4 {
        width: calc(57.2%);
      }
      .progress.progress--5 {
        width: calc(71.5%);
      }
      .progress.progress--6 {
        width: calc(85.8%);
      }
      .progress.progress--7 {
        width: calc(100%);
      }
      .progress.progress--complete {
        width: 100vw;
      }
      
      .progress__bg {
        position: absolute;
        width: 100vw;
        height: 10px;
        background-color: #E5E5E5;
        z-index: -1;
      }
      
      .progress__step {
        position: absolute;
        top: -8px;
        left: 0;
        display: flex;
        flex-direction: column;
        align-items: center;
        text-align: center;
      }
      .progress__step.progress__step--1 {
        left: calc(14.3vw - 9px);
      }
      .progress__step.progress__step--2 {
        left: calc(28.6vw - 9px);
      }
      .progress__step.progress__step--3 {
        left: calc(42.9vw - 9px);
      }
      .progress__step.progress__step--4 {
        left: calc(57.2vw - 9px);
      }
      .progress__step.progress__step--5 {
        left: calc(71.5vw - 9px);
      }
      .progress__step.progress__step--6 {
        left: calc(85.8vw - 9px);
      }
      .progress__step.progress__step--7 {
        left: calc(100vw - 9px);
      }
      .progress__step.progress__step--active {
        color: #2183DD;
        font-weight: 600;
      }
      .progress__step.progress__step--complete .progress__indicator {
        background-color: #009900;
        border-color: #2183DD;
        color: #FFFFFF;
        display: flex;
        align-items: center;
        justify-content: center;
      }
      .progress__step.progress__step--complete .progress__indicator .fa {
        display: block;
      }
      .progress__step.progress__step--complete .progress__label {
        font-weight: 600;
        color: #009900;
      }
      
      .progress__indicator {
        width: 25px;
        height: 25px;
        border: 2px solid #808080;
        border-radius: 50%;
        background-color: #FFFFFF;
        margin-bottom: 10px;
      }
      .progress__indicator .fa {
        display: none;
        font-size: 16px;
        color: #FFFFFF;
      }
      
      .progress__label {
        position: absolute;
        font-size: 10px;
        top: 40px;
      }
      
      .progress__actions {
        position: absolute;
        top: 75px;
        left: 10px;
        display: flex;
        align-items: center;
        width: max-content;
      }
      
      .btn {
        width: fit-content;
        padding: 5px 8px;
        background-color: #FFFFFF;
        border: 1px solid #808080;
        border-radius: 5px;
        cursor: pointer;
        user-select: none;
        visibility: hidden;
      }
      .btn:nth-child(2) {
        margin: 0 10px;
      }
      .stk {
        list-style-type: none;
        margin: 0;
        padding: 0;
        overflow: hidden;
        position: -webkit-sticky; 
        position: sticky;
        top: 0;
      }
    </style>
    <h1 style="color:steelblue; font-family:Microsoft JhengHei">水下觀察機</h1>
    <link rel='stylesheet' href='https://cdnjs.cloudflare.com/ajax/libs/font-awesome/4.7.0/css/font-awesome.min.css'>
    <i id="battery" class='fa fa-battery-0' style="font-size:50px; position:absolute; top:27px; right:12px;"></i>
    <ul>
      <li><a href="#photo">水下影像</a></li>
      <li><a href="#control">控制</a></li>
      <li><a href="#feedback">數值</a></li>
    </ul>
    <div class="stk">
      <p id="photo"></p>
      <iframe
        noresize="noresize"
        style="position: middle; background: transparent; width: 100%; height:320px;"
        src="http://192.168.43.237/">
      </iframe>
    </div>
    
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
      <button class="buttonHdAm" onclick="toggleCheckbox('hold');">抓住</button>
      <button class="buttonHdAm" onclick="toggleCheckbox('release');">放開</button>
      <button class="buttonHdAm" onclick="toggleCheckbox('armup');">手臂抬升</button>
      <button class="buttonHdAm" onclick="toggleCheckbox('armdown');">手臂下降</button>
    </div>
    <div class="btn-group4">
    <button class="buttonStb" style="vertical-align:middle; padding:16.5px;" onclick="toggleCheckbox('light');"><span>燈光</span></button>
    <button class="buttonStb" style="vertical-align:middle; padding:16.5px;" onclick="toggleCheckbox('stable');"><span>穩定機身</span></button>
    <button class="buttonStb" style="vertical-align:middle; padding:6px;" ondblclick="toggleCheckbox('reset');"><span>重置陀螺儀(雙擊)</span></button>
    </div>
    <p><br></p>
    <script src="https://cdn.jsdelivr.net/npm/vue/dist/vue.js"></script>
      <div id="app" :class="progressClasses">
        <div class="progress__bg"></div>
        <template v-for="(step, index) in steps">
          <div :class="stepClasses(index)">
            <div class="progress__indicator">
                <i class="fa fa-check"></i>
            </div>
            <div class="progress__label">
                {{step.label}}
            </div>
          </div>
        </template>
        
        <div class="progress__actions">
          <div
          class="btn"
          v-on:click="nextStep(false)"
          >
          Back
          </div>
          <div
          class="btn"
          id = "NS"
          v-on:click="nextStep"
          >
          Next
          </div>
        </div>
      </div>
    
    <p style="clear:both"><br></p>
    <p style="clear:both"><br></p>
    <div class="card">
      <h2 id="feedback">感測器數值</h2><br>
      <h4>Temperature : <span id="temperValue"></span> (degree C) </h4><br>
      <h4>Pressure : <span id="pressureValue"></span> (mbar) </h4><br>
      <h4>depth : <span id="depthValue"></span> (cm) </h4><br>
    </div>
    <div class="btn-group1">
      <button class="buttonpreres" onclick="depthzero();">深度歸零</button>
    </div>

  <script>
    function toggleCheckbox(x) {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/action?go=" + x, true);
      xhr.send();
    }
    window.onload = document.getElementById("photo").src = window.location.href.slice(0, -1) + ":81/stream";

    setInterval(function() {
      getTemper();
      getPressure();
    }, 2000); //1ms update rate 太短可能導致按鈕失靈(經測試50不錯)
    setInterval(function() {
      getWarn();
    }, 3000);
    setInterval(function(){
      getSpeedstatus();
      sampleClick();
    },500);
    setInterval(function(){
      getBatstatus();
    },9250);
   
    function sampleClick() {
      var btnX = document.getElementById("NS");
      btnX.click();
    }
   
    var warn;
    var PressuValue;
    var oringinalpressure;

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
          window.PressuValue = phttp.responseText;
          document.getElementById("pressureValue").innerHTML = PressuValue;
          var depthcm = (parseFloat(PressuValue) - oringinalpressure).toFixed(2);
          document.getElementById("depthValue").innerHTML = depthcm.toString();
        }
      };
      phttp.open("GET", "readPressure", true);
      phttp.send();
    }
    function depthzero(){
      oringinalpressure = parseFloat(PressuValue);
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
    
    var index;
    
    var app = new Vue({
      el: '#app',
      data: {
      currentStep: null,
      steps: [
        {"label": "back2"},
        {"label": "back1"},
        {"label": "stop"},
        {"label": "forward1"},
        {"label": "forward2"},
        {"label": "forward3"},
      ]
      },
      methods: {
        nextStep(next=true) {
          const steps = this.steps
          const currentStep = this.currentStep
          const currentIndex = steps.indexOf(currentStep)
          if (steps[index]) {
            if(index >= 0 && index <=5)
            {
              return this.currentStep = steps[index]
            }
          }

          this.currentStep = { "label": "complete" }
        },
        stepClasses(index) {
          let result = `progress__step progress__step--${index + 1} `
          if (this.currentStep && this.currentStep.label === 'complete' ||
          index < this.steps.indexOf(this.currentStep)) {
            return result += 'progress__step--complete'
          }
          if (index === this.steps.indexOf(this.currentStep)) {
            return result += 'progress__step--active'
          }
          return result
        }
      },
      computed: {
        progressClasses() {
          let result = 'progress '
          if (this.currentStep && this.currentStep.label === 'complete') {
          return result += 'progress--complete'
          }
          return result += `progress--${this.steps.indexOf(this.currentStep) + 1}`
        }
      }
    })
    function getSpeedstatus() {
      var statushttp = new XMLHttpRequest();
      statushttp.onreadystatechange = function() {
        if (statushttp.readyState == 4 && statushttp.status == 200){
          window.index = parseInt(statushttp.responseText) - 3;
          console.log("index:" + index);  //主控台顯示warn值
        }
      };
      statushttp.open("GET", "readStatus", true);
      statushttp.send();
    }
    function getBatstatus() {
      var batstatushttp = new XMLHttpRequest();
      batstatushttp.onreadystatechange = function() {
        if (batstatushttp.readyState == 4 && batstatushttp.status == 200){
          var batteryvalue = parseInt(batstatushttp.responseText);
          if(batteryvalue == 100)
          {
            document.getElementById("battery").className = "fa fa-battery-4";
            document.getElementById("battery").style.color = "blue";
          }  
          else if(batteryvalue == 80) 
          {
            document.getElementById("battery").className = "fa fa-battery-3";
            document.getElementById("battery").style.color = "blue";
          }
          else if(batteryvalue == 60) 
          {
            document.getElementById("battery").className = "fa fa-battery-2";
            document.getElementById("battery").style.color = "blue";
          }
          else if(batteryvalue == 40) 
          {
            document.getElementById("battery").className = "fa fa-battery-1";
            document.getElementById("battery").style.color = "red";
          }
          else if(batteryvalue == 20) 
          {
            document.getElementById("battery").className = "fa fa-battery-0";
            document.getElementById("battery").style.color = "red";
          }  
        }
      };
      batstatushttp.open("GET", "readBatStatus", true);
      batstatushttp.send();
    }
    </script>
  </body> 
</html>
)rawliteral";

static esp_err_t index_handler(httpd_req_t *req){
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

byte m_send;   //中斷要改變的值設為 volatile
int LEDstatus = LOW;
bool stablestart = false;
bool pressrecord = false;

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
    stablestart = false;
  }
  else if(!strcmp(variable, "left")) {
    Serial.println("l");
    m_send = 'l';
    stablestart = false;
  }
  else if(!strcmp(variable, "right")) {
    Serial.println("r");
    m_send = 'r';
    stablestart = false;
  }
  else if(!strcmp(variable, "backward")) {
    Serial.println("b");
    m_send = 'b';
    stablestart = false;
  }
  else if(!strcmp(variable, "stop")) {
    Serial.println("t");
    m_send = 't';
    stablestart = false;
  }
  else if(!strcmp(variable, "stable")) {
    Serial.println("s");
    m_send = 's';
    stablestart = true;
    pressrecord = true;
  }
  else if(!strcmp(variable, "hold")) {
    Serial.println("h");
    m_send = 'h';
    stablestart = false;
  }
  else if(!strcmp(variable, "release")) {
    Serial.println("m");
    m_send = 'm';
    stablestart = false;
  }
  else if(!strcmp(variable, "armup")) {
    Serial.println("a");
    m_send = 'a';
    stablestart = false;
  }
  else if(!strcmp(variable, "armdown")) {
    Serial.println("q");
    m_send = 'q';
    stablestart = false;
  }
  else if(!strcmp(variable, "dive")) {
    Serial.println("d");
    m_send = 'd';
    stablestart = false;
  }
  else if(!strcmp(variable, "rise")) {
    Serial.println("e");
    m_send = 'e';
    stablestart = false;
  }
  else if(!strcmp(variable, "reset")) {
    Serial.println("x");
    m_send = 'x';
  }
  else if(!strcmp(variable, "light")) {
    Serial.println("light");
    LEDstatus = !LEDstatus;
    digitalWrite(LED_PIN, LEDstatus);
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

byte m_receive;
void handleSpeedstatus(){
  String Status = String(m_receive);
  server.send(200, "text/plane", Status);
}

String batteryvalue;
void handleBatterystatus(){
  String BatStatus = batteryvalue;
  server.send(200, "text/plane", BatStatus);
}

MS5837 sensor;

void setup () {
  Serial.begin(115200); //set baud rate to 115200 for usart
  digitalWrite(SS, HIGH); // disable Slave Select
  SPI.begin(ESP32_CAM_SCK,  ESP32_CAM_MISO, ESP32_CAM_MOSI, ESP32_CAM_SS);
  SPI.setClockDivider(SPI_CLOCK_DIV16);//divide the clock by 8
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  
  Serial.setDebugOutput(false);
  Wire.begin(I2C_SDA, I2C_SCL);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Wi-Fi connection
  WiFi.setSleep(false);
  WiFi.mode (WIFI_STA);
  WiFi.setAutoReconnect(true);

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
  server.on("/readStatus", handleSpeedstatus);
  server.on("/readBatStatus", handleBatterystatus);
  
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
long warnnum = 0;
int originalpressure;
int previouspressure;
int Dpressure;
int Bpressure;
int Apressure;
int Epressure;
int DepthStableCount = 0;
char flag = 'C';
int dampcount = 0;
//int timecount = 0;

void loop () {
  server.handleClient();

  warnnum++;
  if(warnnum >= 1000)
  {
    warnnum = 0;
    if(WiFi.RSSI()<(-200))     //RSSI小於-200跳警告
    {
      w = 1;
    }
    else
    {
      w = 2;
    }
  }

  SPIsendnum++;
  if(SPIsendnum >= 100)
  {
    SPIsendnum = 0;
    char c = m_send;
    digitalWrite(SS, LOW); // enable Slave Select
    m_receive = SPI.transfer(c);

    if(m_receive == 'w') 
    {
      w = 3;
      batteryvalue = 20;
    }
    else if(m_receive == 'o') batteryvalue = 40;
    else if(m_receive == 'i') batteryvalue = 60;
    else if(m_receive == 'u') batteryvalue = 80;
    else if(m_receive == 'y') 
    {
      batteryvalue = 100;
    }
       
    m_send = 0;
    Serial.println(m_receive);
  }

  sensernum++;
  if(sensernum >= 250)  //約200ms讀取一次溫度
  {
    sensernum = 0;
    sensor.read();   //函式裡有 delay 40 ms
    // Serial.print("Temperature: "); 
    // Serial.print(sensor.temperature()); 
    // Serial.println(" deg C");
    // timecount++;
    //Serial.println(timecount);
    tempera = sensor.temperature();
    pressu = sensor.pressure();
    Serial.println(pressu);
    if(stablestart)
    {
      if(pressrecord)
      {
        originalpressure = pressu;
        previouspressure = pressu;
        Dpressure = originalpressure + 3;
        Bpressure = originalpressure - 3;
        Apressure = originalpressure - 5;
        Epressure = originalpressure + 5;
        pressrecord = false;
      }

      DepthStableCount++;
      if(DepthStableCount >= 1)  //約400ms 作動一次
      {
        DepthStableCount = 0;
        if(pressu <= Bpressure)   //B上方
        {
          dampcount = 0;
          if(flag == 'C' || flag == 'D') flag = 'B';       
          if(pressu <= Apressure && pressu <= previouspressure)
          {
            flag = 'A';
            m_send = 'p';   //dive 8
          }
          else   // between A & B
          {
            if(flag == 'B' && pressu <= previouspressure)
            {
              m_send = 'g';   //dive 2
            }
            else if(flag == 'A' && pressu > previouspressure)
            {
              m_send = 'k';   //rise 2
            }
          }
        }
        else if(pressu >= Dpressure)  //D下方
        {
          dampcount = 0;
          if(flag == 'C' || flag == 'B') flag = 'D';       
          if(pressu >= Epressure && pressu >= previouspressure)
          {
            flag = 'E';
            m_send = 'j';   //rise 4
          }
          else   // between D & E
          {
            if(flag == 'D' && pressu >= previouspressure)
            {
              m_send = 'k';   //rise 2
            }
            else if(flag == 'E' && pressu < previouspressure)
            {
              m_send = 'g';   //dive 2
            }
          }
        }
        else  //between B & D
        {
          if(flag == 'A' || flag == 'B')
          {
            flag = 'B';
            dampcount++;
            if(dampcount <= 5)
            {
              m_send = 'k';   //rise 2
            }       
            else if(dampcount <= 10)
            {
              m_send = 'g';   //dive 2
            }
          }
          else if(flag == 'D' || flag == 'E')
          {
            flag = 'D';
            dampcount++;
            if(dampcount <= 5)
            {             
              m_send = 'g';   //dive 2
            }       
            else if(dampcount <= 10)
            {
              m_send = 'k';   //rise 2
            }
          }
        }
      }
      previouspressure = pressu;
    }
    temperature = String(tempera,2);
    pressure = String(pressu,2);
    //Serial.println(WiFi.RSSI());
  }
}