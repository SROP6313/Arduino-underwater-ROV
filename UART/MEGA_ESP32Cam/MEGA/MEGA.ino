void setup() {  
  Serial.begin(115200);
  Serial2.begin(9600);
}

void loop(){
  Serial.println("From CAM：");
  Serial.println(Serial2.readString());
  Serial2.println("HI From Mega");
}
