unsigned int toggle;
void setup() {
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  delay(1000);
  toggle = LOW;
}

void loop() {
  for(int count = 0; count < 10; ++count){
    toggle = toggle_state(toggle);
    digitalWrite(7, toggle);
    delay(100);
  } 
  while(1){
    digitalWrite(7, HIGH);
  }
}

int toggle_state(int toggle){
  if(toggle == LOW){
      toggle = HIGH;
    }
    else{
      toggle = LOW;
    }
    return toggle;
  }
