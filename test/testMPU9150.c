#include <MPU9150.h>

void setup(){
	Serial.begin(57600);
	MPU9150_init();
}
void loop(){
	MPU9150_Read();
	Serial.println(accelX + '\t' + accelY + '\t' + accelZ + '\t' + gyroX + '\t' + gyroY + '\t' + gyroZ + '\t' + magX + '\t' + magY + '\t' + magZ);
	delay(5);

}

int main(int argc, char** arvg){
	setup();
	while(1){
		loop();
	}
}


