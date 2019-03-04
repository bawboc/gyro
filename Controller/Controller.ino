/*
 Name:		Controller.ino
 Created:	2/24/2019 2:35:59 PM
 Author:	jhlaw
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00010";
int count = 1;
char packets[6];
bool aflag, gflag;
char achar[32];
char gchar[32];
void setup() {
	Serial.begin(115200);
	radio.begin();
	radio.openReadingPipe(0, address);
	radio.setPALevel(RF24_PA_MIN);
	radio.startListening();
	aflag = false;
	gflag = false;
	count = 1;
}

int find(char* c, int s, const char f) {
	for (int i = 0; i < 2; i++) {
		if (c[i] == f) return i;
	}
	return -1;
}

void loop() {
	char text[32] = "";
	radio.read(&text, sizeof text);

	if (!aflag && text[0] == 'A' && !gflag)
	{
		snprintf(achar, sizeof text, "%s", text);
		aflag = true;
	}
	else if (aflag && text[0] == 'A') {
		aflag = false;
		gflag = false;
	}
	else if (aflag && !gflag && text[0] == 'G') {
		snprintf(gchar, sizeof text, "%s", text);
		gflag = true;
	}
	else if (!aflag && gflag) {
		aflag = false;
		gflag = false;
	}
	else if (aflag && gflag)
	{
		Serial.println(String(achar) + " " + String(gchar));
		aflag = false;
		gflag = false;
	}
	delay(10);
}

