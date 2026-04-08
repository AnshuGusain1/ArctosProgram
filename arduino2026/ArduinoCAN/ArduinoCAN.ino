#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS_PIN   10
#define CAN_INT_PIN   2
#define LED_PIN       5    // PWM pin D5

#define MY_CAN_ID    0x0D
#define CMD_SET_PWM  0x03

MCP_CAN CAN(CAN_CS_PIN);
volatile bool msgAvailable = false;

void ISR_CAN() {
    msgAvailable = true;
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    analogWrite(LED_PIN, 0);

    while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        Serial.println("CAN init failed, retrying...");
        delay(100);
    }

    // Filter: only accept messages addressed to 0x0D
    CAN.init_Mask(0, 0, 0x7FF);
    CAN.init_Filt(0, 0, MY_CAN_ID);
    CAN.init_Filt(1, 0, MY_CAN_ID);
    CAN.init_Mask(1, 0, 0x7FF);
    CAN.init_Filt(2, 0, MY_CAN_ID);
    CAN.init_Filt(3, 0, MY_CAN_ID);
    CAN.init_Filt(4, 0, MY_CAN_ID);
    CAN.init_Filt(5, 0, MY_CAN_ID);

    CAN.setMode(MCP_NORMAL);
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), ISR_CAN, FALLING);

    Serial.println("Ready. CAN ID=0x0D, waiting for PWM commands...");
}

void loop() {
    if (!msgAvailable) return;
    msgAvailable = false;

    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char buf[8];
    CAN.readMsgBuf(&rxId, &len, buf);

    if (rxId != MY_CAN_ID || len < 8) return;

    // Validate CRC: sum of bytes 0-6, mod 256
    uint8_t expected_crc = 0;
    for (int i = 0; i < 7; i++) expected_crc += buf[i];
    expected_crc &= 0xFF;

    if (buf[7] != expected_crc) {
        Serial.print("CRC fail. got=0x");
        Serial.print(buf[7], HEX);
        Serial.print(" expected=0x");
        Serial.println(expected_crc, HEX);
        return;
    }

    uint8_t command    = buf[3];
    uint8_t brightness = buf[5];

    if (command == CMD_SET_PWM) {
        analogWrite(LED_PIN, brightness);
        Serial.print("LED brightness set to: ");
        Serial.println(brightness);
    } else {
        Serial.print("Unknown command: 0x");
        Serial.println(command, HEX);
    }
}