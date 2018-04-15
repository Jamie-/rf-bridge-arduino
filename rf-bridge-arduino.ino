#include <XBee.h>
#include <SoftwareSerial.h>

// Pins
#define DBG_SERIAL_RX_PIN 6
#define DBG_SERIAL_TX_PIN 7

// Packet headers
#define DATA_REQUEST 16
#define DATA_RESPONSE 17
#define IO_REQUEST 18
#define IO_RESPONSE 19
#define INFO_REQUEST 20
#define INFO_RESPONSE 21
#define SET_REQUEST 22
#define DATA_ALERT 23
#define CTRL_NACK 254
#define CTRL_ACK 255

// IO Types
#define ANALOGUE_1BYTE 0
#define ANALOGUE_2BYTE 1
#define DIGITAL_INPUT 2
#define DIGITAL_OUTPUT 3
#define BYTE_INPUT 4
#define BYTE_OUTPUT 5

// Setup debug serial
SoftwareSerial dbgSer(DBG_SERIAL_RX_PIN ,DBG_SERIAL_TX_PIN);
void dbg(String msg) {
  char out[64];
  msg.toCharArray(out, msg.length() + 1);
  dbgSer.write(out);
}

XBee xbee = XBee();
// Attached sensor info name
String sensorName = "Plant Moisture";

// Dump the payload of a ZBRX_RESP packet to debug
void dbgRxPacket(ZBRxResponse rx) {
  dbg("Data (ints): ");
  for (int i = 0; i < rx.getDataLength(); i++) { 
    dbg(String(rx.getData(i)) + " ");
  }
  dbg("\r\nData (chars): ");
  for (int i = 0; i < rx.getDataLength(); i++) { 
    dbg(String(char(rx.getData(i))));
  }
  dbg("\r\n");
}

// Check a TX_RESPONSE was sent after sending a packet, log to debug
void checkTxResponse() {
  if (xbee.readPacket(100)) {             
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      ZBTxStatusResponse txStatus = ZBTxStatusResponse();
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      if (txStatus.getDeliveryStatus() != SUCCESS) {
        dbg("Delivery failed, status: " + String(txStatus.getDeliveryStatus()) + "\r\n");
      }
    } else {
      dbg("Did not get TX_RESP, got: " + String(xbee.getResponse().getApiId()) + "\r\n");
    }
  }
}

// Send data back to sender (master)
void sendResponse(XBee xbee, ZBRxResponse rx, uint8_t payload[], int payloadSize) {
  ZBTxRequest tx = ZBTxRequest(rx.getRemoteAddress16(), payload, payloadSize);
  xbee.send(tx);
  // Check response was sent
  checkTxResponse();
}

// Send a NACK with data type
void sendNack(XBee xbee, ZBRxResponse rx, uint8_t type) {
  uint8_t payload[2] = {CTRL_NACK, type};
  sendResponse(xbee, rx, payload, sizeof(payload));
}

// Handle an unknown data packet
void handleUnknownPacket(ZBRxResponse rx) {
  dbg("Unknown data packet.\r\n");
  dbgRxPacket(rx);
  uint8_t payload[2] = {CTRL_NACK, rx.getData(0)};
  sendResponse(xbee, rx, payload, sizeof(payload));
}

// Handle an IO_REQUEST
void handleIOReq(XBee xbee, ZBRxResponse rx) {
  dbg("Processing IO_REQUEST.\r\n");
  // Need to send back an IO_RESPONSE
  uint8_t payload[3] = {IO_RESPONSE, (ANALOGUE_1BYTE << 4) + 0};
  sendResponse(xbee, rx, payload, sizeof(payload));
  dbg("Done.\r\n");
}

// Handle an INFO_REQUEST
void handleInfoReq(XBee xbee, ZBRxResponse rx) {
  dbg("Processing INFO_REQUEST.\r\n");
  if (rx.getDataLength() == 2) {
    // Need to send back an INFO_RESPONSE
    uint8_t device = rx.getData(1) >> 4;
    uint8_t index = rx.getData(1) & 15;
    if (device == ANALOGUE_2BYTE && index == 0) {
      uint8_t payload[32] = {INFO_RESPONSE, (device << 4) + index};
      for (int i = 0; i < 14; i++) {
        payload[i + 2] = sensorName[i];
      }
      sendResponse(xbee, rx, payload, 2 + sensorName.length());
    } else {
      dbg("Device " + String(device) + ", and index " + String(index) + " requested, does not exist.\r\n");
      sendNack(xbee, rx, INFO_REQUEST);
    }
  } else {
    dbg("INFO_REQUEST data length not 2, is: " + String(rx.getDataLength()) + "\r\n");
    sendNack(xbee, rx, INFO_REQUEST);
  }
  dbg("Done.\r\n");
}

// Handle all data interactions above ZigBee protocol
void handleData(XBee xbee) {
  // Get data from packet
  ZBRxResponse rx = ZBRxResponse();
  xbee.getResponse().getZBTxStatusResponse(rx);
  if (rx.getData(0) == IO_REQUEST) {
    // Packet is IO_REQUEST
    handleIOReq(xbee, rx);
  } else if (rx.getData(0) == INFO_REQUEST) {
    // Packet is INFO_REQUEST
    handleInfoReq(xbee, rx);
  } else {
    // Unknown data packet
    handleUnknownPacket(rx);
  }
}

void setup() {
  // Start Xbee serial
  Serial.begin(9600);
  xbee.setSerial(Serial);
  // Start debug serial
  dbgSer.begin(9600);
  dbg("Booted\r\n");
}

void loop() {
  xbee.readPacket();
  // Keep checking for packet until one arrives
  if (xbee.getResponse().isAvailable()) {
    dbg("Packet incoming.\r\n");
    
    // Check if data is a data inbound packet
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      handleData(xbee); // Handle data packets
    } else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
      dbg("Recieved modem status packet, ignoring.\r\n");
    } else {
      dbg("Not data packet, is: " + String(xbee.getResponse().getApiId()) + "\r\n");
    }
    dbg("\r\n");
  }
  
  if (xbee.getResponse().isError()) {
    dbg("Error reading packet, code: " + String(xbee.getResponse().getErrorCode()) + "\r\n\r\n");
  }
}
