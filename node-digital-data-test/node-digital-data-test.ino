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
#define INT_1B_OUTPUT 0
#define INT_2B_OUTPUT 1
#define INT_1B_INPUT 2
#define INT_2B_INPUT 3
#define DIGITAL_INPUT 4
#define DIGITAL_OUTPUT 5
#define BYTE_INPUT 6
#define BYTE_OUTPUT 7

// Attached sensor info name
String sensorName = "Digital Data Test";
// Attached sensor type
uint8_t sensorType = DIGITAL_OUTPUT;

// Get digital data
uint8_t getData() {
  return 85;
}

// Setup debug serial
SoftwareSerial dbgSer(DBG_SERIAL_RX_PIN ,DBG_SERIAL_TX_PIN);
void dbg(String msg) {
  char out[64];
  msg.toCharArray(out, msg.length() + 1);
  dbgSer.write(out);
}

XBee xbee = XBee();

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
void sendResponse(ZBRxResponse rx, uint8_t payload[], int payloadSize) {
  ZBTxRequest tx = ZBTxRequest(rx.getRemoteAddress16(), payload, payloadSize);
  xbee.send(tx);
  // Check response was sent
  checkTxResponse();
}

// Send a NACK with data type
void sendNack(ZBRxResponse rx, uint8_t type) {
  uint8_t payload[2] = {CTRL_NACK, type};
  sendResponse(rx, payload, sizeof(payload));
}

// Handle an unknown data packet
void handleUnknownPacket(ZBRxResponse rx) {
  dbg("Unknown data packet.\r\n");
  dbgRxPacket(rx);
  uint8_t payload[2] = {CTRL_NACK, rx.getData(0)};
  sendResponse(rx, payload, sizeof(payload));
}

// Handle an IO_REQUEST
void handleIOReq(ZBRxResponse rx) {
  dbg("Processing IO_REQUEST.\r\n");
  // Need to send back an IO_RESPONSE
  uint8_t payload[2] = {IO_RESPONSE, (sensorType << 4) + 0};
  sendResponse(rx, payload, sizeof(payload));
  dbg("Done.\r\n");
}

// Handle an INFO_REQUEST
void handleInfoReq(ZBRxResponse rx) {
  dbg("Processing INFO_REQUEST.\r\n");
  if (rx.getDataLength() == 2) {
    // Need to send back an INFO_RESPONSE
    uint8_t device = rx.getData(1) >> 4;
    uint8_t index = rx.getData(1) & 15;
    if (device == sensorType && index == 0) {
      uint8_t payload[32] = {INFO_RESPONSE, (device << 4) + index};
      for (int i = 0; i < sensorName.length(); i++) {
        payload[i + 2] = sensorName[i];
      }
      sendResponse(rx, payload, 2 + sensorName.length());
    } else {
      dbg("Device " + String(device) + ", and index " + String(index) + " requested, does not exist.\r\n");
      sendNack(rx, INFO_REQUEST);
    }
  } else {
    dbg("INFO_REQUEST data length not 2, is: " + String(rx.getDataLength()) + "\r\n");
    sendNack(rx, INFO_REQUEST);
  }
  dbg("Done.\r\n");
}

// Handle a DATA_REQUEST
void handleDataReq(ZBRxResponse rx) {
  dbg("Processing DATA_REQUEST.\r\n");
  if (rx.getDataLength() == 2) {
    // Need to send back a DATA_RESPONSE
    uint8_t device = rx.getData(1) >> 4;
    uint8_t index = rx.getData(1) & 15;
    if (device == sensorType && index == 0) {
      // Send back data
      uint8_t payload[3] = {DATA_RESPONSE, (device << 4) + index};
      payload[2] = getData();
      sendResponse(rx, payload, sizeof(payload));
    } else {
      dbg("Device " + String(device) + ", and index " + String(index) + " requested, does not exist.\r\n");
      sendNack(rx, DATA_REQUEST);
    }
  } else {
    dbg("DATA_REQUEST data length not 2, is: " + String(rx.getDataLength()) + "\r\n");
    sendNack(rx, DATA_REQUEST);
  }
  dbg("Done.\r\n");
}

// Handle all data interactions above ZigBee protocol
void handleData() {
  // Get data from packet
  ZBRxResponse rx = ZBRxResponse();
  xbee.getResponse().getZBTxStatusResponse(rx);
  if (rx.getData(0) == IO_REQUEST) {
    // Packet is IO_REQUEST
    handleIOReq(rx);
  } else if (rx.getData(0) == INFO_REQUEST) {
    // Packet is INFO_REQUEST
    handleInfoReq(rx);
  } else if (rx.getData(0) == DATA_REQUEST) {
    // Packet is DATA_REQUEST
    handleDataReq(rx);
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
      handleData(); // Handle data packets from XBee
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
