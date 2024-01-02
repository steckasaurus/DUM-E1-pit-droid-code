#include "sbus.h"
#include "wiring_private.h"
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
extern Stream *rcSerialPort;
bfs::SbusRx sbus_rx(&Serial3);
bfs::SbusData data;
void SERCOM3_Handler() {
  Serial3.IrqHandler();
}
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(2000);
  pinPeripheral(1, PIO_SERCOM);  // Assign RX function to pin 1
  Serial.print("Begin SBUS...");
  sbus_rx.Begin();
  Serial.println("ok");
}
void loop() {
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    for (int8_t i = 0; i < data.NUM_CH; i++) {
      Serial.print(data.ch[i]);
      Serial.print("\t");
    }
    Serial.print(data.lost_frame);
    Serial.print("\t");
    Serial.println(data.failsafe);
  }
}