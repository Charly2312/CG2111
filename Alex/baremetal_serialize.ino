// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial() {
  // To replace later with bare-metal.
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
  // Setting the baud rate to 9600
  //Serial.begin(9600);
  unsigned int b;
  b = (unsigned int) round(16000000 / (16.0 * 9600)) - 1; //by right gives 103
  UBRR0H = (unsigned char) (b >> 8);
  UBRR0L = (unsigned char) b;
  
  //Running in asynchronous mode
  //No parity so bits 5 and 4 are 00
  //1 stop bit so bit 3 is 0
  //8 data bits so UCSZ[2:0] is 011
  //bit 0 is always 0
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
  
  //zero in every bit to prevent data corruption especially U2X0 and MPCM0
  //to prevent us from doubling the transmission speed and enabling multiprocesser mode
  UCSR0A = 0;
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial() {
  // Empty for now. To be replaced with bare-metal code
  // later on.
  // Enabling the receiver and transmitter directly by setting bit 3 and 4
  UCSR0B = 0b00011000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer) {

  int count = 0;
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  //We wait until RXC0 is set to 1, which means USART has received data
  while (UCSR0A & 0b10000000) == 0) 
  buffer[count++] = UDR0;

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len) {
  //Serial.write(buffer, len);
  for (int i = 0; i < len; i ++) {
    UDR0 = buffer[i];
  }
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}
