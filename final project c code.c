/*
 * File:   main.c
 * Target: PIC16F1719 on Explorer 8
 * Compiler: XC8
 *
 * Project: YOLO adaptive traffic light (RED/GREEN) + LCD UI + pedestrian button
 * GROUP MEMBERS:     AYSE CAPACI         MURAT ATCEKEN 
 * Explorer 8 LCD is connected via MCP23S17:
 *   GPA6 -> E
 *   GPA7 -> RS
 *   GPB0..GPB7 -> DB0..DB7 (8-bit mode)
 *
 * Jumpers required for LCD:
 *   J59: RA2 -> MCP23S17 CS
 *   J60: RB5 -> MCP23S17 RESET
 *   J61: LCD power
 *
 * USB-UART:
 *   Ensure J54 is installed (USB-to-UART routing).
 *   Python sends once per second: C:<n>\\n  (e.g. C:07\\n)
 *
 * LEDs:
 *   Explorer 8 D1..D4 are on RD0..RD3. We use:
 *     D1 (RD0) = RED
 *     D2 (RD1) = GREEN
 *
 * Button:
 *   S1 is RB0 (active-low).
 */

 #include <xc.h>                    // PIC XC8 compiler header - provides SFR definitions and compiler intrinsics
 #include <stdint.h>                // Standard integer types (uint8_t, uint16_t, etc.) for precise bit-width control
 
 // Configuration Bits (PIC16F1719 - internal oscillator, no WDT, no LVP)
 // These are written to non-volatile memory during programming and set hardware behavior at reset
 #pragma config FOSC = INTOSC       // Use internal oscillator instead of external crystal - reduces BOM cost and complexity
 #pragma config WDTE = OFF          // Watchdog timer disabled - we use Timer1 for timing, WDT unnecessary here
 #pragma config PWRTE = OFF         // Power-up timer disabled - reduces startup delay (acceptable for stable power)
 #pragma config MCLRE = ON          // MCLR pin enabled - allows external reset capability (important for debugging)
 #pragma config CP = OFF            // Code protection disabled - allows reading/writing flash via programmer
 #pragma config BOREN = OFF         // Brown-out reset disabled - assumes stable power supply (trade-off: less safety)
 #pragma config CLKOUTEN = OFF      // Clock output disabled - saves pin, not needed for this application
 #pragma config IESO = OFF          // Internal/External oscillator switchover disabled - using only internal
 #pragma config FCMEN = OFF         // Fail-safe clock monitor disabled - redundant with IESO=OFF
 #pragma config WRT = OFF           // Flash write protection disabled - program cannot self-modify (security consideration)
 #pragma config PPS1WAY = OFF       // Peripheral Pin Select reconfigurable - allows runtime remapping (flexibility)
 #pragma config ZCDDIS = ON         // Zero-cross detect disabled - not using for power control in this design
 #pragma config PLLEN = OFF         // Phase-locked loop disabled - running at 8MHz without frequency multiplication
 #pragma config STVREN = ON         // Stack overflow/underflow reset enabled - catches firmware bugs early (safety)
 #pragma config BORV = LO           // Brown-out voltage set to low threshold (2.45V) - not used since BOREN=OFF
 #pragma config LPBOR = OFF         // Low-power brown-out reset disabled - consistent with BOREN=OFF
 #pragma config LVP = OFF           // Low-voltage programming disabled - requires 12V for programming (standard practice)
 
 // System clock used by __delay_ms/us
 // Critical: This macro tells the compiler the actual clock frequency for delay calculation accuracy
 #define _XTAL_FREQ 8000000         // 8 MHz system clock - matches OSCCON setting; delay functions use this value
 
 // MCP23S17 Registers (SPI I/O expander for LCD)
 // MCP23S17 is a 16-bit I/O expander accessed via SPI; these are register addresses in its address space
 #define MCP_IODIRA   0x00          // I/O Direction Register for Port A - configures pins as input (1) or output (0)
 #define MCP_IODIRB   0x01          // I/O Direction Register for Port B - same as Port A but for second 8-bit port
 #define MCP_IOCON    0x0A          // I/O Configuration Register - enables address decoding mode (HAEN bit)
 #define MCP_GPIOA    0x12          // General Purpose I/O Register for Port A - reads current pin states
 #define MCP_GPIOB    0x13          // General Purpose I/O Register for Port B - reads current pin states
 #define MCP_OLATA    0x14          // Output Latch Register for Port A - writes to output pins (used for LCD control)
 #define MCP_OLATB    0x15          // Output Latch Register for Port B - writes to output pins (used for LCD data bus)
 
 // MCP23S17 SPI Address (A0=A1=A2=0 on Explorer 8)
 // SPI protocol uses 7-bit addressing: bit7=1 means register operation, bits6-4=0 for address pins (all grounded)
 #define MCP_WRITE    0x40          // Write operation: R/W bit=0, address A2:A0=000, byte-oriented mode
 #define MCP_READ     0x41          // Read operation: R/W bit=1, address A2:A0=000, byte-oriented mode
 
 // LCD mapping on MCP23S17 (Explorer 8 schematic)
 // LCD uses standard HD44780 protocol - these bits control LCD interface signals via MCP23S17 Port A
 #define LCD_E_BIT    0x40          // Enable signal on GPA6 - active-high pulse strobes data into LCD
 #define LCD_RS_BIT   0x80          // Register Select on GPA7 - 0=command, 1=data (fundamental HD44780 control)
 
 // Chip Select + Reset (via jumpers J59/J60)
 // SPI requires CS (Chip Select) to frame transactions; RESET provides hardware initialization capability
 #define CS_LOW()     LATAbits.LATA2 = 0    // Assert CS active-low - signals start of SPI transaction to MCP23S17
 #define CS_HIGH()    LATAbits.LATA2 = 1    // Deassert CS - signals end of transaction (latch received data)
 #define RESET_LOW()  LATBbits.LATB5 = 0    // Assert RESET active-low - forces MCP23S17 into known initial state
 #define RESET_HIGH() LATBbits.LATB5 = 1    // Deassert RESET - allows MCP23S17 to exit reset and become operational
 
 // Delay helpers (used for LCD timing and SPI settle time)
 // These functions wrap compiler intrinsics to create variable-length delays for peripheral timing requirements
 void delay_ms(unsigned int ms) {  // Millisecond delay - LCD commands need precise timing (HD44780 spec: 1-5ms typical)
     while(ms--) {                  // Loop decrements counter - each iteration delays 1ms, total = ms milliseconds
         __delay_ms(1);             // Compiler intrinsic - generates precise NOP loops calibrated to _XTAL_FREQ
     }                              // Critical for LCD: insufficient delay causes command/data corruption
 }
 
 void delay_us(unsigned int us) {  // Microsecond delay - SPI setup/hold times and LCD enable pulse width
     while(us--) {                  // Fine-grained timing control - needed for SPI CS setup and LCD E pulse (10-100µs)
         __delay_us(1);             // Compiler generates instruction-level delays - minimum resolution for timing-critical operations
     }                              // Note: at 8MHz, 1µs = 2 instruction cycles (caveat: loop overhead may add slight error)
 }
 
 // ---------------------------------------------------------------------------
 // SPI Functions (SSP1) -> MCP23S17
 // ---------------------------------------------------------------------------
 // SPI (Serial Peripheral Interface) enables communication with MCP23S17 I/O expander
 // Master mode: PIC generates clock and initiates all transactions
 void SPI_Init(void) {
     // PPS mapping for SPI signals
     // PPS (Peripheral Pin Select) allows remapping hardware peripherals to different physical pins
     // This provides flexibility when board layout conflicts with default pin assignments
     PPSLOCK = 0x55;                // First unlock sequence byte - prevents accidental PPS modifications
     PPSLOCK = 0xAA;                // Second unlock sequence byte - must be written in sequence to unlock
     PPSLOCKbits.PPSLOCKED = 0;     // Clear lock bit - now PPS registers can be modified (prevents race conditions)
     
     RC3PPS = 0x10;      // RC3 = SCK     // Map SSP1 clock output to RC3 - SPI clock signal (generated by master)
     RC5PPS = 0x11;      // RC5 = SDO     // Map SSP1 data output to RC5 - master-out-slave-in (MOSI) signal
     SSPDATPPS = 0x14;   // RC4 = SDI     // Map SSP1 data input from RC4 - master-in-slave-out (MISO) signal
     
     PPSLOCK = 0x55;                // Re-lock sequence - prevents further PPS changes until next unlock sequence
     PPSLOCK = 0xAA;                // Second re-lock byte - ensures PPS configuration remains stable
     PPSLOCKbits.PPSLOCKED = 1;     // Set lock bit - protects PPS settings from accidental modification
     
     // SPI pins direction
     // TRIS register controls I/O direction: 0=output, 1=input (fundamental PIC I/O model)
     TRISCbits.TRISC3 = 0;  // SCK output // Clock must be output by master - drives slave's clock input
     TRISCbits.TRISC4 = 1;  // SDI input  // MISO from slave must be input - PIC receives data from MCP23S17
     TRISCbits.TRISC5 = 0;  // SDO output // MOSI to slave must be output - PIC sends commands/data to MCP23S17
     
     // CS + RESET pins
     // These are software-controlled GPIO pins (not part of SPI hardware but required for SPI protocol)
     TRISAbits.TRISA2 = 0;  // CS output (J59)   // Chip Select must be output - frames SPI transactions
     TRISBbits.TRISB5 = 0;  // RESET output (J60) // Reset control must be output - initializes MCP23S17 on startup
     CS_HIGH();                                  // CS idle state is high - deasserted until transaction starts
     RESET_HIGH();                               // RESET idle state is high - MCP23S17 operational (not in reset)
     
     // SPI Master mode, Fosc/16 (stable for MCP23S17)
     // SSP1CON1 configures SPI mode and clock division; MCP23S17 supports up to 10MHz SPI clock
     SSP1CON1 = 0x21;  // SPI Master, Fosc/16, enabled  // Bit5:SSPEN=1 (enable), Bit4-0:CKP=0,SSPM=001 (Master Fosc/16)
                       // At 8MHz Fosc: SPI clock = 8MHz/16 = 500kHz - well within MCP23S17 10MHz max (safe margin)
     SSP1STAT = 0x40;  // CKE=1                        // Clock edge: sample on rising edge, change on falling (mode 0,0)
                       // CKE=1 means output changes on transition from idle to active clock edge (SPI mode 0 compatibility)
 }
 
 // SPI transfer function - sends one byte and receives one byte simultaneously (full-duplex)
 unsigned char SPI_Transfer(unsigned char data) {
     SSP1BUF = data;                    // Write data to SPI buffer - automatically initiates transmission via hardware
                                        // Hardware shifts out bits on SDO synchronized with SCK clock generation
     while(!SSP1STATbits.BF);           // Wait for Buffer Full flag - indicates both TX complete and RX data ready
                                        // BF is set when transmission finishes AND new data has been received from slave
     return SSP1BUF;                    // Return received byte - SSP1BUF contains data shifted in from SDI during transfer
                                        // In SPI, master receives slave's response while transmitting command (full-duplex)
 }
 
 // ---------------------------------------------------------------------------
 // MCP23S17 Functions
 // ---------------------------------------------------------------------------
 // MCP23S17 is Microchip's SPI-based 16-bit I/O expander - extends PIC's limited I/O pins for LCD interface
 void MCP_Write(unsigned char reg, unsigned char data) {
     CS_LOW();                          // Assert Chip Select - signals start of SPI transaction to MCP23S17
                                        // MCP23S17 requires CS to be low before first clock edge
     delay_us(1);                       // CS setup time - ensures MCP23S17 recognizes transaction start (tCSS spec)
     SPI_Transfer(MCP_WRITE);           // Send write opcode (0x40) - tells MCP23S17 this is a write operation
     SPI_Transfer(reg);                 // Send register address - selects which MCP23S17 register to modify
     SPI_Transfer(data);                // Send data byte - value to write into selected register
     delay_us(1);                       // CS hold time - ensures MCP23S17 latches data before CS deassertion
     CS_HIGH();                         // Deassert CS - signals end of transaction, MCP23S17 processes the write
                                        // MCP23S17 requires CS to remain low during entire transaction
 }
 
 void MCP_Init(void) {
     // Reset expander (J60)
     // Hardware reset ensures MCP23S17 starts in known state - all registers return to power-on defaults
     RESET_LOW();                       // Assert RESET active-low - forces MCP23S17 into reset state
     delay_ms(2);                       // Hold RESET low for minimum reset pulse width (tRESET ~ 1ms, 2ms for safety)
     RESET_HIGH();                      // Release RESET - MCP23S17 exits reset and begins initialization
     delay_ms(2);                       // Recovery time - allows internal state machines to stabilize after reset
     
     // Enable HAEN (address pins) in IOCON
     // HAEN (Hardware Address Enable) allows address pins A0-A2 to control device address (needed for multi-device SPI)
     MCP_Write(MCP_IOCON, 0x08);       // Write IOCON register: bit3 (HAEN)=1 enables hardware addressing
                                        // Since A0=A1=A2=0 (all grounded), device address is 0 (matches MCP_WRITE/MCP_READ)
     
     // Set Port A and Port B as outputs (LCD uses all pins)
     // IODIR register: 0=output, 1=input - all zeros means all pins configured as outputs for LCD control
     MCP_Write(MCP_IODIRA, 0x00);      // Configure Port A (8 bits) as outputs - GPA6=E, GPA7=RS, others unused
     MCP_Write(MCP_IODIRB, 0x00);      // Configure Port B (8 bits) as outputs - GPB0-7 = LCD DB0-7 (8-bit data bus)
     MCP_Write(MCP_OLATA, 0x00);       // Initialize Port A output latches to 0 - ensures LCD E and RS start low
     MCP_Write(MCP_OLATB, 0x00);       // Initialize Port B output latches to 0 - ensures LCD data bus starts at 0
                                        // Clearing outputs prevents spurious LCD commands during initialization
 }
 
 // ---------------------------------------------------------------------------
 // LCD Functions (8-bit via MCP23S17)
 // ---------------------------------------------------------------------------
 // HD44780-compatible LCD controller protocol - industry standard for character displays
 // Uses 8-bit parallel interface through MCP23S17 I/O expander (avoids using PIC's limited I/O pins directly)
 unsigned char lcd_porta = 0;          // Shadow register for Port A - tracks RS and E signal states
                                        // Shadow register pattern: maintain state in software to avoid unnecessary SPI reads
 unsigned char lcd_portb = 0;          // Shadow register for Port B - tracks 8-bit data bus (DB0-DB7) state
                                        // Reduces SPI transactions by only writing when values change
 
 void LCD_Pulse(void) {
     // HD44780 requires Enable (E) pulse to latch data/command - minimum pulse width: 450ns, typical: 1µs
     // This function generates the required E pulse to clock data into LCD's internal registers
     lcd_porta |= LCD_E_BIT;            // Set E bit high in shadow register - enables LCD's input latch
     MCP_Write(MCP_OLATA, lcd_porta);   // Write to MCP23S17 Port A - E signal goes high on LCD's E pin
     delay_us(10);                      // E pulse high time - 10µs exceeds minimum 450ns by large margin (reliable operation)
                                        // HD44780 spec: E high pulse must be 450ns minimum, 10µs provides safety margin
     lcd_porta &= ~LCD_E_BIT;           // Clear E bit in shadow register - prepare to lower E signal
     MCP_Write(MCP_OLATA, lcd_porta);   // Write to MCP23S17 Port A - E signal goes low, latches data into LCD
     delay_us(100);                     // E pulse low time - ensures LCD processes latched data before next operation
                                        // HD44780 requires minimum E low time; 100µs ensures LCD completes internal processing
 }
 
 void LCD_Write8(unsigned char value) {
     // Writes 8-bit value to LCD data bus - used for both commands and character data
     lcd_portb = value;           // DB0..DB7  // Update shadow register with new 8-bit data value
     MCP_Write(MCP_OLATB, lcd_portb);          // Write data to MCP23S17 Port B - appears on LCD DB0-DB7 pins
     LCD_Pulse();                              // Generate E pulse - clocks data into LCD's internal register (command or data RAM)
 }
 
 void LCD_Command(unsigned char cmd) {
     // Sends command byte to LCD - commands control LCD functions (clear, cursor position, display mode, etc.)
     lcd_porta &= ~LCD_RS_BIT;    // RS = 0 (command)  // Clear RS bit in shadow - RS=0 tells LCD this is a command, not data
     MCP_Write(MCP_OLATA, lcd_porta);                  // Write RS state to MCP23S17 - RS signal goes low on LCD's RS pin
     LCD_Write8(cmd);                                  // Write command byte to data bus and pulse E - LCD executes command
     delay_ms(2);                                      // Command execution delay - most HD44780 commands need 1-2ms to complete
                                                        // Critical: insufficient delay causes command to be ignored or corrupted
 }
 
 void LCD_Char(char c) {
     // Sends character data to LCD - displays ASCII character at current cursor position
     lcd_porta |= LCD_RS_BIT;     // RS = 1 (data)     // Set RS bit in shadow - RS=1 tells LCD this is character data, not command
     MCP_Write(MCP_OLATA, lcd_porta);                  // Write RS state to MCP23S17 - RS signal goes high on LCD's RS pin
     LCD_Write8((unsigned char)c);                     // Write character byte to data bus and pulse E - LCD stores in DDRAM
     delay_ms(1);                                      // Character write delay - HD44780 needs ~37µs minimum, 1ms provides margin
                                                        // After write, cursor automatically advances to next position
 }
 
 void LCD_Init(void) {
     // HD44780 initialization sequence - required to place LCD in known operational state
     // LCD needs time to stabilize after power-on before accepting commands
     delay_ms(100);                                    // Power-on delay - allows LCD's internal power supply to stabilize
                                                        // HD44780 typically needs 40-100ms after VCC reaches 4.5V
     
     // Initialize shadow registers and MCP23S17 outputs to known state
     lcd_porta = 0;                                    // Clear Port A shadow - RS=0, E=0 (idle state)
     lcd_portb = 0;                                    // Clear Port B shadow - all data lines low
     MCP_Write(MCP_OLATA, lcd_porta);                  // Set MCP23S17 Port A outputs - ensures RS and E start low
     MCP_Write(MCP_OLATB, lcd_portb);                  // Set MCP23S17 Port B outputs - ensures data bus starts at 0
     delay_ms(50);                                     // Additional stabilization time - ensures LCD is ready for initialization
     
     // 8-bit mode init sequence
     // HD44780 requires specific initialization sequence to reliably enter 8-bit mode
     // Sequence must be sent three times to ensure LCD recognizes 8-bit mode (resets internal state machine)
     LCD_Command(0x38);  // Function set: 8-bit, 2-line  // 0x38 = 8-bit interface, 2-line display, 5x8 font
                         // Bits: DB7-5=001 (8-bit), DB4=1 (2-line), DB3=1 (5x8 dots), DB2-0=000 (unused)
     delay_ms(5);                                       // First command needs longer delay - LCD may still be initializing
     LCD_Command(0x38);                                 // Repeat function set command - ensures LCD latches 8-bit mode
     delay_ms(1);                                       // Shorter delay acceptable for subsequent commands
     LCD_Command(0x38);                                 // Third function set - final confirmation of 8-bit mode
     delay_ms(1);                                       // LCD now guaranteed to be in 8-bit interface mode
     
     LCD_Command(0x0C);  // Display ON, cursor OFF      // 0x0C = display enabled, cursor invisible, cursor blink off
                         // Bits: DB3=1 (display on), DB2=0 (cursor off), DB1=0 (blink off), DB0=0 (unused)
     LCD_Command(0x06);  // Entry mode                   // 0x06 = increment cursor, no display shift
                         // Bits: DB1=1 (increment), DB0=0 (no shift) - cursor moves right after each character
     LCD_Command(0x01);  // Clear display                // 0x01 = clear all DDRAM (Display Data RAM) and return cursor home
                         // This command takes ~1.64ms to execute, requires longer delay than normal commands
     delay_ms(5);                                       // Clear display execution delay - HD44780 needs 1.64ms, 5ms ensures completion
                                                        // After this, LCD is ready for normal operation
 }
 
 void LCD_String(const char *s) {
     // Displays null-terminated C string on LCD - writes each character sequentially
     while(*s) {                                        // Loop until null terminator encountered - standard C string traversal
         LCD_Char(*s++);                                // Write current character and increment pointer - cursor auto-advances
     }                                                  // LCD automatically wraps to second line when first line fills (16 chars)
 }
 
 void LCD_SetCursor(unsigned char row, unsigned char col) {
     // Sets LCD cursor position - DDRAM address determines where next character appears
     // HD44780 DDRAM addresses: Line 1: 0x00-0x0F (0-15), Line 2: 0x40-0x4F (64-79)
     LCD_Command(row == 0 ? (0x80 + col) : (0xC0 + col));  // Set DDRAM address command: bit7=1, bits6-0=address
                                                            // Row 0: 0x80+col (0x80-0x8F), Row 1: 0xC0+col (0xC0-0xCF)
                                                            // Bit7=1 signals DDRAM address set command (vs instruction register)
 }
 
 // ---------------------------------------------------------------------------
 // UART (EUSART) - 9600 baud @ 8MHz
 // Protocol: "C:<n>\\n" where n is 00..99 (vehicle density)
 // ---------------------------------------------------------------------------
 // UART enables asynchronous serial communication with Python script for vehicle detection data
 // EUSART (Enhanced Universal Synchronous/Asynchronous Receiver/Transmitter) hardware handles bit timing
 static void UART_Init(void) {
     // RX/TX pins are fixed on PIC16F1719 (RC7=RX, RC6=TX).
     // Unlike some PICs, these pins cannot be remapped - hardware limitation of this device
     ANSELC = 0;                                       // Disable analog functions on PORTC - pins function as digital I/O
                                                        // Critical: analog mode disables digital UART functionality
     TRISCbits.TRISC7 = 1; // RX                       // RC7 as input - UART receiver requires input pin (receives data)
     TRISCbits.TRISC6 = 0; // TX                       // RC6 as output - UART transmitter requires output pin (sends data)

     // Baud: BRG16=1, BRGH=1 -> Baud = Fosc / (4*(SPBRG+1))
     // Baud rate calculation: BRG16=1 enables 16-bit baud generator, BRGH=1 selects high-speed mode
     // Formula yields: Baud = Fosc / (4 * (SPBRG + 1)) where SPBRG is 16-bit value (SP1BRGH:SP1BRGL)
     // SPBRG ~= (8,000,000/(4*9600)) - 1 ~= 207.33 -> 207
     // Error: |(actual - desired)/desired| = |(8000000/(4*208) - 9600)/9600| ≈ 0.06% (excellent accuracy)
     BAUD1CONbits.BRG16 = 1;                           // Enable 16-bit baud rate generator - provides finer baud rate resolution
     TX1STAbits.BRGH = 1;                              // High-speed baud rate mode - uses Fosc/4 divisor instead of Fosc/16
                                                        // High-speed mode allows higher baud rates with same clock frequency
     SP1BRGH = 0;                                      // High byte of baud rate generator - 16-bit value = 0x00CF = 207 decimal
     SP1BRGL = 207;                                    // Low byte of baud rate generator - actual reload value for 9600 baud
                                                        // Timer reloads with 207, counts down, generates baud rate clock

     RC1STAbits.SPEN = 1;                              // Enable serial port - activates UART hardware (required for operation)
     TX1STAbits.SYNC = 0;                              // Asynchronous mode - UART mode (not synchronous SPI-like mode)
     TX1STAbits.TXEN = 1;                              // Enable transmitter - allows sending data (clears TXIF when buffer empty)
     RC1STAbits.CREN = 1;                              // Enable continuous receiver - receives incoming data and sets RCIF flag
                                                        // CREN=1 enables RX, automatically sets RCIF when byte received in RC1REG
 }
 
 static inline uint8_t UART_Available(void) {
     // Check if UART has received data - non-blocking status check for polling-based receive
     return PIR1bits.RCIF;                             // Return RCIF flag - set by hardware when byte received, cleared by reading RC1REG
                                                        // Inline function: compiler inserts code directly (no function call overhead)
 }
 
 static uint8_t UART_Read(void) {
     // Clear overrun to keep RX alive
     // Overrun error occurs when new byte arrives before previous byte is read - hardware stops receiving
     if (RC1STAbits.OERR) {                            // Check for overrun error flag - indicates receive buffer overflow condition
         RC1STAbits.CREN = 0;                          // Disable receiver - required to clear OERR flag (cannot be cleared directly)
         RC1STAbits.CREN = 1;                          // Re-enable receiver - resets overrun condition, RX resumes normal operation
     }                                                  // Without this, UART stops receiving new data after first overrun
     return (uint8_t)RC1REG;                           // Read received byte from receive register - automatically clears RCIF flag
                                                        // RC1REG is double-buffered: new byte can arrive while previous is being read
 }
 
 static void UART_Write(uint8_t b) {
     // Transmit one byte via UART - blocking function waits for transmit buffer to be empty
     while (!PIR1bits.TXIF) {;}                        // Wait for TXIF flag - indicates transmit buffer is empty and ready for new data
                                                        // Blocking wait: prevents overwriting previous byte before transmission completes
     TX1REG = b;                                       // Write byte to transmit register - hardware automatically starts transmission
                                                        // TXIF is cleared automatically when data loaded, set when transmission completes
 }
 
 static void UART_WriteStr(const char* s) {
     // Transmit null-terminated string via UART - calls UART_Write for each character
     while (*s) UART_Write((uint8_t)*s++);             // Loop through string, transmit each character until null terminator
                                                        // Pointer increment moves to next character after transmission
 }
 
 // ---------------------------------------------------------------------------
 // 1-second tick via Timer1 (10ms base)
 // Fosc=8MHz => Fosc/4=2MHz, prescale 1:8 => tick=4us
 // 10ms => 2500 ticks, preload = 65536-2500 = 63036 = 0xF63C
 // This keeps the main loop non-blocking and enables clean state transitions.
 // ---------------------------------------------------------------------------
 // Timer1 generates periodic interrupts for time-based traffic light state machine
 // 16-bit timer with interrupt enables precise timing without blocking main loop execution
 volatile uint8_t g_tick_10ms = 0;                    // 10ms tick counter - volatile required because modified in ISR
                                                        // Volatile prevents compiler optimization (register caching) across ISR/main context
 volatile uint8_t g_tick_1s = 0;                       // 1-second flag - set by ISR when 100 * 10ms elapsed
                                                        // Volatile ensures main loop always reads latest value set by interrupt handler
 
 void __interrupt() isr(void) {
     // Interrupt Service Routine - handles all interrupt sources (Timer1 overflow in this application)
     // __interrupt() is compiler-specific keyword - generates proper ISR prologue/epilogue (context save/restore)
     if (PIR1bits.TMR1IF) {                            // Check Timer1 interrupt flag - indicates Timer1 overflow/period expired
                                                        // Important: always check interrupt source flags in shared ISR (multiple sources possible)
         // Reload Timer1 with preload value for next 10ms period
         // Preload prevents timer from counting from 0, ensuring exact timing without cumulative error
         TMR1H = 0xF6;                                  // High byte of preload value (0xF63C = 63036 decimal)
         TMR1L = 0x3C;                                  // Low byte of preload value - write low byte last to prevent intermediate overflow
                                                        // Timer1 continues counting from preload value, overflows at 65536 (0x10000)
         PIR1bits.TMR1IF = 0;                           // Clear interrupt flag - required to allow future interrupts
                                                        // Flag must be cleared manually (doesn't auto-clear like some PICs)

         // Update tick counters - maintain timebase for main loop
         g_tick_10ms++;                                 // Increment 10ms counter - tracks number of 10ms periods elapsed
         if (g_tick_10ms >= 100) {                     // Check if 100 * 10ms = 1 second has elapsed
             g_tick_10ms = 0;                           // Reset 10ms counter - wrap around for next second
             g_tick_1s = 1;                             // Set 1-second flag - signals main loop that 1 second has passed
                                                        // Main loop polls this flag to implement time-based state machine
         }                                              // Flag is set to 1, main loop clears it after processing
     }                                                  // If other interrupts exist, add additional if() checks here
 }
 
 static void Timer1_Init_10ms(void) {
     // Timer1 internal clock, prescaler 1:8
     // Timer1 is 16-bit timer (0-65535) with configurable prescaler - can generate long periods with high clock frequency
     T1CONbits.TMR1CS = 0;    // Fosc/4                  // Timer1 clock source: use internal instruction clock (Fosc/4)
                                                        // Fosc/4 = 8MHz/4 = 2MHz - instruction cycle clock (one instruction = 4 clock cycles)
     T1CONbits.T1CKPS1 = 1;                             // Prescaler bit 1: prescaler = 1:8 (both bits set)
     T1CONbits.T1CKPS0 = 1;   // 1:8                    // Prescaler bit 0: prescaler = 1:8 divides timer clock by 8
                                                        // Effective timer clock = (Fosc/4) / 8 = 2MHz / 8 = 250kHz (period = 4µs)
     
     // Set initial timer value for 10ms period
     // Timer counts from preload to 65536: count = 65536 - 63036 = 2500
     // Period = 2500 * 4µs = 10,000µs = 10ms (perfect timing)
     TMR1H = 0xF6;                                      // High byte of preload (63036 = 0xF63C) - timer starts counting from here
     TMR1L = 0x3C;                                      // Low byte of preload - write high byte first, low byte last (prevents rollover)
                                                        // When timer reaches 65536, overflow occurs and interrupt fires
     
     PIR1bits.TMR1IF = 0;                               // Clear Timer1 interrupt flag - ensure no spurious interrupt on startup
     PIE1bits.TMR1IE = 1;                               // Enable Timer1 interrupt - allows ISR to be called when TMR1IF is set
     INTCONbits.PEIE = 1;                               // Enable peripheral interrupts - required for Timer1 interrupt (Timer1 is peripheral)
     INTCONbits.GIE = 1;                                // Enable global interrupts - master interrupt enable (must be on for any ISR to run)
                                                        // GIE enables interrupt system - without this, no interrupts execute regardless of flags
     
     T1CONbits.TMR1ON = 1;                              // Start Timer1 - timer begins counting, interrupt will fire every 10ms
                                                        // Timer runs continuously, generating periodic interrupts for timebase
 }
 
 // ---------------------------------------------------------------------------
 // Traffic logic + UI
 // RED/GREEN only (no yellow). RED can be extended when density is high.
 // ---------------------------------------------------------------------------
 // State machine implements adaptive traffic light control - adjusts timing based on vehicle density
 typedef enum { PH_RED = 0, PH_GREEN = 1 } Phase;      // Traffic light phase enumeration - finite state machine has 2 states
                                                        // PH_RED: vehicles stopped, PH_GREEN: vehicles allowed to proceed
 
 // LED mapping (Explorer 8: D1=RD0, D2=RD1)
 // Physical LED indicators on Explorer 8 board - visual feedback of traffic light state
 #define LED_RED_LAT   LATDbits.LATD0                   // Red LED output latch - writing 1 turns LED on (active-high)
 #define LED_GRN_LAT   LATDbits.LATD1                   // Green LED output latch - writing 1 turns LED on (active-high)
                                                        // LAT register ensures write occurs even if pin is being read simultaneously
 
 static void set_phase(Phase ph) {
     // Updates physical LED outputs to reflect current traffic light phase - only one LED on at a time
     if (ph == PH_RED) {                                // Check if phase is RED - vehicles must stop
         LED_RED_LAT = 1;                               // Turn on red LED - visual indication of stop state
         LED_GRN_LAT = 0;                               // Turn off green LED - ensure only red is visible (mutually exclusive states)
     } else {                                           // Phase must be GREEN - vehicles can proceed
         LED_RED_LAT = 0;                               // Turn off red LED - clear stop indication
         LED_GRN_LAT = 1;                               // Turn on green LED - visual indication of go state
     }                                                  // Safety: ensures only one LED is active (prevents ambiguous states)
 }
 
 // Globals updated by UART (vehicle density 0..99)
 // Vehicle count from Python YOLO detection - updated asynchronously via UART communication
 volatile uint8_t vehicle_count = 0;                   // Volatile required: modified in parse_line_and_update (called from ISR context indirectly)
                                                        // Range 0-99 represents detected vehicle density (used for adaptive timing algorithm)
 
 static void lcd_print_2d(uint8_t v) {
     // Displays 2-digit decimal number on LCD - converts binary value to ASCII representation
     LCD_Char('0' + (v / 10));                         // Calculate tens digit: divide by 10, add to '0' ASCII code to get digit character
                                                        // Example: v=17 → 17/10=1 → '0'+1='1' (tens digit)
     LCD_Char('0' + (v % 10));                         // Calculate ones digit: modulo 10, add to '0' ASCII code
                                                        // Example: v=17 → 17%10=7 → '0'+7='7' (ones digit)
                                                        // Result: displays "17" on LCD
 }
 
 static void lcd_print_phase(Phase ph) {
     // Displays traffic light phase name on LCD - converts enum value to human-readable string
     if (ph == PH_RED) {                                // Check if phase is RED state
         LCD_String("RED");                             // Display "RED" string - 3 characters indicating stop phase
     } else {                                           // Phase must be GREEN state
         LCD_String("GRN");                             // Display "GRN" string - 3 characters indicating go phase (abbreviated for space)
     }                                                  // LCD displays current traffic light state for user visibility
 }
 
 static void lcd_fill_spaces(uint8_t n) {
     // Fills LCD with spaces - clears remainder of line after text output
     while (n--) LCD_Char(' ');                        // Write ' ' character n times - fills unused LCD positions with spaces
                                                        // Prevents old characters from remaining visible after shorter text updates
 }
 
 static void lcd_draw(Phase ph, uint8_t remaining, uint8_t cars, uint8_t ped, uint8_t ext_on) {
     // Updates entire LCD display with current traffic light status - 2-line format showing all relevant information
     // Line1: PH:RED T:12
     LCD_SetCursor(0, 0);                               // Position cursor at line 1, column 0 - start of first display line
     LCD_String("PH:");                                 // Display "PH:" label - indicates Phase information follows
     lcd_print_phase(ph);                               // Display phase name ("RED" or "GRN") - current traffic light state
     LCD_String(" T:");                                 // Display " T:" label - indicates Time remaining information
     lcd_print_2d(remaining);                           // Display remaining time as 2-digit number - seconds until next phase change
     lcd_fill_spaces(16 - 3 - 3 - 3 - 2); // rough pad  // Fill remaining positions with spaces - "PH:"(3) + phase(3) + " T:"(3) + time(2) = 11, pad to 16
 
     // Line2: CAR:07 PED:1 EXT
     LCD_SetCursor(1, 0);                               // Position cursor at line 2, column 0 - start of second display line
     LCD_String("CAR:");                                // Display "CAR:" label - indicates vehicle count information
     lcd_print_2d(cars);                                // Display vehicle count as 2-digit number - detected vehicles from YOLO system
     LCD_String(" PED:");                               // Display " PED:" label - indicates pedestrian button status
     LCD_Char(ped ? '1' : '0');                         // Display pedestrian status: '1' if button active, '0' if inactive (ternary operator)
     LCD_String(" ");                                   // Space separator between fields - improves readability
     LCD_String(ext_on ? "EXT" : "   ");                // Display extension status: "EXT" if red phase extended, spaces otherwise
                                                        // Ternary operator: condition ? value_if_true : value_if_false
     lcd_fill_spaces(16 - 4 - 2 - 5 - 1 - 3); // pad  // Fill remaining positions: "CAR:"(4) + count(2) + " PED:"(5) + status(1) + "EXT"(3) = 15, pad to 16
 }
 
 // UART line parser: expects "C:<n>" where n is 00..99
 // Parses vehicle density data from Python script - simple protocol: "C:" followed by 2-digit number
 static void parse_line_and_update(const char* line) {
     // Protocol validation: line must start with "C:" to be recognized as vehicle count command
     if (line[0] != 'C' || line[1] != ':') return;     // Check first two characters - reject if not "C:" format (invalid protocol)
                                                        // Early return: exits function immediately if format doesn't match expected pattern
     int val = 0;                                       // Initialize accumulator for parsed decimal value
     if (line[2] >= '0' && line[2] <= '9') {           // Check if third character is valid digit (ASCII range '0'-'9')
         val = line[2] - '0';                          // Convert ASCII digit to integer: '0'→0, '1'→1, ..., '9'→9 (ASCII arithmetic)
     } else return;                                     // Invalid tens digit - reject entire message (strict validation)
     if (line[3] >= '0' && line[3] <= '9') {           // Check if fourth character is valid digit (ones place)
         val = val * 10 + (line[3] - '0');             // Build 2-digit number: tens*10 + ones (e.g., "17" → 1*10 + 7 = 17)
     }                                                  // If ones digit missing, val remains single digit (e.g., "C:7" → val=7)
     if (val < 0) val = 0;                             // Clamp minimum to 0 - safety check (shouldn't happen with valid input)
     if (val > 99) val = 99;                           // Clamp maximum to 99 - ensures value fits in uint8_t range (0-255, but protocol limits to 99)
     vehicle_count = (uint8_t)val;                     // Update global vehicle count - cast int to uint8_t, store for traffic logic
                                                        // Volatile qualifier ensures compiler doesn't optimize away reads in main loop
 
     
 }
 
 int main(void) {
     // Main function - program entry point, initializes hardware and runs traffic light control loop
     OSCCON = 0x72;  // 8 MHz                      // Oscillator Control Register: configure internal oscillator to 8 MHz
                                                     // Bits: IRCF=111 (8MHz), SCS=00 (FOSC selected), SPLLEN=0 (no PLL)
                                                     // Internal oscillator provides stable clock without external crystal
     
     // All digital
     // Disable analog functions on all ports - ensures all pins function as digital I/O (required for UART, SPI, GPIO)
     ANSELA = 0;                                     // Port A analog select register - 0 = all pins digital (no ADC channels)
     ANSELB = 0;                                     // Port B analog select register - 0 = all pins digital (button is digital input)
     ANSELC = 0;                                     // Port C analog select register - 0 = all pins digital (UART, SPI are digital)
     ANSELD = 0;                                     // Port D analog select register - 0 = all pins digital (LEDs are digital outputs)
     ANSELE = 0;                                     // Port E analog select register - 0 = all pins digital (if port exists)
     
     // LEDs on PORTD (D1/D2)
     // Configure LED pins as outputs and initialize to off state - prevents LED flash during startup
     TRISDbits.TRISD0 = 0;                           // Configure RD0 as output - red LED control pin (TRIS=0 means output direction)
     TRISDbits.TRISD1 = 0;                           // Configure RD1 as output - green LED control pin (TRIS=0 means output direction)
     LED_RED_LAT = 0;                                // Initialize red LED to off - ensure clean startup state (LED off)
     LED_GRN_LAT = 0;                                // Initialize green LED to off - both LEDs off until state machine starts
 
     // Button S1 on RB0 (active-low)
     // Configure pedestrian button as input with internal pull-up - button press pulls pin low (active-low logic)
     TRISBbits.TRISB0 = 1;                           // Configure RB0 as input - button pin reads external switch state (TRIS=1 means input)
                                                     // Internal weak pull-up (if enabled) keeps pin high when button not pressed
 
     // Init peripherals (UART -> Timer1 -> SPI/LCD)
     // Initialize in dependency order: UART first (for early debug), then timing, then display hardware
     UART_Init();                                    // Initialize UART - enables communication with Python vehicle detection script
     Timer1_Init_10ms();                             // Initialize Timer1 - starts periodic interrupts for 1-second timebase
     SPI_Init();                                     // Initialize SPI - configures communication with MCP23S17 I/O expander
     MCP_Init();                                     // Initialize MCP23S17 - resets and configures I/O expander for LCD interface
     LCD_Init();                                     // Initialize LCD - sends HD44780 initialization sequence, clears display
                                                     // All hardware now ready for normal operation
 
     UART_WriteStr("PIC READY\n");                   // Send startup message via UART - confirms PIC has booted successfully
                                                     // Python script can detect this message to know when PIC is operational
 
     // Traffic constants (tune here)
     // Algorithm parameters - adjust these values to modify traffic light timing behavior
     const uint8_t GREEN_BASE = 10;                  // Base green phase duration in seconds - minimum time vehicles can proceed
     const uint8_t RED_BASE = 10;                    // Base red phase duration in seconds - minimum time vehicles must wait
     const uint8_t RED_MAX = 30;                     // Maximum red phase duration in seconds - prevents indefinite red periods
     const uint8_t EXT_STEP = 3;      // seconds to add per decision  // Increment added to red phase when extending (adaptive algorithm)
     const uint8_t THRESHOLD = 20;    // cars threshold to extend red // Vehicle count threshold - red extends if count >= this value
     const uint8_t PED_HOLD = 5;      // seconds forced red on button press  // Minimum red duration when pedestrian button pressed
 
     // Traffic light state machine variables - track current phase and timing
     Phase phase = PH_GREEN;                         // Initial phase: start with green (vehicles can proceed initially)
     uint8_t remaining = GREEN_BASE;                 // Time remaining in current phase - counts down from base duration
     uint8_t red_extended_total = 0;                 // Total time red phase has been extended - tracking for display purposes
 
     // Pedestrian button state variables - handle button press events
     uint8_t ped_active = 0;                         // Flag: 1 if pedestrian request is active, 0 otherwise (state machine state)
     uint8_t ped_remaining = 0;                      // Time remaining for pedestrian hold - counts down from PED_HOLD
 
     // UART line buffer (small, line-based)
     // Buffer for receiving UART messages - accumulates characters until newline, then parses complete line
     char line[16];                                  // Line buffer array - holds one line of UART data (e.g., "C:07\n")
     uint8_t li = 0;                                 // Line index - current position in buffer (where next character will be stored)
 
     // Initialize display and LEDs to show initial state
     set_phase(phase);                               // Set physical LEDs to match initial phase (green LED on, red LED off)
     lcd_draw(phase, remaining, vehicle_count, 0, 0);  // Update LCD with initial state - shows phase, time, vehicle count (0 initially)
 
     while (1) {                                     // Main loop - runs forever, implements traffic light control state machine
         // Non-blocking UART receive + line parsing
         // Poll UART for received characters - accumulates into line buffer until complete line received
         while (UART_Available()) {                  // Check if UART has received data - loop processes all available bytes
             char c = (char)UART_Read();             // Read one character from UART - removes byte from receive buffer
             if (c == '\r') continue;                // Ignore carriage return (CR) - some systems send CR+LF, we only process LF
             if (c == '\n') {                        // Newline character detected - indicates end of line, ready to parse
                 line[li] = '\0';                    // Null-terminate string - standard C string terminator for parsing functions
                 parse_line_and_update(line);        // Parse complete line - extracts vehicle count and updates global variable
                 li = 0;                             // Reset line index - buffer ready for next line (circular buffer behavior)
             } else if (li < (sizeof(line) - 1)) {   // Check buffer space available - prevents buffer overflow (reserve space for null terminator)
                 line[li++] = c;                     // Store character in buffer and increment index - accumulate line characters
             } else {                                // Buffer overflow condition - line too long, reject entire message
                 li = 0; // overflow -> reset         // Reset index - discard corrupted line, wait for next valid line
             }                                       // Buffer overflow protection prevents memory corruption from malicious/long messages
         }
 
         // Check if 1 second has elapsed - Timer1 ISR sets g_tick_1s flag every second
         if (!g_tick_1s) continue;                   // Skip rest of loop if 1 second hasn't passed - efficient polling (exits early)
         g_tick_1s = 0;                              // Clear 1-second flag - acknowledge time tick, prepare for next second
                                                     // Flag-based polling: main loop checks flag, ISR sets it (interrupt-driven timing)
 
         // Debounced pedestrian (simple): trigger when pressed
         // Detect button press with edge detection - triggers on falling edge (high-to-low transition)
         static uint8_t last_btn = 1;                // Previous button state - static variable preserves value between loop iterations
         uint8_t btn = PORTBbits.RB0;                // Read current button state - PORT register reads actual pin level
         uint8_t ped_req = 0;                        // Pedestrian request flag - set to 1 when valid button press detected
         if (last_btn == 1 && btn == 0) {            // Detect falling edge: button was high, now low (active-low button press)
             delay_ms(10);                           // Debounce delay - wait 10ms to filter out switch contact bounce (mechanical noise)
             if (PORTBbits.RB0 == 0) ped_req = 1;    // Verify button still low after delay - confirms real press, not bounce
         }                                           // Edge detection: only triggers on transition, not continuous low state
         last_btn = btn;                             // Save current state for next iteration - enables edge detection on next cycle
 
         // Process pedestrian request - force red phase when button pressed
         if (ped_req && !ped_active) {               // Check if new pedestrian request and no active pedestrian hold
             // Force RED immediately for pedestrian request
             ped_active = 1;                         // Set pedestrian active flag - prevents multiple button presses from resetting timer
             ped_remaining = PED_HOLD;               // Initialize pedestrian hold timer - pedestrian gets minimum PED_HOLD seconds
             phase = PH_RED;                         // Switch to red phase immediately - vehicles must stop for pedestrian
             remaining = (remaining < PED_HOLD) ? PED_HOLD : remaining;  // Ensure remaining time >= PED_HOLD (extend if needed)
                                                     // Ternary operator: if remaining < PED_HOLD, use PED_HOLD; else keep remaining
             set_phase(phase);                       // Update physical LEDs - red LED on, green LED off (immediate visual feedback)
         }
 
         // Tick down
         // Decrement all timers every second - implements countdown for phase transitions
         if (remaining > 0) remaining--;             // Decrement phase timer - counts down to 0, triggers phase transition when zero
         if (ped_active && ped_remaining > 0) ped_remaining--;  // Decrement pedestrian timer if active - counts down pedestrian hold duration
         if (ped_active && ped_remaining == 0) {     // Check if pedestrian hold completed - timer reached zero
             ped_active = 0;                         // Clear pedestrian active flag - pedestrian request satisfied, normal operation resumes
         }                                           // Pedestrian state machine: button → active → countdown → inactive
 
         // Adaptive: extend RED only while in RED and above threshold
         // Adaptive algorithm: extend red phase if vehicle density is high (prevents traffic congestion)
         uint8_t ext_on = 0;                         // Extension active flag - indicates if red phase is currently being extended
         if (phase == PH_RED && vehicle_count >= THRESHOLD && remaining < RED_MAX) {  // Conditions: red phase, high density, not at max
             uint8_t add = EXT_STEP;                 // Amount to add this cycle - adds EXT_STEP seconds per second when extending
             while (add-- && remaining < RED_MAX) {  // Add seconds up to EXT_STEP or until RED_MAX reached (loop adds multiple seconds)
                 remaining++;                        // Increment remaining time - extends red phase duration (adaptive behavior)
                 red_extended_total++;               // Track total extension - statistics for display/logging purposes
             }                                       // While loop: adds up to EXT_STEP seconds, but stops if RED_MAX would be exceeded
             ext_on = 1;                             // Set extension flag - indicates red phase is actively extended this cycle
         }                                           // Adaptive algorithm: high vehicle count → longer red phase → prevents traffic buildup
 
         // Phase transitions when remaining time reaches 0
         // State machine transitions: green ↔ red based on timer expiration
         if (remaining == 0) {                       // Check if phase timer expired - triggers state transition
             if (phase == PH_GREEN) {                // Transitioning from green to red - vehicles must now stop
                 phase = PH_RED;                     // Change to red phase - update state machine state
                 remaining = RED_BASE;               // Reset timer to base red duration - vehicles wait for RED_BASE seconds
                 red_extended_total = 0;             // Reset extension counter - start fresh for new red phase cycle
             } else {                                // Transitioning from red to green - vehicles can now proceed
                 phase = PH_GREEN;                   // Change to green phase - update state machine state
                 remaining = GREEN_BASE;             // Reset timer to base green duration - vehicles proceed for GREEN_BASE seconds
             }                                       // Binary state machine: only two states, deterministic transitions
             set_phase(phase);                       // Update physical LEDs - reflects new phase immediately (red or green LED)
         }                                           // Timer-driven state machine: time expiration triggers deterministic state changes
 
         // Update LCD display with current state - refreshes display every second with latest information
         lcd_draw(phase, remaining, vehicle_count, ped_active, (ext_on || red_extended_total > 0));  // Draw complete display
                                                     // Parameters: phase, time remaining, vehicle count, pedestrian status, extension status
                                                     // Extension display: shows "EXT" if currently extended OR was extended this cycle
     }                                               // Infinite loop - program runs continuously, implementing real-time traffic control
 }