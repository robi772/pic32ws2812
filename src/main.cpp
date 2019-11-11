#include <Arduino.h>
#include <USB.h>
#include <stdint.h>

USBFS usbDriver;
USBManager USB(usbDriver, 0x0403, 0xA662);
CDCACM uSerial1;

//#define SYS_FREQ             40000000
//#define GetSystemClock()    SYS_FREQ
//#define GetPeripheralClock()    (GetSystemClock() / (1 << OSCCONbits.PBDIV))

#define LED_BUILTIN 48

#define LEDCOUNT 3 // Number of leds inside each pixel
#define BITBYTES LEDCOUNT*8  // How many bytes doe we need to DMA : 1 byte per bit  (24byte for RGB, 32 for WRGB)
#define HALFBITBYTES BITBYTES/2
#define PIXELCOUNT 1024 // Number of complete LED units in the string
/*
#define LO_RS 32 // OCxRS for a 0 bit : 300ns
#define HI_RS 70 // OCxRS for a 1 bit : 600ns
#define PERIOD 280 // OCx Timer period : 1250ns

11hz
*/

#define LO_RS 32 // OCxRS for a 0 bit : 300ns
#define HI_RS 65 // OCxRS for a 1 bit : 600ns
#define PERIOD 100 // OCx Timer period : 1250ns

typedef union {
  struct {
  //  uint8_t white;  // Option : Only for WRGB
    uint8_t blue;
    uint8_t red;
    uint8_t green;
  };
  uint32_t rawdata;
  uint8_t rawbytes[3]; // 3 for RGB, 4 for WRGB
} pixel_t;

pixel_t leds[PIXELCOUNT+1] = {
		{0x40, 0x00, 0x00}
	, {0x00, 0x40, 0x00}
	, {0x00, 0x40, 0x20}
	, {0x10, 0x20, 0x40}
	, {0x00, 0x00, 0x40}
	, {0x00, 0X40, 0x20}
	, {0x00, 0x20, 0x10}
	, {0x00, 0x20, 0X40}
	, {0x40, 0x00, 0x20}
	, {0x10, 0x40, 0x20}

};
// For animating the values
int8_t ddr[PIXELCOUNT][LEDCOUNT];
uint8_t maxintens[LEDCOUNT] = {
	  0b00001111
	, 0b01111111
	, 0b01111111

};
uint8_t maxspeed[LEDCOUNT] = {
	  0b00000001
	, 0b00000111
	, 0b00000111

};


unsigned int sourceAddr;
unsigned int destinationAddr;

// a byte per led bytes plus one to end it
uint8_t junkdata[BITBYTES];
uint16_t ledctr = 0;

extern "C" void __attribute__((interrupt(),nomips16)) DMA1Handler(void);


void setup() {
  // put your setup code here, to run once:
  // initialize LED digital pin as an output.

  sourceAddr = (unsigned int) &junkdata & 0x1FFFFFFF;
	destinationAddr = (unsigned int) &OC4RS & 0x1FFFFFFF;


  ///// Initialize dma first
	DMACON = 0x8000;            // dma module on
	DCRCCON = 0;                // crc module off

	DCH1INT = 0;                // interrupts disabled
	// DCH1INTbits.CHBCIE = 1;  // Int at end of transfer
	DCH1INTbits.CHSDIE = 1;  // Int at source done
	DCH1INTbits.CHSHIE = 1;  // Int at source half empty

	DCH1SSA = sourceAddr;       // source start address
	DCH1DSA = destinationAddr;  // destination start address

	DCH1SSIZ = BITBYTES;               // source size
	DCH1DSIZ = 1;               // destination size - 1 byte
	DCH1CSIZ = 1;
	             // cell size - 1 bytes
	DCH1ECONbits.CHSIRQ = _TIMER_3_IRQ;  // Cell transfer IRQ
	DCH1ECONbits.SIRQEN = 1;  // Transfer a cell when CHSIRQ occurs

	///// Enable dma channel
	DCH1CONbits.CHEN = 0;
	DCH1CONbits.CHAEN = 1;  // Auto enable
	DCH1CONbits.CHPRI = 3;

	//mapPps(18, PPS_OUT_OC4);
	// Setup T3 to run flat out with period of 256
	T3CONbits.TCKPS = 0; // prescale
	TMR3 = 0;
	PR3 = PERIOD;

	//	Setup OC for free running PWM
	OC4CON = OCCON_ON | OCCON_SRC_TIMER3 | OCCON_PWM_FAULT_DISABLE;
	OC4R = 0;
	OC4RS = 0;

  setIntVector(_DMA_1_VECTOR, DMA1Handler);
  clearIntFlag(_DMA1_IRQ);
  setIntPriority(_DMA_1_VECTOR, _CN_IPL_IPC, _CN_SPL_IPC);
  setIntEnable(_DMA1_IRQ);

	uint8_t idx = 0;
	uint8_t jdx = 0;

	// Intro blink to let me know things are working
  TRISDCLR = BIT_2;
/*
	LATDCLR = BIT_2;
	 delay(250);
	 idx = 100;
	 while(idx--){
	 	LATDINV = BIT_2;
	 	delay(25);
  }*/
	T3CONbits.ON = 1;

	LATBCLR = BIT_1;
	// Preload LEDs with random(ish) data
	// Grab value of TMR3
  /*
	 for(idx=0; idx<PIXELCOUNT; idx++){
	 	LATBINV = BIT_1;
	 	for(jdx=0; jdx<LEDCOUNT; jdx++){
	 		// Set direction
	 		ddr[idx][jdx] = ((TMR3 & maxspeed[jdx]) + 1);
	 		// set value (restricted to half intensity)
	 		leds[idx].rawbytes[jdx] = (uint8_t)TMR3 & maxintens[jdx];
    //  leds[idx].rawbytes[jdx] = 0x39;
	 		delay(11);
	 	}
	 	delay(17);
	 }
   */
	// preload LEDs with zeros
/*
	for(idx=0; idx<PIXELCOUNT; idx++){
		LATBINV = BIT_1;
		for(jdx=0; jdx<LEDCOUNT; jdx++){
			leds[idx].rawbytes[jdx] = 0;
			delay(3);
		}
		delay(7);
	}
*/
  TRISBCLR = BIT_0;
  TRISBCLR = BIT_1;
  TRISBCLR = BIT_2;
	LATBCLR = BIT_1;
	LATBCLR = BIT_0;


	delay(100);

	DCH1CONbits.CHEN = 1;
/*
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i=0;i<80;i++){
      pinMode(i, OUTPUT);
  }*/
//Serial.begin(115200);
  // USB.addDevice(uSerial1);
      //USB.begin();


}
uint16_t tdx = 0;
uint16_t jdx = 0;
void loop() {

  // put your main code here, to run repeatedly:
  // turn the LED on (HIGH is the voltage level)
//  Serial.print('.');
  //   uSerial1.println("u");
     /*
  digitalWrite(LED_BUILTIN, HIGH);
  for(int i=0;i<80;i++){
      digitalWrite(i, HIGH);
  }
*/

  // wait for a second
  //delay(10);
  // turn the LED off by making the voltage LOW
/*  digitalWrite(LED_BUILTIN, LOW);
  for(int i=0;i<80;i++){
      digitalWrite(i, LOW);
  }
*/
   // wait for a second
  //delay(10);
  for(tdx=0; tdx<PIXELCOUNT; tdx++){

     leds[tdx].rawbytes[0] = 0x00; //B
     leds[tdx].rawbytes[1] = 0x00; //R
     leds[tdx].rawbytes[2] = 0x01; //G

  }

  //leds[jdx].rawbytes[0] = 0b1111; //G
  //jdx++;
 //if (jdx>PIXELCOUNT) jdx=0;
delay(100);

for(tdx=0; tdx<PIXELCOUNT; tdx++){

   leds[tdx].rawbytes[0] = 0x00; //B
   leds[tdx].rawbytes[1] = 0x01; //R
   leds[tdx].rawbytes[2] = 0x00; //G

}
delay(100);

for(tdx=0; tdx<PIXELCOUNT; tdx++){

   leds[tdx].rawbytes[0] = 0x01; //B
   leds[tdx].rawbytes[1] = 0x00; //R
   leds[tdx].rawbytes[2] = 0x00; //G

}
delay(100);
/*
  for(idx=0; idx<PIXELCOUNT; idx++){
   LATBINV = BIT_1;
   for(jdx=0; jdx<LEDCOUNT; jdx++){
     // Set direction
     ddr[idx][jdx] = ((TMR3 & maxspeed[jdx]) + 1);
     // set value (restricted to half intensity)
     leds[idx].rawbytes[jdx] = (uint8_t)TMR3 & maxintens[jdx];
   //  leds[idx].rawbytes[jdx] = 0x39;
     delay(11);
   }
   delay(17);
  }
  */
}

void __attribute__((interrupt(),nomips16)) DMA1Handler(void){
	uint16_t idx, nextled;
	uint32_t val;


  LATBSET = BIT_1;
	LATBCLR = BIT_2;
	if(ledctr < PIXELCOUNT){
		if(DCH1INTbits.CHSHIF){
			LATBCLR = BIT_0;
			// Source half
			// Fill bits 0 - 11
	//		val = leds[ledctr].rawdata << 8;  // For RGB we need to ignore the first 8 bits
			val = leds[ledctr].rawdata;
			for(idx=0; idx < HALFBITBYTES; idx++){  // Half number of bits
				LATBINV = BIT_0;
				// Test highest bit
				if(val & 0x800000){
					// a one
					junkdata[idx] = HI_RS;
				}else{
					// a zero
					junkdata[idx] = LO_RS;
				}
				val <<= 1;
			}
		}
		if(DCH1INTbits.CHSDIF){
			// Source done
			// Fill bits 12 - 23
			// Next LED
		//	val = leds[ledctr].rawdata << 20;  // For RGB we need to shift
			val = leds[ledctr].rawdata << 12;
			for(idx=HALFBITBYTES; idx < BITBYTES; idx++){ // Half number of bits
				LATBINV = BIT_2;
  //      LATDINV = BIT_2;
				// Test highest bit
				if(val & 0x800000){
					// a one
					junkdata[idx] = HI_RS;
				}else{
					// a zero
					junkdata[idx] = LO_RS;
				}
				val <<= 1;
			}
			ledctr++;
		}
	}else{
		if(DCH1INTbits.CHSDIF){
			// OC4RS = 0;
			// OC4R = 0;
			// RESET
			// Send some empty LEDS to trigger reset
      LATDINV = BIT_2;
			for(idx = 0; idx < BITBYTES+1; idx++){
				junkdata[idx] = 0;
			}
			if(ledctr>PIXELCOUNT+7){
				ledctr = 0;
			}else{
				ledctr++;
			}
		}
	}

	DCH1INTCLR = 0x000000ff; // Clear DMA int flags (bottom 8 bits)
	DCH1CONbits.CHEN = 1;	// Re-enable dma

	// DCH1INTbits.CHBCIE = 1;  // Int at end of transfer
	LATBCLR = BIT_2;
	LATBCLR = BIT_1;
	clearIntFlag(_DMA1_IRQ);
}
