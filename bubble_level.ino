//An ESP32S3 dev board and ICM-20948 accelerometer for a digital bubble level.

//TODO  min/max text added, single click to show/clear
//Serial command to zero offset and clear min/max
//60 degree huge bubble range.  real level ±3° or so

//Digital low pass filter enabled in ICM20948 configuration, extra
//1 euro filter responsive but still flutters once sensor at rest
//second sloppy averaging filter applied to reduce jitter at rest
//when two readings are within ±0.?°

TaskHandle_t lcdTask;

#include <SPI.h>

#include <TFT_eSPI.h>       // Hardware-specific library

#include "ICM_20948.h" 

#include "Button2.h"

#include "1euroFilter.h"

static OneEuroFilter fx;
static OneEuroFilter fy; 
static OneEuroFilter fz; 

static OneEuroFilter bx; 
static OneEuroFilter by; 

//Filter constants
#define FREQUENCY   60   // [Hz] set by timestamps passed to filter
#define MINCUTOFF   1.0   // [Hz] needs to be tuned according to your application
#define BETA        0.1   // needs to be tuned according to your application


#define ALPHA       0.01

#define SERIAL_PORT Serial
#define WIRE_PORT Wire 
#define AD0_VAL 1

ICM_20948_I2C myICM;

Button2 button;

TFT_eSPI tft = TFT_eSPI(); 
TFT_eSprite spr = TFT_eSprite(&tft);  //full screen frame buffer
TFT_eSprite spr_txt = TFT_eSprite(&tft);  //sprite to rotate y bubble text  

QueueHandle_t BubbleQueue;

int average_ctr=0;
double x_sum=0.0;
double y_sum=0.0;
double z_sum=0.0;
double x_average=0.0;
double y_average=0.0;
double z_average=0.0;
double scaledX=0.0;
double scaledY=0.0;
double scaledZ=0.0;
double filteredX=0.0;
double filteredY=0.0;
double filteredZ=0.0;
double x_offset=0.0;
double y_offset=0.0;
double angle_x=0.0;
double angle_y=0.0;
double fangle_x=0.0;
double fangle_y=0.0;
const double p = 3.1415926;
char inputChar=0;

struct bubble_t
{
double x = 0.0;
double y = 0.0;
};

bubble_t bubble;

//calibration constants
#define X_SPAN 0.998725393
#define Y_SPAN 0.997384104
#define Z_SPAN 0.991422437

#define X_ZERO -1.463111228
#define Y_ZERO 18.4064966
#define Z_ZERO 17.21508361

//Screen constants
#define GaugeHalf 50
#define xBubble_centerX 100
#define xBubble_centerY 85

#define yBubble_centerX 250
#define yBubble_centerY 85

#define ARRAY_SIZE 8


void init_ICM()  {
  bool initialized = false;
  while (!initialized)
  {

    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  
  // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
  SERIAL_PORT.println("Device connected!");

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("Software Reset returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }
  delay(250);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setSampleMode returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }
  
 ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm2; 
  myFSS.g = dps250; 

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setFullScale returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

// Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d5bw7_n8bw3; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                  // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)

  myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setDLPcfg returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, false);
  SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: "));
  SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
  SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: "));
  SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));

  SERIAL_PORT.println();
  SERIAL_PORT.println(F("ICM Init complete!"));

}


void setup(void) {
  SERIAL_PORT.begin(115200);

  //bubble queue has only 1 item, always draw the most recent data placed in queue
  BubbleQueue = xQueueCreate( 1, sizeof( bubble_t ) );

  //Create a task to handle the LCD updating seperate from sensor reading
  xTaskCreatePinnedToCore(lcdLoop, "LCD", 10000,  NULL, 0,  &lcdTask,  0); 

  //i2c setup on pins 4 and 5
  WIRE_PORT.begin(4,5);
  WIRE_PORT.setClock(400000);

  init_ICM();

  //init button2 library
  button.begin(0);
  button.setLongClickTime(1000);
  button.setClickHandler(click);
  button.setLongClickHandler(longClick);

  fx.begin(FREQUENCY, MINCUTOFF, BETA);
  fy.begin(FREQUENCY, MINCUTOFF, BETA);
  fz.begin(FREQUENCY, MINCUTOFF, BETA);
  bx.begin(FREQUENCY, MINCUTOFF, BETA);
  by.begin(FREQUENCY, MINCUTOFF, BETA);

  delay(50);

}

void loop() {
  
  button.loop();
  
  if (myICM.dataReady())
  {
    myICM.getAGMT();   
    
    unsigned long t=millis();
    filteredX = fx.filter(myICM.accX(),t) ;
    filteredY = fy.filter(myICM.accY(),t) ;
    filteredZ = fz.filter(myICM.accZ(),t) ;               

    average_ctr++;
    x_sum += filteredX;
    y_sum += filteredY;
    z_sum += filteredZ;
  }

  if(average_ctr == ARRAY_SIZE) {
    x_average=x_sum / average_ctr;
    y_average=y_sum / average_ctr;
    z_average=z_sum / average_ctr;
    
    average_ctr=0;
    x_sum =0;
    y_sum =0;
    z_sum =0;

    scaledX = X_SPAN * (x_average) + X_ZERO;
    scaledY = Y_SPAN * (y_average) + Y_ZERO;
    scaledZ = Z_SPAN * (z_average) + Z_ZERO;

    angle_x = atan2(scaledX,scaledZ)*180.0/p;
    angle_y = atan2(scaledY,scaledZ)*180.0/p;

    // unsigned long m = millis();
    // fangle_x=  bx.filter(angle_x,m);
    // fangle_y = by.filter(angle_y,m);

    fangle_x = (angle_x * ALPHA) + (1.0-ALPHA) * fangle_x;
    fangle_y = (angle_y * ALPHA) + (1.0-ALPHA) * fangle_y;

    if(abs(angle_x-fangle_x) < 0.6)
      bubble.x = (fangle_x - x_offset);
    else
      bubble.x = (angle_x - x_offset);

    if(abs(angle_y-fangle_y)< 0.63)
      bubble.y = (fangle_y - y_offset);
    else
      bubble.y = (angle_y - y_offset);

    //SERIAL_PORT.printf("%f,%f,%f,%f\n", angle_x,angle_y, fangle_x, fangle_y);    

    xQueueOverwrite(BubbleQueue, &bubble);

  }

  if (Serial.available())
    inputChar = Serial.read();
  if(inputChar == 'X')
    SERIAL_PORT.printf("%f,%f\n", bubble.x,bubble.y);
  if(inputChar == 'Z')
    longClick(button);    

  inputChar=0;

}

void click(Button2& btn) {
    Serial.println("click\n");
}

void longClick(Button2& btn) {
    Serial.println("Zero-ed offset\n");
    x_offset=fangle_x;
    y_offset=fangle_y;    
    Serial.println(x_offset);
    Serial.println(y_offset);
}

//scale angle to get more bubble movement near center
//60 is end of dial 
//the fit curve is only valid for 0 to 60
double scaleDial(double angle) {
  double pos=0.0;
  if (angle > 0) pos = angle; else pos = -angle;
  double val = 0.000557*pos*pos*pos - 0.069008*pos*pos + 3.141351*pos - 0.013721;
  if(angle < 0) val = -val;
  if(val > 60) return 60.0;
  if(val < -60) return -60.0;
  return val; 
}

//Endless task on the other core to update LCD screen
void lcdLoop(void * parameter) {

  bubble_t lcdBubble;

  double bubble_x = 0.0;
  double bubble_y = 0.0;
  
  tft.init();
  tft.fillScreen(TFT_BLACK);
  
  tft.setRotation(1);  //Landscape 320, 170, buttons on right hand side

  // Optionally set colour depth to 8 or 16 bits, default is 16 if not specified
  // spr.setColorDepth(8);

  // Create a sprite of defined size
  spr.createSprite(320, 170);  //full size frame buffer

  // Set "cursor" at top left corner of display (0,0) and select font 4
  tft.setCursor(0, 4, 4);
  spr.setCursor(0, 4, 4);

  //sprite to rotate text for y_bubble
  spr_txt.createSprite(170,50);

  // Set the font colour to be white with a black background
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);

  // We can now plot text on screen using the "print" class
  tft.drawString(" Bubble Level v0.6", 150,85);

  delay(3000);
 
  while(1) {

    //Get bubble data from queue
       if( xQueueReceive( BubbleQueue, &( lcdBubble ), ( TickType_t ) 10 ) )
       {
          bubble_x = scaleDial(lcdBubble.x);
          bubble_y = scaleDial(lcdBubble.y);

          spr.fillSprite(TFT_BLACK);

          //print the real values not the scaled ones for the dial
          spr.setTextDatum(MC_DATUM);
          spr.drawFloat(lcdBubble.x ,3, xBubble_centerX, xBubble_centerY+50,4);

          spr.setPivot(yBubble_centerX+50,yBubble_centerY);
          spr_txt.setPivot(85,25);

          spr_txt.fillSprite(TFT_BLACK);
          spr_txt.setTextDatum(MC_DATUM);

          spr_txt.drawFloat(lcdBubble.y,3,85,25,4);
          spr_txt.pushRotated(&spr, -90);
        
          spr.drawWideLine(xBubble_centerX-GaugeHalf, xBubble_centerY, 
                          xBubble_centerX+GaugeHalf, xBubble_centerY,
                          50, TFT_GREEN, TFT_BLACK); 

          spr.drawWideLine(yBubble_centerX, yBubble_centerY-GaugeHalf, 
                          yBubble_centerX, yBubble_centerY+GaugeHalf,
                          50, TFT_GREEN, TFT_BLACK);

          spr.fillSmoothCircle(xBubble_centerX+bubble_x, xBubble_centerY, 15,TFT_YELLOW, TFT_GREEN);  

          spr.fillSmoothCircle(yBubble_centerX, yBubble_centerY-bubble_y, 15, TFT_YELLOW, TFT_GREEN);  

          spr.drawSmoothCircle(xBubble_centerX, xBubble_centerY, 30,TFT_BLACK, TFT_BLACK);  

          spr.drawSmoothCircle(yBubble_centerX, yBubble_centerY, 30, TFT_BLACK, TFT_BLACK);    

          spr.drawSmoothCircle(xBubble_centerX, xBubble_centerY, 19,TFT_BLACK, TFT_GREEN);  

          spr.drawSmoothCircle(yBubble_centerX, yBubble_centerY, 19, TFT_BLACK, TFT_GREEN);            

          spr.pushSprite(0,0); //draw sprite buffer to tft  
      
       }    

       yield(); //run the other core0 tasks to avoid wdt barking
    
  }

}
