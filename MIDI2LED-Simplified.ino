#include <MIDI.h>
#include <FastLED.h>


// How many leds in your strip?
#define NUM_OF_ROWS 64
#define SIZE_OF_ROW 8
#define NUM_LEDS NUM_OF_ROWS*SIZE_OF_ROW
#define LED 17
#define DATA_PIN 2

#define ON_BIT_MASK 0x80



// Globals
static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;
static TaskHandle_t setFrameTask = NULL;
//static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t msg_queue; 
enum { MSG_QUEUE_LEN = 20 }; // Number of slots in message queue
static SemaphoreHandle_t mutex;

//Delays
static const uint32_t showFrameDelay = 100; 
static const uint32_t setFrameDelay = 50; 

MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, midi2);

typedef struct coord{
  int x;
  int y;
}coord;


// Define the array of leds
CRGB leds[NUM_LEDS];
int firstLedRow[NUM_OF_ROWS];
int ledRowsQueue[NUM_OF_ROWS];


int enqueueBit(int byteQueue, int thing) {
  // Value must be 1 or zero
  if (thing == 1) {
    byteQueue = byteQueue << 1;
    byteQueue = byteQueue | thing;
  } else {
    byteQueue = byteQueue << 1;
  }
  return byteQueue;
}

void showFrame(void *parameters){
  
  while(1){

    FastLED.show();
    if (xSemaphoreTake(mutex, 0) == pdTRUE) {
      for(int row = 0; row < NUM_OF_ROWS; row++){
        //Enqueue the first led row into the queue
        ledRowsQueue[row] = enqueueBit(ledRowsQueue[row], firstLedRow[row]);
        int bitMask = 1;
        for(int i = 0; i < SIZE_OF_ROW; i++){
          //Set led to red if bit is 1 and set led to black if bit is 0
          if((bitMask & ledRowsQueue[row]) == bitMask){
            coord point{row, i};
            Serial.println(row);
            int index = MatrixToLedString(point, SIZE_OF_ROW);
            leds[index] = CRGB::Red;
          }else{
            coord point{row, i};
            int index = MatrixToLedString(point, SIZE_OF_ROW);
            leds[index] = CRGB::Black;
          }
          bitMask = bitMask<<1;
        }

      }
        xSemaphoreGive(mutex);
    }
      
    vTaskDelay(showFrameDelay / portTICK_PERIOD_MS);
  }
   
}

void setFrame(void *parameters){
  while(1){
    int pitchToIndex;
    coord point;
    if (xSemaphoreTake(mutex, 0) == pdTRUE) {
      for(int i = 0; i < uxQueueMessagesWaiting( msg_queue); i++){
          if (xQueueReceive(msg_queue, (void *)&pitchToIndex, 10) == pdTRUE) {
            //Set led to red if an on note is sent
            //Serial.println(pitchToIndex);
            if((pitchToIndex & ON_BIT_MASK) == ON_BIT_MASK){
              firstLedRow[(pitchToIndex & (~ON_BIT_MASK))] = 1;//remove "ON" bit component
            }else{
              firstLedRow[pitchToIndex] = 0;
            }   
          }   
      }
      xSemaphoreGive(mutex);
    } 
    vTaskDelay(setFrameDelay / portTICK_PERIOD_MS);
  }
}
   

void setup()
{
  for( int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Black;
  }
  mutex = xSemaphoreCreateMutex();
  msg_queue = xQueueCreate(MSG_QUEUE_LEN, sizeof(int));
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
  midi2.begin(MIDI_CHANNEL_OMNI);  // Listen to all incoming messages
  midi2.setHandleNoteOn(MyHandleNoteOn); // This is important!! This command
    // tells the Midi Library which function you want to call when a NOTE ON command
    // is received. In this case it's "MyHandleNoteOn".
  midi2.setHandleNoteOff(MyHandleNoteOff); // This command tells the Midi Library 
  pinMode (LED, OUTPUT); // Set Arduino board pin 13 to output
  xTaskCreatePinnedToCore(
                        showFrame,   /* Task function. */
                        "Show Frame",     /* name of task. */
                        10000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        1,           /* priority of the task */
                        NULL,      /* Task handle to keep track of created task */
                        pro_cpu);          /* pin task to core 0 */
  xTaskCreatePinnedToCore(
                        setFrame,   /* Task function. */
                        "Set Frame",     /* name of task. */
                        10000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        2,           /* priority of the task */
                        &setFrameTask,      /* Task handle to keep track of created task */
                        pro_cpu);       /* pin task to core 0 */
}

void loop()
{
    // Read incoming messages
    midi2.read();
}


void MyHandleNoteOn(byte channel, byte pitch, byte velocity) { 
  //digitalWrite(LED,HIGH);  //Turn LED on
 // Serial.println(pitch);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  int pitchToIndex = pitch - 36;
  //add on bit so there is a difference between on and off messages
  pitchToIndex = pitchToIndex | ON_BIT_MASK;
  //Serial.println(pitchToIndex);
  if (xQueueSendFromISR(msg_queue, (void *)&pitchToIndex, &xHigherPriorityTaskWoken) != pdTRUE) {
    Serial.println("Queue Full");
  }
 
}
void MyHandleNoteOff(byte channel, byte pitch, byte velocity) { 
  //digitalWrite(LED,LOW);  //Turn LED off
  //Serial.println(pitch);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  byte pitchToIndex = pitch - 36;
  if (xQueueSendFromISR(msg_queue, (void *)&pitchToIndex, &xHigherPriorityTaskWoken) != pdTRUE) {
    Serial.println("Queue Full");
  }
}

int MatrixToLedString(coord coord, int length){

  int index;
  index = length * coord.x;
  if((coord.x % 2) == 0){
    index += coord.y;
  }else{ 
    index = (length -1) - coord.y + index;
  }
  return index;

}

void LedStripToMatrix(int index, coord *coord ){
  if((index/8)%2 == 0){
    coord->y = index%8;
  }else{
    coord->y = 7-(index%8);
  }
  coord->x = index/8;
}
