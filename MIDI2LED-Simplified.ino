#include <MIDI.h>
#include <FastLED.h>

// Number of LEDs in the strip
#define NUM_OF_ROWS 64
// Number of Rows in the grid
#define SIZE_OF_ROW 8
#define NUM_LEDS NUM_OF_ROWS*SIZE_OF_ROW
#define LED 17    //PIN of debug LED
#define DATA_PIN 2  //Data pin of the individually addressable LED                                

#define ON_BIT_MASK 0x80
#define ON    1
#define OFF   0

// Globals for thread processes
static const BaseType_t pro_cpu = 0; // Assign pro_cpu to core 0
static const BaseType_t app_cpu = 1; // Assign app_cpu to core 1
static TaskHandle_t setFrameTask = NULL;
static QueueHandle_t midi_queue; 
#define MIDI_QUEUE_LEN 20 // Number of slots in byte queue
static SemaphoreHandle_t mutex; // Mutex for the byte queue

//Delay in milliseconds
static const uint32_t showFrameDelay = 100; 
static const uint32_t setFrameDelay = 50; 

//Create a MIDI object on the second serial bus
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, midi2);

//Structure to hold LED coordinates
typedef struct coord{
  uint8_t x;
  uint8_t y;
}coord;


// Define the array of leds
CRGB leds[NUM_LEDS];
uint8_t firstLedRow[NUM_OF_ROWS];
uint8_t ledRowsQueue[NUM_OF_ROWS];

// Use a byte as a queue to determine if LED is on or off
uint8_t enqueueBit(uint8_t byteQueue, uint8_t bit) {
  // Value must a boolean
  byteQueue <<= 1;
  byteQueue |= bit; //Enqueue bit
  return byteQueue;
}

//TODO: Make MIDI2LED task spawn tasks using this code example from chatGPT
// // Define a struct to hold parameters for TaskB
// typedef struct {
//     int id;
//     char message[50];
// } TaskBParams;

// void TaskB(void *pvParameters)
// {
//     TaskBParams *params = (TaskBParams *)pvParameters;

//     while (1)
//     {
//         printf("TaskB received: id = %d, message = %s\n", params->id, params->message);
//         vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 second
//     }
// }

// void TaskA(void *pvParameters)
// {
//     TaskBParams *params = pvPortMalloc(sizeof(TaskBParams)); // use FreeRTOS heap
//     if (params == NULL) {
//         printf("Failed to allocate memory for TaskBParams.\n");
//         vTaskDelete(NULL);
//     }

//     // Fill in the parameter structure
//     params->id = 42;
//     snprintf(params->message, sizeof(params->message), "Hello from TaskA!");

//     // Create TaskB and pass the parameters
//     if (xTaskCreate(TaskB, "TaskB", configMINIMAL_STACK_SIZE + 100, params, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
//         printf("Failed to create TaskB.\n");
//         vPortFree(params);  // Clean up if task creation fails
//     }

//     // TaskA can end here
//     vTaskDelete(NULL);
// }

// Show the rendered frame on the LED display
void showFrame(void *parameters){
  
  while(1){
    // Update the LEDs on the screen 
    FastLED.show();
    if (xSemaphoreTake(mutex, 0) == pdTRUE) {
      for(int row = 0; row < NUM_OF_ROWS; row++){
        //Enqueue the first led row into the queue
        ledRowsQueue[row] = enqueueBit(ledRowsQueue[row], firstLedRow[row]);
        int bitMask = 1;
        for(int col = 0; col < SIZE_OF_ROW; col++){
          // Create a coord from row and column value
          coord point{row, col};
          int index = MatrixToLedString(point, SIZE_OF_ROW);
          // Set led to red if bit is 1 and set led to black if bit is 0
          if((bitMask & ledRowsQueue[row]) == bitMask){
            Serial.println(row);
            leds[index] = CRGB::Red;
          }else{
            leds[index] = CRGB::Black;
          }
          bitMask = bitMask<<1;
        }

      }
        xSemaphoreGive(mutex);
    }
    // Delay the task
    vTaskDelay(showFrameDelay / portTICK_PERIOD_MS);
  }
   
}

// Render the frame 
// TODO: Remove all mutex's and semaphores since there should be no multithreading
void setFrame(void *parameters){
  while(1){
    int pitchToIndex;//Take the pitch from MIDI and convert it to the index on the LED matrix
    coord point; //XY coords of LED
    // Take the semaphore before putting messages in the queue
    if (xSemaphoreTake(mutex, 0) == pdTRUE) {
      for(int i = 0; i < uxQueueMessagesWaiting( midi_queue ); i++){
          if (xQueueReceive(midi_queue , (void *)&pitchToIndex, 10) == pdTRUE) {
            //Set led to read if an on note is sent
            //Serial.println(pitchToIndex);
            // Determine if the integer is on or off
            if((pitchToIndex & ON_BIT_MASK) == ON_BIT_MASK){
              firstLedRow[(pitchToIndex & (~ON_BIT_MASK))] = ON;//remove "ON" bit component
            }else{
              firstLedRow[pitchToIndex] = OFF;
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
  // Clear all the LEDs to black 
  for( uint8_t i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Black;
  }
  // Create a mutex for the message queue
  mutex = xSemaphoreCreateMutex();
  // Create a queue of length of the number of rows
  midi_queue  = xQueueCreate(MIDI_QUEUE_LEN, sizeof(uint8_t));
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
  midi2.begin(MIDI_CHANNEL_OMNI);  // Listen to all incoming messages
  midi2.setHandleNoteOn(MyHandleNoteOn); // This is important!! This command
    // tells the Midi Library which function you want to call when a NOTE ON command
    // is received. In this case it's "MyHandleNoteOn".
  midi2.setHandleNoteOff(MyHandleNoteOff); // This command tells the Midi Library 
  pinMode (LED, OUTPUT); // Set Arduino board pin 13 to output

  //TODO: Set all tasks to the CPU not used by the midi controller
  //Pin show Frame task to CPU 0 
  xTaskCreatePinnedToCore(
                        showFrame,   /* Task function. */
                        "Show Frame",     /* name of task. */
                        10000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        1,           /* priority of the task */
                        NULL,      /* Task handle to keep track of created task */
                        pro_cpu);          /* pin task to core 0 */
  //Pin Set Frame task to CPU 1
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
    // Read incoming MIDI messages
    midi2.read();
}

//ISR for when a note on event is detected
void MyHandleNoteOn(byte channel, byte pitch, byte velocity) { 
  //digitalWrite(LED,HIGH);  //Turn debug LED on
 // Serial.println(pitch);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Convert Pitch to index in array
  uint8_t pitchToIndex = pitch - 36;
  // Add on bit so there is a difference between on and off messages
  pitchToIndex = pitchToIndex | ON_BIT_MASK;
  // Serial.println(pitchToIndex);
  // Only enqueue value if queue isn't full
  if (xQueueSendFromISR(midi_queue , (void *)&pitchToIndex, &xHigherPriorityTaskWoken) != pdTRUE) {
    Serial.println("Queue Full");
  }
 
}
//ISR for when a note off event is detected
void MyHandleNoteOff(byte channel, byte pitch, byte velocity) { 
  //digitalWrite(LED,LOW);  //Turn debug LED off
  //Serial.println(pitch);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint8_t pitchToIndex = pitch - 36;
  if (xQueueSendFromISR(midi_queue, (void *)&pitchToIndex, &xHigherPriorityTaskWoken) != pdTRUE) {
    Serial.println("Queue Full");
  }
}

/***********************************************************
Functions to convert coordinates to an index on an LED matrix
The LED matrix is structured in a snaking pattern as shown below
0 | 15 | 16 |... 
1 | 14 | 17 |...
. | .  | .  |...
7 | 8  | 23 |...
************************************************************/
// Converts a coordinate to the index on a LED matrix 
int MatrixToLedString(coord coord, int length){

  uint8_t index;
  index = length * coord.x;
  if((coord.x % 2) == 0){
    index += coord.y;
  }else{ 
    index = (length -1) - coord.y + index;
  }
  return index;

}

// Converts a LED index on a strip to a coordinate
void LedStripToMatrix(int index, coord *coord ){
  if((index/8)%2 == 0){
    coord->y = index%8;
  }else{
    coord->y = 7-(index%8);
  }
  coord->x = index/8;
}
