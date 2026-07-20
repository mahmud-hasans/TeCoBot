//ESP32 S3 Transmitter 
// Written by Sydney Spiegel
// Modified by Mahmud Hasan Saikot

#include <esp_now.h> 
#include <WiFi.h> 

// Global Variables 
#define MAX_INPUT_LENGTH 20 
// Define the wait time for demo
// #define WAIT_TIME 2500
#define WAIT_TIME 1800

// Helper macros to convert the numeric value to a string literal
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

char inputBuffer[MAX_INPUT_LENGTH]; // Buffer to store incoming characters 
int bufferIndex = 0;               // Current position in the buffer 
bool stringComplete = false;

// new for handling multiple input & demo
bool demoMode = false; // Flag to track demo execution
int prewrittenIndex = 0;  // Index for prewritten commands
int activeDemo = 0;       // 1 = demo1, 2 = demo2, 3 = demo3

// Define demo sequences
const char *demo1Commands[] = {
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",   
    "E", "wait 1000",
    "E", "wait 1000",   
    "a", "wait " STR(WAIT_TIME), 
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "A", "wait " STR(WAIT_TIME),
    "e", "wait 1000",
    "e", "wait 1000",
    "e", "wait 1000",
    "e", "wait 1000",
    "e", "wait 1000",
    "e", "wait 1000",
    "e", "wait 1000",
    "e", "wait 1000",
    "a", "wait " STR(WAIT_TIME),
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "E", "wait 1000",
    "A", "wait " STR(WAIT_TIME),
    "e", "wait 1000",
    "e", "wait 1000",
    "e", "wait 1000",
    "e", "wait 1000",
    "e", "wait 1000",
    "e", "wait 1000",
    "e", "wait 1000",
    "e", "wait 1000",
    "stop"  // Final command (no wait after)
}; 

const char *demo2Commands[] = { 
    // heavy push
    "W", "wait 500",
    "q", "wait 2500",
    "E", "wait 300",
    "w", "wait 600",
    // "stop", "wait 100",
    "Q", "wait 2500",
    "e", "wait 500",
    // one cycle
    "W", "wait 900",
    "q", "wait 2000",
    "E", "wait 300",
    "w", "wait 600",
    // "stop", "wait 100",
    "Q", "wait 2000",
    "e", "wait 500",
    // one cycle
    "W", "wait 900",
    "q", "wait 2000",
    "E", "wait 300",
    "w", "wait 600",
    // "stop", "wait 100",
    "Q", "wait 2000",
    "e", "wait 500",
    // one cycle (slower claw)
    // "W", "wait 9000",
    // "q", "wait 2000",
    // "E", "wait 3000",
    // "w", "wait 6000",
    // // "stop", "wait 100",
    // "Q", "wait 3000",
    // //"e", "wait 2000",

    // heavy pull
    // "E", "wait 5000",
    // "q", "wait 3000",
    // "W", "wait 3000",
    // "e", "wait 6000",
    // // "stop", "wait 100",
    // "Q", "wait 4000",
    // "w", "wait 3000",
    // // one cycle
    // "E", "wait 6000",
    // "q", "wait 3000",
    // "W", "wait 3000",
    // "e", "wait 6000",
    // // "stop", "wait 100",
    // "Q", "wait 4000",
    // "w", "wait 3000",
    // // one cycle
    // "E", "wait 6000",
    // "q", "wait 3000",
    // "W", "wait 3000",
    // "e", "wait 6000",
    // // "stop", "wait 100",
    // "Q", "wait 4000",
    // "w", "wait 3000",
    // one cycle
    // "E", "wait 6000",
    // "q", "wait 3000",
    // "W", "wait 3000",
    // "e", "wait 6000",
    // // "stop", "wait 100",
    // "Q", "wait 4000",
    //"w", "wait 3000",
};

const char *demo3Commands[] = {
  // roll opposite
    "s", "wait " STR(WAIT_TIME),
    "d", "wait " STR(WAIT_TIME),
    "S", "wait " STR(WAIT_TIME),
    "a", "wait " STR(WAIT_TIME),
    "D", "wait " STR(WAIT_TIME),
    "s", "wait " STR(WAIT_TIME),
    "A", "wait " STR(WAIT_TIME),
    "d", "wait " STR(WAIT_TIME),
    "S", "wait " STR(WAIT_TIME),
    "a", "wait " STR(WAIT_TIME),
    "D", "wait " STR(WAIT_TIME),
    "s", "wait " STR(WAIT_TIME),
    "A", "wait " STR(WAIT_TIME),
    "d", "wait " STR(WAIT_TIME),
    "S", "wait " STR(WAIT_TIME),
    "a", "wait " STR(WAIT_TIME),
    "D", "wait " STR(WAIT_TIME),
    "s", "wait " STR(WAIT_TIME),
    "A", "wait " STR(WAIT_TIME),
    "d", "wait " STR(WAIT_TIME),
    "S", "wait " STR(WAIT_TIME),
    "a", "wait " STR(WAIT_TIME),
    "D", "wait " STR(WAIT_TIME),
    "s", "wait " STR(WAIT_TIME),
    "A", "wait " STR(WAIT_TIME),
    "d", "wait " STR(WAIT_TIME),
    "S", "wait " STR(WAIT_TIME),
    "a", "wait " STR(WAIT_TIME),
    "D", "wait " STR(WAIT_TIME),
    "s", "wait " STR(WAIT_TIME),
    "A", "wait " STR(WAIT_TIME),

    // roll one direction
    // "s", "wait " STR(WAIT_TIME),
    // "a", "wait " STR(WAIT_TIME),
    // "S", "wait " STR(WAIT_TIME),
    // "d", "wait " STR(WAIT_TIME),
    // "A", "wait " STR(WAIT_TIME),
    // "s", "wait " STR(WAIT_TIME),
    // "D", "wait " STR(WAIT_TIME),
    // "a", "wait " STR(WAIT_TIME),
    // "S", "wait " STR(WAIT_TIME),
    // "d", "wait " STR(WAIT_TIME),
    // "A", "wait " STR(WAIT_TIME),
    // "s", "wait " STR(WAIT_TIME),
    // "D", "wait " STR(WAIT_TIME),
    // "a", "wait " STR(WAIT_TIME),
    // "S", "wait " STR(WAIT_TIME),
    // "d", "wait " STR(WAIT_TIME),
    // "A", "wait " STR(WAIT_TIME),
    // "s", "wait " STR(WAIT_TIME),
    // // more
    // "D", "wait " STR(WAIT_TIME),
    // "a", "wait " STR(WAIT_TIME),
    // "S", "wait " STR(WAIT_TIME),
    // "d", "wait " STR(WAIT_TIME),
    // "A", "wait " STR(WAIT_TIME),
    // "s", "wait " STR(WAIT_TIME),
    // //more
    // "D", "wait " STR(WAIT_TIME),
    // "a", "wait " STR(WAIT_TIME),
    // "S", "wait " STR(WAIT_TIME),
    // "d", "wait " STR(WAIT_TIME),
    // "A", "wait " STR(WAIT_TIME),
    // "s", "wait " STR(WAIT_TIME),
    // "Q", "wait " STR(WAIT_TIME),
    "stop"
};

// const char *demo3Commands[] = { 
//     "s", "wait 1000",
//     "a", "wait 1000",
//     "S", "wait 1000",
//     "d", "wait 1000",
//     "A", "wait 1000",
//     "s", "wait 1000",
//     "D", "wait 1000",
//     "a", "wait 1000",
//     "S", "wait 1000",
//     "d", "wait 1000",
//     "A", "wait 1000",
//     "s", "wait 1000",
//     "D", "wait 1000",
//     "a", "wait 500",
//     "S", "wait 500",
//     "d", "wait 500",
//     "A", "wait 500",
//     "s", "wait 500",
//     "D", "wait 500",
//     "a", "wait 500",
//     "S", "wait 500",
//     "d", "wait 500",
//     "A", "wait 500",
//     "s", "wait 500",
//     "D", "wait 500",
//     "a", "wait 500",
//     "S", "wait 500",
//     "d", "wait 500",
//     "A", "wait 500",
//     "s", "wait 500",
//     "D", "wait 500",
//     "a", "wait 500",
//     "S", "wait 500",
//     "d", "wait 500",
//     "A", "wait 500",
//     "s", "wait 500",
//     "D", "wait 500",
//     "a", "wait 500",
//     "S", "wait 500",
//     "d", "wait 500",
//     "A", "wait 500",
//     "s", "wait 500",
//     "stop" 
// };

const int numDemo1Commands = sizeof(demo1Commands) / sizeof(demo1Commands[0]);
const int numDemo2Commands = sizeof(demo2Commands) / sizeof(demo2Commands[0]);
const int numDemo3Commands = sizeof(demo3Commands) / sizeof(demo3Commands[0]);

const char **currentDemoCommands = nullptr;
int numCurrentDemoCommands = 0;

// Modified readSerialInput(): 
// It reads all available characters into inputBuffer. 
// When no more data is available (i.e. after pressing send), the buffer is null-terminated and flagged as complete.
void readSerialInput() { 
    while (Serial.available() > 0) { 
        char incomingChar = Serial.read(); 
        if (bufferIndex < MAX_INPUT_LENGTH - 1) { 
            inputBuffer[bufferIndex++] = incomingChar; 
        }
    }
    // If we have data and no more characters are waiting, mark the input as complete.
    if (bufferIndex > 0 && Serial.available() == 0) {
         inputBuffer[bufferIndex] = '\0';
         stringComplete = true;
    }
}

// -- (The rest of your unchanged code remains below) --
//R2 = 64:E8:33:70:1B:A8
//R4 = 74:4D:BD:AA:BE:3C

// Updated MAC Addresses for ESP32 S3 Boards
uint8_t broadcastAddress1[] = { 0x74, 0x4D, 0xBD, 0xA9, 0xEC, 0x90}; 
uint8_t broadcastAddress2[] = { 0x64, 0xE8, 0x33, 0x70, 0x1B, 0xA8 }; 
uint8_t broadcastAddress3[] = { 0x74, 0x4D, 0xBD, 0xAA, 0xBD, 0xE4 }; 
uint8_t broadcastAddress4[] = { 0x74, 0x4D, 0xBD, 0xAA, 0xBE, 0x3C }; 
uint8_t broadcastAddress5[] = { 0x84, 0xFC, 0xE6, 0x7B, 0xA7, 0xE4 }; 

// Structure for robot commands
typedef struct robotCommands_struct { 
    bool connected;       // true or false 
    int robotNumber;      // 1,2,3,4, etc.
    int manualInput;      // for manual control 
    int move;             // pre programmed movements 
    int iterations;       // number of iterations for a gait sequence.
    int gait;             // gait sequence type 
} robotCommands_struct; 

robotCommands_struct commands_1; 
robotCommands_struct commands_2; 
robotCommands_struct commands_3; 
robotCommands_struct commands_4;
robotCommands_struct commands_5;

// Global flag to indicate command changes 
bool commandChanged = false; 

esp_now_peer_info_t peerInfo; 

// Callback for ESP-NOW data sent status
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { 
    Serial.print("\r\nLast Packet Send Status:\t"); 
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail"); 
} 

// Robot connection flags 
bool robot_1_connected = false; 
bool robot_2_connected = false; 
bool robot_3_connected = false; 
bool robot_4_connected = false;
bool robot_5_connected = false;

void setup(){ 
    Serial.begin(115200); 
    WiFi.mode(WIFI_STA); 

    if (esp_now_init() != ESP_OK) { 
        Serial.println("Error initializing ESP-NOW"); 
        return; 
    } 

    esp_now_register_send_cb(OnDataSent); 

    // Register peers
    peerInfo.channel = 0; 
    peerInfo.encrypt = false; 
    memcpy(peerInfo.peer_addr, broadcastAddress1, 6); 
    if (esp_now_add_peer(&peerInfo) != ESP_OK) { 
        Serial.println("Failed to add peer 1"); 
        return; 
    } 
    memcpy(peerInfo.peer_addr, broadcastAddress2, 6); 
    if (esp_now_add_peer(&peerInfo) != ESP_OK) { 
        Serial.println("Failed to add peer 2"); 
        return; 
    } 
    memcpy(peerInfo.peer_addr, broadcastAddress3, 6); 
    if (esp_now_add_peer(&peerInfo) != ESP_OK) { 
        Serial.println("Failed to add peer 3"); 
        return; 
    } 
    memcpy(peerInfo.peer_addr, broadcastAddress4, 6); 
    if (esp_now_add_peer(&peerInfo) != ESP_OK) { 
        Serial.println("Failed to add peer 4"); 
        return; 
    }
    memcpy(peerInfo.peer_addr, broadcastAddress5, 6); 
    if (esp_now_add_peer(&peerInfo) != ESP_OK) { 
        Serial.println("Failed to add peer 5"); 
        return; 
    }  

    // Initialize iterations
    commands_1.iterations = 1; 
    commands_2.iterations = 1; 
    commands_3.iterations = 1; 
    commands_4.iterations = 1;
    commands_5.iterations = 1; 

    Serial.println("Ready.");
}

void connectRobots() { 
    if (strcmp(inputBuffer, "1") == 0) {
        robot_1_connected = !robot_1_connected; 
        commands_1.connected = robot_1_connected; 
        commandChanged = true; 
        Serial.print("robot_1 "); Serial.println(robot_1_connected ? "connected" : "disconnected"); 
    } 
    else if (strcmp(inputBuffer, "2") == 0) { 
        robot_2_connected = !robot_2_connected; 
        commands_2.connected = robot_2_connected; 
        commandChanged = true; 
        Serial.print("robot_2 "); Serial.println(robot_2_connected ? "connected" : "disconnected"); 
    } 
    else if (strcmp(inputBuffer, "3") == 0) { 
        robot_3_connected = !robot_3_connected; 
        commands_3.connected = robot_3_connected; 
        commandChanged = true; 
        Serial.print("robot_3 "); Serial.println(robot_3_connected ? "connected" : "disconnected"); 
    } 
    else if (strcmp(inputBuffer, "4") == 0) { 
        robot_4_connected = !robot_4_connected; 
        commands_4.connected = robot_4_connected; 
        commandChanged = true; 
        Serial.print("robot_4 "); Serial.println(robot_4_connected ? "connected" : "disconnected"); 
    }
    else if (strcmp(inputBuffer, "5") == 0) { 
        robot_5_connected = !robot_5_connected; 
        commands_5.connected = robot_5_connected; 
        commandChanged = true; 
        Serial.print("robot_5 "); Serial.println(robot_5_connected ? "connected" : "disconnected"); 
    }
}

void handleSingleInput(char command) { 
    bool localCommandChanged = false;

    switch (command) {
        case 'a': 
            commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 5; 
            Serial.println("Motor 1 Forward"); 
            localCommandChanged = true;
            break;
        case 'A': 
            commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 6; 
            Serial.println("Motor 1 Reverse"); 
            localCommandChanged = true;
            break;
        case 's':  
            commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 7; 
            Serial.println("Motor 2 Forward"); 
            localCommandChanged = true;
            break;
        case 'S': 
            commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 8; 
            Serial.println("Motor 2 Reverse"); 
            localCommandChanged = true;
            break;
        case 'd': 
            commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 9; 
            Serial.println("Motor 3 Forward"); 
            localCommandChanged = true;
            break;
        case 'D': 
            commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 10; 
            Serial.println("Motor 3 Reverse"); 
            localCommandChanged = true;
            break;
        case 'q': 
            commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 14; 
            Serial.println("Fully Collapse"); 
            localCommandChanged = true;
            break;
        case 'Q': 
            commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 15; 
            Serial.println("Zero Command"); 
            localCommandChanged = true;
            break;
        case 'w': 
            commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 1; 
            Serial.println("Upper Claw Open"); 
            localCommandChanged = true;
            break;
        case 'W': 
            commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 2; 
            Serial.println("Upper Claw Close"); 
            localCommandChanged = true;
            break;
        case 'e': 
            commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 3; 
            Serial.println("Lower Claw Open"); 
            localCommandChanged = true;
            break;
        case 'E': 
            commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 4; 
            Serial.println("Lower Claw Close"); 
            localCommandChanged = true;
            break;
        default:
            return;
    }
    
    if (localCommandChanged) {
        esp_now_send(broadcastAddress1, (uint8_t *)&commands_1, sizeof(commands_1)); 
        esp_now_send(broadcastAddress2, (uint8_t *)&commands_2, sizeof(commands_2)); 
        esp_now_send(broadcastAddress3, (uint8_t *)&commands_3, sizeof(commands_3)); 
        esp_now_send(broadcastAddress4, (uint8_t *)&commands_4, sizeof(commands_4));
        esp_now_send(broadcastAddress5, (uint8_t *)&commands_5, sizeof(commands_5));
        Serial.println("Command Sent");
    }
}

void executeDemo() {
    Serial.print("Starting Demo ");
    Serial.println(activeDemo);
    demoMode = true;
    prewrittenIndex = 0;

    while (demoMode && prewrittenIndex < numCurrentDemoCommands) {
        const char *command = currentDemoCommands[prewrittenIndex++];

        if (strncmp(command, "wait", 4) == 0) {
            int waitTime = atoi(command + 5);
            Serial.print("Waiting for ");
            Serial.print(waitTime);
            Serial.println(" ms...");
            delay(waitTime);
            continue;
        }

        strncpy(inputBuffer, command, MAX_INPUT_LENGTH);
        stringComplete = true;
        Serial.print("Executing command: ");
        Serial.println(inputBuffer);
        processInputBuffer();

        sendCommands();

        if (strncmp(inputBuffer, "end", 4) == 0) {
            Serial.println("Demo Mode Stopped.");
            demoMode = false;
            break;
        }
    }
}

void processInputBuffer() { 
    if (strcmp(inputBuffer, "x") == 0) { 
        commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 11; 
        Serial.println("Small Displacement");
        commandChanged = true;
    }
    else if (strcmp(inputBuffer, "m") == 0) { 
        commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 12; 
        Serial.println("Medium Displacement");
        commandChanged = true;
    }
    else if (strcmp(inputBuffer, "l") == 0) { 
        commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 13; 
        Serial.println("Large Displacement");
        commandChanged = true;
    }
    else if (strcmp(inputBuffer, "i") == 0) { 
        commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 19; 
        Serial.println("Super Large Claw Time");
        commandChanged = true;
    }
    else if (strcmp(inputBuffer, "r") == 0) { 
        commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 18; 
        Serial.println("Huge Displacement");
        commandChanged = true;
    }
    else if (strcmp(inputBuffer, "demo1") == 0) { 
        activeDemo = 1;
        currentDemoCommands = demo1Commands;
        numCurrentDemoCommands = numDemo1Commands;
        executeDemo();
    } 
    else if (strcmp(inputBuffer, "demo2") == 0) { 
        activeDemo = 2;
        currentDemoCommands = demo2Commands;
        numCurrentDemoCommands = numDemo2Commands;
        executeDemo();
    } 
    else if (strcmp(inputBuffer, "demo3") == 0) { 
        activeDemo = 3;
        currentDemoCommands = demo3Commands;
        numCurrentDemoCommands = numDemo3Commands;
        executeDemo();
    }
    else if (strcmp(inputBuffer, "stop") == 0) { 
        commands_1.manualInput = commands_2.manualInput = commands_3.manualInput = commands_4.manualInput = commands_5.manualInput = 17; 
        Serial.println("STOP!");
        commandChanged = true;
        // demoMode = false;
    }
    else { 
        // Process each character individually if not a full-word command
        for (int i = 0; i < strlen(inputBuffer); i++) { 
            handleSingleInput(inputBuffer[i]);  
        }
    }
}

void sendCommands() { 
    if (commandChanged) { 
        esp_now_send(broadcastAddress1, (uint8_t *)&commands_1, sizeof(commands_1)); 
        esp_now_send(broadcastAddress2, (uint8_t *)&commands_2, sizeof(commands_2)); 
        esp_now_send(broadcastAddress3, (uint8_t *)&commands_3, sizeof(commands_3)); 
        esp_now_send(broadcastAddress4, (uint8_t *)&commands_4, sizeof(commands_4)); 
        esp_now_send(broadcastAddress5, (uint8_t *)&commands_5, sizeof(commands_5)); 
        Serial.println("Commands Sent");
        commandChanged = false; 
    }
}

void loop() { 
    readSerialInput(); 

    if (stringComplete && !demoMode) {
        connectRobots();
        processInputBuffer(); 
        sendCommands(); 
        // Clear the buffer after processing
        memset(inputBuffer, 0, MAX_INPUT_LENGTH); 
        stringComplete = false;
        bufferIndex = 0;
    } 
}
