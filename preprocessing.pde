import processing.serial.*; // Import the serial library

Serial myPort; // Serial port for Arduino communication

int cabinY;             // Position of the elevator cabin
int targetY;            // Target position for the cabin
boolean moving;         // Whether the cabin is moving
int floorHeight = 192;  // Height of each floor
int numFloors = 3;      // Total number of floors
int currentFloor;       // Current floor of the elevator

// Button state variables received from Arduino
boolean groundButton = false;
boolean firstFloorButton = false;
boolean secondFloorButton = false;
boolean stopButton = false;

void setup() {
  size(800, 800);        // Window size
  cabinY = height - 250; // Start at the ground floor
  targetY = cabinY;
  moving = false;
  currentFloor = 0;      // Start on the ground floor

  // Initialize serial communication
  String portName = Serial.list()[0]; // Select the first available port
  myPort = new Serial(this, portName, 9600); // Set baud rate to match Arduino
}

void draw() {
  background(220, 240, 255); // Light blue background

  // Elevator shaft
  fill(60, 60, 60);
  rect(200, 150, 200, height - 195);

  // Floors
  fill(80);
  for (int i = 0; i <= numFloors; i++) {
    rect(200, 150 + i * floorHeight, 200, 30);
  }

  // Elevator cabin
  fill(180, 180, 200); // Light gray
  rect(200, cabinY, 200, floorHeight);
  fill(100, 100, 150); // Slightly darker for interior
  rect(210, cabinY + 10, 180, floorHeight - 20);

  // Simulate cabin movement
  if (moving) {
    if (abs(cabinY - targetY) > 2) { // Smooth transition to target
      cabinY += (targetY - cabinY) / abs(targetY - cabinY) * 2; // Move incrementally
    } else {
      cabinY = targetY; // Snap to target
      moving = false;   // Stop movement
      currentFloor = numFloors - (cabinY - 50) / floorHeight; // Update floor
    }
  }

  // Draw 7-segment displays
  drawSevenSegmentDisplay(500, 200, currentFloor); // Top display
  drawSevenSegmentDisplay(500, 400, currentFloor); // Middle display
  drawSevenSegmentDisplay(500, height - 200, currentFloor); // Bottom display

  // Update target based on button state
  if (groundButton && !moving) {
    targetY = height - 250; // Ground floor position
    moving = true;
    resetButtons();
  } else if (firstFloorButton && !moving) {
    targetY = height - 250 - floorHeight; // First floor position
    moving = true;
    resetButtons();
  } else if (secondFloorButton && !moving) {
    targetY = height - 250 - 2 * floorHeight; // Second floor position
    moving = true;
    resetButtons();
  } else if (stopButton) {
    moving = false; // Stop the elevator
    resetButtons();
  }
}

void drawSevenSegmentDisplay(int x, int y, int number) {
  int segWidth = 10;
  int segHeight = 20;
  int spacing = 5;

  // Draw outer display
  fill(30);
  rect(x, y, segWidth * 3 + spacing * 2, segHeight * 2 + spacing * 4, 6);

  // Draw segments based on number
  fill(255, 0, 0); // Red for lit segments
  boolean[] segments = new boolean[7];
  switch (number) {
    case 0: segments = new boolean[] {true, true, true, false, true, true, true}; break; // Ground Floor
    case 1: segments = new boolean[] {false, true, true, false, false, false, false}; break; // First Floor
    case 2: segments = new boolean[] {true, true, false, true, true, false, true}; break; // Second Floor
  }
  if (segments[0]) rect(x + segWidth + spacing, y, segWidth, spacing); // Top
  if (segments[1]) rect(x + 2 * segWidth + 2 * spacing, y + spacing, spacing, segHeight); // Top-right
  if (segments[2]) rect(x + 2 * segWidth + 2 * spacing, y + 2 * spacing + segHeight, spacing, segHeight); // Bottom-right
  if (segments[3]) rect(x + segWidth + spacing, y + 2 * (spacing + segHeight), segWidth, spacing); // Bottom
  if (segments[4]) rect(x, y + 2 * spacing + segHeight, spacing, segHeight); // Bottom-left
  if (segments[5]) rect(x, y + spacing, spacing, segHeight); // Top-left
  if (segments[6]) rect(x + segWidth + spacing, y + spacing + segHeight, segWidth, spacing); // Middle
}

void resetButtons() {
  groundButton = false;
  firstFloorButton = false;
  secondFloorButton = false;
  stopButton = false;
}

void serialEvent(Serial myPort) {
  String input = myPort.readStringUntil('\n');
  if (input != null) {
    input = input.trim(); // Remove extra whitespace or newline

    // Parse input and update button states
    if (input.equals("GROUND")) {
      groundButton = true;
    } else if (input.equals("FIRST")) {
      firstFloorButton = true;
    } else if (input.equals("SECOND")) {
      secondFloorButton = true;
    } else if (input.equals("STOP")) {
      stopButton = true;
    }
  }
}
