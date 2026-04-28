# LF_gate_project

## 📖 Description
An advanced, smart timing gate system designed for Line Follower and other robotics competitions. Built on the **Raspberry Pi Pico W**, this project ensures highly precise lap time measurements using IR sensor interrupts. It features wireless time synchronization between Start and Finish gates, competitor authentication, and real-time data communication with a central server.

## ✨ Key Features
* **Precise Time Measurement:** Utilizes hardware interrupts via an IR sensor for millisecond accuracy.
* **Wireless Gate Synchronization:** Uses NRF24L01 modules to synchronize the Start and Finish gates and transmit timestamps.
* **Smart Authentication:** 
  * **RFID Scanner (RC522):** Used for competitor and judge authentication.
  * **QR/Barcode Scanner (I2C):** Used to identify and verify specific robots before the run.
* **Live Display:** Outputs dynamic messages and live timers directly to a P10 LED Matrix Display.
* **Wi-Fi Cloud Integration:** Connects to a central server to fetch gate settings, validate users/robots, and submit final run scores using JSON payloads.
* **Physical Control Panel:** Integrated hardware buttons for accepting results, forfeiting/disqualifying, or canceling the run.

## 🛠 Technologies & Dependencies
* **Platform:** Raspberry Pi Pico W
* **Framework:** Arduino (via PlatformIO)
* **Main Libraries:**
  * `RF24` (nRF24L01 communication)
  * `Freenove_RFID_Lib_for_Pico` (RC522 RFID handling)
  * `FirebaseJson` (JSON parsing for server communication)

## 📌 Hardware Pinout
Based on the default configuration in `constants.hpp`:

| Component | Pin (GPIO) | Description |
| :--- | :--- | :--- |
| **SPI (RFID & NRF)** | 2, 3, 4 | SCK, MOSI, MISO |
| **RFID RC522** | 5, 6 | SS (SDA), RST |
| **NRF24L01** | 7, 10, 11 | IRQ, CE, CSN |
| **QR Scanner (I2C)** | 8, 9 | SDA, SCL |
| **P10 Display** | 16, 17, 18, 19, 20, 21 | OE, A, B, CLK, LATCH, DATA |
| **IR Sensor** | 22 | Hardware Interrupt (IRQ) |
| **Buttons** | 26, 14, 15 | Accept, Cancel, Forfeit |
| **LEDs** | 12, 13 | Red, Green |

## 🚀 Installation & Setup
This project is built using **PlatformIO**.

```bash
# Clone the repository
git clone https://github.com/Matt7S/LF_gate_project.git

# Navigate to the project directory
cd LF_gate_project

# Open the project in VS Code with the PlatformIO extension installed.
# Build and upload the code to your Raspberry Pi Pico W:
pio run --target upload
```

## ⚙️ How It Works (State Machine)
The gate operates using a continuous state machine to handle the flow of the competition:
1. **IDLE / USER_AUTHENTICATION:** The gate waits for a competitor to scan their RFID card. The server validates the user.
2. **ROBOT_AUTHENTICATION:** The competitor scans the robot's QR code.
3. **TIME_SYNCHRONIZATION:** Start and Finish gates synchronize their internal clocks over NRF24L01 radio modules.
4. **START / FINISH:** The IR sensor detects the robot passing through, capturing high-precision timestamps.
5. **JUDGE_CONFIRMATION:** If required, a judge scans their RFID card to validate the run.
6. **CONFIRMATION:** The operator uses physical buttons to Accept, Cancel, or mark the run as a Forfeit, sending the final JSON payload to the server.

## 📁 Project Structure
```
LF_gate_project/
├── include/
│   └── constants.hpp              # Centralized constants and pin definitions
├── lib/                           # External libraries
├── src/
│   ├── main.cpp                   # Main orchestrator (lean, ~400 lines)
│   ├── main.hpp                   # Global types and function prototypes
│   ├── StateManager.hpp/.cpp      # State machine management (11 states)
│   ├── HardwareControl.hpp/.cpp   # GPIO abstraction (LEDs, buttons, display)
│   ├── Authentication.hpp/.cpp    # RFID and QR code scanning
│   ├── Measurement.hpp/.cpp       # Time calculations and result formatting
│   ├── RadioCommunication.hpp/.cpp    # NRF24 wireless synchronization
│   └── ServerCommunication.hpp/.cpp   # JSON server communication
├── test/                          # Unit tests
├── platformio.ini                 # PlatformIO configuration
├── CODING_STANDARDS.md            # Code style guide
└── README.md                      # This file
```

## 🏗️ Modular Architecture

The codebase has been refactored into specialized modules following clean code principles:

### Core Modules:
- **StateManager** - Handles 11-state finite state machine for competition flow
  - `getCurrentState()` - Get current state
  - `transitionTo(State)` - Change state with logging
  - `waitForStateTransition(ms)` - Delay between state changes
  - `reset()` - Reset to IDLE state

- **HardwareControl** - GPIO abstraction layer
  - LED control (Red, Green status indicators)
  - Button polling (Accept, Cancel, Forfeit)
  - P10 LED Matrix display messages and timers
  - IR sensor interrupt attachment

- **Authentication** - Competitor and robot verification
  - RFID card scanning for users and judges
  - QR/Barcode scanning for robots
  - Result caching to avoid duplicate reads

- **Measurement** - Time and data management
  - Competition run measurements (MeasurementManager singleton)
  - Time synchronization state tracking
  - Result formatting (MM:SS:mmm format)

- **RadioCommunication** - NRF24L01 wireless sync
  - Time synchronization between Start/Finish gates
  - Start/Finish command transmission
  - Interrupt-driven signal handling

- **ServerCommunication** - Wi-Fi JSON communication
  - Gate settings retrieval
  - User/Robot data fetching
  - Score submission
  - Connection status monitoring

### Key Files:
- `constants.hpp` - 130+ centralized constants (pins, timeouts, addresses)
- `main.hpp` - Measurement, Gate, and State struct definitions
- `main.cpp` - Lean orchestrator delegating to modules (~400 lines)
- `CODING_STANDARDS.md` - Professional C++ conventions

## 📚 Code Quality Improvements
This project follows professional C++ coding standards:
- **Naming Conventions:** camelCase for variables/functions, UPPERCASE for constants
- **Documentation:** JSDoc-style comments for all functions and structures
- **Constants:** Centralized in `constants.hpp` to avoid magic numbers
- **Code Organization:** Clear sections with descriptive headers
- **Error Handling:** Meaningful error messages in Serial output

## 🤝 Contributing
Contributions, issues, and feature requests are welcome!
Feel free to check the [issues page](https://github.com/Matt7S/LF_gate_project/issues) if you want to contribute.

## 👤 Author
**Matt7S** - GitHub: [@Matt7S](https://github.com/Matt7S)
