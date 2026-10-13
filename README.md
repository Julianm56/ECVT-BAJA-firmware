# Baja Wildcat Racing — Electrical & Telemetry Firmware

The software (and hardware designs) behind the electrical and telemetry system on the Baja Wildcat Racing Baja SAE car, 2025–2026 season.

**Electrical Lead:** Julian Pimienta

**Software Lead:** Karsten Yin

---

## What this repo is

The car has a network of small sensor boards connected over a CAN bus to a central Raspberry Pi, the **car computer**. The car computer polls every sensor, logs the readings to a database on an SSD, drives the dashboard, and streams live data to the pit over a LoRa radio.

```
Sensor nodes (Nano / ESP32)                           Dash (ESP32)
IMU, GPS, engine RPM, wheel speed,                    servo gauges +
CVT temp, shock travel, brake pressure                alphanumeric displays
        │                                                  │
════════╧════════════════════════╤═════════════════════════╧════  CAN bus, 500 kbit/s
                                 │
                    Car computer (Raspberry Pi)
                            src/CarMain
                        │                 │
                        ▼                 ▼
           SQLite database on the SSD     LoRa 915 MHz ──▶ Pit radio (ESP32) ──USB──▶ pit laptop
```

## Repository layout

| Path | What's there |
|---|---|
| [`src/CarMain/`](src/CarMain) | The car computer program (C++17, runs on the Raspberry Pi). Built by the root [`CMakeLists.txt`](CMakeLists.txt). |
| [`src/Onboard/`](src/Onboard) | One PlatformIO project per microcontroller: the sensor nodes, the dash, and the pit radio. |
| [`src/Onboard/CANTProtocol/`](src/Onboard/CANTProtocol) | The CAN library every node uses (see [the CAN bus](#the-can-bus-and-the-cant-protocol)). |
| [`include/`](include) | Enums shared between the car computer and the pit side: [`Commands.h`](include/Commands.h), [`DataTypes.h`](include/DataTypes.h), [`Devices.h`](include/Devices.h). |
| [`hardware/`](hardware) | KiCad schematics and PCBs, gerbers, 3D models, and datasheets for the boards. |
| [`doc/`](doc) | Pinouts, system diagrams (`.drawio`, open with diagrams.net), and the [web app spec](doc/Webapp.md). |
| [`tools/`](tools) | `DatabaseParsing/DatabaseToCSV.py`, a small GUI that exports logged data to CSV. It was written for an older database layout and needs updating for the current one. |
| [`tests/`](tests) | Scratch projects and experiments. Not part of the car. |

---

## How the car computer works

The car computer ([`src/CarMain`](src/CarMain)) is modeled on FIRST Robotics' WPILib. There are four ideas to understand:

- **Devices** ([`devices/`](src/CarMain/src/devices)) — one class per CAN node (`Tachometer`, `IMU`, `GPS`, `Dash`, …). Each inherits from `CANDevice` and turns "give me the engine RPM" into a CAN request.
- **Subsystems** ([`subsystems/`](src/CarMain/src/subsystems)) — group devices into parts of the car (`DrivetrainSubsystem`, `IMUSubsystem`, `GPSSubsystem`, `DashSubsystem`) and convert raw readings into real units (for example, ADC counts into psi).
- **Procedures** ([`procedures/`](src/CarMain/src/procedures)) — the behavior. Each procedure runs at its own rate, reads from subsystems, and logs the data, radios it to the pit, or sends it to the dash.
- **Commands** ([`include/Commands.h`](include/Commands.h)) — start and stop procedures. They come from the pit over the radio, plus `DEFAULT_CAR_START`, which the car sends itself at boot.

[`CarContainer.cpp`](src/CarMain/src/CarContainer.cpp) is where these are wired together: it builds the subsystems and binds each procedure to the commands that start and stop it. It's the file you'll edit most.

### The main loop

[`main.cpp`](src/CarMain/src/main.cpp) creates a `Car` ([`Car.cpp`](src/CarMain/src/Car.cpp)), which:

1. Creates, in order: `DataStorage` (opens a new SQLite file), `ProcedureScheduler`, `CANDispatcher` (opens the `can0` interface), `Coms` (starts the radio), and `CarContainer` (builds the subsystems and binds the procedures).
2. Sends itself `DEFAULT_CAR_START`, which starts every procedure bound to it.
3. Loops at **360 Hz** (`BASE_CAR_FREQUENCY` in [`Constants.h`](src/CarMain/include/Constants.h)) until it gets Ctrl-C or SIGTERM. Each tick it:
   - runs every active procedure that is due (a 30 Hz procedure runs every 12th tick),
   - sends up to 6 queued CAN frames, and drops requests that have gone unanswered for 100 ticks,
   - updates `CarTime` (seconds since start, and Unix time) and the radio's timestamp.

The slow work happens on three background threads: one reads CAN replies, one writes queued database rows every 100 ms, and one runs the radio.

### What runs on the car

Logging procedures start on `DEFAULT_CAR_START` or `START_LOG` and stop on `END_LOG`. The dash procedure starts at boot and never stops.

| Procedure | Rate | What it does |
|---|---|---|
| `IMULoggingProcedure` | 30 Hz | Logs and radios IMU rotation and acceleration |
| `RPMLoggingProcedure` | 30 Hz | Logs and radios engine RPM |
| `SpeedLoggingProcedure` | 30 Hz | Logs and radios wheel RPM |
| `DisplacementLoggingProcedure` | 30 Hz | Logs and radios shock travel at all four corners |
| `BrakePressureLoggingProcedure` | 30 Hz | Logs and radios front and rear brake pressure |
| `TemperatureLoggingProcedure` | 10 Hz | Logs and radios CVT temperature |
| `GPSLoggingProcedure` | 10 Hz | Waits for a GPS fix, then logs and radios position |
| `CarStateProcedure` | 60 Hz | Integrates wheel speed into distance traveled; radios speed and distance; sends distance to the dash |
| `DashProcedure` | 5 Hz | Sends CVT temperature, engine RPM, speed, and elapsed time to the dash |

Procedure rates must divide evenly into 360.

### Reading a sensor is asynchronous

Device getters such as `Tachometer::getEngineRPM()` queue a CAN request and immediately return the **last** value received. When the reply arrives, the CAN thread overwrites that cached value. So each reading is the answer to the previous request, about one procedure cycle old.

### Logging data

`DataStorage` writes to one SQLite file per run: `/home/bajaelectrical/DataStorage/baja_data_YYYY-MM-DD_HH-MM-SS.db`, where `DataStorage` is the SSD's mount point. Each kind of sensor has its own table (`imu`, `gps`, `tachometer`, `temperature`, `pressure`, `linear_actuator`, `vehicle_state`, `log`), and every row is keyed by Unix time in milliseconds, the vehicle name, and the sensor name.

The schema the program actually uses is the `DATABASE_SCHEMA` string in [`SqlCommands.h`](src/CarMain/include/SqlCommands.h). [`database_schema.sql`](src/CarMain/src/database_schema.sql) is a readable copy, so keep the two in sync.

Debug messages, warnings, and errors go to `/home/bajaelectrical/car.log` through `CarLogger`.

### The radio link to the pit

`Coms` ([`coms/Coms.cpp`](src/CarMain/src/coms/Coms.cpp)) streams live data over an RFM95 LoRa module at 915 MHz using the RadioHead library. Procedures call `coms.sendData(DataType::..., value)`. Only the newest value of each data type is kept, and the radio thread sends what's queued 10 times a second by default, packing as many values into each packet as fit:

```
[ data type: 1 byte ][ timestamp: float, seconds since start ][ length: 1 byte ][ data ] [ next value... ]
```

Every 2 seconds the car flags a packet to say it's about to listen, then waits briefly for commands from the pit. The pit can send:

- `0` followed by 4 bytes — change how often the car transmits (Hz)
- `1` followed by 32 bytes — a 256-bit mask choosing which data types to send
- any other byte — a `Command` (such as `START_LOG = 100` or `END_LOG = 101`), passed to the procedure scheduler

The pit radio ([`src/Onboard/PitRadio`](src/Onboard/PitRadio)) forwards every packet it receives to the pit laptop over USB serial as `[0xAAAAAAAA][header flags: 1 byte][RSSI: int16][packet]`. It also takes command bytes from the laptop and sends them during the car's next listening window. The header flags are: bit 0, the car is about to listen; bit 1, the car received the last commands; bit 2, the car's radio is overloaded. The pit laptop software is not in this repo.

---

## The CAN bus and the CANT protocol

Every node uses an MCP2515 CAN controller at 500 kbit/s and the shared [`CANTProtocol`](src/Onboard/CANTProtocol) library. On the Pi side, `CANDispatcher` speaks the same protocol through Linux SocketCAN. Messages use 29-bit extended CAN IDs:

| Bits | 28–24 | 23–20 | 19–0 |
|---|---|---|---|
| Field | Device ID ([`Devices.h`](include/Devices.h)) | Command (0–15) | Callback ID (0 = no reply expected) |

There are three kinds of message:

- **Request** — the Pi asks a node for data. The node's handler replies with `CAN.sendRequestResponse(value, callbackID)`. Values bigger than 8 bytes are split across several frames automatically, and the Pi reassembles them. Every sensor works this way.
- **Lossy command** — the Pi sends data to a node and expects no reply. The dash works this way.
- **Lossless command** — the same, but the node sends back an acknowledgement. (Resending when no acknowledgement arrives isn't implemented yet.)

Each node calls `CAN.execute()` in `loop()`, which handles up to 2 requests and 6 commands per call.

### The nodes

| Folder in `src/Onboard/` | Node | CAN ID | Board | What it does |
|---|---|---|---|---|
| [`IMU`](src/Onboard/IMU) | IMU | `0x01` | ESP32 DevKit | BNO055: orientation and linear acceleration |
| [`Temperature`](src/Onboard/Temperature) | CVT temperature | `0x02` | Arduino Nano | MLX90614 infrared thermometer |
| [`Spedometer`](src/Onboard/Spedometer) | Wheel speed | `0x03` | Arduino Nano | Three analog inputs → front-left, front-right, and rear wheel RPM |
| [`Tachometer`](src/Onboard/Tachometer) | Engine RPM | `0x04` | Arduino Nano | Analog voltage → RPM through a curve fit |
| [`Displacement`](src/Onboard/Displacement) | Shock travel | `0x05` front, `0x06` rear | Arduino Nano | Two analog inputs (left and right) |
| [`Brake Pressure Code`](src/Onboard/Brake%20Pressure%20Code) | Brake pressure | `0x07` front, `0x08` rear | Arduino Nano | One analog input |
| [`GPS`](src/Onboard/GPS) | GPS | `0x0A` | ESP32 DevKit | u-blox GNSS: position, Unix time, and fix status |
| [`Dash REV2`](src/Onboard/Dash%20REV2) | Dash (newest revision) | `0x09` | Unexpected Maker TinyS3 | Two servo gauges (tach and speedo), two 4-character displays, and a button to cycle readouts |
| [`Dash`](src/Onboard/Dash) | Dash REV1 | `0x09` | Seeed XIAO ESP32-S3 | Earlier dash revision |
| [`Old Dash`](src/Onboard/Old%20Dash) | 2024–25 dash | — | ESP32 DevKit | Predates CANTProtocol |
| [`PitRadio`](src/Onboard/PitRadio) | Pit radio | not on CAN | ESP32 DevKit + RFM95 | Bridges LoRa to USB serial |

The front and rear displacement boards run the same firmware, as do the front and rear brake pressure boards. Change the device ID in `main.cpp` before flashing the rear one.

The dash accepts five commands: `0` engine RPM, `1` speed (mph), `2` CVT temperature (°F), `3` elapsed time (s), and `4` distance (miles).

---

## Building and running

### Car computer (Raspberry Pi)

Requirements: Linux with SocketCAN, CMake 3+, a C++17 compiler, the SQLite3 development library, and RadioHead installed with a `radiohead` pkg-config file.

From the repository root:

```bash
mkdir build && cd build
cmake ..
make
./carComputer
```

`sudo make install` copies `carComputer` into the system `bin` directory.

Before running:

- The `can0` interface must be up at the same bitrate as the nodes, e.g. `sudo ip link set can0 up type can bitrate 500000`.
- The SSD must be mounted at `/home/bajaelectrical/DataStorage`. The `/etc/fstab` line for this is in [`src/CarMain/readme.md`](src/CarMain/readme.md#data-storage).
- The interface name and file paths are set in [`Car.h`](src/CarMain/include/Car.h).

Ctrl-C or SIGTERM shuts the program down cleanly: procedures are ended and the database is closed.

### Microcontrollers (PlatformIO)

Each folder in `src/Onboard/` is a PlatformIO project. With PlatformIO installed (the VS Code extension or the CLI):

```bash
cd src/Onboard/Tachometer
pio run -t upload
```

- Each `platformio.ini` pulls in `CANTProtocol` with `symlink://../CANTProtocol`, so every node builds against the same copy of the library. A change there affects every node.
- Each project defines `CANT_ARDUINO` or `CANT_ESP32` in `build_flags`. CANTProtocol won't compile without one.
- **Dash REV2 is the exception.** It uses its own copy of the library (`CANTProtocol_DASHONLY`) and a modified LED backpack library, both shipped in [`lib.zip`](src/Onboard/Dash%20REV2). Unzip it into `src/Onboard/Dash REV2/lib/` before building; the `lib/` folder isn't tracked by git.

---

## Adding a new sensor

1. **Write the firmware.** Copy an existing project in `src/Onboard/` (the [Tachometer](src/Onboard/Tachometer/src/main.cpp) is the simplest), pick an unused device ID (0–31), and register a request handler:

   ```cpp
   CANTProtocol CAN(10, 2, 0x4);   // CS pin, interrupt pin, device ID

   void onRequest(unsigned char dataLength, byte* incomingData, unsigned long callbackID) {
       float rpm = getRPM();
       CAN.sendRequestResponse(rpm, callbackID);   // any value or struct
   }

   void setup() {
       CAN.registerRequest(0x00, onRequest);       // command 0–15
       while (!CAN.begin()) delay(100);
   }

   void loop() {
       CAN.execute();
   }
   ```

2. **Register the device ID** in [`include/Devices.h`](include/Devices.h).
3. **Add a device class** in [`src/CarMain/src/devices/`](src/CarMain/src/devices), modeled on `Tachometer.h`/`.cpp`: inherit from `CANDevice` and call `sendCanRequest(command, &member, sizeof(member))`. The type you read into must match what the firmware sends, byte for byte.
4. **Add it to a subsystem**, constructed with `Device::Devices::YOUR_SENSOR`, with a getter that converts the reading into real units.
5. **Store and transmit it.**
   - To log it: add a table to `DATABASE_SCHEMA` and an `INSERT_...` statement in [`SqlCommands.h`](src/CarMain/include/SqlCommands.h), then a `storeData` overload in [`DataStorage`](src/CarMain/src/DataStorage.cpp).
   - To radio it: add a data type below 256 to [`include/DataTypes.h`](include/DataTypes.h).
6. **Write a procedure.** Copy [`RPMLoggingProcedure.cpp`](src/CarMain/src/procedures/RPMLoggingProcedure.cpp), set `this->frequency`, `#include` it in [`CarContainer.h`](src/CarMain/include/CarContainer.h), and bind it in [`CarContainer.cpp`](src/CarMain/src/CarContainer.cpp):

   ```cpp
   procedureScheduler.bindCommand<RPMLoggingProcedure>(
       std::unordered_set<Command>({Command::DEFAULT_CAR_START, Command::START_LOG}),  // start on
       std::unordered_set<Command>({Command::END_LOG}),                                // stop on
       drivetrainSubsystem, dataStorage, coms                                          // constructor arguments
   );
   ```

CMake picks up new `.cpp` files under `src/CarMain/src/` automatically, but you need to re-run `cmake ..` after adding one.

---

## More documentation

- [`src/CarMain/readme.md`](src/CarMain/readme.md) — a deeper walkthrough of the car computer, including SSD and SQLite setup
- [`doc/Webapp.md`](doc/Webapp.md) — the spec for a planned telemetry web app
- [`doc/pinouts/`](doc/pinouts) — ESP32, Arduino Nano, and Raspberry Pi pinouts
