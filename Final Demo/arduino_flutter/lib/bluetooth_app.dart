import 'dart:convert';
import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';

// define UUIDs as constants - these should match the Arduino code
const String serviceUUID = "00000000-5EC4-4083-81CD-A10B8D5CF6EC";
const String characteristicUUID = "00000001-5EC4-4083-81CD-A10B8D5CF6EC";

// define for custom button for ramp
class SequenceStep {
  final int command;
  final Duration duration;

  SequenceStep({required this.command, required this.duration});
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});

  final String title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  final _ble = FlutterReactiveBle();

  StreamSubscription<DiscoveredDevice>? _scanSub; // subscribe to bluetooth scanning stream
  StreamSubscription<ConnectionStateUpdate>? _connectSub; // subscribe to bluetooth connection stream
  StreamSubscription<List<int>>? _notifySub;

  List<DiscoveredDevice> _devices = [];
  String? _selectedDeviceId; // will hold the device ID selected for connection
  String? _selectedDeviceName; // will hold the device name selected for connection
  var _stateMessage = 'Scanning...'; // displays app status
  QualifiedCharacteristic? _writeCharacteristic;

  bool _isConnected = false; // flag to indicate connection
  bool _isSequenceRunning = false; //flag

  // on initialization scan for devices
  Timer? _scanTimer;

  @override
  void initState() {
    super.initState();
    _scanSub = _ble.scanForDevices(withServices: []).listen(_onScanUpdate);
    _scanTimer = Timer.periodic(Duration(seconds: 5), (timer) {
      _scanSub?.cancel();
      _scanSub = _ble.scanForDevices(withServices: []).listen(_onScanUpdate);
    });
  }

  // when terminating cancel all the subscriptions
  @override
  void dispose() {
    _notifySub?.cancel();
    _connectSub?.cancel();
    _scanSub?.cancel();
    super.dispose();
  }

  // update devices that found with "NINA" in their name
  void _onScanUpdate(DiscoveredDevice d) {
    if (d.name.contains("NINA") &&
        !_devices.any((device) => device.id == d.id)) {
      setState(() {
        _devices.add(d);
      });
    }
  }

  // Connect to the devices that was selected by user
  void _connectToDevice() {
    if (_selectedDeviceId != null) {
      setState(() {
        _stateMessage = 'Connecting to $_selectedDeviceName...';
      });

      _connectSub = _ble.connectToDevice(id: _selectedDeviceId!).listen(
        (update) {
          if (update.connectionState == DeviceConnectionState.connected) {
            setState(() {
              _stateMessage = 'Connected to $_selectedDeviceName!';
              _isConnected = true;
            });
            _onConnected(_selectedDeviceId!);
          }
        },
        onError: (error) {
          setState(() {
            _stateMessage = 'Connection error: $error';
          });
        },
      );
    }
  }

  // Handle disconnection
  void _disconnectFromDevice() {
    try {
      if (_notifySub != null) {
        _notifySub?.cancel();
        _notifySub = null;
      }

      if (_connectSub != null) {
        _connectSub?.cancel();
        _connectSub = null;
      }

      setState(() {
        _isConnected = false;
        _stateMessage = 'Disconnected from $_selectedDeviceName.';
        _writeCharacteristic = null;
      });
    } catch (e) {
      setState(() {
        _stateMessage = 'Error during disconnection: $e';
      });
    }
  }

  void _onConnected(String deviceId) {
    final characteristic = QualifiedCharacteristic(
      deviceId: deviceId,
      serviceId: Uuid.parse(serviceUUID), // Use the constant here
      characteristicId: Uuid.parse(characteristicUUID), // Use the constant here
    );

    _writeCharacteristic = characteristic;

    _notifySub = _ble.subscribeToCharacteristic(characteristic).listen((bytes) {
      setState(() {
        _stateMessage = 'Data received: $bytes';
      });
    });
  }

  Future<void> _sendCommand(int command) async {
    if (_writeCharacteristic != null) {
      try {
        await _ble.writeCharacteristicWithResponse(
          _writeCharacteristic!,
          value: [command],
        );
        setState(() {
          _stateMessage = "Command '$command' sent!";
        });
      } catch (e) {
        setState(() {
          _stateMessage = "Error sending command: $e";
        });
      }
    }
  }

  // // 🟢 PLACE THE HELPER METHOD RIGHT HERE 🟢
  // Widget __buildTapButtonButton({required IconData icon, required int command}) {
  //   return GestureDetector(
  //     onTapDown: _isConnected ? (_) => _sendCommand(command) : null,
  //     onTapUp: _isConnected ? (_) => _sendCommand(0) : null, //STOP
  //     onTapCancel: _isConnected ? () => _sendCommand(0) : null, //STOP
  //     child: Container(
  //       padding: const EdgeInsets.all(16),
  //       decoration: BoxDecoration(
  //         color: _isConnected ? Colors.blue : Colors.grey[400],
  //         borderRadius: BorderRadius.circular(12),
  //       ),
  //       child: Icon(icon, color: Colors.white, size: 28),
  //     ),
  //   );
  // }

  // Loops dynamically through sequence steps, waiting out each duration before ending on Stop (5)
  void _runCustomSequence(List<SequenceStep> steps) async {
    if (!_isConnected || _isSequenceRunning) return;

    setState(() {
      _isSequenceRunning = true;
    });

    try {
      for (int i = 0; i < steps.length; i++) {
        final step = steps[i];
        
        setState(() {
          _stateMessage = "Macro Step ${i + 1}/${steps.length}: Sending ${step.command} for ${step.duration.inMilliseconds}ms";
        });

        await _sendCommand(step.command);
        await Future.delayed(step.duration);
      }

      setState(() {
        _stateMessage = "Sequence complete. Issuing STOP.";
      });
      await _sendCommand(5); // Stop command at the end

    } catch (e) {
      setState(() {
        _stateMessage = "Sequence failed: $e";
      });
    } finally {
      setState(() {
        _isSequenceRunning = false;
      });
    }
  }

  // Function that sends a signal, custom timing duration
  Future<void> _sendTimedSignal(int command, Duration duration) async {
    if (!_isConnected) return;
    
    // Send the turn signal command (e.g., 2 for Left, 4 for Right)
    await _sendCommand(command);
    
    // Wait for whatever duration is passed to this method
    await Future.delayed(duration);
    
    // Send the stop command (5)
    await _sendCommand(5); 
  }



    // Simple tap button (no hold) for directional commands
  Widget _buildTapButton({required IconData icon, required int command}) {
    return GestureDetector(
      onTap: _isConnected ? () => _sendCommand(command) : null,
      child: Container(
        padding: const EdgeInsets.all(16),
        decoration: BoxDecoration(
          color: _isConnected ? Colors.blue : Colors.grey[400],
          borderRadius: BorderRadius.circular(12),
        ),
        child: Icon(icon, color: Colors.white, size: 28),
      ),
    );
  }

  //widget that adds two timed turn buttons
  Widget _buildTimedSignalButton({ required IconData icon, required String label, 
  required int command, 
  required Duration duration,
  }) {
    return ElevatedButton.icon(
      onPressed: _isConnected ? () => _sendTimedSignal(command, duration) : null,
      icon: Icon(icon),
      label: Text(label),
      style: ElevatedButton.styleFrom(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Text(widget.title),
      ),
      body: Column(
        children: [
          Container(
            padding: const EdgeInsets.all(16.0),
            color: Colors.grey[300],
            width: double.infinity,
            child: Text(
              _stateMessage,
              style: Theme.of(context).textTheme.titleMedium,
              textAlign: TextAlign.center,
            ),
          ),
          if (_devices.isNotEmpty)
            Padding(
              padding: const EdgeInsets.all(16.0),
              child: DropdownButton<String>(
                isExpanded: true,
                hint: const Text("Select a BLE Device"),
                value: _selectedDeviceId,
                items: _devices.map((device) {
                  return DropdownMenuItem(
                    value: device.id,
                    child: Text(device.name),
                  );
                }).toList(),
                onChanged: (value) {
                  setState(() {
                    _selectedDeviceId = value;
                    _selectedDeviceName = _devices
                        .firstWhere((device) => device.id == value)
                        .name;
                  });
                },
              ),
            ),
          if (!_isConnected)
            ElevatedButton(
              onPressed: _selectedDeviceId != null ? _connectToDevice : null,
              child: const Text('Connect'),
            ),
          if (_isConnected)
            ElevatedButton(
              onPressed: _disconnectFromDevice,
              child: const Text('Disconnect'),
            ),
          // **************** command buttons ****************
          Expanded(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [

                // MULTI-STEP TIMED BUTTON FOR RAMP─── 
                // +5: 8
                // +10: 11
                // stay 10: 10
                // -10: 12
                // -5: 9
                // stop: 5
                //BUTTON FOR 5 DEGREE RAMP
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    ElevatedButton.icon(
                      onPressed: (_isConnected && !_isSequenceRunning)
                          ? () => _runCustomSequence([
                                SequenceStep(command: 8, duration: const Duration(seconds: 15)), // +5 for 15 seconds
                                SequenceStep(command: 9, duration: const Duration(seconds: 15)), // -5 for 15 seconds
                              ])
                          : null,
                      icon: const Icon(Icons.play_circle_filled_rounded),
                      label: const Text("Run 5 Ramp"),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.green,
                        foregroundColor: Colors.white,
                        padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 14),
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 10),
                //BUTTON FOR 10 DEGREE RAMP
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    ElevatedButton.icon(
                      onPressed: (_isConnected && !_isSequenceRunning)
                          ? () => _runCustomSequence([
                                SequenceStep(command: 8, duration: const Duration(seconds: 1)), // +5 for 1 second
                                SequenceStep(command: 11, duration: const Duration(seconds: 10)), // +10 for 10 seconds
                                SequenceStep(command: 10, duration: const Duration(seconds: 8)), // hold 10 for 8 seconds
                                SequenceStep(command: 9, duration: const Duration(seconds: 5)), // -5 for 5 seconds
                              ])
                          : null,
                      icon: const Icon(Icons.play_circle_filled_rounded),
                      label: const Text("Run 10 Ramp"),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.green,
                        foregroundColor: Colors.white,
                        padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 14),
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 25),

                // Joystick Buttons
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    _buildTimedSignalButton(
                      icon: Icons.arrow_upward,
                      label: "Forward (9s)",
                      command: 1, // FORWARD
                      duration: const Duration(milliseconds: 9000), // Adjust runtime here
                    ),
                  ],
                ),
                const SizedBox(height: 10),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    _buildTapButton(
                      icon: Icons.arrow_back,
                      command: 2, //LEFT
                    ),
                    const SizedBox(width: 10),
                    _buildTimedSignalButton(
                      icon: Icons.arrow_downward,
                      label: "Backward (9s)",
                      command: 3, // BACKWARD
                      duration: const Duration(milliseconds: 9000), // Adjust runtime here
                    ),
                    const SizedBox(width: 10),
                    _buildTapButton(
                      icon: Icons.arrow_forward,
                      command: 4, //RIGHT
                    ),
                  ],
                ),
                const SizedBox(height: 25),

                // Button for Ramp
                // Row(
                //   mainAxisAlignment: MainAxisAlignment.center,
                //   children: [
                //     // Fixed: Replaced custom template parameters with standard Tap Button structure
                //     ElevatedButton.icon(
                //       onPressed: _isConnected ? () => _sendCommand(8) : null,
                //       icon: const Icon(Icons.terrain),
                //       label: const Text("Ramp"),
                //     ),
                //   ],
                // ),
 
                // Timed turn button signals with individualized durations
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    _buildTimedSignalButton(
                      icon: Icons.keyboard_double_arrow_left,
                      label: "1s Left",
                      command: 2, 
                      duration: const Duration(milliseconds: 1000), // Left 
                    ),
                    const SizedBox(width: 15),
                    _buildTimedSignalButton(
                      icon: Icons.keyboard_double_arrow_right,
                      label: "1s Right",
                      command: 4, 
                      duration: const Duration(milliseconds: 1000), // Right 
                    ),
                  ],
                ),
                const SizedBox(height: 20),

                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    ElevatedButton(
                      onPressed: _isConnected ? () => _sendCommand(5) : null,
                      child: const Text('Stop'), //changed from Stop A
                    ),
                    const SizedBox(width: 10),
                    ElevatedButton(
                      onPressed: _isConnected ? () => _sendCommand(6) : null,
                      child: const Text('Send B'),
                    ),
                    const SizedBox(width: 10),
                    ElevatedButton(
                      onPressed: _isConnected ? () => _sendCommand(7) : null,
                      child: const Text('Send C'),
                    ),
                  ],
                ),
              ],
            ),
          ),
          // **************** end of command buttons ****************
        ],
      ),
    );
  }
}



// --------------------------- RAMP TESTING CODE ---------------------------------------------------

// import 'dart:convert';
// import 'dart:async';
// import 'package:flutter/material.dart';
// import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';

// // define UUIDs as constants - these should match the Arduino code
// const String serviceUUID = "00000000-5EC4-4083-81CD-A10B8D5CF6EC";
// const String characteristicUUID = "00000001-5EC4-4083-81CD-A10B8D5CF6EC";

// class MyHomePage extends StatefulWidget {
//   const MyHomePage({super.key, required this.title});

//   final String title;

//   @override
//   State<MyHomePage> createState() => _MyHomePageState();
// }

// class _MyHomePageState extends State<MyHomePage> {
//   final _ble = FlutterReactiveBle();

//   StreamSubscription<DiscoveredDevice>? _scanSub; // subscribe to bluetooth scanning stream
//   StreamSubscription<ConnectionStateUpdate>? _connectSub; // subscribe to bluetooth connection stream
//   StreamSubscription<List<int>>? _notifySub;

//   List<DiscoveredDevice> _devices = [];
//   String? _selectedDeviceId; // will hold the device ID selected for connection
//   String? _selectedDeviceName; // will hold the device name selected for connection
//   var _stateMessage = 'Scanning...'; // displays app status
//   QualifiedCharacteristic? _writeCharacteristic;

//   bool _isConnected = false; // flag to indicate connection

//   // on initialization scan for devices
//   Timer? _scanTimer;

//   @override
//   void initState() {
//     super.initState();
//     _scanSub = _ble.scanForDevices(withServices: []).listen(_onScanUpdate);
//     _scanTimer = Timer.periodic(Duration(seconds: 5), (timer) {
//       _scanSub?.cancel();
//       _scanSub = _ble.scanForDevices(withServices: []).listen(_onScanUpdate);
//     });
//   }

//   // when terminating cancel all the subscriptions
//   @override
//   void dispose() {
//     _notifySub?.cancel();
//     _connectSub?.cancel();
//     _scanSub?.cancel();
//     super.dispose();
//   }

//   // update devices that found with "NINA" in their name
//   void _onScanUpdate(DiscoveredDevice d) {
//     if (d.name.contains("NINA") &&
//         !_devices.any((device) => device.id == d.id)) {
//       setState(() {
//         _devices.add(d);
//       });
//     }
//   }

//   // Connect to the devices that was selected by user
//   void _connectToDevice() {
//     if (_selectedDeviceId != null) {
//       setState(() {
//         _stateMessage = 'Connecting to $_selectedDeviceName...';
//       });

//       _connectSub = _ble.connectToDevice(id: _selectedDeviceId!).listen(
//         (update) {
//           if (update.connectionState == DeviceConnectionState.connected) {
//             setState(() {
//               _stateMessage = 'Connected to $_selectedDeviceName!';
//               _isConnected = true;
//             });
//             _onConnected(_selectedDeviceId!);
//           }
//         },
//         onError: (error) {
//           setState(() {
//             _stateMessage = 'Connection error: $error';
//           });
//         },
//       );
//     }
//   }

//   // Handle disconnection
//   void _disconnectFromDevice() {
//     try {
//       if (_notifySub != null) {
//         _notifySub?.cancel();
//         _notifySub = null;
//       }

//       if (_connectSub != null) {
//         _connectSub?.cancel();
//         _connectSub = null;
//       }

//       setState(() {
//         _isConnected = false;
//         _stateMessage = 'Disconnected from $_selectedDeviceName.';
//         _writeCharacteristic = null;
//       });
//     } catch (e) {
//       setState(() {
//         _stateMessage = 'Error during disconnection: $e';
//       });
//     }
//   }

//   void _onConnected(String deviceId) {
//     final characteristic = QualifiedCharacteristic(
//       deviceId: deviceId,
//       serviceId: Uuid.parse(serviceUUID), // Use the constant here
//       characteristicId: Uuid.parse(characteristicUUID), // Use the constant here
//     );

//     _writeCharacteristic = characteristic;

//     _notifySub = _ble.subscribeToCharacteristic(characteristic).listen((bytes) {
//       setState(() {
//         _stateMessage = 'Data received: $bytes';
//       });
//     });
//   }

//   Future<void> _sendCommand(int command) async {
//     if (_writeCharacteristic != null) {
//       try {
//         await _ble.writeCharacteristicWithResponse(
//           _writeCharacteristic!,
//           value: [command],
//         );
//         setState(() {
//           _stateMessage = "Command '$command' sent!";
//         });
//       } catch (e) {
//         setState(() {
//           _stateMessage = "Error sending command: $e";
//         });
//       }
//     }
//   }

//   // // 🟢 PLACE THE HELPER METHOD RIGHT HERE 🟢
//   // Widget __buildTapButtonButton({required IconData icon, required int command}) {
//   //   return GestureDetector(
//   //     onTapDown: _isConnected ? (_) => _sendCommand(command) : null,
//   //     onTapUp: _isConnected ? (_) => _sendCommand(0) : null, //STOP
//   //     onTapCancel: _isConnected ? () => _sendCommand(0) : null, //STOP
//   //     child: Container(
//   //       padding: const EdgeInsets.all(16),
//   //       decoration: BoxDecoration(
//   //         color: _isConnected ? Colors.blue : Colors.grey[400],
//   //         borderRadius: BorderRadius.circular(12),
//   //       ),
//   //       child: Icon(icon, color: Colors.white, size: 28),
//   //     ),
//   //   );
//   // }

//     // Simple tap button (no hold) for directional commands
//   Widget _buildTapButton({required IconData icon, required int command}) {
//     return GestureDetector(
//       onTap: _isConnected ? () => _sendCommand(command) : null,
//       child: Container(
//         padding: const EdgeInsets.all(16),
//         decoration: BoxDecoration(
//           color: _isConnected ? Colors.blue : Colors.grey[400],
//           borderRadius: BorderRadius.circular(12),
//         ),
//         child: Icon(icon, color: Colors.white, size: 28),
//       ),
//     );
//   }

//   @override
//   Widget build(BuildContext context) {
//     return Scaffold(
//       appBar: AppBar(
//         backgroundColor: Theme.of(context).colorScheme.inversePrimary,
//         title: Text(widget.title),
//       ),
//       body: Column(
//         children: [
//           Container(
//             padding: const EdgeInsets.all(16.0),
//             color: Colors.grey[300],
//             width: double.infinity,
//             child: Text(
//               _stateMessage,
//               style: Theme.of(context).textTheme.titleMedium,
//               textAlign: TextAlign.center,
//             ),
//           ),
//           if (_devices.isNotEmpty)
//             Padding(
//               padding: const EdgeInsets.all(16.0),
//               child: DropdownButton<String>(
//                 isExpanded: true,
//                 hint: const Text("Select a BLE Device"),
//                 value: _selectedDeviceId,
//                 items: _devices.map((device) {
//                   return DropdownMenuItem(
//                     value: device.id,
//                     child: Text(device.name),
//                   );
//                 }).toList(),
//                 onChanged: (value) {
//                   setState(() {
//                     _selectedDeviceId = value;
//                     _selectedDeviceName = _devices
//                         .firstWhere((device) => device.id == value)
//                         .name;
//                   });
//                 },
//               ),
//             ),
//           if (!_isConnected)
//             ElevatedButton(
//               onPressed: _selectedDeviceId != null ? _connectToDevice : null,
//               child: const Text('Connect'),
//             ),
//           if (_isConnected)
//             ElevatedButton(
//               onPressed: _disconnectFromDevice,
//               child: const Text('Disconnect'),
//             ),
//           // **************** command buttons **********************************************************************
//           Expanded(
//             child: Column(
//               mainAxisAlignment: MainAxisAlignment.center,
//               children: [

//                 // buttons for up and down 5 degrees ramp buttons
//                 Row(
//                   mainAxisAlignment: MainAxisAlignment.center,
//                   children: [
//                     ElevatedButton.icon(
//                       onPressed: _isConnected ? () => _sendCommand(9) : null, // Command 9
//                       icon: const Icon(Icons.remove),
//                       label: const Text("-5 Target"),
//                     ),
//                     const SizedBox(width: 15),
//                     ElevatedButton.icon(
//                       onPressed: _isConnected ? () => _sendCommand(8) : null, // Command 8
//                       icon: const Icon(Icons.add),
//                       label: const Text("+5 Target"),
//                     ),
//                     const SizedBox(width: 15),
//                   ],
//                 ),

//                 //buttons for up and down 10 degree ramp
//                 Row(
//                   mainAxisAlignment: MainAxisAlignment.center,
//                   children: [
//                     ElevatedButton.icon(
//                       onPressed: _isConnected ? () => _sendCommand(12) : null, // Command 12
//                       icon: const Icon(Icons.remove),
//                       label: const Text("-10 Target"),
//                     ),
//                     const SizedBox(width: 15),
//                     ElevatedButton.icon(
//                       onPressed: _isConnected ? () => _sendCommand(11) : null, // Command 11
//                       icon: const Icon(Icons.add),
//                       label: const Text("+10 Target"),
//                     ),
//                     const SizedBox(width: 15),
//                   ],
//                 ),

//                 //stop buttons for each ramp section
//                 Row(
//                   mainAxisAlignment: MainAxisAlignment.center,
//                   children: [
//                     ElevatedButton.icon(
//                       onPressed: _isConnected ? () => _sendCommand(10) : null, // Command 10
//                       icon: const Icon(Icons.add),
//                       label: const Text("stop ramp"),
//                     ),
//                     const SizedBox(width: 15),
//                   ]
//                 ),
//                 const SizedBox(height: 10),

//                 // Joystick Buttons
//                 Row(
//                   mainAxisAlignment: MainAxisAlignment.center,
//                   children: [
//                     _buildTapButton(
//                       icon: Icons.arrow_upward,
//                       command: 1, //FORWARD
//                     ),
//                   ],
//                 ),
//                 const SizedBox(height: 10),
//                 Row(
//                   mainAxisAlignment: MainAxisAlignment.center,
//                   children: [
//                     _buildTapButton(
//                       icon: Icons.arrow_back,
//                       command: 2, //LEFT
//                     ),
//                     const SizedBox(width: 10),
//                     _buildTapButton(
//                       icon: Icons.arrow_downward,
//                       command: 3, //BACKWARD
//                     ),
//                     const SizedBox(width: 10),
//                     _buildTapButton(
//                       icon: Icons.arrow_forward,
//                       command: 4, //RIGHT
//                     ),
//                   ],
//                 ),
//                 const SizedBox(height: 20),

//                 Row(
//                   mainAxisAlignment: MainAxisAlignment.center,
//                   children: [
//                     ElevatedButton(
//                       onPressed: _isConnected ? () => _sendCommand(5) : null,
//                       child: const Text('Stop'), //changed from Stop A
//                     ),
//                     const SizedBox(width: 10),
//                     ElevatedButton(
//                       onPressed: _isConnected ? () => _sendCommand(6) : null,
//                       child: const Text('Send B'),
//                     ),
//                     const SizedBox(width: 10),
//                     ElevatedButton(
//                       onPressed: _isConnected ? () => _sendCommand(7) : null,
//                       child: const Text('Send C'),
//                     ),
//                   ],
//                 ),
//               ],
//             ),
//           ),
//           // **************** end of command buttons ****************
//         ],
//       ),
//     );
//   }
// }
