// Flutter MQTT Image Viewer Example
// Add to pubspec.yaml:
// dependencies:
//   mqtt_client: ^10.2.0

import 'package:flutter/material.dart';
import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import 'dart:typed_data';

class ESP32CamViewer extends StatefulWidget {
  @override
  _ESP32CamViewerState createState() => _ESP32CamViewerState();
}

class _ESP32CamViewerState extends State<ESP32CamViewer> {
  MqttServerClient? client;
  Uint8List? currentFrame;
  Uint8List? _frameBuffer;
  bool isConnected = false;
  String statusMessage = 'Disconnected';
  int frameCount = 0;
  DateTime? _lastFrameTime;
  double fps = 0.0;

  // MQTT Configuration - UPDATE THESE WITH YOUR BROKER DETAILS
  final String broker = 'broker.hivemq.com'; // Or your HiveMQ cluster URL
  final int port = 1883;
  final String username = ''; // Your username (empty for public broker)
  final String password = ''; // Your password (empty for public broker)

  // Topics (must match ESP32 topics)
  final String topicFrame = 'camera/esp32cam/frame';
  final String topicStatus = 'camera/esp32cam/status';
  final String topicControl = 'camera/esp32cam/control';

  @override
  void initState() {
    super.initState();
    connectToMQTT();
  }

  Future<void> connectToMQTT() async {
    setState(() {
      statusMessage = 'Connecting...';
    });

    client = MqttServerClient(
      broker,
      'flutter_viewer_${DateTime.now().millisecondsSinceEpoch}',
    );
    client!.port = port;
    client!.keepAlivePeriod = 20; // More frequent keep-alive
    client!.autoReconnect = true;
    client!.logging(on: false);
    client!.setProtocolV311();
    client!.onDisconnected = _onDisconnected;
    client!.onConnected = _onConnected;

    final connMessage = MqttConnectMessage()
        .withClientIdentifier(
          'flutter_viewer_${DateTime.now().millisecondsSinceEpoch}',
        )
        .startClean()
        .withWillQos(MqttQos.atMostOnce)
        .keepAliveFor(60);

    client!.connectionMessage = connMessage;

    try {
      print('Connecting to MQTT broker at $broker:$port...');
      await client!.connect(
        username.isNotEmpty ? username : null,
        password.isNotEmpty ? password : null,
      );

      if (client!.connectionStatus!.state == MqttConnectionState.connected) {
        setState(() {
          isConnected = true;
          statusMessage = 'Connected to broker';
        });
        print('Connected to MQTT broker');

        // Subscribe to frame topic
        client!.subscribe(topicFrame, MqttQos.atMostOnce);

        // Subscribe to status topic
        client!.subscribe(topicStatus, MqttQos.atMostOnce);

        // Listen for messages
        client!.updates!.listen((
          List<MqttReceivedMessage<MqttMessage>> messages,
        ) {
          final MqttPublishMessage message =
              messages[0].payload as MqttPublishMessage;
          final String topic = messages[0].topic;

          if (topic == topicFrame) {
            // Received image frame
            final Uint8List imageBytes = Uint8List.fromList(
              message.payload.message.toList(),
            );

            // Calculate FPS
            final now = DateTime.now();
            if (_lastFrameTime != null) {
              final timeDiff = now.difference(_lastFrameTime!).inMilliseconds;
              if (timeDiff > 0) {
                fps = 1000.0 / timeDiff;
              }
            }
            _lastFrameTime = now;

            // Update frame immediately without waiting for setState
            _frameBuffer = imageBytes;
            frameCount++;

            // Use post frame callback for smoother updates
            WidgetsBinding.instance.addPostFrameCallback((_) {
              if (mounted && _frameBuffer != null) {
                setState(() {
                  currentFrame = _frameBuffer;
                  statusMessage =
                      'Live • ${fps.toStringAsFixed(1)} FPS • Frame #$frameCount';
                });
              }
            });
          } else if (topic == topicStatus) {
            // Received status message
            final String status = String.fromCharCodes(message.payload.message);
            print('Status: $status');
          }
        });
      } else {
        setState(() {
          isConnected = false;
          statusMessage =
              'Connection failed: ${client!.connectionStatus!.returnCode}';
        });
        print('Connection failed - Status: ${client!.connectionStatus}');
      }
    } catch (e) {
      setState(() {
        isConnected = false;
        statusMessage = 'Error: ${e.toString().split('\n').first}';
      });
      print('Exception: $e');
    }
  }

  void sendControlCommand(String command) {
    if (client != null &&
        client!.connectionStatus!.state == MqttConnectionState.connected) {
      final builder = MqttClientPayloadBuilder();
      builder.addString(command);
      client!.publishMessage(
        topicControl,
        MqttQos.atLeastOnce,
        builder.payload!,
      );
      print('Sent command: $command');
    }
  }

  void _onConnected() {
    print('MQTT Connected');
  }

  void _onDisconnected() {
    print('MQTT Disconnected');
    if (mounted) {
      setState(() {
        isConnected = false;
        statusMessage = 'Disconnected - Reconnecting...';
      });
    }
  }

  @override
  void dispose() {
    client?.disconnect();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('ESP32-CAM MQTT Viewer'),
        backgroundColor: Colors.blue,
      ),
      body: Column(
        children: [
          // Status Bar
          Container(
            padding: EdgeInsets.all(16),
            color: isConnected ? Colors.green : Colors.red,
            width: double.infinity,
            child: Text(
              statusMessage,
              style: TextStyle(color: Colors.white, fontSize: 16),
              textAlign: TextAlign.center,
            ),
          ),

          // Camera Feed
          Expanded(
            child: Center(
              child: currentFrame != null
                  ? Image.memory(
                      currentFrame!,
                      fit: BoxFit.contain,
                      gaplessPlayback: true,
                      filterQuality: FilterQuality.low, // Faster rendering
                      cacheWidth: null,
                      cacheHeight: null,
                      errorBuilder: (context, error, stackTrace) {
                        return Icon(Icons.error, size: 48, color: Colors.red);
                      },
                    )
                  : Text(
                      'Waiting for camera feed...',
                      style: TextStyle(fontSize: 18, color: Colors.grey),
                    ),
            ),
          ),

          // Control Buttons
          Container(
            padding: EdgeInsets.all(16),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                ElevatedButton.icon(
                  onPressed: () => sendControlCommand('start'),
                  icon: Icon(Icons.play_arrow),
                  label: Text('Start'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.green,
                  ),
                ),
                ElevatedButton.icon(
                  onPressed: () => sendControlCommand('stop'),
                  icon: Icon(Icons.stop),
                  label: Text('Stop'),
                  style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
                ),
                ElevatedButton.icon(
                  onPressed: () {
                    if (!isConnected) connectToMQTT();
                  },
                  icon: Icon(Icons.refresh),
                  label: Text('Reconnect'),
                  style: ElevatedButton.styleFrom(backgroundColor: Colors.blue),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

// Example main.dart usage:
void main() {
  runApp(
    MaterialApp(
      home: ESP32CamViewer(),
      theme: ThemeData(primarySwatch: Colors.blue),
    ),
  );
}
