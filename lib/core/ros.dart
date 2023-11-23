// Copyright (c) 2019 Conrad Heidebrecht.

import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'package:web_socket_channel/io.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

// ignore: uri_does_not_exist
// ignore: unused_import
import 'ros_stub.dart'
    // ignore: uri_does_not_exist
    if (dart.library.html) 'ros_html.dart'
    // ignore: uri_does_not_exist
    if (dart.library.io) 'ros_io.dart';

import 'request.dart';

/// Status enums.
enum Status { none, connecting, connected, closed, errored }

enum TopicStatus {
  subscribed,
  unsubscribed,
  publisher,
  advertised,
  unadvertised,
}

/// The class through which all data to and from a ROS node goes through.
/// Manages status and key information about the connection and node.
class Ros {
  /// Initializes the [_statusController] as a broadcast.
  /// The [url] of the ROS node can be optionally specified at this point.
  Ros({this.url}) {
    _statusController = StreamController<(Status status, String? reason)>.broadcast();
  }

  /// The url of ROS node running the rosbridge server.
  dynamic url;

  /// Total subscribers to ever connect.
  int subscribers = 0;

  /// Total number of advertisers to ever connect.
  int advertisers = 0;

  /// Total number of publishers to ever connect.
  int publishers = 0;

  /// Total number of callers to ever call a service.
  int serviceCallers = 0;

  /// The sum to generate IDs with.
  int get ids => subscribers + advertisers + publishers + serviceCallers;

  /// The websocket connection to communicate with the ROS node.
  late WebSocketChannel _channel;

  /// Subscription to the websocket stream.
  late StreamSubscription _channelListener;

  /// JSON broadcast websocket stream.
  late Stream<Map<String, dynamic>> stream;

  /// The controller to update subscribers on the state of the connection.
  late StreamController<(Status status, String? reason)> _statusController;

  /// Subscribable stream to listen for connection status changes.
  Stream<(Status status, String? reason)> get statusStream => _statusController.stream;

  /// Status variable that can be used when not interested in getting live updates.
  Status status = Status.none;

  void _changeStatus(Status status, [String? reason]) {
    this.status = status;
    _statusController.add((status, reason));
  }

  /// Connect to the ROS node, the [url] can override what was provided in the constructor.
  void connect({dynamic url}) async {
    this.url = url ?? this.url;
    url ??= this.url;

    _changeStatus(Status.connecting);

    // try {
    //   _channel = WebSocketChannel.connect(Uri.parse(url));
    // } catch (error) {
    //   _changeStatus(Status.errored, error.toString());
    // }

    // Uri uri = Uri.parse('ws://' + host + ':' + port.toString());

    // timeout based on https://github.com/dart-lang/web_socket_channel/issues/61
    final httpClient = HttpClient();
    httpClient.connectionTimeout = const Duration(seconds: 15);
    await runZonedGuarded<Future<void>>(() async {
      WebSocket.connect(
        url,
        customClient: httpClient,
      ).then((ws) {
        _channel = IOWebSocketChannel(ws);
        _changeStatus(Status.connected);

        stream = _channel.stream.asBroadcastStream().map((raw) => json.decode(raw));

        // Listen for messages on the connection to update the status.
        _channelListener = stream.listen(
          (data) {
            //print('INCOMING: $data');
          },
          onError: (error) {
            _changeStatus(Status.errored, error.toString());
          },
          onDone: () {
            _changeStatus(Status.closed);
          },
          cancelOnError: true,
        );
      }).onError((error, stackTrace) {
        if (error is TimeoutException) {
          _changeStatus(Status.errored, "connection timed out");
        } else if (error is SocketException) {
          if (error.message.substring(0, 25) == "HTTP connection timed out") {
            _changeStatus(Status.errored, "connection timed out");
          } else {
            _changeStatus(Status.errored, error.message);
          }
        } else {
          _changeStatus(Status.errored, error.toString());
        }
      });
    }, (error, stack) async {
      _changeStatus(Status.errored, error.toString());
    });

    // try {
    //   await runZonedGuarded<Future<void>>(() async {
    //     try {
    //       WebSocket.connect(url).timeout(const Duration(seconds: 10)).then((ws) {
    //         _channel = IOWebSocketChannel(ws);

    //         stream = _channel.stream.asBroadcastStream().map((raw) => json.decode(raw));

    //         // Update the connection status.
    //         _changeStatus(Status.connected);

    //         // Listen for messages on the connection to update the status.
    //         _channelListener = stream.listen((data) {
    //           //print('INCOMING: $data');
    //         }, onError: (error) {
    //           _changeStatus(Status.errored, error.toString());
    //         }, onDone: () {
    //           _changeStatus(Status.closed);
    //         });
    //       });
    //     } catch (error) {
    //       _changeStatus(Status.errored, error.toString());
    //     }
    //   }, (error, stack) async {
    //     if (error is TimeoutException) {
    //       _changeStatus(Status.errored, "connection timed out");
    //     } else {
    //       _changeStatus(Status.errored, error.toString());
    //     }

    //     await _channel.ready;
    //     _channel.sink.close();
    //   });
    // } catch (error) {
    //   _changeStatus(Status.errored, error.toString());
    //   return;
    // }

    // try {
    //   // Initialize the connection to the ROS node with a Websocket.
    //   WebSocket.connect(url).then((socket) {
    //     // Convert the WebSocket to a WebSocketChannel.
    //     _channel = IOWebSocketChannel(socket);
    //     stream = _channel.stream.asBroadcastStream().map((raw) => json.decode(raw));

    //     // Update the connection status.
    //     _changeStatus(Status.connected);

    //     // Listen for messages on the connection to update the status.
    //     _channelListener = stream.listen((data) {
    //       //print('INCOMING: $data');
    //     }, onError: (error) {
    //       _changeStatus(Status.errored, error);
    //     }, onDone: () {
    //       _changeStatus(Status.closed);
    //     });
    //   }, onError: (e) {
    //     _changeStatus(Status.errored, e.toString());
    //   }).catchError((e) {
    //     _changeStatus(Status.errored, e.toString());
    //   });
    // } catch (e) {
    //   if (e is WebSocketChannelException || e is SocketException) {
    //     _changeStatus(Status.errored, e.toString());
    //   } else {
    //     print("rethrowing");
    //     rethrow;
    //   }
    // }
  }

  /// Close the connection to the ROS node, an exit [code] and [reason] can
  /// be optionally specified.
  Future<void> close([int? code, String? reason]) async {
    /// Close listener and websocket.
    await _channelListener.cancel();
    await _channel.sink.close(code, reason);

    /// Update the connection status.
    _changeStatus(Status.closed);
  }

  /// Send a [message] to the ROS node
  bool send(dynamic message) {
    // If we're not connected give up.
    if (status != Status.connected) return false;
    // Format the message into JSON and then stringify.
    final toSend = (message is Request)
        ? json.encode(message.toJson())
        : (message is Map || message is List)
            ? json.encode(message)
            : message;
    //print('OUTGOING: $toSend');
    // Actually send it to the node.
    _channel.sink.add(toSend);
    return true;
  }

  void authenticate({
    required String mac,
    required String client,
    required String dest,
    required String rand,
    required DateTime t,
    required String level,
    required DateTime end,
  }) async {
    send({
      'mac': mac,
      'client': client,
      'dest': dest,
      'rand': rand,
      't': t.millisecondsSinceEpoch,
      'level': level,
      'end': end.millisecondsSinceEpoch,
    });
  }

  /// Sends a set_level request to the server.
  /// [level] can be one of {none, error, warning, info}, and
  /// [id] is the optional operation ID to change status level on
  void setStatusLevel({String? level, int? id}) {
    send({
      'op': 'set_level',
      'level': level,
      'id': id,
    });
  }

  /// Request a subscription ID.
  String requestSubscriber(String name) {
    subscribers++;
    return 'subscribe:' + name + ':' + ids.toString();
  }

  /// Request an advertiser ID.
  String requestAdvertiser(String name) {
    advertisers++;
    return 'advertise:' + name + ':' + ids.toString();
  }

  /// Request a publisher ID.
  String requestPublisher(String name) {
    publishers++;
    return 'publish:' + name + ':' + ids.toString();
  }

  /// Request a service caller ID.
  String requestServiceCaller(String name) {
    serviceCallers++;
    return 'call_service:' + name + ':' + ids.toString();
  }

  @override
  bool operator ==(other) {
    return other.hashCode == hashCode;
  }

  @override
  int get hashCode =>
      url.hashCode +
      subscribers.hashCode +
      advertisers.hashCode +
      publishers.hashCode +
      _channel.hashCode +
      _channelListener.hashCode +
      stream.hashCode +
      _statusController.hashCode +
      status.hashCode;
}
