import 'package:flutter/material.dart';
import 'package:roslib/roslib.dart';

void main() {
  runApp(const ExampleApp());
}

class ExampleApp extends StatelessWidget {
  const ExampleApp({Key? key}) : super(key: key);
  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      title: 'Roslib Example',
      home: HomePage(),
    );
  }
}

class HomePage extends StatefulWidget {
  const HomePage({Key? key}) : super(key: key);
  @override
  _HomePageState createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  late Ros ros;
  late Topic chatter;

  @override
  void initState() {
    ros = Ros(url: 'ws://127.0.0.1:9090');
    chatter = Topic(
        ros: ros,
        name: '/topic',
        type: "std_msgs/String",
        reconnectOnClose: true,
        queueLength: 10,
        queueSize: 10);
    super.initState();
  }

  void initConnection() async {
    ros.connect();
    await chatter.subscribe();
    setState(() {});
  }

  void destroyConnection() async {
    await chatter.unsubscribe();
    await ros.close();
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Roslib Subscriber Example'),
      ),
      body: StreamBuilder<Object>(
          stream: ros.statusStream,
          builder: (context, snapshot) {
            return Center(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.center,
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[
                  StreamBuilder<Map<String, dynamic>>(
                    stream: chatter.subscription,
                    builder: (BuildContext context2,
                        AsyncSnapshot<Map<String, dynamic>> snapshot2) {
                      if (snapshot2.hasData && snapshot2.data != null) {
                        return Text('${snapshot2.data!['msg']}');
                      } else {
                        return const CircularProgressIndicator();
                      }
                    },
                  ),
                  ActionChip(
                    label: Text(snapshot.data == Status.connected
                        ? 'DISCONNECT'
                        : 'CONNECT'),
                    backgroundColor: snapshot.data == Status.connected
                        ? Colors.green[300]
                        : Colors.grey[300],
                    onPressed: () {
                      // print(snapshot.data);
                      if (snapshot.data != Status.connected) {
                        initConnection();
                      } else {
                        destroyConnection();
                      }
                    },
                  ),
                ],
              ),
            );
          }),
    );
  }
}
