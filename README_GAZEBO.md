# Gazebo Sim / ArduPilot SITL kullanımı

Bu sürümde görev algoritmasına dokunulmadı. Yalnızca kamera kaynağı OpenCV `VideoCapture` yerine Gazebo Transport `gz.msgs.Image` topic'ine çevrildi.

## Gazebo Jetty bağımlılıkları

Jetty için Gazebo Transport 15.x ve Gazebo Msgs 12.x kullanılır. Ubuntu 24.04 sisteminde Gazebo'nun kurulu olduğu ortamda Python binding'lerinin erişilebilir olması gerekir.

Önce kontrol:

```bash
python3 -c 'from gz.transport import Node; print("gz.transport OK")'
python3 -c 'from gz.msgs.image_pb2 import Image; print("gz.msgs.Image OK")'
```

Kamera topic'ini kontrol et:

```bash
gz topic -l | grep '/sensor/camera/image'
```

Beklenen topic:

```text
/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

SITL çalışırken:

```bash
python3 iha_test1_gazebo.py
```

ArduPilot SITL bağlantısı mevcut kodda olduğu gibi:

```text
tcp:127.0.0.1:5760
```

olarak bırakıldı.
