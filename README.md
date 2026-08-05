# İHA Gazebo Autonomy

Gazebo Jetty 10.5.0 + ArduPilot SITL için Python tabanlı, state-machine mimarili otonom görev yazılımıdır.

Dokümandaki ana akış:
1. 10 m takeoff
2. Kamera ile mavi altıgen / kırmızı üçgen hedef arama
3. Renk + şekil ile sınıflandırma
4. Hedef GPS konumunu kaydetme
5. Hedefe yaklaşma ve 1 m'de hassas merkezleme
6. Dummy payload release
7. İkinci hedef için aynı işlem
8. İlk hedef GPS konumuna dönüş
9. Ters renkli payload arama
10. Payload'a hassas iniş
11. Dummy pickup
12. 1 m yükselme ve kamera ile doğrulama
13. En fazla 5 pickup denemesi
14. Başarılı/başarısız durumda RTL
15. Görev 3 bitene kadar kod tarafından DISARM gönderilmez.

## Kurulum

Jetty'nin Python Gazebo Transport / Messages paketleri:

```bash
sudo apt update
sudo apt install python3-gz-transport15 python3-gz-msgs12
```

Python:

```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -r requirements.txt
```

`--system-site-packages`, Ubuntu paketlerinden gelen `gz.transport` ve `gz.msgs`
Python modüllerine erişmek için önerilir.

## Çalıştırma

Gazebo + ardupilot_gazebo + ArduPilot SITL çalışırken:

```bash
source .venv/bin/activate
python3 main.py
```

Varsayılan MAVLink bağlantısı:

```text
udp:127.0.0.1:14550
```

Değiştirmek için:

```bash
export MAVLINK_CONNECTION="udp:127.0.0.1:14550"
```

## Kamera

`config.py` içindeki `CAMERA_TOPIC` dokümandaki Gazebo topic'ine ayarlanmıştır.

Kamera FOV ve çözünürlüğü de gerçek SDF kamera parametreleriyle eşleştirilmelidir.
Pixel → metre hesabı bu değerleri kullanır.

## Test

```bash
pytest -q
```

Testler Gazebo veya MAVLink bağlantısı gerektirmez.

## Önemli entegrasyon notları

- `payload/payload_controller.py` gerçek servo yerine dummy/mock abstraction'dır.
- `main.py` tek uzun script değildir; görev state machine ile yönetilir.
- Kod hiçbir yerde otomatik DISARM çağrısı yapmaz.
- Hedef tek frame ile kabul edilmez; ardışık frame stabilitesi aranır.
- Hedef alçalma sırasında kaybolursa kör şekilde devam edilmez.
- İlk hedef ve ikinci hedef merkezi veri yapısında tutulur.
- Hedef konumları önceden world/SDF'den okunmaz; kamera tespitinden sonra mevcut
  MAVLink GPS konumu kaydedilir.
