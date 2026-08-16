# Raspberry Pi 5 Deployment

Bu klasör, Raspberry Pi 5 üzerinde çalıştırma için **ince bir launcher
klasörüdür** — donanım implementasyonu İÇERMEZ.

Tüm gerçek donanım kodu, tek kaynak (single source of truth) olarak ortak
paketlerdedir ve HIL ile REAL modlarında **birebir aynıdır**:

| Katman | Tek kaynak | HIL sunucusu | REAL uygulama |
|---|---|---|---|
| Payload backend | `payload/backends/local_rpi.py` | ✔ | ✔ |
| Stepper + mıknatıs | `payload/hardware/*` | ✔ | ✔ |
| Kamera (CSI Cam3) | `vision/rpi_camera.py` | ✔ | ✔ |
| Config | `payload/config.yaml` (tek dosya) | ✔ | ✔ |

---

## HIL (gerçek kamera + gerçek payload, Ubuntu/Gazebo ile)

Raspberry Pi üzerinde (repo klonlandıktan sonra):

```bash
# Bağımlılıklar (RPi'de bir kez)
pip install -r requirements.txt
sudo apt install python3-picamera2        # Raspbian'da çoğu zaman ön kurulu

# HIL sunucusunu başlat (payload 5700 + kamera 5701)
bash raspberry-pi-5/run_hil_server.sh
#   veya: python3 -m services.hil_entrypoint
```

Ubuntu üzerinde (aynı repo, ikinci makine):

```bash
# payload/config.yaml → backend: hil  (zaten varsayılan)
#                       hil_camera: remote_rpi (gerçek RPi kamerayı kullanır)
python main.py
```

> Kamera montaj yönü ters ise `payload/config.yaml → camera.orientation`
> değerini ayarlayın (örn. 'ROTATE_180') — HIL sunucu ve REAL aynı değeri
> kullanır (TEK kaynak). B↔R kanal takası sabittir —
> RPiCam3 içinde IMX708 için otomatik uygulanır; ayrıca ayar gerekmez.

---

## REAL (Cube Orange + gerçek uçuş — aynı deployment)

HIL başarılıysa aynı repo Raspberry Pi'de kalır; yalnızca config değişir:

```bash
# 1) payload/config.yaml → backend: real
# 2) Aynı dosyada flight_controller.connection → Cube Orange TELEM2 UART
#    (device: /dev/serial0, baudrate: 921600)
# 3) video_stream → ground station IP/port (GStreamer yayını)
python main.py
```

HIL'de çalışan `RPiCam3`, `LocalRPiBackend` ve `payload/hardware/*` kodları
REAL'de **aynen** kullanılır — fark yalnızca giriş noktası (`main.py`),
MAVLink adresi (`flight_controller.connection`) ve görüntünün GStreamer ile
ground station'a yayınlanmasıdır (REAL'de yerel GUI/pencere açılmaz).

### REAL CONFIGURATION (tek dosya: `payload/config.yaml`)

```yaml
backend: real        # simulation | hil | real

# Cube Orange TELEM2 → Raspberry Pi UART
flight_controller:
  connection:
    device: /dev/serial0
    baudrate: 921600
  heartbeat_timeout_s: 10.0

# Ground station'a GStreamer yayını
video_stream:
  enabled: true
  host: 192.168.1.100   # ← Ground Station IP (en sık değişen)
  port: 5000            # ← UDP port (alıcı komutuyla eşleşmeli)
  codec: h264
  bitrate: 2000000      # bps
  fps: 30

# Gerçek RPi Camera Module 3 — TEK fiziksel kamera (HIL + REAL ortak)
camera:
  orientation: 'ROTATE_180'   # fiziksel montaj yönü
  jpeg_quality: 80

# Kamera çözünürlük/FPS/FOV → config.py (geometriyle tutarlı TEK kaynak):
#   CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX, REAL_CAMERA_FPS, RPI_CAMERA_NUM,
#   CAMERA_HORIZONTAL_FOV_DEG, CAMERA_VERTICAL_FOV_DEG
# SIM/Gazebo kamera yönü → config.py → CAMERA_ORIENTATION ('NORMAL')
```

Değişiklik rehberi:

| Ne değişti? | Nereden? | Parametre |
|---|---|---|
| Ground Station IP | `payload/config.yaml` | `video_stream.host` |
| GStreamer portu | `payload/config.yaml` | `video_stream.port` |
| Video bitrate / FPS | `payload/config.yaml` | `video_stream.bitrate` / `video_stream.fps` |
| Video encode preset | `payload/config.yaml` | `video_stream.preset` (`ultrafast`…`veryfast` önerilir) |
| Cube Orange UART cihazı | `payload/config.yaml` | `flight_controller.connection.device` |
| UART baudrate | `payload/config.yaml` | `flight_controller.connection.baudrate` |
| UART flow control (RTS/CTS) | `payload/config.yaml` | `flight_controller.connection.flow_control` |
| Heartbeat timeout | `payload/config.yaml` | `flight_controller.heartbeat_timeout_s` |
| Kamera montaj yönü (REAL+HIL) | `payload/config.yaml` | `camera.orientation` |
| Kamera çözünürlük / FPS / CSI port | `config.py` | `CAMERA_WIDTH_PX` / `CAMERA_HEIGHT_PX` / `REAL_CAMERA_FPS` / `RPI_CAMERA_NUM` |
| 1m'de ek hizalama süresi | `config.py` | `PAYLOAD_DROP_ALIGN_TIMEOUT_S` (0 = kapalı, varsayılan 5 sn) |
| Görev irtifaları / hızlar / timeout'lar | `config.py` | örn. `TAKEOFF_ALTITUDE`, `MISSION_TIMEOUT_S`, `TAKEOFF_TIMEOUT`, `STATE_TIMEOUT`, `RTL_ARRIVAL_TIMEOUT_S` |

> IP/port/device/baudrate Python kodunda HARD-CODE DEĞİLDİR — hepsi
> `payload/config.yaml`'dan okunur. Aynı mantıksal parametre tek kaynaktadır.

### UART (Cube Orange TELEM2 ↔ Raspberry Pi)

```bash
sudo raspi-config    # Interface Options → Serial Port →
                     #   login shell: NO, serial hardware: YES
sudo usermod -aG dialout $USER   # serial cihazına erişim
sudo reboot
# Doğrulama:
ls -l /dev/serial0   # → GPIO UART symlink'i görünmeli
```

### GStreamer (ground station'a video yayını)

```bash
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good \
                 gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
                 python3-gi
```

Ground station (Ubuntu PC) alıcı komutu:

Ground station (Ubuntu PC) alıcı komutu:

```bash
gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
  ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

> Yayınlanan frame, görev ekranının (HUD) birebir aynısıdır: 960x540,
> ~1.5-2 Mbps, `tune=zerolatency` + leaky queue (encoder yavaşsa frame
> düşer, görev asla bloklanmaz). GStreamer'ın kopması uçuşu durdurmaz.

**Görüntü işleme + yayın TEST scripti (yalnızca yayın — uçuş komutu YOK):**

Görev akışından bağımsız, yalnızca görüntü işleyip yayın yapan script
(`vision_stream_only.py`, repo kökü):

```bash
# İHA 11.5m'de Mission Planner ile HOLD'da iken (ikinci terminal):
python3 vision_stream_only.py
```

- Projenin gerçek kamerayı (RPiCam3) ve mevcut tespit pipeline'ını
  (TargetDetector) kullanır; MAVİ ALTIGEN + KIRMIZI ÜÇGEN tespit eder ve
  tespit overlay + HUD bandını `payload/config.yaml → video_stream.host`
  adresine GStreamer ile yayınlar.
- **MAVLink AÇMAZ / hiçbir hareket komutu gönderemez** (flight/payload
  import etmez), pencere açmaz (headless). Ground station alıcı komutu
  yukarıdaki ile aynıdır.

**Görüntü kalitesi ayarı (tuning):**

| Sorun | Çözüm |
|---|---|
| Karıncalanma / bloklaşma (sahne değişince) | `video_stream.preset` → `faster`/`medium` veya `video_stream.bitrate` ↑ |
| Kare kaybı / donma | `video_stream.fps` ↓ (30→25) veya `video_stream.bitrate` ↑ |
| Ağ burst → bozulma | Kod içi sabit: `vbv-buf-capacity` (bitrate/2) bit patlamalarını sınırlar |
| Gecikme hissi (stale frame) | Kod içi sabit: `queue leaky=upstream` — canlı yayında EN YENİ frame kazanır |

### İlk donanım testi (GÜVENLİ — uçuş komutu yok)

Cube Orange bağlıyken sırayla yalnızca doğrula:

```text
MAVLink connection → heartbeat
→ GPS telemetry
→ altitude telemetry
→ flight mode telemetry
```

ARM / TAKEOFF / GOTO / RTL komutları ilk test aşamasına dahil EDİLMEZ.

> Not: ArduPilot telemetri stream'lerini (SRx_*) yalnızca GCS algıladığında
> başlatır. Testler ve ana uygulama bağlantı açılışında GCS heartbeat gönderir
> (uçuş komutu değil, MAVLink el sıkışması). Yine de yalnızca HEARTBEAT
> geliyorsa Mission Planner'dan `SR2_POSITION`, `SR2_EXTRA1`, `SR2_EXTRA2`
> değerlerinin 0 olmadığını ve `SERIAL2_PROTOCOL=2` olduğunu doğrula.

### ArduPilot failsafe (BİRİNCİL güvenlik katmanı)

Raspberry Pi tarafındaki heartbeat watchdog ikincildir. Cube Orange üzerinde
TELEM2/bağlantı kaybına karşı ArduPilot failsafe parametreleri ayarlanmalıdır
(örn. `FS_THR_ENABLE`, `FS_EKF_ACTION`, `FS_GCS_ENABLE` gibi ilgili
parametreler) — görev sırasında bağlantı koparsa FC kendi RTL/LAND davranışını
yürütür.

---

## Donanım testleri (Raspberry Pi 5 üzerinde)

Tüm testler **READ-ONLY / güvenli** başlar — hiçbiri ARM / TAKEOFF / GOTO /
LAND / RTL komutu göndermez. Ubuntu'da `pytest` çalıştırılırsa bu testler
otomatik SKIP edilir (`Raspberry Pi hardware required`).

Önerilen sıra: MAVLink → kamera → payload → video stream.

```bash
# 1) Tek komutla genel donanım durumu (read-only)
python tests/hardware/check_real_hardware.py

# 2) MAVLink READ-ONLY telemetri (heartbeat, GPS, altitude, mode, battery)
python tests/hardware/test_mavlink_connection.py --duration 10

# 3) Payload mekanizması — küçük güvenli hareket (~0.2 mm)
python tests/hardware/test_payload_hardware.py --steps 50
#    Tam pickup sekansı istiyorsan (dikkatli):
#    python tests/hardware/test_payload_hardware.py --full-cycle

# 4) Kamera + GStreamer — loopback alıcı ile FPS/drop/CPU ölçümü
python tests/hardware/test_video_stream.py --loopback --duration 10
#    Gerçek ground station'a yayın:
#    python tests/hardware/test_video_stream.py --duration 10
```

Her test `--no-...` bayraklarıyla bireysel kontrolü atlayabilir
(`check_real_hardware.py --no-mavlink` vb.).

---

## Servis olarak çalıştırma (isteğe bağlı)

```bash
sudo cp deploy/iha-payload.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now iha-payload
```

REAL görev için (`backend: real`):

```bash
sudo cp deploy/iha-real.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now iha-real

# Loglar:
journalctl -u iha-real -f
journalctl -u iha-real --since "10 minutes ago"
```
