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

> Kamerada renkler ters ise `payload/config.yaml → camera_server.swap_rb_channels`
> (RPi tarafı) veya `remote_camera.swap_rb_channels` (Ubuntu tarafı) değerlerini
> ayarlayın. Kamera montaj yönü için `orientation` kullanılır.

---

## REAL (Cube Orange + gerçek uçuş — aynı deployment)

HIL başarılıysa aynı repo Raspberry Pi'de kalır; yalnızca iki şey değişir:

```bash
# 1) payload/config.yaml → backend: real
# 2) config.py → MAVLINK_CONNECTION (Cube Orange serial, örn. "/dev/ttyACM0:115200")
python main.py
```

HIL'de çalışan `RPiCam3`, `LocalRPiBackend` ve `payload/hardware/*` kodları
REAL'de **aynen** kullanılır — fark yalnızca giriş noktası (sunucu yerine
`main.py`) ve MAVLink adresidir.

---

## Servis olarak çalıştırma (isteğe bağlı)

```bash
sudo cp deploy/iha-payload.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now iha-payload
```
