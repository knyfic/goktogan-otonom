Haklısın, önce her şeyi açıklayayım. Kod yazmak yok bu cevapta.

---

## Sistemin şu anki çalışma şekli

Projen şu an **3 katmandan** oluşuyor:

```
State Machine  (görev mantığı)
     ↓
PayloadController  (ara katman — state machine bunu çağırıyor)
     ↓
Backend  (gerçek iş burada yapılıyor)
```

State machine hiç değişmedi. Hâlâ aynı şeyleri çağırıyor:
```python
payload.release_payload(color)     # yük bırak
payload.send_pickup_command(color) # yük almaya başla
payload.finish_pickup(color)       # yük alma tamamlandı
payload.clear_attached()           # sıfırla
```

Değişen tek şey: `PayloadController` artık bu işleri backend'e devrediyor. Hangi backend olduğunu bilmiyor.

---

## 3 Backend nedir, ne zaman hangisi kullanılır?

**1. SimulationBackend** — şu an kullandığın bu
- GPIO yok, gerçek donanım yok
- Gazebo + Ubuntu'da çalışır
- Yük bırakma/alma logda görünür, bellekte simüle edilir

**2. RemoteRPiBackend** — HIL (Hardware In the Loop) modu
- Ubuntu'da çalışır ama komutları Wi-Fi üzerinden Raspberry Pi'ye gönderir
- Raspberry Pi'de gerçek GPIO çalışır
- Drone uçmadan önce masaüstünde donanım testi için

**3. LocalRPiBackend** — gerçek uçuş
- Raspberry Pi üzerinde çalışır
- Network yok, direkt GPIO
- `raspberry-pi-5/` klasörü içindeki kod

---

## Simülasyonu nasıl çalıştıracaksın? (Şu an yaptığın şey)

`main.py`'e bak:

```python
BACKEND_MODE = "simulation"   # ← bu satır
```

Bu satır `"simulation"` olduğu sürece hiçbir şey değişmez. Gazebo'yu eskisi gibi başlatıyorsun:

```bash
# Terminal 1 — Gazebo
gz sim ...

# Terminal 2 — Mission
cd /home/ubuntu/Documents/4-2garip
python3 main.py
```

Log'da şunu görürsün:
```
INFO | Payload backend modu: simulation
INFO | [SIM] initialize: Magnet1=ON Magnet2=ON Stepper=SAFE
```

Yük bırakıldığında:
```
INFO | [SIM] release_payload_1: Stepper↓ → Magnet1 OFF → Stepper↑
```

Fark etmez, drone eskisi gibi uçar. SimulationBackend sadece log yazar.

---

## Drone'da (Raspberry Pi'de) nasıl çalıştıracaksın?

İki adım var:

### Adım 1 — Raspberry Pi'ye kodu kopyala

```bash
# Ubuntu terminalinde:
scp -r /home/ubuntu/Documents/4-2garip/raspberry-pi-5/ pi@<RPi_IP>:~/payload-service/
```

### Adım 2 — Raspberry Pi'de servisi başlat

```bash
# Raspberry Pi SSH:
cd ~/payload-service
pip install RPi.GPIO PyYAML
python3 main.py
```

Servis şu mesajı verir:
```
INFO | Payload TCP servisi başlıyor: 0.0.0.0:5700
```

Ve bekler. Komut almak için hazır.

### Adım 3 — Ubuntu'da HIL moduna geç

`main.py`'de sadece bu satırı değiştir:

```python
BACKEND_MODE = "hil"   # "simulation" → "hil"
```

Ve `payload/config.yaml`'da Raspberry Pi'nin IP'sini yaz:

```yaml
remote:
  host: 192.168.1.42   # RPi'nin gerçek IP'si
```

Mission'ı başlatınca artık:
```
Ubuntu → payload.release_payload() → Wi-Fi → RPi → GPIO → Mıknatıs
```

---

## Gerçek uçuşa geçince ne değişir?

Ubuntu yoktur. Raspberry Pi hem uçuş kodunu hem payload kodunu çalıştırır.

Şu an `raspberry-pi-5/main.py` sadece TCP server. Gerçek uçuşta buraya `state_machine.py`'yi de taşıman gerekir — ama bu ayrı bir konu, şimdi buna gerek yok.

Şu an için akış:
```
Ubuntu (state machine + Gazebo + mission) → HIL → RPi (GPIO)
```

Yarışmada büyük ihtimalle yine Ubuntu laptop'tan uçuracaksın:
```
Laptop (mission) → Wi-Fi → RPi (GPIO)
```

Bu zaten HIL modu, ekstra bir şey değiştirmene gerek yok.

---

## 21 commit var, ne yapmalısın?

Hepsini test etmen gerekmez. Şöyle düşün:

**Şu an uçuş simülasyonu etkilendi mi?** → Hayır. `BACKEND_MODE = "simulation"` ile her şey eskisi gibi çalışır.

**Ne zaman test edersin?**
1. Gazebo simülasyonunu çalıştır — eskisi gibi çalışıyorsa payload katmanı sorunsuz
2. Donanım geldiğinde HIL moduna geç

**Commitleri şöyle gruplayabilirsin:**
- Payload sistemi değişiklikleri (payload/ klasörü + main.py) → 1 commit
- Raspberry Pi servisi (raspberry-pi-5/ klasörü) → 1 commit
- Testler → 1 commit

---

## Hızlı sağlık kontrolü (şu an yapabilirsin)

```bash
cd /home/ubuntu/Documents/4-2garip
.venv/bin/python -m pytest tests/ -v
```

28 payload testi + 6 mevcut test = 34 test geçiyor. Bu yeterli.

Simülasyon testi için:
```bash
.venv/bin/python -c "
from payload.backends.simulation import SimulationBackend
b = SimulationBackend()
b.initialize()
print('Magnet 1:', b.status().magnets.magnet_1_on)   # True
print('Magnet 2:', b.status().magnets.magnet_2_on)   # True
b.release_payload_1()
print('Sonra Magnet 1:', b.status().magnets.magnet_1_on)  # False
print('Sonra Magnet 2:', b.status().magnets.magnet_2_on)  # True (dokunulmadı)
"
```





---


Hayır, yarışmada bu senaryo için tasarımı değiştirmem gerekiyor. Yanlış anladım.

---

## Gerçek durum

Yarışmada tüm kod İHA'nın üzerindeki Raspberry Pi 5'te çalışacak:

```
Raspberry Pi 5 (İHA üstünde)
  ├── State Machine
  ├── Mission kodu
  ├── Kamera
  ├── MAVLink / uçuş kontrolü
  └── GPIO → PCB → Stepper + Mıknatıslar
```

Ubuntu laptop yok, Wi-Fi yok, TCP yok. Her şey lokal.

---

## `payload/config.yaml` sorusu

**Hayır, o dosya önemli değil.** Şu an backend seçimi `main.py`'deki şu satırla yapılıyor:

```python
BACKEND_MODE = "simulation"   # ← tek kontrol noktası bu
```

`payload/config.yaml`'daki `backend: simulation` satırı şu an hiç okunmuyor bile, sadece dokümantasyon amaçlı orada duruyor. Kafanı karıştırmasın.

---

## Yarışma için ne değişmeli

Şu an sistem şöyle tasarlanmış:

```
Ubuntu → mission kodu → SimulationBackend (GPIO yok)
Ubuntu → mission kodu → RemoteRPiBackend → RPi GPIO (HIL)
```

Ama gerçek senaryo şu:

```
RPi → mission kodu → LocalRPiBackend → GPIO
```

Yani **mission kodunun da Raspberry Pi'de çalışması gerekiyor.** Şu an `raspberry-pi-5/` klasöründe sadece TCP servisi var, mission kodu yok.

**Yapılması gereken:** `raspberry-pi-5/main.py`'i, mevcut `main.py`'in yaptığını yapacak şekilde genişletmek — ama `LocalRPiBackend` ile. Şöyle:

```python
# raspberry-pi-5/main.py olacak hali
backend = LocalRPiBackend.from_config("config/payload.yaml")
payload = PayloadController(backend=backend, logger=logger)
mission = MissionStateMachine(navigation, camera, payload, logger)
mission.run()
```

Bunu yapmamı ister misin? Yoksa önce simülasyonu doğrulamak ve sonra bu adıma geçmek mi istiyorsun?

Önce mevcut simülasyonun çalışmasını onaylaman daha mantıklı. Sıra şöyle olabilir:

1. Simülasyonu test et (şu anki haliyle)
2. Donanım testini yap (HIL modu, RPi bağlı ama drone uçmuyor)  
3. Her şey çalışınca mission kodunu da RPi'ye taşı