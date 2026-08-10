from dataclasses import dataclass

MAVLINK_CONNECTION = "udp:127.0.0.1:14550"

CAMERA_TOPIC = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"

GAZEBO_MSGS_MODULE = "gz.msgs"
GAZEBO_TRANSPORT_MODULE = "gz.transport"

TAKEOFF_ALTITUDE = 8.0
TARGET_APPROACH_ALTITUDE = 4.0

# Yük bırakma: görüntü işleme ile hizalanılan irtifa
PAYLOAD_RELEASE_ALTITUDE = 2.0
# Hizalama tamamlandıktan sonra sabit iniş ile yükün bırakılacağı irtifa
PAYLOAD_DROP_ALTITUDE = 1.0
# Görev 2: küçük yükler yüksekten görülmez — arama bu irtifada başlar
PAYLOAD_SEARCH_ALTITUDE = 2.0
# Her kademede: yatay ortala → bu kadar metre in → tekrar ortala ...
ALIGNMENT_DESCENT_STEP_M = 2.0

# Geriye dönük uyumluluk
FINAL_TARGET_ALTITUDE = PAYLOAD_RELEASE_ALTITUDE

# Yük alımı: İHA tam yere (0 m) inmeli çünkü mekanizma kısa
PAYLOAD_CONTACT_ALTITUDE = 0.0

# Yük doğrulama için yükselme irtifası
PAYLOAD_VERIFY_ALTITUDE = 1

# Validation irtifa toleransı: bu bantın dışındaki frame'ler sayılmaz.
# ALTITUDE_TOLERANCE_M (0.20m) yerine daha sıkı tutuyoruz.
VERIFY_ALT_TOLERANCE = 0.12   # metre

# Doğrulama irtifasına ulaştıktan sonra stabilizasyon için bekleme süresi (s)
VERIFY_STABILIZE_S = 1.0

# Referans area ile karşılaştırma — bant yaklaşımı.
# Pickup sonrası medyan area referansın bu bandında kalıyorsa payload hâlâ yerde.
# [ref * 0.55, ref * 1.65]: aynı nesnenin ±%45–65 noise/pozisyon varyasyonu normal.
# Band dışına çıkarsa ya da hiç görünmüyorsa → alındı.
VERIFY_BAND_LOW  = 0.55   # referans alanın alt sınırı (x ref)
VERIFY_BAND_HIGH = 1.65   # referans alanın üst sınırı (x ref)

# Verification temporal window: kaç ardışık frame'de "target kayboldu" görülmeli
# ki pickup başarılı sayılsın. False-positive'e karşı koruma.
VERIFY_ABSENT_FRAMES_REQUIRED = 4

# Signature benzerlik eşiği: area, width, height normalize farkı bu değerin
# altındaysa candidate "pre-pickup target ile aynı nesne" sayılır → payload hâlâ yerde.
VERIFY_SIMILARITY_THRESHOLD = 0.50  # %50 normalize fark toleransı

# Signature olmadan (fallback) verification'da kullanılacak confidence eşiği
VERIFY_FALLBACK_CONFIDENCE = 0.50

# Yük alımı hassas kademeli iniş adım büyüklüğü (metre)
# 10cm'lik yük için kamera çok yakından bakmalı — küçük adımlarla in
PAYLOAD_DESCENT_STEP_M = 0.4

# Hizalama döngüsü: pickup esnasında piksel tolerans eşiği
# Normal hizalamadan daha sıkı — yük çok küçük
PAYLOAD_PIXEL_TOLERANCE_PX = 15.0

# Her hizalama basamağında maksimum bekleme süresi (saniye).
# Bu süre dolunca zorla bir basamak aşağı inilir — daha yakında daha iyi görülür.
PAYLOAD_ALIGN_TIMEOUT_S = 4.0

# Görev 2 navigasyon sonrası hedef merkezleme zaman aşımı (sn)
ALIGN_ON_TARGET_TIMEOUT_S = 5

# Servo/mekanik mekanizmanın çalışması için bekleme süresi (saniye).
# Simulation modunda yapay bekleme için kullanılır.
# Gerçek uçuşta (HIL/real) bu değer stepper süresinden hesaplanır:
#   steps_for_travel × step_delay_s × 2 = 11429 × 0.001 × 2 ≈ 22.9s
# Buradaki değer yalnızca simulation backend için geçerlidir.
PAYLOAD_SERVO_WAIT_S = 1

FIELD_WIDTH = 30.0
FIELD_LENGTH = 100.0

HEXAGON_SIDE_LENGTH = 2.0
TRIANGLE_SIDE_LENGTH = 1.0

# Yarışma yükü: 10 cm × 10 cm kare. Boyut filtresi bu fiziksel ölçüye göre yapılır.
PAYLOAD_WIDTH = 0.10
PAYLOAD_HEIGHT = 0.10

MAX_PICKUP_ATTEMPTS = 5

# Payload arama: beklenen boyutun bu katını aşan tespitler payload değil hedeftir.
# Payload 10cm kare, hedefler metre ölçeğinde olduğundan 3x lineer tolerans güvenlidir.
MAX_PAYLOAD_SIZE_FACTOR = 3.0

SEARCH_TIMEOUT = 12.0
TARGET_LOST_TIMEOUT = 2.0
PAYLOAD_SEARCH_TIMEOUT = 20.0

CAMERA_WIDTH_PX = 1920
CAMERA_HEIGHT_PX = 1080
CAMERA_HORIZONTAL_FOV_DEG = 66.0
# 16:9 sensörde H-FOV=78° iken V-FOV = 2*atan(tan(78°/2) * 9/16) ≈ 46.4°
# 4:3 sensörde eski değer 60° idi; 16:9'a geçince V-FOV düşer.
CAMERA_VERTICAL_FOV_DEG = 41.0

# Kamera montaj orientation — görüntüyü pipeline başında normalize etmek için kullanın.
# Geçerli değerler: 'NORMAL', 'ROTATE_90', 'ROTATE_180', 'ROTATE_270', 'FLIP_H', 'FLIP_V'
# Varsayılan: 'NORMAL'
CAMERA_ORIENTATION = 'NORMAL'

# Gerçek kamera kaynağı — yalnızca backend: real modunda kullanılır.
# Gazebo / HIL modunda bu değer okunmaz.
# 0 = ilk USB kamera, "/dev/video0" veya RTSP URL de olabilir.
REAL_CAMERA_SOURCE = 0
REAL_CAMERA_FPS    = 30

MIN_TARGET_AREA_PX = 400.0
MIN_PAYLOAD_AREA_PX = 40.0
DETECTION_HISTORY_SIZE = 5
MIN_STABLE_DETECTIONS = 3
MIN_DETECTION_CONFIDENCE = 0.60

SCAN_LINE_SPACING = 8.0
SEARCH_MARGIN = 2.0

# Arama sırası yatay hız
SEARCH_SPEED_MPS = 3.0
# Hedefe yaklaşma hızı (kaba)
APPROACH_SPEED_MPS = 2.0
# Hizalama sırası yatay hız (ince)
ALIGN_SPEED_MPS = 0.8
# İrtifa değişim hızı
VERTICAL_SPEED_MPS = 1.0

POSITION_TOLERANCE_M = 0.35
ALTITUDE_TOLERANCE_M = 0.20
PIXEL_ALIGNMENT_TOLERANCE_PX = 30.0

# Hizalama komutu gönderme aralığı (saniye)
# Her step'te değil, bu aralıkta bir goto_body_offset gönderilir.
ALIGN_CMD_INTERVAL_S = 0.4

# GPS kalite biriktirme: hizalama döngüsünde "hizalı" anında GPS + confidence
# kaydedilir; en az bu kadar örnek birikmeden GPS kesinleşmez.
GPS_CANDIDATE_MIN_COUNT = 3

# Zaman bazlı hedef kilidi: ilk görülen hedef bu süre boyunca görüntede kalırsa
# 1. hedef olarak kilitlenir (saniye).
TARGET_LOCK_DURATION_S = 1.0

ENABLE_LIVE_PREVIEW = True
PREVIEW_WINDOW_NAME = "İHA Kamera"

CONTROL_RATE_HZ = 10.0
CAMERA_TIMEOUT = 3.0
STATE_TIMEOUT = 120.0
TAKEOFF_TIMEOUT = 45.0
RTL_TIMEOUT = 120.0

# Yarışma görevi maksimum süresi (saniye).
# Kalkış tamamlandığı anda sayaç başlar.
# Bu süre dolunca sistem hangi state'te olursa olsun RTL'ye geçer.
MISSION_TIMEOUT_S = 11 * 60  # 660 saniye
