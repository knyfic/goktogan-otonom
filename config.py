import math

# Simulation/HIL MAVLink adresi (Ubuntu → Gazebo/SITL UDP).
# REAL modda bu değer KULLANILMAZ — bağlantı payload/config.yaml →
# flight_controller.connection'dan gelir (Cube Orange TELEM2 serial).
MAVLINK_CONNECTION = "udp:127.0.0.1:14550"

CAMERA_TOPIC = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"

GAZEBO_MSGS_MODULE = "gz.msgs"
GAZEBO_TRANSPORT_MODULE = "gz.transport"

# Kalkış + HER İKİ hedef araması (kırmızı üçgen / mavi altıgen) bu irtifada yapılır.
# 11.5m'de kamera 30×100m alanı görür → her arama tek geçiş (single_pass) ile yapılır.
TAKEOFF_ALTITUDE = 9
TARGET_APPROACH_ALTITUDE = 8.0

# ── Yük bırakma — HEDEFE ÖZEL konfigürasyon ────────────────────────────────
# Görev kuralı (hedef/payload eşlemesi — payload_for_target ile birebir):
#   Payload 1 (MAVİ payload) → KIRMIZI ÜÇGEN
#   Payload 2 (KIRMIZI payload) → MAVİ ALTIGEN
#
# ORTAK akış (iki hedef için de):
#   görüntü tabanlı merkezleme → RED_PAYLOAD_VERIFY_ALTITUDE (3m) → OFFSET
#   doğrulaması → görüntü işlemesiz dik iniş → PAYLOAD_DROP_ALTITUDE (1m) → bırak.
# 3m/8m BIRAKMA irtifası DEĞİLDİR — yalnızca görüntü işlemenin bittiği irtifadır.
#
#   RED_PAYLOAD_VERIFY_ALTITUDE (3m): görüntü tabanlı merkezlemenin bittiği ve
#       offset doğrulamasının başladığı ORTAK irtifa (her iki hedef için).
#   BLUE_PAYLOAD_VERIFY_ALTITUDE (8m): mavi altıgen için ŞEKİL DOĞRULAMALI
#       detection/kilit aşamasının bittiği irtifa. 8m'den sonra şekil tekrar
#       doğrulanmaz; 8m'de kilitlenen hedefin MAVİ BÖLGESİ 3m'ye kadar takip
#       edilir (hedef büyüyüp kadrajdan taşsa bile kaybolmaz). Kırmızı üçgende
#       ayrı bir şekil-kilit irtifası yoktur — aynı 3m geçerlidir.
#   *_OFFSET_M: OFFSET doğrulama adımında İHA'nın hedef merkez doğrultusunda
#       ileri gideceği mesafe (mekanizma arkada → hedefin ortasına denk gelir).
# Kırmızı/mavi değerler BİRBİRİNDEN BAĞIMSIZDİR (saha kalibrasyonuna göre ayrı ayarlanır).
RED_PAYLOAD_VERIFY_ALTITUDE  = 3.0
RED_PAYLOAD_OFFSET_M         = 1.5
BLUE_PAYLOAD_VERIFY_ALTITUDE = 8.0
BLUE_PAYLOAD_OFFSET_M        = 1.8

# Hizalama + offset düzeltmesi sonrası DİK iniş ile inilen ve yükün bırakıldığı
# irtifa (her iki hedef için ortak — görüntü işleme bu inişte KULLANILMAZ).
PAYLOAD_DROP_ALTITUDE = 1.0
# 1m'de görüntüye göre EK hizalama denemesi (saniye). 0 → devre dışı:
# yeni akış 2.5m'de hizalama + offset düzeltmesi yapar, sonra dik iner ve bırakır.
PAYLOAD_DROP_ALIGN_TIMEOUT_S = 0.0
# Görev 2: küçük yükler yüksekten görülmez — arama bu irtifada başlar
PAYLOAD_SEARCH_ALTITUDE = 3.0
# Her kademede: yatay ortala → bu kadar metre in → tekrar ortala ...
ALIGNMENT_DESCENT_STEP_M = 2.0

# ── Geriye dönük uyumluluk ──────────────────────────────────────────────────
# Eski ORTAK değerler — yeni kod hedefe özel RED/BLUE_PAYLOAD_* değerlerini
# kullanır; bu takma adlar yalnızca eski referanslar/FINAL_TARGET_ALTITUDE için
# korunur. Yeni akışta bunlara bağlanma.
PAYLOAD_RELEASE_ALTITUDE = BLUE_PAYLOAD_VERIFY_ALTITUDE   # eski ortak verify (8m)
FINAL_TARGET_ALTITUDE = PAYLOAD_RELEASE_ALTITUDE

# Yük alımı: İHA tam yere (0 m) inmeli çünkü mekanizma kısa
PAYLOAD_CONTACT_ALTITUDE = 0.0

# Yük doğrulama için yükselme irtifası
PAYLOAD_VERIFY_ALTITUDE = 1

# Empirical pixel expectation for 1m altitude (measured in HIL tests).
# Use these to prefer payload candidates when at PAYLOAD_VERIFY_ALTITUDE.
# The user measured ~110..120 px width/height at 1m; use midpoint default 115px.
# Vision YARI çözünürlükte (960x352) çalıştığı için aynı fiziksel ölçümün
# piksel karşılığı da yarıya iner (~55..60 px → 57.5px).
PAYLOAD_EXPECTED_W_PX_AT_1M = 57.5
PAYLOAD_EXPECTED_H_PX_AT_1M = 57.5

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

# Pickup öncesi son yatay hizalama (kamera merkezleme) irtifası (metre).
# İHA bu irtifada payload'ı ortaladıktan sonra CAMERA_TO_MECHANISM_OFFSET_M
# kadar ileri gidip yere iner ve mekanizmayı aktive eder.
PAYLOAD_ALIGN_ALTITUDE_M = 0.4

# Payload alımı: bu irtifanın ALTINDA köşe/kenar tespitleri de geçerlidir.
# Alçak irtifada İHA payload'ı ortalayamayabilir ve payload köşelere/kenarlara
# kayabilir; bu yüzden normalde uygulanan %15 kenar marjı (köşe eleme) kaldırılır
# ve "mavi puan (color_purity) + alan" skoru EN YÜKSEK olan aday gerçek payload
# kabul edilip ona yönelinir. Bu seçim yalnızca bu eşiğin altında geçerlidir.
PAYLOAD_CORNER_PICK_ALT_M = 0.5

# Hizalama döngüsü: pickup esnasında piksel tolerans eşiği
# Normal hizalamadan daha sıkı — yük çok küçük
# Yarı çözünürlükte aynı fiziksel tolerans için değer de yarıya iner.
PAYLOAD_PIXEL_TOLERANCE_PX = 7.5

# Her hizalama basamağında maksimum bekleme süresi (saniye).
# Bu süre dolunca zorla bir basamak aşağı inilir — daha yakında daha iyi görülür.
PAYLOAD_ALIGN_TIMEOUT_S = 4.0

# Kamera ile görev mekanizması (mıknatıs/stepper) arasındaki boyuna offset (metre).
# Kamera İHA burnuna göre mekanizmanın 8cm önündedir; pickup öncesi İHA bu
# kadar ileri giderek mekanizmayı payload'ın tam üzerine getirir.
CAMERA_TO_MECHANISM_OFFSET_M = 0.16

# ── Payload alımı inişi — GUIDED yer-teması doğrulaması (LAND mode YOK) ──────
# ArduPilot LAND detektöründen uyarlanan, GUIDED altında çalışan yer teması
# algılama. İniş dikey HIZ komutuyla yapılır (ArduPilot LAND gibi) — altitude
# estimator hatasına bağışık: FC fiziksel zemine kadar inmeye devam eder,
# "estimator 0 gösterdi" diye havada durmaz. Temas, aşağıdaki koşulların
# GROUND_CONTACT_HOLD_S boyunca KESİNTİSİZ sağlanmasıyla doğrulanır (dwell /
# hysteresis — kısa süreli yanlış koşullar sayılmaz).
GROUND_CONTACT_DESCENT_SPEED_MPS = 0.3   # iniş dikey hızı (m/s, aşağı)
GROUND_CONTACT_MAX_ALT_M         = 0.30  # irtifa bu değerin ALTINDA olmalı
GROUND_CONTACT_MAX_VS_MPS        = 0.15  # |dikey hız| bu değerin ALTINDA olmalı
                                          # (ArduPilot descent_rate_low karşılığı)
GROUND_CONTACT_MAX_THROTTLE_PCT  = 20.0  # motor çıkışı (VFR_HUD) bu değerin ALTINDA —
                                          # yerdeyken motor yükü kaldıramaz
                                          # (ArduPilot motor_at_lower_limit karşılığı).
                                          # VFR_HUD yoksa/bayatsa koşul atlanır.
GROUND_CONTACT_MAX_TILT_DEG      = 8.0   # |roll|,|pitch| bu değerin ALTINDA (ATTITUDE) —
                                          # zemine oturmuş, sallanmıyor (ArduPilot
                                          # large_angle_request/error karşılığı).
                                          # ATTITUDE yoksa/bayatsa koşul atlanır.
GROUND_CONTACT_HOLD_S            = 1.5   # koşullar bu süre KESİNTİSİZ → LANDED_HOLD
                                          # (ArduPilot LAND_DETECTOR_TRIGGER_SEC ≈ 0.5s'ten
                                          # sıkı — payload teması için daha temkinli)
GROUND_CONTACT_CMD_INTERVAL_S    = 0.2   # iniş hızı komutu yenileme aralığı (5 Hz)
GROUND_CONTACT_TIMEOUT_S         = 20.0  # temas doğrulanamazsa güvenli RTL
# Temas doğrulandıktan sonra LANDED_HOLD bekleme süresi (pickup öncesi)
LANDED_HOLD_WAIT_S               = 1.0

# Yük BIRAKMA offseti — ARTIK HEDEFE ÖZEL: RED_PAYLOAD_OFFSET_M (kırmızı üçgen)
# ve BLUE_PAYLOAD_OFFSET_M (mavi altıgen) ayrı ayrı tanımlıdır (yukarıya bak).
# Bu takma ad yalnızca eski referanslar için korunur; bırakma akışı hedefe
# özel değeri kullanır.
DROP_OFFSET_M = RED_PAYLOAD_OFFSET_M

# Offset düzeltmesinde ileri hareketin tamamlanması için verilen
# maksimum süre (saniye). Bu süre dolunca alçalmaya geçilir.
OFFSET_CORRECTION_WAIT_S = 2.0

# Görev 2 navigasyon sonrası hedef merkezleme zaman aşımı (sn)
ALIGN_ON_TARGET_TIMEOUT_S = 5

# Yük alımı görüntü işleme fazı MİNİMUM süresi (saniye).
# İHA hedef şekle gelince merkezleme başlar; tek frame'de tesadüfi hizalı
# görünüm (jitter) İHA'yı daha kendini ortalamadan SEARCHING_PAYLOAD'a
# gönderiyordu → hedefin üzerinde olmadığı için payload'ı göremiyordu.
# Bu süre dolmadan (hem merkezleme hem payload hizalama fazlarında)
# "hazır / payload yok" kararı verilmez.
PAYLOAD_PICKUP_VISION_MIN_S = 3.0

# Payload alımı bekleme süresi (saniye) — STEP MOTOR KULLANILMIYOR.
# Görev mekanizması sabit durur; yükler mıknatıslarla alınır. İHA yere
# indikten sonra yükün tutulması için bu kadar beklenir.
# Simulation modunda (backend'ten süre gelmezse) bu değer kullanılır;
# real/HIL'de payload/config.yaml → pickup_wait_s (varsayılan 3.0s) geçerlidir.
PAYLOAD_SERVO_WAIT_S = 3

FIELD_WIDTH = 30.0
FIELD_LENGTH = 100.0

# ── Yarışma alanı (competition) GPS köşeleri ─────────────────────────────────
# 30×100 m dikdörtgen yarışma alanının dört GPS köşesi. Sıralama (kavramsal):
#   A = alt-sol, B = üst-sol, C = üst-sağ, D = alt-sağ
# CompetitionField (search/coverage_planner.py) bu köşelerden SABİT local frame
# kurar; İHA'nın launch konumu runtime GPS'ten alınır ve bu frame'e çevrilir.
# İHA alan dışında kalkarsa search başlatılmaz (RTL + hata logu).
#
# Değerler SITL home'una göre düzenlenmiştir: SITL --home 41.0082,28.9784
# alanın tam MERKEZİNE (local 15, 50) denk gelir. Gerçek yarışma alanına
# geçerken yalnızca bu dört değer güncellenir (kod değişmez).
# COMPETITION_CORNERS = {
#     "A": (41.0077508, 28.9782215),
#     "B": (41.0086492, 28.9782215),
#     "C": (41.0086492, 28.9785785),
#     "D": (41.0077508, 28.9785785),
# }
COMPETITION_CORNERS = {
    "A": (37.9336259, 34.6175672),
    "B": (37.9338936, 34.6175042),
    "C": (37.9339258, 34.6177221),
    "D": (37.9336640, 34.6177952),
}

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

# Kamera montaj orientation — YALNIZCA GAZEBO/SIM kamerası içindir.
# Gerçek RPi Camera Module 3'ün montaj yönü payload/config.yaml → camera.orientation
# altındadır (TEK kaynak — HIL sunucu + REAL ortak; fiziksel kamera ROTATE_180).
# Geçerli değerler: 'NORMAL', 'ROTATE_90', 'ROTATE_180', 'ROTATE_270', 'FLIP_H', 'FLIP_V'
# Varsayılan: 'NORMAL' (simülasyon/Gazebo)
CAMERA_ORIENTATION = 'NORMAL'

# Gerçek kamera — Raspberry Pi Camera Module 3 (CSI), Picamera2 tabanlı.
# REAL modun TEK gerçek kamera implementasyonu: vision/rpi_camera.py (RPiCam3).
# Gazebo / HIL modunda bu değerler okunmaz.
# B↔R kanal takası RPiCam3 içinde SABİT uygulanır (IMX708 — config değil).
# Kamera çözünürlük/FPS/FOV geometriye bağlıdır ve TEK kaynak burasıdır;
# payload/config.yaml → camera bölümü yalnızca donanım montajı içerir
# (orientation, jpeg_quality) — burada tekrar tanımlanmaz.
RPI_CAMERA_NUM = 0            # CSI port (0 = CAM0)
REAL_CAMERA_FPS    = 30

# ── Vision kırpma + YARI çözünürlük (payload mekanizması + performans) ───────
# Görev mekanizması kameranın DİKEY kadrajının alt kısmını kaplıyor. Kamera
# 1920x1080 üretmeye devam eder; kadrajın ÜSTTEN itibaren VISION_CROP_HEIGHT_PX
# satırı kırpılır (alt kısım — mekanizma — yok sayılır; FOV/geometri değişmez).
# Pipeline GÖRÜNTÜ İŞLEMEYİ YARI çözünürlükte yapar (1920x704 → 960x352 —
# piksel sayısı 1/4, işleme ~4x ucuzlar; vision 11fps'te kalmak yerine rahatlar).
# Ölçek (m/piksel) aynı kalır: içerik birebir korunur; yalnızca piksel boyutları
# yarıya iner. Bu yüzden piksel cinsinden FİZİKSEL sabitler (toleranslar, alan
# eşikleri, beklenen boyutlar) da aynı oranda ölçeklenir (aşağıdaki yorumlara bak).
VISION_CROP_HEIGHT_PX = 704        # kameradan kırpılan satır sayısı (alt — mekanizma)
VISION_DOWNSCALE       = 0.5       # pipeline frame küçültme oranı (2x → piksel 1/4)
VISION_WIDTH_PX        = int(CAMERA_WIDTH_PX * VISION_DOWNSCALE)              # 960
VISION_HEIGHT_PX       = int(VISION_CROP_HEIGHT_PX * VISION_DOWNSCALE)         # 352
# Vision'ın GÖRDÜĞÜ kadrajın dikey FOV'u: sensörün yalnızca 704/1080'i görüldüğü
# için m/piksel hesabında TAM sensör V-FOV'u değil, kırpılmış kadrajın V-FOV'u
# kullanılır (VISION_WIDTH/HEIGHT_PX ile birlikte eski (1920,1080) modeline
# birebir eşdeğer sonuç verir — ölçek gerçekten değişmez).
VISION_VERTICAL_FOV_DEG = 2.0 * math.degrees(math.atan(
    math.tan(math.radians(CAMERA_VERTICAL_FOV_DEG) / 2.0)
    * VISION_CROP_HEIGHT_PX / CAMERA_HEIGHT_PX
))

# Fiziksel boyut filtreleri (px²) — yarı çözünürlükte alan 2/4'e iner;
# aynı fiziksel eşik için değerler de 2/4'e ölçeklenir.
MIN_TARGET_AREA_PX = 101.0
MIN_PAYLOAD_AREA_PX = 10.0
DETECTION_HISTORY_SIZE = 5
MIN_STABLE_DETECTIONS = 3
MIN_DETECTION_CONFIDENCE = 0.60

# Detektör seviyesinde B↔R kanal takası (yalnızca acil durum ağı).
# Varsayılan KAPALI: kanal takasının sahibi kamera katmanıdır
# (RPiCam3 — IMX708 için sabit B↔R takası).
# Kameradan BGR doğru geliyorsa (önizlemede renkler doğruysa) bu bayrak
# False kalmalıdır; True yapılırsa TargetDetector kendi V3 metrik
# kalibrasyonuyla takas kararı verir (gürültülü sahnede yanlış tetiklenebilir).
TARGET_DETECTOR_AUTO_SWAP = False

SCAN_LINE_SPACING = 10
SEARCH_MARGIN = 2.0

# Arama sırası yatay hız
SEARCH_SPEED_MPS = 0.5
# Hedefe yaklaşma hızı (kaba)
APPROACH_SPEED_MPS = 0.5
# Hizalama sırası yatay hız (ince)
ALIGN_SPEED_MPS = 0.5
# İrtifa değişim hızı
VERTICAL_SPEED_MPS = 0.5
# Hedef kaba GPS'ine bu yarıçaptan UZAKTAYKEN APPROACH_SPEED_MPS kullanılır;
# yarıçapa girince ALIGN_SPEED_MPS'ye düşülüp görüntü hizalaması başlar.
# (Eski davranış: tüm hedef transit'i 0.8 m/s'de sürünüyordu — bu kapı
# "uzak=APPROACH / yakın=ALIGN" iki aşamalı tasarımı gerçekleştirir.)
APPROACH_RADIUS_M = 5.0

POSITION_TOLERANCE_M = 0.35
ALTITUDE_TOLERANCE_M = 0.20
# Hizalama piksel toleransı. 2.5m'de hedef büyük olduğundan sıkı tolerans (30px)
# centroid jitter'ı yüzünden uzun süren ortalama üretiyor → 60px yeterli (hedef
# 4m çap; bırakma hedefin içinde kalır). Payload alımı kendi sıkı toleransını
# (PAYLOAD_PIXEL_TOLERANCE_PX) kullanmaya devam eder.
# Yarı çözünürlükte aynı FİZİKSEL tolerans için değer yarıya iner (60 → 30px;
# aksi halde aşağıdaki yorumda "jitter üretiyor" denilen rejime geri dönülürdü).
PIXEL_ALIGNMENT_TOLERANCE_PX = 30.0

# Hizalama komutu gönderme aralığı (saniye)
# Her step'te değil, bu aralıkta bir goto_global gönderilir.
ALIGN_CMD_INTERVAL_S = 0.4

# Hizalama komut ölçeği (gain) — piksel hatasının NE KADARI metre düzeltmesine
# çevrilir. 0.8: hızlı yakınsama (uzun süreli hassas ortalama gerekmiyor);
# hedef görüntüyü doldurduğunda 'hedef üzerinde' kabul edilip inişe geçilir.
ALIGN_GAIN = 0.8

# ── Uyarlanabilir hizalama kazancı (yüksek irtifa salınımını önleme) ─────────
# 11.5m'de aynı piksel hatası çok daha büyük bir fiziksel adıma karşılık gelir;
# İHA komut aralığında (ALIGN_CMD_INTERVAL_S) adımı bitiremezse hareket halinde
# alınan yeni ölçümle hedef aşılır → sağ-sol salınımı. Bu davranış YALNIZCA
# 8m ÜSTÜNDE devreye girer; 8m ve altında ESKİ/stabil alignment hesabı birebir
# korunur (sabit ALIGN_GAIN, adım sınırı yok).
#   0) Davranış modu eşikleri: 8m ve altında eski hesap; 8–9m arasında iki hesap
#      ağırlıkla KARIŞTIRILIR (yumuşak geçiş — ani davranış sıçraması olmaz);
#      9m ve üstünde tam adaptive hesap.
ALIGN_ALT_MODE_LO_M = 9.0
ALIGN_ALT_MODE_HI_M = 10.0
#   1) Hata oranı: piksel hatası bu eşiğin ÜZERİNDEYSE kazanç tam ALIGN_GAIN'dir;
#      altına indikçe kazanç (hata/eşik) oranında küçülür → hedefe yaklaştıkça
#      adımlar otomatik küçülür.
#      Yarı çözünürlükte aynı FİZİKSEL hata piksel cinsinden yarıya iner;
#      davranışın birebir aynı kalması için eşik de yarıya iner (150 → 75px).
ALIGN_GAIN_FULL_ERROR_PX = 75.0
#   2) İrtifa: bu irtifanın ALTINDA kazanç tam ALIGN_GAIN'dir; ÜSTÜNDE kazanç
#      (ref_alt / irtifa) oranında küçülür — yüksek irtifada daha küçük adım.
ALIGN_ALT_GAIN_REF_M = 3.0
#   3) Adım sınırı: tek düzeltme komutunda hedefe en fazla bu kadar metre
#      yaklaşılır (komut aralığında tamamlanabilecek fiziksel adım).
ALIGN_MAX_STEP_M = 0.5
# Adım sınırı PİKSEL HATAYA GÖRE UYARLANIR (adaptive step): uzak hedef için
# büyük adım, hedefe yaklaştıkça otomatik küçülen adım. Sınır, hatanın
# ALIGN_STEP_FULL_ERROR_PX'e oranıyla ALIGN_MAX_STEP_M (0.25m) →
# ALIGN_MAX_STEP_LARGE_M (0.5m) arasında doğrusal büyür; eşiğin üstünde
# doygunluğa ulaşır. Böylece: büyük hata → ~0.4-0.5m, orta hata → ~0.25-0.35m,
# küçük hata → gain zaten küçük adım ürettiği için sınırın altında kalır.
ALIGN_MAX_STEP_LARGE_M = 1
# Aynı gerekçeyle (yarı çözünürlük → yarı piksel hata) eşik de yarıya iner
# (250 → 125px) — fiziksel adım davranışı değişmez.
ALIGN_STEP_FULL_ERROR_PX = 180.0

# ── Alignment KABUL eşikleri (ACCEPTANCE — hareket hızları DEĞİŞMEZ) ────────
# "Ortalandım" kararı irtifaya göre uyarlanır: yüksek irtifada hedef küçüktür ve
# rüzgâr / küçük pitch-yaw kamera jitter'ı centroid'i birkaç/onlarca piksel
# oynatabilir; sabit sıkı tolerans bu jitter'ı "hizasız" sanıp confirmation'ı
# sürekli sıfırlar → iniş hiç başlamaz.
#   irtifa ≤ ALIGN_TOL_ALT_LO_M → ALIGN_TOL_LO_PX (alçakta HASSAS — mevcut 30px)
#   irtifa ≥ ALIGN_TOL_ALT_HI_M → ALIGN_TOL_HI_PX (yüksekte jitter'a toleranslı)
#   arada → doğrusal geçiş
# Bu SADECE "hizalı" KABUL kararını etkiler; gain/adım/hareket mantığına
# DOKUNMAZ (PositionController.aligned_tolerance_px — tek kullanım noktası).
ALIGN_TOL_ALT_LO_M = 3.0            # RED_PAYLOAD_VERIFY_ALTITUDE — bırakma öncesi
ALIGN_TOL_ALT_HI_M = 8.0            # TARGET_APPROACH_ALTITUDE — yüksek irtifa hizalaması
ALIGN_TOL_LO_PX    = PIXEL_ALIGNMENT_TOLERANCE_PX   # 30.0 (geriye dönük uyum)
ALIGN_TOL_HI_PX    = 50.0

# Confirmation hysteresis (jitter dayanıklılığı): bir kez "hizalı" sayıldıktan
# sonra, tek bir frame'in ENTER toleransı dışı ama EXIT toleransı içindeki
# küçük oynaması sayacı SIFIRLAMAZ (artmaz da) — yalnızca EXIT dışı gerçek
# sapma sıfırlar. Böylece ALIGNED ↔ NOT_ALIGNED flip-flop'u önlenir; gerçek
# sapmada sayaç yine de sıfırlanır (yanlış alignment kabul edilmez).
ALIGN_TOL_EXIT_FACTOR = 1.6         # exit_tol = enter_tol * bu çarpan

# "Hizalı" kararı için GEREKLİ ardışık frame sayısı (temporal confirmation).
# Tek bir frame'de hedefin milisaniyelik merkeze denk gelmesi (jitter / tesadüfi
# geçiş) İHA'yı yanlış "ortaladım" sonucuna götürmesin diye, hedefin merkezde
# ARDIŞIK bu kadar frame kalması şart koşulur; araya hizasız frame girerse sayaç
# sıfırlanır. ALIGN_CMD_INTERVAL_S (0.4s) aralığında sayıldığından 3 onay ≈ 1.2s
# stabil merkezde kalma demektir.
ALIGNED_FRAMES_REQUIRED = 3

# YÜK BIRAKMA hedef hizalaması için ayrı ve DAHA SIKI ardışık onay sayısı.
# Search irtifasında (8m) hedef küçük ve yalpa/yaw nedeniyle çerçevede kayabilir;
# şansa merkeze denk gelen birkaç frame "hizalı" sanılıp erken irtifa düşüşü
# (hedef kaybolur → boşluğa bırakma) üretiyordu. 8 onay ≈ 3.2s (0.4s aralık)
# ARDIŞIK merkezde kalma ister; araya hizasız frame girerse sayaç sıfırlanır.
TARGET_ALIGNED_FRAMES_REQUIRED = 2

# GPS kalite biriktirme: hizalama döngüsünde "hizalı" anında GPS + confidence
# kaydedilir; en az bu kadar örnek birikmeden GPS kesinleşmez.
GPS_CANDIDATE_MIN_COUNT = 3

# Zaman bazlı hedef kilidi: ilk görülen hedef bu süre boyunca görüntede kalırsa
# 1. hedef olarak kilitlenir (saniye).
TARGET_LOCK_DURATION_S = 1.0

ENABLE_LIVE_PREVIEW = True
PREVIEW_WINDOW_NAME = "İHA Kamera"

# Canlı önizleme penceresi gösterim genişliği (piksel).
# Görüntü işleme pipeline YARI çözünürlükte (960x352) yapılır; YALNIZCA pencere
# gösterimi bu genişliğe küçültülür → imshow/HUD maliyeti ekstra ~4x azalır.
PREVIEW_DISPLAY_WIDTH_PX = 960

# ── Vision kayıt (vision_record_only.py — debug recorder, SD kart) ───────────
# Yalnızca recorder tarafından kullanılır; görev akışı bu değerleri okumaz.
VISION_RECORDING_DIR = "recordings"      # proje root'una göre (çalışma konumundan bağımsız)
# Kayıt hedef kalitesi (bps) — canlı stream'den (2 Mbps) yüksek debug kaydı.
# cv2.VideoWriter bitrate'i doğrudan ayarlamaz; gerçek çıktı dosya boyutundan
# ölçülüp kayıt sonunda loglanır (960x352 avc1 ile ölçülen tipik ~5 Mbps).
VISION_RECORDING_BITRATE = 10_000_000
# Kayıt çözünürlüğü: pipeline yarı çözünürlükte (960x352) çalışır; kayıt
# genişliği de pipeline genişliğiyle aynıdır (960 → frame ölçeklenmez).
VISION_RECORDING_WIDTH_PX = 960

# RTL sırasında önizleme güncelleme hızı (Hz).
# 4 Hz çok seyrek (pencere takılıyor); 15 Hz akıcı ve küçültülmüş
# gösterimle CPU/GPU için ucuzdur.
RTL_PREVIEW_HZ = 15.0

# Tek bir state işlemi için genel zaman aşımı (state_machine —
# örn. ilk hedefe navigasyon beklemesi)
STATE_TIMEOUT = 120.0
# Kalkış irtifasına ulaşma zaman aşımı (state_machine _do_takeoff)
TAKEOFF_TIMEOUT = 45.0
# RTL: home'a varış ve disarm bekleme süreleri (state_machine _do_wait_rtl)
RTL_ARRIVAL_TIMEOUT_S = 300.0   # home'a varış bekleme (5 dk)
RTL_DISARM_TIMEOUT_S  = 300.0   # disarm bekleme (ek 5 dk)

# Yarışma görevi maksimum süresi (saniye).
# Kalkış tamamlandığı anda sayaç başlar.
# Bu süre dolunca sistem hangi state'te olursa olsun RTL'ye geçer.
MISSION_TIMEOUT_S = 11 * 60  # 660 saniye

# ── Pilot takeover (RC flight mode değişikliği) — GÜVENLİK ───────────────────
# Mission yalnızca GUIDED modda uçar. Pilot RC'den başka bir mod seçerse
# (LAND/LOITER/RTL/STABILIZE/ALT_HOLD/...) mission PILOT_CONTROL state'inde
# durur ve Python artık hiçbir uçuş komutu göndermez; ArduPilot pilotun
# seçtiği modda kalır (pilot her zaman önceliklidir).
# Bu süre içinde tazelenen HEARTBEAT mod bilgisi geçerli sayılır; bayat mod
# bilgisiyle takeover kararı verilmez (yanlış pozitif riski).
PILOT_TAKEOVER_MODE_STALE_S = 5.0
