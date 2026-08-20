# Simulation/HIL MAVLink adresi (Ubuntu → Gazebo/SITL UDP).
# REAL modda bu değer KULLANILMAZ — bağlantı payload/config.yaml →
# flight_controller.connection'dan gelir (Cube Orange TELEM2 serial).
MAVLINK_CONNECTION = "udp:127.0.0.1:14550"

CAMERA_TOPIC = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"

GAZEBO_MSGS_MODULE = "gz.msgs"
GAZEBO_TRANSPORT_MODULE = "gz.transport"

# Kalkış + HER İKİ hedef araması (kırmızı üçgen / mavi altıgen) bu irtifada yapılır.
# 11.5m'de kamera 30×100m alanı görür → her arama tek geçiş (single_pass) ile yapılır.
TAKEOFF_ALTITUDE = 11.5
TARGET_APPROACH_ALTITUDE = 4.0

# Yük bırakma: görüntü işleme ile HASSAS hizalanılan irtifa.
# Kadraj kırpma sonrası (1920x704) görüş daraldığı için 1m gibi alçak irtifada
# hizalama yapılamıyor → son hizalama 2.5m'de yapılır.
PAYLOAD_RELEASE_ALTITUDE = 3
# Hizalama + offset düzeltmesi sonrası DİK iniş ile inilen ve yükün bırakıldığı irtifa
PAYLOAD_DROP_ALTITUDE = 1.0
# 1m'de görüntüye göre EK hizalama denemesi (saniye). 0 → devre dışı:
# yeni akış 2.5m'de hizalama + offset düzeltmesi yapar, sonra dik iner ve bırakır.
PAYLOAD_DROP_ALIGN_TIMEOUT_S = 0.0
# Görev 2: küçük yükler yüksekten görülmez — arama bu irtifada başlar
PAYLOAD_SEARCH_ALTITUDE = 3.0
# Her kademede: yatay ortala → bu kadar metre in → tekrar ortala ...
ALIGNMENT_DESCENT_STEP_M = 1.0

# Geriye dönük uyumluluk
FINAL_TARGET_ALTITUDE = PAYLOAD_RELEASE_ALTITUDE

# Yük alımı: İHA tam yere (0 m) inmeli çünkü mekanizma kısa
PAYLOAD_CONTACT_ALTITUDE = 0.0

# Yük doğrulama için yükselme irtifası
PAYLOAD_VERIFY_ALTITUDE = 1

# Empirical pixel expectation for 1m altitude (measured in HIL tests).
# Use these to prefer payload candidates when at PAYLOAD_VERIFY_ALTITUDE.
# The user measured ~110..120 px width/height at 1m; use midpoint default 115px.
PAYLOAD_EXPECTED_W_PX_AT_1M = 115.0
PAYLOAD_EXPECTED_H_PX_AT_1M = 115.0

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
PAYLOAD_PIXEL_TOLERANCE_PX = 15.0

# Her hizalama basamağında maksimum bekleme süresi (saniye).
# Bu süre dolunca zorla bir basamak aşağı inilir — daha yakında daha iyi görülür.
PAYLOAD_ALIGN_TIMEOUT_S = 4.0

# Kamera ile görev mekanizması (mıknatıs/stepper) arasındaki boyuna offset (metre).
# Kamera İHA burnuna göre mekanizmanın 8cm önündedir; pickup öncesi İHA bu
# kadar ileri giderek mekanizmayı payload'ın tam üzerine getirir.
CAMERA_TO_MECHANISM_OFFSET_M = 0.16

# Yük BIRAKMA offseti (metre): 2.5m'de (PAYLOAD_RELEASE_ALTITUDE) görüntü
# hizalaması bittikten sonra İHA, hedef merkez doğrultusu boyunca bu kadar
# İLERİ gider (görev mekanizması arkada → hedefin tam ortasına denk gelir).
# Bu adımda görüntü işleme kapalıdır; config'ten istenildiği gibi değiştirilir.
DROP_OFFSET_M = 1.5

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
COMPETITION_CORNERS = {
    "A": (41.0077508, 28.9782215),
    "B": (41.0086492, 28.9782215),
    "C": (41.0086492, 28.9785785),
    "D": (41.0077508, 28.9785785),
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
# Gerçek kamera FPS hedefi (IMX708 / Camera Module 3 — 1920x1080'de 10-bit mod
# max ~56 FPS destekler, dolayısıyla 50 FPS güvenli ve desteklenen değerdir).
# RPiCam3 bu değeri FrameDurationLimits = 20.000 µs/frame ile SABİTLER.
REAL_CAMERA_FPS    = 50

# ── Vision kırpma (payload mekanizması) ─────────────────────────────────────
# Görev mekanizması kameranın DİKEY kadrajının alt kısmını kaplıyor. Bu değer,
# kadrajın ÜSTTEN itibaren kaç pikselinin vision pipeline'a verileceğidir:
#   704 → vision 1920x704 kullanır (alt 376 px — mekanizma — yok sayılır),
#   kamera yine 1920x1080 üretmeye devam eder (FOV/geometri değişmez).
VISION_HEIGHT_PX = 704   # px — vision'a verilen frame yüksekliği (tek ayar)

MIN_TARGET_AREA_PX = 400.0
MIN_PAYLOAD_AREA_PX = 40.0
DETECTION_HISTORY_SIZE = 5
MIN_STABLE_DETECTIONS = 3
MIN_DETECTION_CONFIDENCE = 0.60

# Detection ÇALIŞMA çözünürlüğü: giriş frame'i (1920x704) bu genişliğe ölçeklenir
# (1920 → 960 = 4x daha az piksel; LAB-CLAHE'siz pipeline ile ~2-3x hız kazancı).
# Detection koordinatları her zaman ORİJİNAL frame çözünürlüğüne geri ölçeklenir;
# state machine / align / meters_per_pixel hesabı (1920 bazlı) değişmez.
VISION_DETECT_WIDTH_PX = 960

# ── Görüntü işleme pipeline konfigürasyonu (sıfır magic number) ──────────────
# Tüm eşikler, toleranslar, HSV değerleri, ağırlıklar ve sınırlar buradan gelir.
# <...> olan değerler saha kalibrasyonuyla netleştirilecek başlangıç değerleridir.

# Kamera geometrisi — GSD (Ground Sampling Distance). GSD irtifa ile doğrusal:
#   GSD(h) = gsd_ref_m_per_px * (h / gsd_ref_altitude_m)
# 11.5 m'de yatay görüş ≈ 14.93 m → GSD ≈ 0.01555 m/px (saha ölçümüyle doğrulandı).
# NOT: eski 30 m / 960 px varsayımı KULLANILMAZ.
VISION_CAMERA_GEOMETRY = {
    "resolution": [960, 352],          # pipeline çalışma çözünürlüğü
    "hfov_deg": 66.0,
    "vfov_deg": 41.0,
    "gsd_ref_m_per_px": 0.01555,
    "gsd_ref_altitude_m": 11.5,
}

# Hedef tipleri — her hedef bağımsız yapılandırılır. HSV aralıkları
# [H_lo, H_hi, S_min, V_min]; kırmızı için iki aralık (wrap-around) OR'lanır.
VISION_TARGETS = {
    "RED_TRIANGLE": {
        "color": "RED",
        "shape": "TRIANGLE",
        "real_edge_m": 1.0,
        "hsv_ranges": [[0, 10, 60, 50], [160, 179, 60, 50]],
        "circularity_range": [0.52, 0.72],  # üçgen circ ≈ 0.60 (pikselleşmiş ölçüm 0.55); blur'da düşer/artar
        "solidity_min": 0.90,
        "vertex_count_range": [3, 3],     # EŞKENAR üçgen 3 köşelidir; 4+ köşe (yamuk/kare) ASLA üçgen değil
        "max_extent": 0.85,          # üçgen extent ≈ 0.50 (blur'da ~0.60) — üstü elenir
        "angle_target_deg": 60.0,    # eşkenar üçgen iç açısı
        "min_edge_equality": 0.80,   # EŞKENARLIK: kenar uzunlukları eşitliği. Gerçek eşkenar ≥0.94
                                        # (jagged/jitter'da 0.94-0.99); ikizkenar üçgen ≤0.74 → sert elenir
        "max_angle_deviation_deg": 15.0,  # köşe açıları 60°'ye yakın olmalı (gerçek ≤7°; ikizkenar ≥21°)
        "area_tolerance": [0.60, 1.40],
    },
    "BLUE_HEXAGON": {
        "color": "BLUE",
        "shape": "HEXAGON",
        "real_edge_m": 2.0,
        "hsv_ranges": [[90, 130, 60, 50]],
        "circularity_range": [0.80, 0.92],
        "solidity_min": 0.90,
        "vertex_count_range": [5, 7],   # 8+ köşe = eğri kontur; 4- köşe = altıgen değil (sert)
        "max_extent": 0.92,          # altıgen extent ≈ 0.75-0.83; kare 1.0 → ELENİR
        "angle_target_deg": 120.0,   # düzgün altıgen iç açısı
        "min_edge_equality": 0.45,   # sanite tabanı: 6 kenar eşitliği. Asıl eşkenarlık kanıtı AÇI kapısıdır
                                        # (gerçek jagged altıgen edgeEq 0.57'ye düşebilir; bozuk decoy ≥41° açı sapmasıyla elenir)
        "max_angle_deviation_deg": 20.0,  # köşe açıları 120°'ye yakın olmalı (gerçek ≤17°; bozuk şekil ≥41°)
        "area_tolerance": [0.60, 1.40],
    },
}

# Morphology — renk maskesi temizliği (kernel tek boyut; elips kernel).
VISION_MORPHOLOGY = {
    "kernel_size": 5,
    "open_iterations": 1,
    "close_iterations": 1,
}

# Şekil skorlaması: circ + solidity + vertex + angle ağırlıklı birleşim [0,1].
# Hiçbir TEK özellik tek başına karar veremez. Köşe AÇILARI decoy ayrımında
# kritiktir (üçgen 60°, kare 90°, altıgen 120°).
VISION_SHAPE_SCORING = {
    "circularity_weight": 0.25,
    "solidity_weight": 0.20,
    "vertex_weight": 0.25,
    "angle_weight": 0.30,
    "angle_tolerance_deg": 45.0,   # hedef açıdan sapma toleransı (açı skoru)
    "collinear_merge_deg": 20.0,   # iç açısı > 180°−20° (=160°) olan köşeler doğrusal kabul edilir
                                    # ve birleştirilir (blur/jagged maske gürültü köşeleri)
    "min_shape_score": 0.55,
}

# KARE DECOY modeli — gerçek hedefleri kandırmak için görev alanına konan
# kareler (mavi kare 2m / kırmızı kare 1m). Bu model YALNIZCA rakip skoru
# üretir: pipeline her aday için target_score vs square_score karşılaştırır
# (VISION_DECOY_REJECTION).
VISION_SQUARE = {
    "circularity_range": [0.72, 0.82],   # kare circ ≈ 0.785 (DAR — altıgen 0.85 girmesin)
    "solidity_min": 0.90,
    "vertex_count_range": [3, 6],        # sert sınır (analyze için)
    "vertex_score_map": {4: 1.0, 3: 0.5, 5: 0.2, 6: 0.0},  # kareye özgü köşe skoru
    "max_extent": 1.05,                  # kare extent ≈ 1.0 → skorlanabilmeli
    "minimum_score": 0.45,
    "min_edge_equality": 0.50,           # kare 4 eşit kenar (dikdörtgen elensin)
    "angle_target_deg": 90.0,            # kare iç açısı
    "angle_tolerance_deg": 35.0,         # DAR — altıgenin 120° köşeleri uzak kalır
    "max_angle_deviation_deg": 25.0,     # sert açı kapısı: 120°'lik altıgen köşeleri kare analyze'den geçemez
    "aspect_target": 1.0,
    "aspect_tolerance": 0.30,            # axis-aligned bbox aspect sapması
    "min_rect_aspect_tolerance": 0.35,   # minAreaRect aspect sapması (rotated kare sağlam)
    "weights": {
        "vertex": 0.15,
        "circularity": 0.20,
        "solidity": 0.10,
        "angle": 0.25,
        "aspect": 0.15,
        "min_rect_aspect": 0.15,
    },
}

# Decoy reddi — target skor ile square skoru karşılaştırması.
# Kabul şartı:
#   target_score >= minimum_target_score
#   AND target_score - square_score >= minimum_margin
# Fark minimum_margin ile uncertain_margin arasındaysa UNCERTAIN (reject —
# state machine'e kesin detection gönderilmez).
VISION_DECOY_REJECTION = {
    "minimum_target_score": 0.55,
    "minimum_margin": 0.15,
    "uncertain_margin": 0.08,
    "margin_score_ref": 0.30,   # confidence margin_score normalizasyon referansı
}

# Final confidence bileşenleri (toplam 1.0 olacak şekilde kalibre edilir).
#   confidence = w_shape·shape + w_color·color(purity) + w_size·size
#                + w_margin·margin(target-square) [doğrulananlara + w_temporal]
VISION_CONFIDENCE_WEIGHTS = {
    "shape": 0.30,
    "color": 0.25,
    "size": 0.15,
    "decoy_margin": 0.15,
    "temporal": 0.15,
}

# Kontur içi renk saflığı (purity) — karışık blob / düşük doygunluk zemin
# elenmesi. Baskın renk oranı + diğer renge 2.0x üstünlük şartı (yarı-yarıya
# veya 40/60 karışık blob'lar baskın renk ÜRETEMEZ → elenir; gerçek hedefler
# saf renklidir). Saha testiyle ayarlanabilir.
VISION_COLOR_PURITY = {
    "min_ratio": 0.50,
    "dominance_ratio": 2.0,
    "vivid_s_min": 60,
    "vivid_v_min": 50,
}

# approxPolyDP epsilon oranları (çoğunluk oyu — köşe sayısı güvenilirliği).
VISION_VERTEX_EPSILONS = [0.02, 0.04, 0.06]

# Boyut filtresi — altitude bilinmiyorsa (state machine eski `detect(frame)`
# çağrısı) mutlak eşikler kullanılır; altitude verilirse GSD tabanlı beklenen
# alan + area_tolerance devreye girer.
VISION_SIZE_FILTER = {
    "min_area_px": 100.0,          # 960 ölçeğinde mutlak alt sınır (altitude yokken)
    "max_blob_fraction": 0.75,     # kadrajın bu oranından büyük blob hedef değildir
}

# Frame kapısı — gyro (açısal hız) ile motion blur riskli frame'leri ele.
# Gyro verisi yoksa pipeline ÇALIŞMAYA DEVAM eder (require_gyro_data=False).
VISION_FRAME_GATE = {
    "enabled": True,
    "max_gyro_deg_s": 30.0,
    "require_gyro_data": False,
}

# Piksel → zemin → dünya dönüşümü. Nadir görüş varsayımı: küçük pitch/roll
# toleransı config'ten; aşılırsa projection geçersiz işaretlenir (confidence düşer).
VISION_GEO_PROJECTION = {
    "simplified_nadir_projection": True,
    "max_pitch_deg": 5.0,
    "max_roll_deg": 5.0,
    "position_uncertainty_radius_m": 2.0,
}

# Temporal doğrulama — son N frame'deki adaylar dünya koordinatında tutarlı
# olmalı. Dünya koordinatı yoksa piksel-uzay yarıçap fallback'i kullanılır.
# DEFAULT KAPALI: state machine'in stable_detection() zaten temporal tutarlılık
# sağlar (çift temporal = mevcut davranışı bozar). World-coordinate consistency
# gerektiren saha testlerinde enabled=True yapılır.
VISION_TEMPORAL = {
    "enabled": False,
    "window_size": 6,
    "min_consistent_ratio": 0.60,
    "min_consistent_frames": 3,   # en az bu kadar frame görülmeli (tek frame asla "kesin" değil)
    "position_match_radius_m": 1.5,
    "pixel_match_radius_px": 40.0,  # world koord yokken (960 ölçeğinde)
}

# Kabul edilebilir minimum işleme FPS'i — altına düşünce log uyarısı (engel değil).
VISION_MIN_PROCESSING_FPS = 20.0

# Kamera exposure — mode: "auto" = mevcut davranış (AeEnable otomatik).
# "fixed" saha kalibrasyonuyla doldurulup RPiCam3'e uygulanabilir (şu an kullanılmaz).
VISION_CAMERA_EXPOSURE = {
    "mode": "auto",
    "exposure_time_us": 5000,
    "analogue_gain": 2.0,
}

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
SEARCH_SPEED_MPS = 1.0
# Hedefe yaklaşma hızı (kaba)
APPROACH_SPEED_MPS = 2.0
# Hizalama sırası yatay hız (ince)
ALIGN_SPEED_MPS = 0.8
# İrtifa değişim hızı
VERTICAL_SPEED_MPS = 1.0
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
PIXEL_ALIGNMENT_TOLERANCE_PX = 60.0

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
ALIGN_GAIN_FULL_ERROR_PX = 150.0
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
ALIGN_STEP_FULL_ERROR_PX = 250.0

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
# Görüntü işleme TAM çözünürlükte (1920×1080) yapılır; YALNIZCA pencere
# gösterimi bu genişliğe küçültülür → imshow/HUD maliyeti ~4x azalır.
PREVIEW_DISPLAY_WIDTH_PX = 480

# ── Vision kayıt (vision_record_only.py — debug recorder, SD kart) ───────────
# Yalnızca recorder tarafından kullanılır; görev akışı bu değerleri okumaz.
VISION_RECORDING_DIR = "recordings"      # proje root'una göre (çalışma konumundan bağımsız)
# Kayıt hedef kalitesi (bps) — canlı stream'den (2 Mbps) yüksek debug kaydı.
# cv2.VideoWriter bitrate'i doğrudan ayarlamaz; gerçek çıktı dosya boyutundan
# ölçülüp kayıt sonunda loglanır (1920x704 avc1 ile ölçülen tipik ~19 Mbps).
VISION_RECORDING_BITRATE = 10_000_000
# Kayıt çözünürlüğü: tespit TAM çözünürlükte yapılır; yalnızca YAZILAN frame bu
# genişliğe ölçeklenir (canlı yayın 480 px; debug kaydı için 960 px önerilir).
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
