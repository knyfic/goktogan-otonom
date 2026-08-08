Önce her şeyi okuyayım, sonra değerlendireyim:Tüm kodu okudum. Şimdi state_machine.py'nin geri kalan kısmını da göreyim:Tüm kodu inceledim. Değerlendirmemi yazıyorum:

---

## Mimari Değerlendirme

### 1. Güçlü Taraflar

**Modül sınırları büyük ölçüde doğru.** `Navigation`, `PositionController`, `PayloadController`, `TargetDetector`, `PayloadDetector`, `LawnmowerPlanner` kendi sorumluluklarını gerçekten taşıyor. State machine bunları doğrudan "nasıl yapacağım?" diye sormak yerine "yap" diye çağırıyor.

**Non-blocking mimari tutarlı.** `_send_altitude` + `_altitude_ok()` döngüsü hem doğru hem tekrar kullanılıyor. `_pump_preview` tek noktadan yönetiliyor.

**State geçişleri okunabilir.** `_tick()` içindeki `if/elif` zinciri tüm akışı tek bakışta gösteriyor.

**Test altındaki kodun çalıştığını biliyoruz** — Gazebo'da çalışıyor, yani büyük bir mimari yeniden yazım risk almaya değmez.

---

### 2. Gerçekten Problemli Noktalar

**Sorun A — `DESCENDING_TO_PAYLOAD` state'i ölü kod:**
`models.py`'de tanımlı, `_tick()`'te handler'ı var ama handler direkt `PICKING_UP_PAYLOAD`'a yönlendiriyor ve log'da "bu state artık kullanılmıyor" yazıyor. `_do_align_payload` hiçbir zaman bu state'e geçmiyor.

**Sorun B — `TASK_1_COMPLETE` state'i anlamsız bir durak:**
```python
elif s == MissionState.TASK_1_COMPLETE:
    self._goto(MissionState.NAVIGATING_TO_FIRST_TARGET)
```
Bir tick geçişi. Hiçbir iş yapmıyor. Davranışa katkısı sıfır.

**Sorun C — `_do_align_target` içinde duplicate `_no_detection_count` kontrolü:**
Aynı if bloğu `detection is None` için iki kez yazılmış:

```python
if detection is None:
    if self._no_detection_count > self._NO_DETECTION_LIMIT:
        ...
    return

if self._no_detection_count > self._NO_DETECTION_LIMIT:  # AYNI KONTROL TEKRAR
    ...
    return
```

**Sorun D — `_do_release_returned_payload` üç fazı tek handler'da yönetiyor:**
`HORIZONTAL | DESCENDING | RELEASING` — bu `_do_align_target`'ın aynısı ama farklı değişkenlerle. Duplicate mantık.

**Sorun E — `_do_search_target` içinde `_locked_color` güncellemesi iki yerde:**
`update_time_lock` sonrası bir kez set ediliyor, `stable is not None` bloğunda tekrar kontrol ediliyor ve tekrar set edilebiliyor. İki farklı kod yolu aynı değişkeni farklı koşullarda yazıyor — hangisi kesin olduğu takip edilmesi zor.

**Sorun F — `navigation.py`'de `import math` iki kez yazılmış:**
Küçük ama açık hata.

**Sorun G — `config.py`'de `ALIGNMENT_DESCENT_STEP_M` iki kez tanımlı:**
```python
ALIGNMENT_DESCENT_STEP_M = 2.0   # satır 17
...
ALIGNMENT_DESCENT_STEP_M = 2.0   # satır 101 (tekrar)
```

**Sorun H — `AltitudeController` kullanılmıyor:**
`state_machine.py` `self.altitude_controller` oluşturuyor ama hiçbir yerde çağırmıyor. Non-blocking sisteme geçilince devre dışı kaldı ama instance yaratılmaya devam ediyor.

**Sorun I — `MissionData.home_latitude/longitude` kullanılmıyor:**
`_do_init` bu değerleri set ediyor ama kod boyunca hiç okunmuyor. `self.nav.home_lat/lon` kullanılıyor.

---

### 3. Karmaşık Görünüp Aslında Doğru Olan Noktalar

**`_do_align_payload` içindeki iç faz yönetimi (`HORIZONTAL/DESCENDING`) aslında mimari olarak doğru.** Alternatif iki ayrı state yapmak olurdu ama bu durumda geçiş sayısı artardı ve `_align_phase_start_t` gibi bağlam değişkenleri yine gerekli olurdu. Tek handler içinde tutmak bu durumda okunabilirliği artırıyor.

**`__init__` içindeki uzun değişken listesi gerçek bir problem değil.** Bunların çoğu aynı lifecycle'ı paylaşan state bağlamı. Ayrı bir `AlignContext` dataclass'ına çıkarmak mümkün ama davranışı değiştirmeden bunu yapmak high-risk olur, kazanç düşük.

**`_do_navigate_to_first` içindeki blocking `while` döngüsü:** Diğer state'lerle tutarsız görünüyor ama bu state'in tek görevi bir noktaya gitmek ve GÜVENLİ ŞEKİLDE beklemek. Preview de beslendiği için donma yok. Davranış açısından sorun değil.

---

### 4. Potansiyel Bug / State Transition Problemi

**`_do_search_target`'ta `locked_color` yarış durumu:**
`stable is not None` iken `_locked_color is None` ise state machine `_locked_color`'ı set ediyor. Ama `update_time_lock` zaten `time_locked_color`'ı da set etmiş olabilir. İki kaynak aynı değişkeni yazıyor. Şu an çalışıyor çünkü her ikisi de aynı değeri yazıyor, ama mantık karmaşık.

**`_do_return_payload` navigasyon faz geçişi belirsiz:**
`_alt_waiting` False iken irtifa kontrol et, yükselt, `return`. Sonraki tick'te `_alt_waiting` True oluyor, `_altitude_ok()` çağrılıyor, `False` dönünce `return`. Doğru çalışıyor ama `if not self._alt_waiting` + `if self._alt_waiting` zinciri okunması zor.

---

### 5. Gereksiz / Ölü Kod

| Öğe | Durum | Güven |
|---|---|---|
| `DESCENDING_TO_PAYLOAD` state + handler | Ölü, hiç ulaşılmıyor | Yüksek |
| `TASK_1_COMPLETE` state | Sadece geçiş, anlamsız durak | Yüksek |
| `AltitudeController` instance | Yaratılıyor, kullanılmıyor | Yüksek |
| `MissionData.home_latitude/longitude` | Set ediliyor, okunmuyor | Yüksek |
| `import math` x2 `navigation.py` | Duplicate import | Kesin |
| `ALIGNMENT_DESCENT_STEP_M` x2 `config.py` | Duplicate tanım | Kesin |
| `FINAL_TARGET_ALTITUDE` config'de | Alias, sadece geriye uyumluluk için | Orta |
| `MAX_HORIZONTAL_SPEED_MPS`, `COARSE_APPROACH_SPEED_MPS`, `FINE_APPROACH_SPEED_MPS` | "Eski isimler" yorumuyla config'de, kullanılmıyor | Yüksek |
| `queue` import `camera.py`'de | İmport edilmiş, kullanılmıyor | Kesin |

---

### 6. State Machine'de Sorumluluk İhlalleri

**Minor:** `_do_pickup` içinde `cv2.putText` çağrısı var — display mantığı `_set_display_*` metodlarına delegelenmeli, doğrudan state handler içinde olmamalı. Ama bu kritik değil.

**Minor:** `_do_search_target` içinde `target_detector.history.clear()` ve `reset_time_lock()` çağrıları state machine içinde yapılıyor. Detector'ın iç state'i SM tarafından yönetiliyor — ideal değil ama alternatif (detector'da reset metodu) zaten var ve çağrılıyor, sorun büyük değil.

---

### 7. Önerilen Refactor Planı ve Risk Seviyeleri

| # | Değişiklik | Dosya | Risk | Açıklama |
|---|---|---|---|---|
| 1 | `import math` duplikasyonunu kaldır | `navigation.py` | **LOW** | Sadece ilk satırı sil |
| 2 | `ALIGNMENT_DESCENT_STEP_M` duplikasyonunu kaldır | `config.py` | **LOW** | İkinci tanımı sil |
| 3 | `queue` import'unu kaldır | `vision/camera.py` | **LOW** | Kullanılmıyor |
| 4 | `FINAL_TARGET_ALTITUDE`, `MAX_HORIZONTAL_SPEED_MPS`, `COARSE_APPROACH_SPEED_MPS`, `FINE_APPROACH_SPEED_MPS` kullanımlarını kontrol et, kullanılmıyorsa kaldır | `config.py` | **LOW** | Grep ile teyit et |
| 5 | `DESCENDING_TO_PAYLOAD` state'i ve handler'ı kaldır | `models.py`, `state_machine.py` | **LOW** | Handler log yazıyor "kullanılmıyor" diyor, `_do_align_payload` bu state'e geçmiyor |
| 6 | `AltitudeController` instance'ını kaldır | `state_machine.py` | **LOW** | `self.altitude_controller` hiç çağrılmıyor |
| 7 | `MissionData.home_latitude/longitude` kaldır | `models.py`, `state_machine.py` | **LOW** | `nav.home_lat/lon` kullanılıyor zaten |
| 8 | `TASK_1_COMPLETE` state'ini kaldır, `_do_release`'den direkt `NAVIGATING_TO_FIRST_TARGET`'a geç | `models.py`, `state_machine.py` | **LOW** | Tek satır geçişi, test edilebilir |
| 9 | `_do_align_target`'ta duplicate `_no_detection_count` kontrolünü birleştir | `state_machine.py` | **LOW** | Pure cleanup, mantık değişmiyor |
| 10 | `_do_pickup` içindeki raw `cv2.putText`'i `_set_display_*` ile kapsülle | `state_machine.py` | **LOW** | Display tutarlılığı |
| 11 | `_do_search_target` içindeki `_locked_color` set mantığını tek noktaya topla | `state_machine.py` | **MEDIUM** | İki yerden yazılıyor, birleştirince davranış değişme riski var |
| 12 | `_do_return_payload` faz yönetimini netleştir | `state_machine.py` | **MEDIUM** | Mantık aynı kalır ama alt/wait zinciri okunması zor |
| 13 | `_do_release_returned_payload` ile `_do_align_target`'ın duplicate mantığını ortak bir helper'a çıkar | `state_machine.py` | **HIGH** | İki handler'ın parametreleri farklı (target tipi, color filtresi, altitude floor) — yanlış abstraction riski var |

---

### Karar

Madde 11, 12, 13 davranış değiştirme riski taşıdığından **uygulamayacağım**. Bunları sana bildiriyorum ama dokunmuyorum.

LOW riskli 1-10 arası değişiklikleri şimdi uyguluyorum:Başlıyorum. Her değişiklikten önce mevcut kodu doğruluyorum:Mevcut durum teyit edildi. Değişiklikleri uyguluyorum:

**Adım 1 — navigation.py: duplicate `import math`****Adım 2 — camera.py: kullanılmayan `queue` import****Adım 3 — config.py: duplicate ve kullanılmayan sabitler**

Önce config'deki tüm şüpheli isimlerin gerçekten kullanılmadığını doğrulayalım:`ALIGNMENT_DESCENT_STEP_M` ve `FINAL_TARGET_ALTITUDE` `state_machine.py`'de kullanılıyor. `MAX_HORIZONTAL_SPEED_MPS`, `COARSE_APPROACH_SPEED_MPS`, `FINE_APPROACH_SPEED_MPS` kullanılmıyor. Şimdi config'i düzeltelim:`POSITION_TOLERANCE_M` da kullanılıp kullanılmadığını kontrol edeyim:Bunlar da kullanılmıyor. Ama bunları kaldırmak daha temkinli — `POSITION_TOLERANCE_M`, `SEARCH_TIMEOUT` gibi isimler gerçek donanım geçişinde kullanılabilir. Sadece açıkça "geriye dönük uyumluluk" yorumu olan `MAX_HORIZONTAL_SPEED_MPS`, `COARSE_APPROACH_SPEED_MPS`, `FINE_APPROACH_SPEED_MPS` ile `ALIGNMENT_DESCENT_STEP_M` duplikasyonuna odaklanıyorum:Sonucu doğrula:`ALIGNMENT_DESCENT_STEP_M` artık tek bir yerde tanımlı, üç kullanılmayan sabit kaldırıldı.**Adım 4 — Dead code temizliği: 4 bağımsız değişiklik**

Önce her birinin gerçekten güvenli olduğunu teyit edeyim:Teyit edildi. Şimdi 4 değişikliği sırayla uyguluyorum:

**4a — `DESCENDING_TO_PAYLOAD`: models.py'den state, state_machine.py'den `_tick` satırı ve handler kaldır**Handler metodunu da kaldır:**4b — `TASK_1_COMPLETE`: `_do_release` içindeki geçişi direkt `NAVIGATING_TO_FIRST_TARGET`'a çevir, state ve tick satırını kaldır****4c — `AltitudeController`: import ve instance kaldır**`position_controller` ve `rtl_controller` satırlarını tekrar eklemem gerekiyor — yalnızca `altitude_controller` satırını silmem gerekiyordu:**4d — `home_latitude` / `home_longitude`: models.py'den field, state_machine.py'den set satırları kaldır**Hızlı derleme kontrolü:Kalan referans kalmadığını doğrula:Temiz — hiç kalmadı. (exit 1 sadece grep eşleşme bulamadığında döner, hata değil.)**Adım 5 — `_do_align_target`: duplicate `_no_detection_count` kontrolü**

Önce mevcut bloğu tam olarak okuyayım:Mantığı anlıyorum. İki blok aynı koşulu kontrol ediyor:

- **Birinci blok:** `detection is None` → limit aşıldıysa recover et → `return`
- **İkinci blok:** `detection is not None` ama `_no_detection_count > limit` → recover et → `return`

İkinci blok, `detection` var ama önceki tick'lerde sayaç dolmuşsa tetikleniyor. Mantık doğru ama `detection is None: return` zaten birinci bloktan döndüğü için ikinci blok aslında **sadece `detection is not None` olan durumda** çalışıyor. Yani şartlar birbirini dışlamıyor, ikisi farklı kod yolları.

Birleştirme: `_no_detection_count` kontrolünü `detection` varlığından bağımsız olarak yukarı çek, tek bir kontrol yap:Davranış aynı mı kalıyor? Evet — eskiden de sayaç limit'i aştığında her iki durumda (`detection is None` veya `detection is not None`) `_recover_to_rough_position()` çağrılıp `return` yapılıyordu. Yeni kod bunu tek noktada yapıyor, sonra `detection is None` ise yine `return` yapıyor.

Derleme kontrolü:**Adım 6 — `_do_pickup`: raw `cv2.putText` → `_draw_hud` ile kapsülle**

Önce mevcut kodu okuyayım:Şu an iki ham `cv2.putText` çağrısı var. `_draw_hud` iki satır alıyor: `line1` ve `line2`. Servo bekleme bilgisi `line1`'e, irtifa bilgisi `_hud_lines` metodunun zaten döndürdüğü `line2`'ye giriyor. `_hud_lines(extra=...)` kullanarak tamamen kapsülleyebiliriz:`_draw_hud` `@staticmethod` olduğu için `target_panel` parametresi de alıyor — varsayılan `None` zaten, servo ekranında panel göstermek doğru (first_target kaydedilmiş olduğundan paneli göstermek anlamsız değil). Tutarlılık için `target_panel` da geçirelim:Derleme kontrolü:Başka ham `cv2` çağrısı kalmadığını doğrula:Bunlar `_draw_hud`, `_set_display_payload` gibi display yardımcıları içinde — state handler'da değil, doğru yerde.**Adım 7 — Kapsamlı derleme ve state ulaşılabilirlik testi**Tüm modüller temiz derlendi. Şimdi kapsamlı state testi:Tüm testler geçti. Mevcut testleri de çalıştıralım:6/6 test geçti.Refactor tamamlandı. Özet:

---

**Ne değişti — 6 dosya, 7 değişiklik kategorisi:**

| Dosya | Değişiklik | Etki |
|---|---|---|
| `flight/navigation.py` | `import math` duplikasyonu kaldırıldı | 1 satır |
| `vision/camera.py` | Kullanılmayan `import queue` kaldırıldı | 1 satır |
| `config.py` | `ALIGNMENT_DESCENT_STEP_M` duplikasyonu + 3 kullanılmayan sabit kaldırıldı | 7 satır |
| `models.py` | `DESCENDING_TO_PAYLOAD`, `TASK_1_COMPLETE`, `home_latitude/longitude` kaldırıldı | 5 satır |
| `state_machine.py` | `_do_descend_to_payload` handler, `AltitudeController`, `TASK_1_COMPLETE` tick, `home_lat/lon` set, duplicate `_no_detection_count` bloğu, raw `cv2.putText` kaldırıldı/değiştirildi | ~40 satır |

**Ne değişmedi:** Tüm state transition mantığı, görev akışı, navigation davranışı, detection algoritması, payload pickup/release, timing/timeout değerleri, Gazebo simülatörü davranışı.

**Doğrulama:** 15 modül hatasız derlendi, 24 state'in tamamı erişilebilir, 6 mevcut test geçti.