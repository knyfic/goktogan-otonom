|     İrtifa | Yatay genişlik | Dikey yükseklik |
| ---------: | -------------: | --------------: |
|        1 m |        1.298 m |         0.487 m |
|        2 m |        2.596 m |         0.974 m |
|        3 m |        3.894 m |         1.461 m |
|        4 m |        5.192 m |         1.948 m |
|        5 m |        6.490 m |         2.435 m |
|        6 m |        7.788 m |         2.922 m |
|        7 m |        9.086 m |         3.409 m |
|        8 m |       10.384 m |         3.896 m |
|        9 m |       11.682 m |         4.383 m |
|       10 m |       12.980 m |         4.870 m |
|   **11 m** |   **14.278 m** |     **5.357 m** |
| **11.5 m** |   **14.927 m** |     **5.601 m** |
|       12 m |       15.576 m |         5.844 m |
|       13 m |       16.874 m |         6.331 m |
|       14 m |       18.172 m |         6.818 m |
|       15 m |       19.470 m |         7.305 m |

Natural Hfov = 66d 1920px 
Natural Vfov = 41d 1080px

current Hfov = 66d 960px
current Vfov = 27.38d 352px (cropped)

current vision Hm = 14.93m
current vision Vm = 5.605m


test:
kırmızı ucgene yonelde:
3m irtifaya kadar goruntu islem yaptıktan sonra offset vericaiton a girdi(2 saniye boyunca)
ardından 1 metre irtfaya dustu ve yuku bıraktı

mavi altıgene ulastı:
3m irtifaya kadar goruntu islem yaptıktan sonra offset vericaiton a girdi(2 saniye boyunca)
ardından 1 metre irtfaya dustu ve yuku bıraktı

returning payload:
4m iratada returning payload durumuna giriyor
3m irtifaya kadar goruntu islem yaptıktan sonra offset vericaiton a girdi(2 saniye boyunca)
ardından 1 metre irtfaya dustu ve yuku bıraktı

----------
config degistik
kırmızı 8m oldu

mavi de 1m deyken oldu
fakat offset duzeltme artık hic yeterli olmuyor

---


Tabii. Saha tuning'i için pratik bir karar rehberi hazırladım — önce **ne gözlemleyeceğinizi**, sonra **hangi belirtide hangi parametreyi hangi yönde, neden** değiştireceğinizi gösteriyorum.

## Adım 0 — Değiştirmeden Önce: Logdan Teşhis

Konsolda şu iki satırı okuyun (bunlar size tam teşhis verir):

```
ALIGN HORIZONTAL: merkez=(...) dx=12 dy=-8 onay=1/2 → hizalı=False   ← her komutta (0.4 sn)
ALIGN aktif: faz=HORIZONTAL irtifa=8.1m kayıp_sayac=3 kilit=RED/TRIANGLE takip=şekil-dogrulu  ← 2 sn'de bir
```

Gördüğünüz `dx/dy` değerleri asıl veridir:

| Logda gördüğünüz | Anlamı | Çözüm |
|---|---|---|
| `dx/dy` hep küçük (5–30px) ama `hizalı=False` | Tolerans eşiğinin hemen dışında jitter | **tolerans ↑** |
| `onay=1/2 → 0/2 → 1/2` sürekli sıfırlanıyor | Tek jitter frame'i sayacı düşürüyor | **exit factor ↑** |
| `dx/dy` hep büyük (50–150px) | Gerçekten merkezden uzaksınız | tolerans değil, **gain/hareket** sorunu |
| `kayıp_sayac` hızla büyüyor | Hedef görüntüden çıkıyor | farklı sorun — tolerans çözmez |

---

## Adım 1 — Belirti → Parametre → Yön → Neden

### A) Kabul eşikleri (birincil — en sık değiştireceğiniz)

| # | Belirti (real uçuşta) | Parametre | Yön | Neden / Nasıl |
|---|---|---|---|---|
| A1 | **Yüksek irtifada** (8m+) fiziksel olarak merkezdeyim ama hâlâ `hizalı=False` | `ALIGN_TOL_HI_PX` (şu an 50) | **↑** (50→60, sonra 70) | Yüksek irtifa kabul eşiği. 10'ar artırıp deneyin. 50px @8m ≈ 54cm fiziksel — 60px ≈ 65cm. |
| A2 | **Alçak irtifada** (3m civarı, iniş öncesi) merkezdeyim ama kabul etmiyor | `ALIGN_TOL_LO_PX` (şu an 30) | **↑** (30→35→40) | Alçak irtifa eşiği. **DİKKAT:** bırakma hassasiyeti buna bağlı — 40'ı geçirmeyin. 30px @3m ≈ 12cm fiziksel. |
| A3 | Merkezden **belirgin uzaktayken** kabul ediyor (yüksek irtifa) | `ALIGN_TOL_HI_PX` | **↓** (50→40) | Fazla gevşek. 10 azaltın. |
| A4 | Merkezden **belirgin uzaktayken** kabul ediyor (3m, bırakma öncesi) | `ALIGN_TOL_LO_PX` | **↓** (30→25) | Bırakma doğruluğu riski — hemen sıkın. |
| A5 | İrtifa geçişi yanlış yerde oluyor (örn. 6m'de hâlâ "yüksek irtifa" kadar gevşek) | `ALIGN_TOL_ALT_LO_M` (3.0) / `ALIGN_TOL_ALT_HI_M` (8.0) | LO↑ / HI↓ | Geçiş bandını kendi irtifalarınıza göre ayarlayın. Bandı daraltmak geçişi dikleştirir. |

### B) Confirmation (zaman/onay — ikincil)

| # | Belirti | Parametre | Yön | Neden / Nasıl |
|---|---|---|---|---|
| B1 | Tek jitter frame'i `onay` sayacını sürekli düşürüyor (flip-flop) | `ALIGN_TOL_EXIT_FACTOR` (şu an 1.6) | **↑** (1.6→1.8→2.0) | Exit toleransı = eşik × bu çarpan. Sayacı koruyan bant genişler. 2.0'da 50px eşik → 100px'e kadar korur. |
| B2 | Kabul ediyor ama **çok çabuk** iniyor (tesadüfi şanslı frame'ler) | `TARGET_ALIGNED_FRAMES_REQUIRED` (şu an 2) | **↑** (2→3) | Bırakma onayı. Her onay 0.4 sn → 3 onay ≈ 1.2 sn aralıksız merkezde kalma şartı. |
| B3 | Kabul ediyor ama çok çabuk (pickup/merkezleme) | `ALIGNED_FRAMES_REQUIRED` (şu an 3) | **↑** (3→4) | Pickup öncesi merkezleme. |
| B4 | **Hiç** onay dolmuyor / çok yavaş | `TARGET_ALIGNED_FRAMES_REQUIRED` ↓ (2→1) veya `ALIGN_CMD_INTERVAL_S` ↓ (0.4→0.3) | **↓** | Son çare. Önce A1/A2 toleransı deneyin; onayı düşürmek güvenliği azaltır. |

### C) Hareket tarafı — YALNIZCA salınım/gecikme görürseniz (hız değil ama davranış)

Hızlar (`*_SPEED_MPS`) değişmiyor; ama aşağıdakiler uçuş davranışını değiştirir:

| Belirti | Parametre | Yön | Neden |
|---|---|---|---|
| Hedef etrafında **sağ-sol salınım** (yüksek irtifada) | `ALIGN_GAIN` (0.8) veya `ALIGN_MAX_STEP_M` (0.5) | **↓** (0.8→0.6 / 0.5→0.4) | Adım başına düzeltme büyük → aşıyor. |
| **Çok yavaş** yakınsıyor (küçük adımlar) | `ALIGN_GAIN` | **↑** (0.8→0.9) | Kazanç düşük. |
| Yüksek irtifada hâlâ salınım | `ALIGN_ALT_GAIN_REF_M` (3.0) | **↑** (3→4) | İrtifa kompanzasyonu daha erken küçültür. |

**Ayrım:** `dx/dy` küçük ama `hizalı=False` → A/B grubu. `dx/dy` büyük ve İHA hep geçiyor → C grubu.

---

## Adım 2 — Yöntem (trial-error disiplini)

1. **Her uçuşta YALNIZCA bir parametre** değiştirin. İkisini birden değiştirirseniz hangisinin işe yaradığını bilemezsiniz.
2. Küçük adımlarla gidin (yukarıdaki önerilen adımlar).
3. Her uçuştan sonra logdaki `dx/dy` dağılımını not edin (örn. "8m'de dx 10-25px arası, hizalı hep False").
4. İstediğiniz sonuç: **yüksek irtifada** `dx/dy` 20–50px bandındayken `hizalı=True`; **3m'de** 10–25px bandındayken `hizalı=True`; ama 80px+ her zaman False.

---

## Adım 3 — Fiziksel anlam (karar verirken kafanızda olsun)

Kameralı (960x352, H-FOV 66°):

| İrtifa | 1 piksel ≈ | 30px ≈ | 50px ≈ |
|---|---|---|---|
| 3 m | 4.1 mm | **12 cm** | 20 cm |
| 8 m | 10.8 mm | 32 cm | **54 cm** |
| 11.5 m | 15.6 mm | 47 cm | **78 cm** |

Bu tablo size "şu tolerans kaç cm merkezden sapma kabul ediyor" der. Örn. 8m'de 50px = 54cm — bu, 1m'lik üçgenin yarısından biraz az; yüksek irtifa kabulü için makul, ama bırakma (3m) için asla bu kadar gevşek olmamalı (orada 30px = 12cm).

---

## Özet karar tablosu (kopyalanıp sahaya götürülebilir)

| Semptom | Parametre | Yön | Adım |
|---|---|---|---|
| 8m+'da merkezdeyim, kabul etmiyor | `ALIGN_TOL_HI_PX` | ↑ | +10 |
| 3m'de merkezdeyim, kabul etmiyor | `ALIGN_TOL_LO_PX` | ↑ | +5 (maks 40) |
| Onay sayacı jitter'da düşüyor | `ALIGN_TOL_EXIT_FACTOR` | ↑ | +0.2 |
| Uzaktayken kabul ediyor (yüksek) | `ALIGN_TOL_HI_PX` | ↓ | −10 |
| Uzaktayken kabul ediyor (alçak) | `ALIGN_TOL_LO_PX` | ↓ | −5 |
| Çok çabuk iniyor | `TARGET_ALIGNED_FRAMES_REQUIRED` | ↑ | +1 |
| Hiç inemiyor (tolerans denendikten sonra) | `TARGET_ALIGNED_FRAMES_REQUIRED` | ↓ | −1 |
| Salınım (dx büyük, hep geçiyor) | `ALIGN_GAIN` / `ALIGN_MAX_STEP_M` | ↓ | −0.1 / −0.1 |
| Çok yavaş yakınsama | `ALIGN_GAIN` | ↑ | +0.1 |

**En kritik kural:** Değişiklikleri kalıcı yapmadan önce her uçuşta logdaki `dx/dy` değerlerine bakın — "merkezde ama kabul etmiyor" (tolerans sorunu) ile "gerçekten uzakta" (hareket sorunu) aynı şey değil; yanlış grubu değiştirirseniz hiçbir parametre işe yaramaz. İsterseniz bu tabloyu `hata.md`'ye tuning notu olarak ekleyebilirim — kod değişikliği gerekmez.
