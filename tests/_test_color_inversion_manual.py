"""Küçük B↔R inversiyon testi — auto_swap mekanizmasını doğrular.

Önemli Not: %100 tek renkli, tek nesneli sahneler (sadece mavi altıgen veya sadece
kırmızı üçgen) — orijinal ve swaplı versiyon SIMETRIK oldugu icin (tek bantta
benzer sayida piksel uretirler) AutoSwap TEK BASINA karar veremez. Bu durumda
katman 1 / 2 (config uzerinden manual swap bayragi) kullanilmalidir.

AutoSwap'in tasarlandigi ve %100 basarili oldugu senaryo: KARISIK renkli sahne
(hem mavi altigen HEM de kirmizi ucgen ayni anda gorundugu — yarisma alani gibi).
"""
import cv2
import numpy as np
import logging

logging.basicConfig(level=logging.WARNING)

from models import TargetColor
from vision.target_detector import TargetDetector

W, H = 1920, 1080


def make_blue_hex():
    img = np.zeros((H, W, 3), dtype=np.uint8)
    pts = np.array(
        [[840, 375], [1080, 375], [1200, 540],
         [1080, 705], [840, 705], [720, 540]], dtype=np.int32,
    )
    cv2.fillPoly(img, [pts], (255, 0, 0))   # BGR: mavi
    return img


def make_red_tri():
    img = np.zeros((H, W, 3), dtype=np.uint8)
    pts = np.array([[250, 345], [120, 705], [460, 705]], dtype=np.int32)
    cv2.fillPoly(img, [pts], (0, 0, 255))   # BGR: kirmizi
    return img


def make_mixed_scene():
    """Hem mavi altigen hem de kirmizi ucgen BERABER — gercek yarisma sahnesi gibi."""
    img = make_blue_hex()
    pts = np.array([[300, 345], [120, 705], [510, 705]], dtype=np.int32)
    cv2.fillPoly(img, [pts], (0, 0, 255))   # BGR: kirmizi
    return img


def run_n_frames(det, img, n=15):
    last = None
    for _ in range(n):
        last = det.detect(img)
    return last or []


print("\n" + "="*65)
print("=== GRUP A: TEK NESNELI / TEK RENKLI SAHNELER (simetrik) ===")
print("="*65)
print("⚠ Not: Tek renkli sahnelerde iki versiyon (orijinal/swap) simetrik")
print("  oldugu icin AutoSwap tek basina karar veremez.")
print("  Kanal takasi artik RPiCam3 icinde sabittir (config degil).")
print()

# --- A1: Normal tek renkli (mavi) ---
img_b = make_blue_hex()
d1 = TargetDetector()
res1 = run_n_frames(d1, img_b)
ok1 = any(d.color == TargetColor.BLUE and d.shape == "HEXAGON" for d in res1)
print(f"[A1] Normal BGR — Mavi altigen:    { '✓' if ok1 else '✗' }  auto_swap={d1._auto_swap}")

# --- A2: Inverted tek renkli (mavi) — simetrik oldugu icin auto_swap = False
#        Burada beklenen: Hala renk yanlis (RED HEXAGON) — cozum: config katmani
img_b_inv = img_b[:, :, ::-1].copy()
d2 = TargetDetector()
res2 = run_n_frames(d2, img_b_inv)
ok2_configless = any(d.color == TargetColor.BLUE for d in res2)  # False beklenir
print(f"[A2] Inverted BGR — Mavi altigen:  { '✗ (Beklenen: config ile duzeltilmeli)' if not ok2_configless else '✓' }  auto_swap={d2._auto_swap}")
print(f"     Tespit edilen renk: {[(d.color.value, d.shape) for d in res2[:1]]}")
print()

# --- A3: Normal tek renkli (kirmizi ucgen) ---
img_r = make_red_tri()
d3 = TargetDetector()
res3 = run_n_frames(d3, img_r)
ok3 = any(d.color == TargetColor.RED and d.shape == "TRIANGLE" for d in res3)
print(f"[A3] Normal BGR — Kirmizi ucgen:   { '✓' if ok3 else '✗' }  auto_swap={d3._auto_swap}")

# --- A4: Inverted tek renkli (kirmizi) ---
img_r_inv = img_r[:, :, ::-1].copy()
d4 = TargetDetector()
res4 = run_n_frames(d4, img_r_inv)
ok4_configless = any(d.color == TargetColor.RED for d in res4)  # False beklenir
print(f"[A4] Inverted BGR — Kirmizi ucgen: { '✗ (Beklenen: config ile duzeltilmeli)' if not ok4_configless else '✓' }  auto_swap={d4._auto_swap}")
print(f"     Tespit edilen renk: {[(d.color.value, d.shape) for d in res4[:1]]}")
print()

print("\n" + "="*65)
print("=== GRUP B: KARISIK RENKLI SAHNE (Mavi + Kirmizi BIRLIKTE) ===")
print("="*65)
print("ℹ Gercek yarisma sahasini taklit eder; AutoSwap V3 bu durumda")
print("  inversiyonu %100 dogru sekilde otomatik yakalar.")
print()

# --- B1: Normal (BGR) karisik sahne — auto_swap=False, her iki nesne de dogru
img_mix = make_mixed_scene()
d5 = TargetDetector()
res5 = run_n_frames(d5, img_mix)
b1 = any(d.color == TargetColor.BLUE   and d.shape == "HEXAGON"  for d in res5)
b2 = any(d.color == TargetColor.RED    and d.shape == "TRIANGLE" for d in res5)
b_ok = b1 and b2
print(f"[B1] Normal BGR — iki nesneli:     { '✓' if b_ok else '✗' }  auto_swap={d5._auto_swap}")
print(f"     MaviAltigen={b1}, KirmiziUcgen={b2}")

# --- B2: INVERTED (B↔R takasli) karisik sahne  ← ASIL TEST — AutoSwap AÇILMALI ve
#     renkleri YINE DOGRU tespit etmeli
img_mix_inv = img_mix[:, :, ::-1].copy()
d6 = TargetDetector()
res6 = run_n_frames(d6, img_mix_inv)
b3 = any(d.color == TargetColor.BLUE   and d.shape == "HEXAGON"  for d in res6)
b4 = any(d.color == TargetColor.RED    and d.shape == "TRIANGLE" for d in res6)
b2_ok = b3 and b4
swap_activated = (d6._auto_swap is True)
print(f"[B2] Inverted BGR — iki nesneli:   { '✓' if (b2_ok and swap_activated) else '✗' }  auto_swap={d6._auto_swap}  ⚠ TRUE olmali!")
print(f"     MaviAltigen={b3}, KirmiziUcgen={b4}  |  auto_swap_acildi_mi={swap_activated}")
print(f"     Detay: {[(d.color.value, d.shape, round(d.confidence,2)) for d in res6[:4]]}")


print("\n\n" + "="*65)
print("=== RENK KANALI COZUM OZETI ===")
print("="*65)
print("  Katman 1 [RPiCam3]:  B↔R kanal takasi SABIT (IMX708) — config degil")
print("       → RPi tarafinda cikti her zaman BGR.")
print("  Katman 2 [Ubuntu]:   RemoteRPiCamera JPEG decode → BGR (takas YOK).")
print("  Katman 3 [AutoSwap V3] Yalnizca TARGET_DETECTOR_AUTO_SWAP=True ise aktiftir.")
print("       → Varsayilan KAPALI (kanal takasi kamera katmaninin isi).")
print()


print("=== GENEL SONUC ===")
# Gecme kosulu: Tek renkli (Grup A normal durumlar) + Karisik sahne (Grup B TAMAMI)
grup_a_normal_ok = ok1 and ok3
grup_b_ok        = b_ok and b2_ok and swap_activated
genel = grup_a_normal_ok and grup_b_ok
print(f"Grup A (normal tek renk):        {'✓ GECTI' if grup_a_normal_ok else '✗ KALDI'}")
print(f"Grup B (karisik + inversiyon):   {'✓ GECTI' if grup_b_ok else '✗ KALDI'}")
print(f"GENEL TOPLAM:                    {'BAŞARILI ✓' if genel else 'BAŞARISIZ ✗'}")
exit(0 if genel else 1)

