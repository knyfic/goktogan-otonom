"""
Payload Candidate Tracker
=========================
Amacı: Ardışık frame'lerde aynı fiziksel nesneyi takip edip,
drone yaklaşırken "en küçük area = gerçek payload" kuralının
bozulduğu durumlarda (büyük decoy kadraj dışına çıkmaya başlayınca)
doğru hedefe kilitli kalmak.

Nasıl çalışır
-------------
1. Her frame'de PayloadDetector'dan gelen raw candidate listesi
   ``update(candidates, frame_shape)`` ile tracker'a verilir.

2. Tracker her candidate'ı önceki frame'deki track'lerle eşleştirir
   (IoU + merkez mesafesi).  Eşleşen track güncellenir, eşleşmeyen
   yeni track açılır, hiç güncellenmeyenlerin ``miss`` sayacı artar.

3. Her track bir **score** alır:
   - area_trend  : son N frame'deki area artışı (drone yaklaşıyorsa
                   gerçek payload büyümeli)
   - stability   : merkez tutarlılığı (az zıplıyor mu?)
   - border_pen  : görüntü kenarına temas → negatif faktör
   - age_bonus   : kaç frame'dir görülüyor

4. ``best_candidate()`` mevcut lock'u hysteresis ile korur:
   - Kilitli track hâlâ görünüyorsa, rakibin score'u
     LOCK_SWITCH_MARGIN kadar üstüne çıkmadan geçiş olmaz.
   - Kilitli track MISS_THRESHOLD frame boyunca kaybolursa kilit düşer.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Optional

from vision.payload_detector import PayloadDetection


# ------------------------------------------------------------------ #
# Sabitler
# ------------------------------------------------------------------ #
HISTORY_LEN       = 8    # Her track için kaç frame geçmişi tutulsun
MISS_THRESHOLD    = 5    # Bu kadar frame kaybolursa track silinir
IOU_MATCH_THRESH  = 0.15 # Track eşleştirme için minimum IoU
# Merkez piksel mesafesi eşiği — yarı çözünürlükte (960x352) aynı fiziksel
# mesafe piksel cinsinden yarıya iner; davranışın aynı kalması için 80 → 40.
DIST_MATCH_THRESH = 40
LOCK_SWITCH_MARGIN = 0.15 # Rakibin score'u bu kadar üstüne çıkmalı ki geçiş olsun
# Border'a temas eşiği (px) — yarı çözünürlükte aynı fiziksel temas için 8 → 4.
BORDER_MARGIN_PX  = 4
MIN_AGE_FOR_LOCK  = 2    # En az bu kadar frame görülmeden lock olmaz

# Score ağırlıkları
W_AREA_TREND  = 0.35
W_STABILITY   = 0.25
W_CONFIDENCE  = 0.25
W_AGE         = 0.15


# ------------------------------------------------------------------ #
@dataclass
class _Track:
    track_id:   int
    detection:  PayloadDetection          # en son frame tespiti
    area_hist:  deque = field(default_factory=lambda: deque(maxlen=HISTORY_LEN))
    center_hist: deque = field(default_factory=lambda: deque(maxlen=HISTORY_LEN))
    age:        int   = 0   # kaç frame'dir görüldü (toplam hit)
    miss:       int   = 0   # ardışık kaç frame'de kayboldu
    touches_border: bool = False
    score:      float = 0.0

    def update(self, det: PayloadDetection, frame_w: int, frame_h: int,
               border_margin: int = BORDER_MARGIN_PX) -> None:
        self.detection = det
        self.area_hist.append(det.area_px)
        self.center_hist.append((det.center_x, det.center_y))
        self.age  += 1
        self.miss  = 0

        # Border kontrolü: bbox frame kenarına değiyor mu?
        x0 = det.center_x - det.width_px / 2
        y0 = det.center_y - det.height_px / 2
        x1 = det.center_x + det.width_px / 2
        y1 = det.center_y + det.height_px / 2
        self.touches_border = (
            x0 <= border_margin
            or y0 <= border_margin
            or x1 >= frame_w - border_margin
            or y1 >= frame_h - border_margin
        )

    # ---------- Score hesabı ----------
    def compute_score(self) -> float:
        """
        0..1 arası bütünleşik güven skoru.
        Yüksek score → bu track gerçek payload olabilir.
        """
        # 1) Area trend: son frame'lere göre artıyor mu?
        area_trend = self._area_trend_score()

        # 2) Konum stabilitesi: merkez sıçraması küçük mü?
        stability = self._stability_score()

        # 3) Detector confidence (son frame)
        conf = self.detection.confidence

        # 4) Yaş bonusu: uzun süre görülüyorsa daha güvenilir
        age_bonus = min(1.0, self.age / HISTORY_LEN)

        # 5) Border cezası: kadraj kenarına temas → area güvenilmez
        border_pen = 0.4 if self.touches_border else 0.0

        raw = (
            W_AREA_TREND * area_trend
            + W_STABILITY * stability
            + W_CONFIDENCE * conf
            + W_AGE       * age_bonus
        )
        self.score = max(0.0, raw - border_pen)
        return self.score

    def _area_trend_score(self) -> float:
        """
        Drone yaklaşıyorsa gerçek payload'ın area'sı artmalı.
        Trend pozitifse (artıyor) yüksek skor, negatifse (azalıyor) düşük skor.
        Border'a temas eden nesne azalıyor gibi görünebilir → zaten border_pen
        ile ayrıca cezalandırılıyor.
        """
        hist = list(self.area_hist)
        if len(hist) < 2:
            return 0.5  # Yeterli geçmiş yok → nötr

        # Basit lineer trend: son yarı ile ilk yarıyı karşılaştır
        mid = len(hist) // 2
        avg_early = sum(hist[:mid]) / mid
        avg_late  = sum(hist[mid:]) / len(hist[mid:])

        if avg_early <= 0:
            return 0.5

        change = (avg_late - avg_early) / avg_early  # -1..+inf

        # change = +0.5 → %50 büyüdü → iyi
        # change = -0.5 → %50 küçüldü → kötü (büyük decoy kadraj dışına çıkıyor)
        # Normalize: 0..1
        score = 0.5 + min(0.5, max(-0.5, change * 0.5))
        return score

    def _stability_score(self) -> float:
        """Merkez hareketi küçükse kararlı tracking → yüksek skor."""
        hist = list(self.center_hist)
        if len(hist) < 2:
            return 0.5
        diffs = [
            ((hist[i][0] - hist[i-1][0])**2 + (hist[i][1] - hist[i-1][1])**2) ** 0.5
            for i in range(1, len(hist))
        ]
        avg_motion = sum(diffs) / len(diffs)
        # 0px hareket → 1.0, 25px+ hareket → 0.0
        # (yarı çözünürlük: aynı fiziksel hareket piksel cinsinden yarıya iner)
        return max(0.0, 1.0 - avg_motion / 25.0)


# ------------------------------------------------------------------ #
class PayloadTracker:
    """
    Kullanım:
        tracker = PayloadTracker()
        # Her frame'de:
        best = tracker.update(candidates, frame.shape)
        if best is not None:
            # best.detection → seçilen PayloadDetection
    """

    def __init__(self) -> None:
        self._tracks:   list[_Track] = []
        self._next_id:  int          = 0
        self._lock_id:  Optional[int] = None   # kilitli track ID

    def reset(self) -> None:
        self._tracks  = []
        self._next_id = 0
        self._lock_id = None

    # ------------------------------------------------------------------ #
    def update(
        self,
        candidates: list[PayloadDetection],
        frame_shape: tuple,          # (height, width, channels)
    ) -> Optional[PayloadDetection]:
        """
        Bir frame'deki candidate listesiyle tracker'ı güncelle.
        Seçilen en iyi candidate'ın PayloadDetection'ını döndür.
        Hiçbir şey yoksa None döner.
        """
        frame_h, frame_w = frame_shape[:2]

        # 1. Her track'in miss sayacını artır (bu frame'de görülmedi varsayımı)
        for t in self._tracks:
            t.miss += 1

        # 2. Candidate'ları mevcut track'lerle eşleştir
        matched_track_ids: set[int] = set()
        unmatched_candidates = list(candidates)

        for candidate in candidates:
            best_track, best_metric = None, -1.0
            for track in self._tracks:
                if track.track_id in matched_track_ids:
                    continue
                metric = self._match_score(track, candidate)
                if metric > best_metric:
                    best_metric = metric
                    best_track  = track

            if best_track is not None and best_metric > 0:
                best_track.update(candidate, frame_w, frame_h)
                best_track.miss = 0
                matched_track_ids.add(best_track.track_id)
                if candidate in unmatched_candidates:
                    unmatched_candidates.remove(candidate)

        # 3. Eşleşmeyen candidate'lar için yeni track aç
        for candidate in unmatched_candidates:
            new_track = _Track(
                track_id=self._next_id,
                detection=candidate,
            )
            new_track.update(candidate, frame_w, frame_h)
            new_track.age  = 1
            new_track.miss = 0
            self._tracks.append(new_track)
            self._next_id += 1

        # 4. Çok uzun kayıp track'leri temizle
        self._tracks = [t for t in self._tracks if t.miss < MISS_THRESHOLD]

        # 5. Lock'u kontrol et — kilitli track hâlâ yaşıyor mu?
        if self._lock_id is not None:
            if not any(t.track_id == self._lock_id for t in self._tracks):
                self._lock_id = None  # Track silindi, kilit düştü

        # 6. Score hesapla
        for t in self._tracks:
            t.compute_score()

        # 7. En iyi candidate'ı seç (hysteresis ile)
        return self._select_best()

    # ------------------------------------------------------------------ #
    def _match_score(self, track: _Track, candidate: PayloadDetection) -> float:
        """Track ↔ candidate eşleşme kalitesi. 0 = eşleşmez."""
        # Merkez mesafesi
        dx = track.detection.center_x - candidate.center_x
        dy = track.detection.center_y - candidate.center_y
        dist = (dx*dx + dy*dy) ** 0.5

        # Bounding box IoU
        iou = self._bbox_iou(track.detection, candidate)

        if iou >= IOU_MATCH_THRESH:
            return iou
        if dist <= DIST_MATCH_THRESH:
            return max(0.01, 1.0 - dist / DIST_MATCH_THRESH)
        return 0.0

    @staticmethod
    def _bbox_iou(a: PayloadDetection, b: PayloadDetection) -> float:
        ax0 = a.center_x - a.width_px  / 2;  ax1 = a.center_x + a.width_px  / 2
        ay0 = a.center_y - a.height_px / 2;  ay1 = a.center_y + a.height_px / 2
        bx0 = b.center_x - b.width_px  / 2;  bx1 = b.center_x + b.width_px  / 2
        by0 = b.center_y - b.height_px / 2;  by1 = b.center_y + b.height_px / 2

        inter_w = max(0.0, min(ax1, bx1) - max(ax0, bx0))
        inter_h = max(0.0, min(ay1, by1) - max(ay0, by0))
        inter   = inter_w * inter_h
        if inter == 0:
            return 0.0
        union = a.width_px * a.height_px + b.width_px * b.height_px - inter
        return inter / max(union, 1.0)

    # ------------------------------------------------------------------ #
    def _select_best(self) -> Optional[PayloadDetection]:
        """Hysteresis ile en iyi track'i seç."""
        # Aktif (bu frame'de görülen) track'ler
        active = [t for t in self._tracks if t.miss == 0]
        if not active:
            return None

        # En yüksek score'lu aday
        top = max(active, key=lambda t: t.score)

        # Mevcut lock aktif mi?
        locked = next((t for t in active if t.track_id == self._lock_id), None)

        if locked is None:
            # Lock yok veya kayboldu — yeni lock kur ama MIN_AGE_FOR_LOCK'a bak
            if top.age >= MIN_AGE_FOR_LOCK:
                self._lock_id = top.track_id
                return top.detection
            # Yeterince olgunlaşmamış — yine de döndür ama kilitlemez
            return top.detection

        # Lock var: rakip, kilitlinin üzerinde LOCK_SWITCH_MARGIN kadar değilse geçme
        if top.track_id != self._lock_id:
            margin = top.score - locked.score
            if margin < LOCK_SWITCH_MARGIN:
                return locked.detection  # Hysteresis: eski lock'ta kal
            # Rakip ciddi fark açtı → geçiş yap
            self._lock_id = top.track_id

        return top.detection

    # ------------------------------------------------------------------ #
    @property
    def locked_track_id(self) -> Optional[int]:
        return self._lock_id

    def track_count(self) -> int:
        return len(self._tracks)
