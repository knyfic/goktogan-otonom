"""
Göktoğan Vision Debug Video Server — recordings/*.mp4 → Local Network → PC.

Yalnızca recordings/ dizinini HTTP üzerinden sunar; projenin geri kalanına
erişim YOKTUR (path traversal engellidir, yalnızca .mp4 dosyaları servis edilir).

Özellikler:
  • HTML liste + HTML5 <video> oynatıcı ([▶ İzle]) + indirme ([↓ İndir])
  • HTTP Range Requests (tarayıcıda ileri/geri sarma — 206 Partial Content)
  • Yalnızca Python standard library (harici framework yok)

GÜVENLİK:
  Bu sunucu yalnızca LOCAL NETWORK kullanımı içindir. Authentication YOKTUR;
  0.0.0.0 üzerinde dinlediği için internete AÇMAYIN.

KULLANIM (Raspberry Pi üzerinde, test sonrası):
  python3 tools/video_server.py [--host 0.0.0.0] [--port 8000] [--dir recordings]

PC tarayıcıdan:
  http://RASPBERRY_PI_IP:8000
"""
from __future__ import annotations

import argparse
import re
import sys
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# Kayıt dizini proje root'una göre (çalıştırma konumundan bağımsız).
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_DEFAULT_DIR = _PROJECT_ROOT / "recordings"

_CHUNK = 64 * 1024


def parse_range(header: str | None, size: int) -> tuple[int, int] | None:
    """'Range: bytes=start-end' başlığını çözümler (tek aralık).

    Dönen: (start, end) — end DAHİL; geçersizse None (→ 200 tam gönderim).
    Boş start (suffix) ve açık uçlu end desteklenir.
    """
    if not header or size <= 0:
        return None
    m = re.fullmatch(r"\s*bytes=(\d*)-(\d*)\s*", header)
    if not m:
        return None
    s, e = m.group(1), m.group(2)
    if not s and not e:
        return None
    if not s:  # son N byte: 'bytes=-500'
        n = int(e)
        if n <= 0:
            return None
        return (max(0, size - n), size - 1)
    start = int(s)
    end = int(e) if e else size - 1
    if start >= size:
        return None
    end = min(end, size - 1)
    if end < start:
        return None
    return (start, end)


class _Handler(BaseHTTPRequestHandler):
    server_version = "GoktoganVisionServer/1.0"

    # make_handler() tarafından doldurulur — yalnızca bu dizin sunulur.
    recordings_dir: Path = _DEFAULT_DIR

    # ------------------------------------------------------------------ #
    def do_GET(self) -> None:  # noqa: N802
        self._handle(method="GET")

    def do_HEAD(self) -> None:  # noqa: N802
        self._handle(method="HEAD")

    # ------------------------------------------------------------------ #
    def _handle(self, method: str) -> None:
        try:
            path = urllib.parse.unquote(urllib.parse.urlparse(self.path).path)
            if path in ("/", "/index.html"):
                self._send_listing(method)
            elif path.startswith("/v/"):
                self._send_player(method, path[len("/v/"):])
            elif path.startswith("/dl/"):
                self._send_file(method, path[len("/dl/"):], as_attachment=True)
            elif path.startswith("/raw/"):
                self._send_file(method, path[len("/raw/"):], as_attachment=False)
            else:
                self._send_error(404, "Not Found")
        except (BrokenPipeError, ConnectionResetError):
            pass  # istemci kapattı — önemsiz

    # ------------------------------------------------------------------ #
    def _resolve(self, name: str) -> Path | None:
        """Güvenli dosya çözümü: yalnızca recordings/ içindeki .mp4'ler."""
        if not name or name in (".", "..") or name.startswith("."):
            return None
        if "/" in name or "\\" in name or name != Path(name).name:
            return None  # alt dizin / traversal denemesi → red
        if not name.lower().endswith(".mp4"):
            return None
        root = self.recordings_dir.resolve()
        p = (root / name).resolve()
        try:
            p.relative_to(root)
        except ValueError:
            return None  # root dışına kaçış → red
        if not p.is_file():
            return None
        return p

    # ------------------------------------------------------------------ #
    def _send_listing(self, method: str) -> None:
        videos = sorted(
            (p for p in self.recordings_dir.glob("*.mp4") if p.is_file()),
            key=lambda p: p.name, reverse=True,
        )
        rows = "".join(
            f"<li><span class='name'>{p.name}</span> "
            f"<a class='btn' href='/v/{urllib.parse.quote(p.name)}'>▶ İzle</a> "
            f"<a class='btn' href='/dl/{urllib.parse.quote(p.name)}'>↓ İndir</a></li>"
            for p in videos
        ) or "<li class='empty'>Kayıt yok — önce vision_record_only.py çalıştırın.</li>"
        body = self._page(
            "GÖKTOĞAN VISION DEBUG — Recordings",
            f"<h1>GÖKTOĞAN VISION DEBUG</h1><h2>Recordings</h2>"
            f"<p class='hint'>Yalnızca local network — {len(videos)} video</p>"
            f"<ul class='list'>{rows}</ul>",
        )
        self._send_bytes(body.encode("utf-8"), "text/html; charset=utf-8", method)

    def _send_player(self, method: str, name: str) -> None:
        if self._resolve(name) is None:
            self._send_error(404, "Not Found")
            return
        quoted = urllib.parse.quote(name)
        body = self._page(
            f"İzle — {name}",
            f"<h1><a href='/'>← Recordings</a></h1>"
            f"<h2>{name}</h2>"
            f"<video controls preload='metadata' src='/raw/{quoted}' "
            f"style='width:100%;max-width:960px;background:#000'></video>"
            f"<p><a class='btn' href='/dl/{quoted}'>↓ İndir</a></p>",
        )
        self._send_bytes(body.encode("utf-8"), "text/html; charset=utf-8", method)

    def _send_file(self, method: str, name: str, as_attachment: bool) -> None:
        p = self._resolve(name)
        if p is None:
            self._send_error(404, "Not Found")
            return
        size = p.stat().st_size
        rng = parse_range(self.headers.get("Range"), size)
        disposition = (
            f'attachment; filename="{p.name}"' if as_attachment else None
        )
        try:
            if rng is None:
                self.send_response(200)
                self.send_header("Content-Type", "video/mp4")
                self.send_header("Content-Length", str(size))
                self.send_header("Accept-Ranges", "bytes")
                if disposition:
                    self.send_header("Content-Disposition", disposition)
                self.end_headers()
                if method == "HEAD":
                    return
                with open(p, "rb") as f:
                    self._copy(f, size)
            else:
                start, end = rng
                length = end - start + 1
                self.send_response(206)
                self.send_header("Content-Type", "video/mp4")
                self.send_header("Content-Length", str(length))
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
                self.send_header("Accept-Ranges", "bytes")
                if disposition:
                    self.send_header("Content-Disposition", disposition)
                self.end_headers()
                if method == "HEAD":
                    return
                with open(p, "rb") as f:
                    f.seek(start)
                    self._copy(f, length)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _copy(self, f, remaining: int) -> None:
        while remaining > 0:
            chunk = f.read(min(_CHUNK, remaining))
            if not chunk:
                break
            self.wfile.write(chunk)
            remaining -= len(chunk)

    # ------------------------------------------------------------------ #
    def _page(self, title: str, content: str) -> str:
        return f"""<!DOCTYPE html>
<html lang="tr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
<style>
  body {{ font-family: system-ui, sans-serif; max-width: 900px; margin: 2rem auto;
          padding: 0 1rem; background: #101418; color: #e8e8e8; }}
  h1 {{ color: #4caf50; }} h2 {{ color: #ccc; }}
  .list {{ list-style: none; padding: 0; }}
  .list li {{ padding: .5rem 0; border-bottom: 1px solid #2a2f36;
              display: flex; align-items: center; gap: .75rem; }}
  .name {{ flex: 1; font-family: monospace; }}
  .btn {{ color: #fff; background: #1e6f3f; padding: .3rem .7rem; border-radius: 4px;
          text-decoration: none; font-size: .9rem; }}
  a {{ color: #4caf50; }}
  .hint {{ color: #888; font-size: .85rem; }}
  .empty {{ color: #888; }}
</style></head><body>{content}</body></html>"""

    def _send_bytes(self, data: bytes, ctype: str, method: str) -> None:
        try:
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            if method != "HEAD":
                self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_error(self, code: int, message: str) -> None:
        body = f"<!DOCTYPE html><html><body><h1>{code} {message}</h1></body></html>"
        try:
            self.send_response(code)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body.encode("utf-8"))
        except (BrokenPipeError, ConnectionResetError):
            pass

    # İstek logunu sessiz tut (video seek çok sayıda istek üretir).
    def log_message(self, fmt: str, *args) -> None:  # noqa: N802
        pass


def make_handler(recordings_dir: str | Path) -> type[_Handler]:
    """recordings dizinine bağlı handler sınıfı üretir (test edilebilirlik)."""
    d = Path(recordings_dir)
    if not d.is_absolute():
        d = _PROJECT_ROOT / d
    d = d.resolve()

    class Handler(_Handler):
        recordings_dir = d

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description="Göktoğan Vision Debug Video Server")
    parser.add_argument("--host", default="0.0.0.0", help="dinlenecek adres (varsayılan 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8000, help="port (varsayılan 8000)")
    parser.add_argument("--dir", default="recordings", help="kayıt dizini (varsayılan recordings/)")
    args = parser.parse_args()

    recordings = Path(args.dir)
    if not recordings.is_absolute():
        recordings = _PROJECT_ROOT / recordings
    recordings.mkdir(parents=True, exist_ok=True)
    recordings = recordings.resolve()

    handler = make_handler(recordings)
    httpd = ThreadingHTTPServer((args.host, args.port), handler)
    print("Göktoğan Vision Video Server")
    print(f"Directory: {recordings}")
    print(f"Address:   http://{args.host}:{args.port}")
    print("⚠ Yalnızca LOCAL NETWORK — authentication yok, internete açmayın.")
    videos = sorted(recordings.glob("*.mp4"))
    print(f"Available videos: {len(videos)}")
    for v in videos:
        print(f"  - {v.name}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nDurduruldu.")
    finally:
        httpd.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
