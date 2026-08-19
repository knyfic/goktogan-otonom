"""
tools/video_server.py — recordings/ HTTP sunucusu testleri.

Gerçek sunucu yerel portta (127.0.0.1:0) başlatılır; yalnızca recordings/ dizini
servis edilir, path traversal engellenir, Range request çalışır.
"""
import http.client
import threading

import pytest

from tools.video_server import make_handler, parse_range
from http.server import ThreadingHTTPServer

_FAKE_MP4 = b"fake-mp4-bytes"  # 14 byte


@pytest.fixture
def server(tmp_path):
    (tmp_path / "a.mp4").write_bytes(_FAKE_MP4)
    (tmp_path / "b.mp4").write_bytes(b"second-file-content")
    (tmp_path / "secret.txt").write_text("gizli")
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(tmp_path))
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    yield httpd, httpd.server_address[1]
    httpd.shutdown()
    httpd.server_close()


def _req(port, method, path, headers=None):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request(method, path, headers=headers or {})
    resp = conn.getresponse()
    body = resp.read()
    conn.close()
    return resp.status, dict(resp.getheaders()), body


def test_index_lists_only_mp4(server):
    _, port = server
    status, headers, body = _req(port, "GET", "/")
    assert status == 200
    text = body.decode("utf-8")
    assert "GÖKTOĞAN" in text
    assert "a.mp4" in text and "b.mp4" in text
    assert "secret.txt" not in text
    assert "İzle" in text and "İndir" in text


def test_player_page(server):
    _, port = server
    status, _, body = _req(port, "GET", "/v/a.mp4")
    assert status == 200
    text = body.decode("utf-8")
    assert "<video" in text
    assert "/raw/a.mp4" in text


def test_raw_serves_file_with_video_mime(server):
    _, port = server
    status, headers, body = _req(port, "GET", "/raw/a.mp4")
    assert status == 200
    assert headers.get("Content-Type") == "video/mp4"
    assert body == _FAKE_MP4
    assert headers.get("Accept-Ranges") == "bytes"


def test_missing_file_404(server):
    _, port = server
    assert _req(port, "GET", "/raw/yok.mp4")[0] == 404


def test_non_mp4_not_served(server):
    _, port = server
    assert _req(port, "GET", "/raw/secret.txt")[0] == 404


def test_project_files_not_exposed(server, tmp_path):
    _, port = server
    # root dışındaki dosyalar (örn. config.py) asla servis edilmez
    assert _req(port, "GET", "/config.py")[0] == 404
    assert _req(port, "GET", "/../config.py")[0] == 404
    assert _req(port, "GET", "/raw/../config.py")[0] == 404
    assert _req(port, "GET", "/raw/%2e%2e/config.py")[0] == 404
    assert _req(port, "GET", "/raw/..%2fconfig.py")[0] == 404


def test_download_attachment(server):
    _, port = server
    status, headers, body = _req(port, "GET", "/dl/a.mp4")
    assert status == 200
    assert "attachment" in headers.get("Content-Disposition", "")
    assert 'filename="a.mp4"' in headers.get("Content-Disposition", "")
    assert body == _FAKE_MP4


def test_range_request(server):
    _, port = server
    status, headers, body = _req(port, "GET", "/raw/a.mp4", {"Range": "bytes=0-3"})
    assert status == 206
    assert headers.get("Content-Range") == "bytes 0-3/14"
    assert headers.get("Content-Length") == "4"
    assert body == b"fake"


def test_range_open_ended(server):
    _, port = server
    status, _, body = _req(port, "GET", "/raw/a.mp4", {"Range": "bytes=10-"})
    assert status == 206
    assert body == b"ytes"  # 14-4 = son 4 bayt (indeks 10..13)


def test_head_returns_no_body(server):
    _, port = server
    status, _, body = _req(port, "HEAD", "/raw/a.mp4")
    assert status == 200
    assert body == b""


# ── parse_range birim testleri ─────────────────────────────────────────────── #

def test_parse_range_variants():
    assert parse_range("bytes=0-3", 14) == (0, 3)
    assert parse_range("bytes=10-", 14) == (10, 13)
    assert parse_range("bytes=-4", 14) == (10, 13)
    assert parse_range("bytes=5-9", 14) == (5, 9)
    assert parse_range(None, 14) is None
    assert parse_range("bytes=20-", 14) is None          # başlangıç boyutun dışında
    assert parse_range("bytes=-0", 14) is None
    assert parse_range("bytes=abc", 14) is None
    assert parse_range("bytes=9-3", 14) is None          # end < start
