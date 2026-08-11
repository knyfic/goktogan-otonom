#!/usr/bin/env bash
# Raspberry Pi 5 — HIL sunucusunu başlatır (payload TCP 5700 + kamera TCP 5701).
#
# Kullanım:
#   bash raspberry-pi-5/run_hil_server.sh
#
# Not: Bu klasör HİÇBİR donanım implementasyonu içermez. Tüm kod ortak
# kaynaklardadır (payload/, vision/, services/). Bu script yalnızca
# services/hil_entrypoint.py'yi repo kökünden çalıştırır.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "İHA HIL servisi başlatılıyor (repo: $REPO_ROOT)..."
exec python3 -m services.hil_entrypoint "$@"
