"""
HIL servisleri — Raspberry Pi 5 üzerinde çalışan TCP sunucuları.

Bu paket HIL modunda RPi donanımını Ubuntu/Gazebo tarafına sunar:

  payload_server.py   → PayloadTCPServer (port 5700) — gerçek payload komutları
  payload_dispatcher.py → JSON komut → payload/backends/local_rpi.py adaptörü

Kritik tasarım kuralı: Bu paket HİÇBİR donanım implementasyonu içermez.
Gerçek donanım kodu TEK kaynaktır:

  payload/backends/local_rpi.py  +  payload/hardware/*   (REAL ile aynı kod)
  vision/rpi_camera.py                                  (REAL ile aynı kod)

Yani HIL sunucusu ile REAL uygulaması birebir aynı donanım sürücülerini kullanır.
"""
