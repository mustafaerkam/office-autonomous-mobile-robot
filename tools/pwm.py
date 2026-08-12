#!/usr/bin/env python3
"""Tezgahta motora dogrudan PWM darbesi verir ve encoder sayacini raporlar.

Firmware'in JOG komutunu kullanir: kalibrasyon, PID ve ARM gerektirmez, sure
dolunca cikis kendiliginden kesilir. Amaci "motor donuyor mu, encoder sayiyor mu"
sorusunu tek komutla yanitlamaktir.

Kullanim:
    python3 tools/pwm.py                      sol motor ileri, pwm 200, 2 sn
    python3 tools/pwm.py --wheel 1            sag motor
    python3 tools/pwm.py --dir -1             geri
    python3 tools/pwm.py --pwm 120 --ms 1000  daha yavas, 1 saniye
    python3 tools/pwm.py --both               once ileri sonra geri
    python3 tools/pwm.py --status             yalniz durumu yazdir
    python3 tools/pwm.py --watch              encoder sayacini canli izle (CPR icin)

Not: Bu script seri portu tek basina kullanir. ROS 2 koprusu calisirken port
mesgul olur; once koprunun durdurulmasi gerekir.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial kurulu degil:  pip install pyserial")

# Firmware motor_control.cpp icindeki tavanlarla ayni; buradan buyuk deger
# gonderilirse firmware zaten INVALID_PARAMETER dondurur.
JOG_MAX_PWM = 200
JOG_MAX_DURATION_MS = 2000


def encoder_sayaci(port, taraf):
    """STATUS ciktisindan tek bir tekerin count degerini okur."""
    port.reset_input_buffer()
    port.write(b"STATUS\n")
    time.sleep(0.7)
    anahtar = "left_count" if taraf == 0 else "right_count"
    for satir in port.read(8192).decode("utf-8", errors="replace").splitlines():
        if anahtar in satir:
            try:
                return int(satir.split("=")[1])
            except (IndexError, ValueError):
                return None
    return None


def darbe(port, teker, yon, pwm, sure_ms):
    once = encoder_sayaci(port, teker)
    port.reset_input_buffer()
    port.write(f"JOG {teker} {yon} {pwm} {sure_ms}\n".encode("ascii"))

    # Darbe suresi + firmware'in cevabini ve son telemetriyi yakalamak icin pay.
    time.sleep(sure_ms / 1000.0 + 1.0)
    cevap = port.read(8192).decode("utf-8", errors="replace")
    if "HATA" in cevap or "ERR" in cevap:
        for satir in cevap.splitlines():
            if "HATA" in satir or "ERR" in satir:
                print(f"  firmware reddetti: {satir.strip()}")
                return None

    sonra = encoder_sayaci(port, teker)
    if once is None or sonra is None:
        print("  encoder sayaci okunamadi (STATUS cevabi gelmedi)")
        return None

    delta = sonra - once
    hiz = abs(delta) / (sure_ms / 1000.0)
    ad = "SOL" if teker == 0 else "SAG"
    isaret = "ileri" if yon > 0 else "geri "
    print(f"  {ad} {isaret}  pwm={pwm:>3}  {once:>7} -> {sonra:<7}"
          f"  delta={delta:+7d}  ({hiz:6.0f} count/sn)")
    return delta


def izle(port, tur_sayisi):
    """Encoder sayacini canli gosterir; CPR kalibrasyonu icin kullanilir.

    Firmware zaten 500 ms'de bir V1 TEL yayinladigi icin komut gondermeye gerek
    yoktur; yalniz gelen telemetri okunur. Baslangic degeri sifir kabul edilir ve
    delta buna gore hesaplanir, boylece tekerlegi cevirirken net sayim gorulur.
    """
    print("Encoder sayaci izleniyor. Tekerlegi cevirin. Bitirmek icin Ctrl+C.\n")
    baslangic_sol = baslangic_sag = None
    tampon = ""
    try:
        while True:
            tampon += port.read(4096).decode("utf-8", errors="replace")
            *satirlar, tampon = tampon.split("\n")
            for satir in satirlar:
                if not satir.startswith("V1 TEL"):
                    continue
                alanlar = satir.split()
                if len(alanlar) < 16:
                    continue
                sol, sag = int(alanlar[7]), int(alanlar[8])
                if baslangic_sol is None:
                    baslangic_sol, baslangic_sag = sol, sag
                d_sol = sol - baslangic_sol
                d_sag = sag - baslangic_sag
                cpr = f"{abs(d_sol) / tur_sayisi:8.1f}" if tur_sayisi > 0 else "     ---"
                print(f"\r  SOL {sol:>8}  (delta {d_sol:+8d})   "
                      f"SAG {sag:>8}  (delta {d_sag:+8d})   "
                      f"SOL CPR({tur_sayisi} tur) = {cpr}   ",
                      end="", flush=True)
    except KeyboardInterrupt:
        print("\n")
        if baslangic_sol is None:
            print("Hic telemetri alinmadi.")
            return
        d_sol = sol - baslangic_sol
        d_sag = sag - baslangic_sag
        print(f"SONUC  SOL delta = {d_sol:+d}    SAG delta = {d_sag:+d}")
        if tur_sayisi > 0 and d_sol:
            print(f"       SOL effective CPR = |{d_sol}| / {tur_sayisi} tur "
                  f"= {abs(d_sol) / tur_sayisi:.1f} count/tur")
        if tur_sayisi > 0 and d_sag:
            print(f"       SAG effective CPR = |{d_sag}| / {tur_sayisi} tur "
                  f"= {abs(d_sag) / tur_sayisi:.1f} count/tur")


def main():
    ayristirici = argparse.ArgumentParser(description=__doc__,
                                          formatter_class=argparse.RawDescriptionHelpFormatter)
    ayristirici.add_argument("--port", default="/dev/ttyUSB0")
    ayristirici.add_argument("--wheel", type=int, choices=(0, 1), default=0,
                             help="0=sol, 1=sag")
    ayristirici.add_argument("--dir", type=int, choices=(-1, 1), default=1,
                             help="1=ileri, -1=geri")
    ayristirici.add_argument("--pwm", type=int, default=200,
                             help=f"1..{JOG_MAX_PWM}")
    ayristirici.add_argument("--ms", type=int, default=2000,
                             help=f"1..{JOG_MAX_DURATION_MS}")
    ayristirici.add_argument("--both", action="store_true",
                             help="once ileri, sonra geri darbe uygular")
    ayristirici.add_argument("--status", action="store_true",
                             help="motoru surmeden yalniz STATUS yazdirir")
    ayristirici.add_argument("--watch", action="store_true",
                             help="encoder sayacini canli izler; motoru surmez")
    ayristirici.add_argument("--turns", type=float, default=2.0,
                             help="--watch sirasinda CPR hesabi icin cevrilen tur sayisi")
    a = ayristirici.parse_args()

    if not 1 <= a.pwm <= JOG_MAX_PWM:
        sys.exit(f"pwm 1..{JOG_MAX_PWM} araliginda olmali")
    if not 1 <= a.ms <= JOG_MAX_DURATION_MS:
        sys.exit(f"ms 1..{JOG_MAX_DURATION_MS} araliginda olmali")

    try:
        port = serial.Serial(a.port, 115200, timeout=0.2)
    except serial.SerialException as hata:
        sys.exit(f"Seri port acilamadi ({a.port}): {hata}\n"
                 "ESP32 takili mi? ROS 2 koprusu calisiyor olabilir mi?")

    # Acilisla birlikte gelen telemetri birikintisini temizle.
    time.sleep(0.5)
    port.reset_input_buffer()

    try:
        if a.watch:
            izle(port, a.turns)
            return

        if a.status:
            port.write(b"STATUS\n")
            time.sleep(1.0)
            print(port.read(8192).decode("utf-8", errors="replace").strip())
            return

        print(f"=== JOG  teker={a.wheel}  pwm={a.pwm}  sure={a.ms} ms ===")
        darbe(port, a.wheel, a.dir, a.pwm, a.ms)
        if a.both:
            time.sleep(1.0)
            darbe(port, a.wheel, -a.dir, a.pwm, a.ms)
    finally:
        port.close()


if __name__ == "__main__":
    main()
