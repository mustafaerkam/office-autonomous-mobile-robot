#!/usr/bin/env python3
"""Kok klasordeki .vscode/c_cpp_properties.json dosyasini, PlatformIO'nun
oamr-esp32-firmware/.vscode/ altina urettigi gercek derleme konfigurasyonundan
yeniden uretir.

Neden gerekli: VS Code C/C++ eklentisi c_cpp_properties.json dosyasini yalnizca
acik olan workspace klasorunun kokunde arar. Kok klasor olarak
office-autonomous-mobile-robot acildiginda PlatformIO'nun alt klasordeki dosyasi
yok sayilir ve <Arduino.h> bulunamaz (hata 1696).

Tercih edilen kullanim oamr.code-workspace dosyasini acmaktir; o zaman bu script
gerekmez. Kok klasoru dogrudan acmaya devam edenler icin:

    python3 tools/sync-vscode-cpp-config.py

PlatformIO framework/platform surumu her degistiginde tekrar calistirin.
"""

import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PIO_CONFIG = os.path.join(ROOT, "oamr-esp32-firmware", ".vscode", "c_cpp_properties.json")
OUT_CONFIG = os.path.join(ROOT, ".vscode", "c_cpp_properties.json")

ROS_DISTRO = os.environ.get("ROS_DISTRO", "humble")


def load_jsonc(path):
    """PlatformIO ciktisi basinda // yorum satirlari icerir, JSON parser bunu kabul etmez."""
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    return json.loads(re.sub(r"^\s*//.*$", "", text, flags=re.M))


def main():
    if not os.path.isfile(PIO_CONFIG):
        sys.exit(
            "PlatformIO konfigurasyonu bulunamadi: %s\n"
            "Once firmware klasorunde bir kez derleyin: pio run" % PIO_CONFIG
        )

    pio = load_jsonc(PIO_CONFIG)["configurations"][0]

    esp32 = {
        "name": "ESP32 (PlatformIO)",
        "includePath": [p for p in pio["includePath"] if p],
        # PlatformIO listelerin sonuna bos string birakiyor; -D"" derleyiciyi kizdirir.
        "defines": [d for d in pio.get("defines", []) if d],
        "cStandard": pio.get("cStandard", "gnu99"),
        "cppStandard": pio.get("cppStandard", "gnu++11"),
        "compilerPath": pio["compilerPath"],
        "compilerArgs": [a for a in pio.get("compilerArgs", []) if a],
        "browse": {
            "path": [p for p in pio.get("browse", {}).get("path", []) if p],
            "limitSymbolsToIncludedHeaders": True,
        },
    }

    ros2 = {
        "name": "ROS 2 (%s)" % ROS_DISTRO,
        "includePath": [
            "${workspaceFolder}/oamr-ros2-ws/src/**",
            "${workspaceFolder}/oamr-ros2-ws/install/**/include/**",
            "/opt/ros/%s/include/**" % ROS_DISTRO,
        ],
        "defines": [],
        "cStandard": "gnu11",
        "cppStandard": "c++17",
        "compilerPath": "/usr/bin/g++",
        "intelliSenseMode": "linux-gcc-x64",
        "browse": {
            "path": [
                "${workspaceFolder}/oamr-ros2-ws/src",
                "/opt/ros/%s/include" % ROS_DISTRO,
            ],
            "limitSymbolsToIncludedHeaders": True,
        },
    }

    os.makedirs(os.path.dirname(OUT_CONFIG), exist_ok=True)
    with open(OUT_CONFIG, "w", encoding="utf-8") as handle:
        json.dump({"configurations": [esp32, ros2], "version": 4}, handle, indent=2)
        handle.write("\n")

    print("Yazildi: %s" % OUT_CONFIG)
    print("  ESP32 includePath: %d yol, %d define" % (len(esp32["includePath"]), len(esp32["defines"])))
    print("  compilerPath: %s" % esp32["compilerPath"])


if __name__ == "__main__":
    main()
