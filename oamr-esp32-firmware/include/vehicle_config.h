// Bu dosya ESP32 tarafinin derleme-zamani arac parametre kaynagidir.
//
// Insan-okunur ve ROS 2 tarafinin kullandigi ana referans `config/vehicle.yaml`
// dosyasidir. Arduino/ESP32 YAML dosyasini derleme aninda okuyamadigi icin burada
// ayni fiziksel degerlerin C++ karsiligi bulunur. Bir geometri veya timeout degeri
// degistirildiginde iki dosya birlikte guncellenmelidir.

#pragma once

#include <stdint.h>

namespace VehicleConfig
{
// SI birimleri kullanilir: metre, radyan ve milisaniye. constexpr, bu fiziksel
// degerlerin calisma aninda degistirilememesini ve derleyicinin sabit olarak
// kullanabilmesini saglar.
constexpr float WHEEL_RADIUS_M = 0.035F;
constexpr float WHEELBASE_M = 0.170F;
constexpr float REAR_TRACK_M = 0.150F;

// Haberlesme kesilirse ESP32 bu sure sonunda hedefleri sifirlayip PWM cikislarini
// kapatir. Bu deger, YAML dosyasindaki safety.command_timeout_ms ile aynidir.
constexpr uint32_t COMMAND_TIMEOUT_MS = 500U;
}  // namespace VehicleConfig
