// #pragma once bu bildirim dosyasinin ayni .cpp dosyasina dolayli yollarla
// birden fazla kez eklenmesini engeller. Boylece struct ve fonksiyonlar ikinci
// kez tanimlanmaz. Klasik header guard ile ayni amaca daha sade bicimde hizmet eder.
#pragma once

// Bu dosya yalnizca arac geometrisi ile ilgili tipleri ve fonksiyonlari bildirir.
// GPIO, encoder ve PWM ayrintilarinin burada bulunmamasi, matematigin masaustu
// bilgisayarda bile donanimdan bagimsiz test edilebilmesini saglar.
namespace VehicleKinematics
{
// struct, ayni hesaplamaya ait birden cok sonucu tek anlamli veri paketi halinde
// dondurur. float ESP32'de yeterli hassasiyetle daha az bellek/islem maliyeti sunar.
struct Targets
{
    float leftWheelRadS;     // Sol teker hedef acisal hizi [rad/s].
    float rightWheelRadS;    // Sag teker hedef acisal hizi [rad/s].
    float steeringAngleRad;  // Sanal Ackermann direksiyon acisi [rad].
    bool valid;              // Istenen hareket fiziksel modelle uyumlu mu?
};

// Degerle alinan float parametreler cok kucuk oldugu icin pointer/reference
// kullanmak fayda saglamaz. Fonksiyon yan etkisizdir: ayni girdiye ayni sonucu verir.
Targets calculate(float linearXMps, float angularZRadS);
}  // namespace VehicleKinematics
