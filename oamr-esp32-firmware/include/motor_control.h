// Bu header motor kontrol modulunun disariya sundugu arayuzdur. Uygulama ayrintilari
// .cpp dosyasinda kalir; main.cpp pinlere veya interrupt sayaclarina dogrudan erismez.
#pragma once

#include <Arduino.h>

namespace MotorControl
{
// Her tekerin hedef, olcum ve cikis degerlerini ayni anda raporlayan struct,
// telemetry'nin birbiriyle iliskili alanlari acik isimlerle okumasini saglar.
struct WheelState
{
    int32_t encoderCount;       // 1x quadrature ham/yon-duzeltilmis toplam count.
    float targetRadS;           // Istenen teker hizi [rad/s].
    float measuredRadS;         // Encoder ile hesaplanan teker hizi [rad/s].
    float errorRadS;            // target - measured [rad/s].
    float controllerOutput;     // Isaretli, doyurulmus PWM komutu [-255, 255].
    uint8_t pwm;                // Direction'dan ayrilmis PWM buyuklugu [0, 255].
    int8_t direction;           // -1 geri, 0 stop, +1 ileri.
    float countsPerWheelRev;    // Deneysel efektif count/teker turu.
    int8_t encoderSign;         // Fiziksel montaja gore -1 veya +1; 0 bilinmiyor.
};

struct SystemState
{
    WheelState left;
    WheelState right;
    bool armed;                 // Kullanici fiziksel cikisa acikca izin verdi mi?
    bool feedbackReady;         // CPR, encoder isareti ve kazanc bilgisi tamam mi?
    bool commandTimedOut;       // Son hareket komutu guvenlik suresini asti mi?
};

// setup() yalniz main.cpp'de kalir; begin() onun icinden bir kez cagrilir.
void begin();

// Non-blocking update kontrol zamani geldiginde hesap yapar, aksi halde hemen doner.
void update(uint32_t nowMicros, uint32_t nowMillis);

// Hedefler signed rad/s'dir. nowMillis timeout sayacini guncellemek icin aktarilir.
void setTargets(float leftRadS, float rightRadS, uint32_t nowMillis);
void stop();

// ARM ancak tum fiziksel parametreler hazirken basarili olur. DISARM PWM'i aninda keser.
bool arm();
void disarm();

// CPR ve encoder isaretleri tahmin edilmez; calibration ve yon testi sonrasinda
// kullanici tarafindan bu fonksiyonlarla verilir.
bool setCountsPerWheelRevolution(float leftCpr, float rightCpr);
bool setEncoderSigns(int8_t leftSign, int8_t rightSign);

// PID kazanimlari runtime'da verilir. En az bir pozitif kazanc olmadan sistem hazir
// sayilmaz; sifir varsayilan deger motorun kazara calismamasini saglar.
bool setPidGains(float kp, float ki, float kd);

// Acik-cevrim tezgah darbesi: CPR/SIGN/GAINS/ARM zincirini atlar, PID calistirmaz.
// PWM buyuklugu ve sure motor_control.cpp icindeki tavanlarla sinirlanir; sure
// dolunca cikislar otomatik kesilir. Once mevcut kontrolu disarm eder.
bool jog(uint8_t wheel, int8_t direction, uint8_t pwmMagnitude, uint32_t durationMs,
         uint32_t nowMillis);

// Encoder sayaclari ISR tarafindan degistigi icin tutarli kopya critical section'da
// alinir. Reference (&) cikislari iki degeri tek fonksiyonla kopyalamaya yarar.
void readEncoderCounts(int32_t &leftCount, int32_t &rightCount);
SystemState getState();
}  // namespace MotorControl
