#include "motor_control.h"

#include "commands.h"
#include "vehicle_config.h"

#include <cmath>

namespace MotorControl
{
namespace
{
// Pin sabitleri kablo semasinin yazilim karsiligidir. constexpr kullanmak yanlislikla
// calisma aninda degistirilmelerini engeller ve pin eslesmesini tek yerde toplar.
//
// SOL MOTOR (1. motor) -- fiziksel olarak baglanmis ve olculmustur:
//   encoder B  -> GPIO16      encoder A  -> GPIO17
//   surucu IN1 -> GPIO18      surucu IN2 -> GPIO19
//   surucu ENA -> GPIO25
//   Motorun (-) ucu OUT1'e, (+) ucu OUT2'ye baglidir. Bu, IN1=HIGH iken motorun
//   hangi yone donecegini belirler; "ileri" olup olmadigi ancak yon testiyle
//   anlasilir. Ters cikarsa OUT1/OUT2 kablolari fiziksel olarak degistirilmelidir.
constexpr uint8_t LEFT_ENCODER_A_PIN = 17;
constexpr uint8_t LEFT_ENCODER_B_PIN = 16;
constexpr uint8_t LEFT_IN1_PIN = 18;
constexpr uint8_t LEFT_IN2_PIN = 19;
// GPIO25 strapping pini degildir ve boot sirasinda serbest kalir; ENA icin GPIO5'e
// gore daha guvenli bir secimdir (GPIO5 boot boyunca dahili pull-up ile HIGH durur).
constexpr uint8_t LEFT_PWM_PIN = 25;

// SAG MOTOR (2. motor) -- henuz fiziksel olarak baglanmadi; asagidaki degerler
// planlanan pinlerdir ve kablolama yapilinca dogrulanmalidir.
constexpr uint8_t RIGHT_ENCODER_A_PIN = 32;
constexpr uint8_t RIGHT_ENCODER_B_PIN = 33;
constexpr uint8_t RIGHT_IN1_PIN = 26;
constexpr uint8_t RIGHT_IN2_PIN = 27;
constexpr uint8_t RIGHT_PWM_PIN = 13;

// Projede kurulu Arduino-ESP32 2.0.17 oldugu icin kanal tabanli ledcSetup ve
// ledcAttachPin API'si kullanilir. Iki ayri kanal iki motorun duty degerlerini
// birbirinden bagimsiz yapar.
constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;

// L298N'nin bipolar transistor yapisi icin 1 kHz pratik bir baslangictir. 8 bit
// resolution, duty komutunu 0..255 araligina boler. PWM elektriksel ortalama gerilimi
// etkiler; yuk, surtunme ve besleme nedeniyle PWM sayisi fiziksel hiz demek degildir.
constexpr uint32_t PWM_FREQUENCY_HZ = 1000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr float MAX_PWM = 255.0F;

// 10 ms kontrol periyodu 100 Hz demektir. Hiz olcumu ve integral/turev hesaplarinin
// anlamli olmasi icin duzenli ornekleme gerekir. Telemetry bundan daha yavas basilir.
constexpr uint32_t CONTROL_PERIOD_US = 10000;
constexpr float RADIANS_PER_REVOLUTION = 6.28318530718F;
constexpr float ZERO_TARGET_RAD_S = 1.0e-4F;

// volatile, bu sayaclarin normal loop disinda ISR tarafindan degisebilecegini
// derleyiciye bildirir. volatile tek basina atomik/tutarli snapshot garantisi vermez;
// bu nedenle asagida critical section da kullanilir.
volatile int32_t leftEncoderCount = 0;
volatile int32_t rightEncoderCount = 0;


// Iki ISR ve normal program ayni sayac verisine ulasirken bu kilit cok kisa sure
// tutulur. ESP32'nin iki cekirdeginde yalniz interrupts() kullanmaktan daha guvenlidir.
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

struct ControllerMemory
{
    float integral;
    float previousError;
    int32_t previousCount;
};

WheelState leftState{0, 0.0F, 0.0F, 0.0F, 0.0F, 0, 0, 0.0F, 0};
WheelState rightState{0, 0.0F, 0.0F, 0.0F, 0.0F, 0, 0, 0.0F, 0};
ControllerMemory leftController{0.0F, 0.0F, 0};
ControllerMemory rightController{0.0F, 0.0F, 0};

// Kazanimlar kasitli olarak sifirdir; motor/aktarim olculmeden guvenli bir PID
// katsayisi uydurulamaz. Kullanici GAINS komutuyla deneysel deger girmelidir.
float kp = 0.0F;
float ki = 0.0F;
float kd = 0.0F;
bool gainsConfigured = false;
bool armed = false;
bool commandTimedOut = true;
uint32_t lastCommandMs = 0;
uint32_t previousControlUs = 0;

// Jog tavanlari kasitli olarak MAX_PWM'in ve komut zaman asiminin altindadir:
// tezgahta motoru gormeye yeter, kacak bir komut zarar veremez.
constexpr uint8_t JOG_MAX_PWM = 200U;
constexpr uint32_t JOG_MAX_DURATION_MS = 2000U;
bool jogActive = false;
uint32_t jogStartMillis = 0;
uint32_t jogDurationMs = 0;

// ISR (interrupt service routine) encoder A'nin yalniz RISING kenarinda calisir.
// B seviyesine bakarak yon secmek "1x decoding"dir: A'nin iki kenari ve B kenarlari
// sayilmaz. Bu nedenle kalibre edilen effective CPR tam olarak bu 1x yonteme aittir.
// IRAM_ATTR, fonksiyonu flash erisiminin gecici olarak kullanilamadigi anda da hizli
// erisilebilen RAM bolgesine yerlestirmesini ister. ISR'da Serial, delay, PID veya
// floating-point hesap yoktur; interrupt gecikmesini kisa tutar.
void IRAM_ATTR leftEncoderIsr()
{
    const bool phaseBHigh = digitalRead(LEFT_ENCODER_B_PIN);
    portENTER_CRITICAL_ISR(&encoderMux);
    leftEncoderCount += phaseBHigh ? 1 : -1;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR rightEncoderIsr()
{
    const bool phaseBHigh = digitalRead(RIGHT_ENCODER_B_PIN);
    portENTER_CRITICAL_ISR(&encoderMux);
    rightEncoderCount += phaseBHigh ? 1 : -1;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

float clampValue(const float value, const float minimum, const float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

bool feedbackIsReady()
{
    const bool cprReady = leftState.countsPerWheelRev > 0.0F &&
                          rightState.countsPerWheelRev > 0.0F;
    const bool signsReady = std::abs(leftState.encoderSign) == 1 &&
                            std::abs(rightState.encoderSign) == 1;
    return cprReady && signsReady && gainsConfigured;
}

void writeMotor(const uint8_t in1, const uint8_t in2, const uint8_t pwmChannel,
                const float signedOutput, WheelState &state)
{
    // Kontrol cikisi isaret ve magnitude olarak ayrilir. Direction pinleri enerji
    // yonunu, PWM ise o yondeki duty miktarini belirler.
    const float saturated = clampValue(signedOutput, -MAX_PWM, MAX_PWM);
    state.controllerOutput = saturated;

    if (std::fabs(saturated) < 0.5F)
    {
        state.direction = 0;
        state.pwm = 0;
        ledcWrite(pwmChannel, 0);
        // Iki girisin LOW olmasi L298N kanalini guvenli coast/stop durumuna getirir.
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        return;
    }

    state.direction = saturated > 0.0F ? 1 : -1;
    state.pwm = static_cast<uint8_t>(std::round(std::fabs(saturated)));
    digitalWrite(in1, state.direction > 0 ? HIGH : LOW);
    digitalWrite(in2, state.direction > 0 ? LOW : HIGH);
    ledcWrite(pwmChannel, state.pwm);
}

void hardStopOutputs()
{
    // Once PWM'i sifirlamak yon pinleri degisirken motora darbe gitmesini engeller.
    ledcWrite(LEFT_PWM_CHANNEL, 0);
    ledcWrite(RIGHT_PWM_CHANNEL, 0);
    digitalWrite(LEFT_IN1_PIN, LOW);
    digitalWrite(LEFT_IN2_PIN, LOW);
    digitalWrite(RIGHT_IN1_PIN, LOW);
    digitalWrite(RIGHT_IN2_PIN, LOW);
    leftState.pwm = 0;
    rightState.pwm = 0;
    leftState.direction = 0;
    rightState.direction = 0;
    leftState.controllerOutput = 0.0F;
    rightState.controllerOutput = 0.0F;
}

float runPid(WheelState &state, ControllerMemory &memory, const float dtSeconds)
{
    // Hata, fiziksel olarak istenen hiz ile encoderin gosterdigi hiz farkidir.
    state.errorRadS = state.targetRadS - state.measuredRadS;

    // Sifir hedefte integrali sifirlamak, onceki hareketten biriken enerjinin motoru
    // tekrar itmesini onler. Motor hiz kontrolunde once P/PI ile baslamak genellikle
    // daha sadedir; turev terimi encoder nicemleme gurultusunu buyutebilir.
    if (std::fabs(state.targetRadS) < ZERO_TARGET_RAD_S)
    {
        memory.integral = 0.0F;
        memory.previousError = state.errorRadS;
        return 0.0F;
    }

    const float derivative = (state.errorRadS - memory.previousError) / dtSeconds;
    const float candidateIntegral = memory.integral + (state.errorRadS * dtSeconds);
    const float candidateOutput = (kp * state.errorRadS) +
                                  (ki * candidateIntegral) +
                                  (kd * derivative);
    const float saturatedOutput = clampValue(candidateOutput, -MAX_PWM, MAX_PWM);

    // Saturation, L298N/PWM'nin uretebilecegi siniri asan komutu 255'te keser.
    // Cikis zaten sinirda ve hata onu daha da ayni yone itiyorsa integrali kabul
    // etmeyerek integral windup'i onleriz. Hata ters yone donunce integral cozulur.
    const bool pushingFurtherIntoSaturation =
        (candidateOutput > MAX_PWM && state.errorRadS > 0.0F) ||
        (candidateOutput < -MAX_PWM && state.errorRadS < 0.0F);
    if (!pushingFurtherIntoSaturation)
    {
        memory.integral = candidateIntegral;
    }

    memory.previousError = state.errorRadS;
    return saturatedOutput;
}

}  // namespace

void begin()
{
    // Pinler OUTPUT olur olmaz LOW yazilarak ESP32 boot sonrasi motorun kendiliginden
    // hareket etmemesi saglanir. PWM kurulmadan once de yon cikislari guvenlidir.
    pinMode(LEFT_IN1_PIN, OUTPUT);
    pinMode(LEFT_IN2_PIN, OUTPUT);
    pinMode(RIGHT_IN1_PIN, OUTPUT);
    pinMode(RIGHT_IN2_PIN, OUTPUT);
    digitalWrite(LEFT_IN1_PIN, LOW);
    digitalWrite(LEFT_IN2_PIN, LOW);
    digitalWrite(RIGHT_IN1_PIN, LOW);
    digitalWrite(RIGHT_IN2_PIN, LOW);

    // Arduino-ESP32 2.x LEDC API'sinde frekans/cozunurluk kanala kurulur, sonra pin
    // o kanala baglanir. ledcWrite da pin degil kanal numarasi alir.
    ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(LEFT_PWM_PIN, LEFT_PWM_CHANNEL);
    ledcAttachPin(RIGHT_PWM_PIN, RIGHT_PWM_CHANNEL);
    hardStopOutputs();

    // INPUT_PULLUP acik-kollektor olabilen encoder cikisini bosta kalmaktan korur.
    pinMode(LEFT_ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(LEFT_ENCODER_B_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_B_PIN, INPUT_PULLUP);

    // A fazinin RISING kenari interrupt kaynagidir; B fazi ISR icinde yon icin okunur.
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A_PIN), leftEncoderIsr, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A_PIN), rightEncoderIsr, RISING);

    readEncoderCounts(leftController.previousCount, rightController.previousCount);
    previousControlUs = micros();

}

void update(const uint32_t nowMicros, const uint32_t nowMillis)
{
    // Unsigned cikarma millis/micros sayaci tastiginda da dogru sure farkini verir.
    // Periyot dolmadiysa beklemek yerine hemen donmek non-blocking programlamadir;
    // seri komutlar ve guvenlik kontrolleri bu sirada yanit verebilir.
    const uint32_t elapsedUs = nowMicros - previousControlUs;
    if (elapsedUs < CONTROL_PERIOD_US)
    {
        return;
    }
    previousControlUs = nowMicros;
    const float dtSeconds = static_cast<float>(elapsedUs) / 1000000.0F;

    int32_t rawLeft = 0;
    int32_t rawRight = 0;
    readEncoderCounts(rawLeft, rawRight);

    const int32_t deltaLeft = rawLeft - leftController.previousCount;
    const int32_t deltaRight = rawRight - rightController.previousCount;
    leftController.previousCount = rawLeft;
    rightController.previousCount = rawRight;

    // Isaret bilinmiyorsa ham count telemetry'de gorulur ama gercek hiz hesaplanmaz.
    leftState.encoderCount = rawLeft * (leftState.encoderSign == 0 ? 1 : leftState.encoderSign);
    rightState.encoderCount = rawRight * (rightState.encoderSign == 0 ? 1 : rightState.encoderSign);
    leftState.measuredRadS = 0.0F;
    rightState.measuredRadS = 0.0F;

    if (leftState.countsPerWheelRev > 0.0F && std::abs(leftState.encoderSign) == 1)
    {
        // delta count / CPR = teker turu; tur * 2*pi = radyan; radyan / dt = rad/s.
        leftState.measuredRadS =
            (static_cast<float>(deltaLeft * leftState.encoderSign) /
             leftState.countsPerWheelRev) * RADIANS_PER_REVOLUTION / dtSeconds;
    }
    if (rightState.countsPerWheelRev > 0.0F && std::abs(rightState.encoderSign) == 1)
    {
        rightState.measuredRadS =
            (static_cast<float>(deltaRight * rightState.encoderSign) /
             rightState.countsPerWheelRev) * RADIANS_PER_REVOLUTION / dtSeconds;
    }

    // Jog aktifken normal armed/PID yolu calistirilmaz; aksi halde asagidaki
    // "!armed" dali jog'un yazdigi PWM'i bir sonraki 10 ms tikte sifirlardi.
    // Sure dolunca cikislar kesilir ve normal akisa donulur.
    if (jogActive)
    {
        if (nowMillis - jogStartMillis >= jogDurationMs)
        {
            hardStopOutputs();
            jogActive = false;
        }
        return;
    }

    // Yalniz yeni ve gecerli hareket komutlari timeout saatini yeniler. Yaklasik
    // 500 ms komutsuzlukta hedefler sifirlanir ve PWM hemen kesilir.
    if (armed && (nowMillis - lastCommandMs > VehicleConfig::COMMAND_TIMEOUT_MS))
    {
        commandTimedOut = true;
        leftState.targetRadS = 0.0F;
        rightState.targetRadS = 0.0F;
    }

    if (!armed || !feedbackIsReady() || commandTimedOut)
    {
        leftController.integral = 0.0F;
        rightController.integral = 0.0F;
        leftState.errorRadS = leftState.targetRadS - leftState.measuredRadS;
        rightState.errorRadS = rightState.targetRadS - rightState.measuredRadS;
        hardStopOutputs();
        return;
    }

    const float leftOutput = runPid(leftState, leftController, dtSeconds);
    const float rightOutput = runPid(rightState, rightController, dtSeconds);
    writeMotor(LEFT_IN1_PIN, LEFT_IN2_PIN, LEFT_PWM_CHANNEL, leftOutput, leftState);
    writeMotor(RIGHT_IN1_PIN, RIGHT_IN2_PIN, RIGHT_PWM_CHANNEL, rightOutput, rightState);
}

void setTargets(const float leftRadS, const float rightRadS, const uint32_t nowMillis)
{
    // Disarmed durumda hedef biriktirmemek, daha sonra ARM denildiginde eski bir
    // komutun aniden motora uygulanmasini engeller.
    if (!armed || !std::isfinite(leftRadS) || !std::isfinite(rightRadS))
    {
        return;
    }
    leftState.targetRadS = leftRadS;
    rightState.targetRadS = rightRadS;
    lastCommandMs = nowMillis;
    commandTimedOut = false;
}

void stop()
{
    leftState.targetRadS = 0.0F;
    rightState.targetRadS = 0.0F;
    leftController.integral = 0.0F;
    rightController.integral = 0.0F;
    commandTimedOut = true;
    hardStopOutputs();
}

bool arm()
{
    if (!feedbackIsReady())
    {
        armed = false;
        return false;
    }
    // ARM sonrasi da ilk TWIST gelene kadar stop korunur.
    stop();
    armed = true;
    return true;
}

void disarm()
{
    armed = false;
    stop();
}

bool setCountsPerWheelRevolution(const float leftCpr, const float rightCpr)
{
    if (!std::isfinite(leftCpr) || !std::isfinite(rightCpr) ||
        leftCpr <= 0.0F || rightCpr <= 0.0F)
    {
        return false;
    }
    disarm();
    leftState.countsPerWheelRev = leftCpr;
    rightState.countsPerWheelRev = rightCpr;
    return true;
}

bool setEncoderSigns(const int8_t leftSign, const int8_t rightSign)
{
    if (std::abs(leftSign) != 1 || std::abs(rightSign) != 1)
    {
        return false;
    }
    disarm();
    leftState.encoderSign = leftSign;
    rightState.encoderSign = rightSign;
    return true;
}

bool setPidGains(const float newKp, const float newKi, const float newKd)
{
    // Negatif veya sayi olmayan kazanimlar bu basit kontrol yapisinda beklenmez.
    if (!std::isfinite(newKp) || !std::isfinite(newKi) || !std::isfinite(newKd) ||
        newKp < 0.0F || newKi < 0.0F || newKd < 0.0F ||
        (newKp == 0.0F && newKi == 0.0F && newKd == 0.0F))
    {
        return false;
    }
    disarm();
    kp = newKp;
    ki = newKi;
    kd = newKd;
    gainsConfigured = true;
    return true;
}

bool jog(const uint8_t wheel, const int8_t direction, const uint8_t pwmMagnitude,
         const uint32_t durationMs, const uint32_t nowMillis)
{
    // Sinirlar cagirandan bagimsiz olarak burada zorlanir; hicbir dispatcher
    // hatasi bu tavanlarin ustune cikamaz.
    if ((wheel != WHEEL_LEFT && wheel != WHEEL_RIGHT) ||
        (direction != -1 && direction != 1) ||
        pwmMagnitude == 0U || pwmMagnitude > JOG_MAX_PWM ||
        durationMs == 0U || durationMs > JOG_MAX_DURATION_MS)
    {
        return false;
    }

    // disarm() PID/ARM durumunu ve mevcut cikislari guvenle kapatir; jog PWM'i
    // ondan sonra dogrudan yazilir.
    disarm();

    WheelState &state = (wheel == WHEEL_LEFT) ? leftState : rightState;
    const uint8_t in1 = (wheel == WHEEL_LEFT) ? LEFT_IN1_PIN : RIGHT_IN1_PIN;
    const uint8_t in2 = (wheel == WHEEL_LEFT) ? LEFT_IN2_PIN : RIGHT_IN2_PIN;
    const uint8_t channel = (wheel == WHEEL_LEFT) ? LEFT_PWM_CHANNEL : RIGHT_PWM_CHANNEL;

    state.direction = direction;
    state.pwm = pwmMagnitude;
    digitalWrite(in1, direction > 0 ? HIGH : LOW);
    digitalWrite(in2, direction > 0 ? LOW : HIGH);
    ledcWrite(channel, pwmMagnitude);

    jogActive = true;
    jogStartMillis = nowMillis;
    jogDurationMs = durationMs;
    return true;
}

void readEncoderCounts(int32_t &leftCount, int32_t &rightCount)
{
    // Critical section boyunca ISR sayaclari degistiremez; iki count ayni zaman
    // penceresinden tutarli bir snapshot olarak kopyalanir. Bolge kasitli olarak kisadir.
    portENTER_CRITICAL(&encoderMux);
    leftCount = leftEncoderCount;
    rightCount = rightEncoderCount;
    portEXIT_CRITICAL(&encoderMux);
}

SystemState getState()
{
    // Deger dondurmek, dis kodun modulun ic durumunu pointer ile degistirmesini onler.
    return SystemState{leftState, rightState, armed, feedbackIsReady(), commandTimedOut};
}
}  // namespace MotorControl
