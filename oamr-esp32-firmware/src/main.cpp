#include <Arduino.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "motor_control.h"
#include "vehicle_kinematics.h"

namespace
{
// Serial hizi bilgisayardaki monitor ayariyla ayni olmalidir. 115200 bit/s,
// telemetry icin yeterince hizli ve ESP32 gelistirme kartlarinda yaygin bir degerdir.
constexpr uint32_t SERIAL_BAUD = 115200;

// Sabit boyutlu satir tamponu dinamik String nesnelerinin heap parcalanmasi riskini
// kaldirir. size_t negatif olamayan dizi boyutlari icin uygun unsigned tiptir.
constexpr size_t COMMAND_BUFFER_SIZE = 96;
char commandBuffer[COMMAND_BUFFER_SIZE] = {};
size_t commandLength = 0;
bool commandOverflow = false;

// Telemetry kontrol dongusunden daha yavas tutulur. millis() ile zaman kontrolu
// delay kullanmadan yapildigi icin encoder/PID dongusu seri yaziyi beklemez.
constexpr uint32_t TELEMETRY_PERIOD_MS = 500;
uint32_t lastTelemetryMs = 0;

// Son komut degerlerini saklamak STATUS ciktisinda girdiden motora tum zinciri
// gostermeyi saglar. Bunlar SI birimlerindedir: m/s ve rad/s.
float lastLinearXMps = 0.0F;
float lastAngularZRadS = 0.0F;
VehicleKinematics::Targets lastTargets{0.0F, 0.0F, 0.0F, true};

// Kalibrasyon baslangic/bitis snapshot'lari motor surmeden, elle tam 10 turden
// effective 1x counts-per-wheel-revolution hesaplamak icin tutulur.
constexpr float CALIBRATION_TURNS = 10.0F;
bool calibrationActive = false;
bool calibrationCandidateReady = false;
int32_t calibrationStartLeft = 0;
int32_t calibrationStartRight = 0;
float candidateLeftCpr = 0.0F;
float candidateRightCpr = 0.0F;

void printHelp()
{
    Serial.println(F("\nKOMUTLAR:"));
    Serial.println(F("  TWIST <linear_x[m/s]> <angular_z[rad/s]>"));
    Serial.println(F("  STOP                  : hedefleri sifirla ve PWM'i kes"));
    Serial.println(F("  STATUS                : tum veri zincirini yaz"));
    Serial.println(F("  CAL START             : iki encoder baslangic count'unu al"));
    Serial.println(F("  CAL END               : her teker 10 tur sonra aday CPR hesapla"));
    Serial.println(F("  CAL APPLY             : aday CPR degerlerini RAM'de uygula"));
    Serial.println(F("  SIGN <-1|1> <-1|1>    : ileri donuste encoder isaretlerini ata"));
    Serial.println(F("  GAINS <Kp> <Ki> <Kd>  : olculup ayarlanacak PID kazanimlari"));
    Serial.println(F("  ARM                    : tum ayarlar hazirsa motor cikisini ac"));
    Serial.println(F("  DISARM                 : motor cikisini aninda ve kalici kapat"));
    Serial.println(F("\nGuvenli sira: CAL -> SIGN -> GAINS -> ARM -> TWIST."));
    Serial.println(F("ARM sonrasi TWIST en gec 500 ms araliklarla yenilenmelidir."));
}

void printStatus()
{
    // State degerle dondurulur; telemetry modul icindeki degiskenleri degistiremez.
    const MotorControl::SystemState state = MotorControl::getState();
    constexpr float DEGREES_PER_RADIAN = 57.2957795131F;

    Serial.println(F("\nCMD:"));
    Serial.print(F("  linear_x       = "));
    Serial.print(lastLinearXMps, 4);
    Serial.println(F(" m/s"));
    Serial.print(F("  angular_z      = "));
    Serial.print(lastAngularZRadS, 4);
    Serial.println(F(" rad/s"));

    Serial.println(F("KINEMATICS:"));
    Serial.print(F("  valid          = "));
    Serial.println(lastTargets.valid ? F("true") : F("false"));
    Serial.print(F("  left_target    = "));
    Serial.print(lastTargets.leftWheelRadS, 4);
    Serial.println(F(" rad/s"));
    Serial.print(F("  right_target   = "));
    Serial.print(lastTargets.rightWheelRadS, 4);
    Serial.println(F(" rad/s"));
    Serial.print(F("  steering       = "));
    Serial.print(lastTargets.steeringAngleRad, 4);
    Serial.print(F(" rad = "));
    Serial.print(lastTargets.steeringAngleRad * DEGREES_PER_RADIAN, 2);
    Serial.println(F(" deg (yalniz sanal hedef; aktuator yok)"));

    Serial.println(F("ENCODERS:"));
    Serial.print(F("  left_count     = "));
    Serial.println(state.left.encoderCount);
    Serial.print(F("  right_count    = "));
    Serial.println(state.right.encoderCount);
    Serial.print(F("  left_speed     = "));
    Serial.print(state.left.measuredRadS, 4);
    Serial.println(F(" rad/s"));
    Serial.print(F("  right_speed    = "));
    Serial.print(state.right.measuredRadS, 4);
    Serial.println(F(" rad/s"));
    Serial.print(F("  CPR L/R        = "));
    Serial.print(state.left.countsPerWheelRev, 3);
    Serial.print(F(" / "));
    Serial.println(state.right.countsPerWheelRev, 3);
    Serial.print(F("  SIGN L/R       = "));
    Serial.print(state.left.encoderSign);
    Serial.print(F(" / "));
    Serial.println(state.right.encoderSign);

    Serial.println(F("CONTROL:"));
    Serial.print(F("  left_error     = "));
    Serial.print(state.left.errorRadS, 4);
    Serial.println(F(" rad/s"));
    Serial.print(F("  right_error    = "));
    Serial.print(state.right.errorRadS, 4);
    Serial.println(F(" rad/s"));
    Serial.print(F("  left_pwm/dir   = "));
    Serial.print(state.left.pwm);
    Serial.print(F(" / "));
    Serial.println(state.left.direction);
    Serial.print(F("  right_pwm/dir  = "));
    Serial.print(state.right.pwm);
    Serial.print(F(" / "));
    Serial.println(state.right.direction);
    Serial.print(F("  ready/armed    = "));
    Serial.print(state.feedbackReady ? F("true") : F("false"));
    Serial.print(F(" / "));
    Serial.println(state.armed ? F("true") : F("false"));
    Serial.print(F("  cmd_timeout    = "));
    Serial.println(state.commandTimedOut ? F("true") : F("false"));
}

void printPeriodicTelemetryNonBlocking()
{
    const MotorControl::SystemState state = MotorControl::getState();

    // Otomatik telemetry tek, kisa bir satira onceden bicimlendirilir. UART'in bos
    // alanı tum satira yetmiyorsa bu ornek atlanir; Serial.write'in tampon bosalana
    // kadar bekleyip 100 Hz kontrol dongusunu geciktirmesine izin verilmez. Ayrintili
    // ve cok satirli veri kullanici STATUS yazdiginda basilir.
    char line[128] = {};
    const int length = std::snprintf(
        line, sizeof(line),
        "TEL tgt=%.2f/%.2f meas=%.2f/%.2f pwm=%u/%u arm=%d timeout=%d\n",
        state.left.targetRadS, state.right.targetRadS,
        state.left.measuredRadS, state.right.measuredRadS,
        state.left.pwm, state.right.pwm,
        state.armed ? 1 : 0, state.commandTimedOut ? 1 : 0);

    if (length > 0 && length < static_cast<int>(sizeof(line)) &&
        Serial.availableForWrite() >= length)
    {
        Serial.write(reinterpret_cast<const uint8_t *>(line),
                     static_cast<size_t>(length));
    }
}

void startCalibration()
{
    // Kalibrasyon elle yapilir; olcum baslamadan motor cikisini kapatmak zorunludur.
    MotorControl::disarm();
    MotorControl::readEncoderCounts(calibrationStartLeft, calibrationStartRight);
    calibrationActive = true;
    calibrationCandidateReady = false;
    Serial.println(F("CAL basladi. Motorlar DISARM."));
    Serial.print(F("Baslangic raw count L/R: "));
    Serial.print(calibrationStartLeft);
    Serial.print(F(" / "));
    Serial.println(calibrationStartRight);
    Serial.println(F("Her tekeri elle TAM 10 tur cevirin, sonra CAL END yazin."));
}

void finishCalibration()
{
    if (!calibrationActive)
    {
        Serial.println(F("HATA: Once CAL START yazin."));
        return;
    }

    int32_t endLeft = 0;
    int32_t endRight = 0;
    MotorControl::readEncoderCounts(endLeft, endRight);
    const int32_t deltaLeft = endLeft - calibrationStartLeft;
    const int32_t deltaRight = endRight - calibrationStartRight;

    Serial.print(F("Bitis raw count L/R: "));
    Serial.print(endLeft);
    Serial.print(F(" / "));
    Serial.println(endRight);
    Serial.print(F("Delta count L/R: "));
    Serial.print(deltaLeft);
    Serial.print(F(" / "));
    Serial.println(deltaRight);

    calibrationActive = false;
    if (deltaLeft == 0 || deltaRight == 0)
    {
        calibrationCandidateReady = false;
        Serial.println(F("HATA: Iki tekerde de count gorulmedi; aday uygulanmadi."));
        return;
    }

    // abs(delta)/10 yonu kaldirir ve bir tam teker turundeki effective 1x count'u verir.
    candidateLeftCpr = std::fabs(static_cast<float>(deltaLeft)) / CALIBRATION_TURNS;
    candidateRightCpr = std::fabs(static_cast<float>(deltaRight)) / CALIBRATION_TURNS;
    calibrationCandidateReady = true;
    Serial.print(F("Aday effective CPR L/R: "));
    Serial.print(candidateLeftCpr, 3);
    Serial.print(F(" / "));
    Serial.println(candidateRightCpr, 3);
    Serial.println(F("Tur sayisini ve sonucu dogruladiysaniz CAL APPLY yazin."));
}

void applyCalibration()
{
    if (!calibrationCandidateReady ||
        !MotorControl::setCountsPerWheelRevolution(candidateLeftCpr, candidateRightCpr))
    {
        Serial.println(F("HATA: Uygulanabilir aday yok; CAL START/END yapin."));
        return;
    }
    calibrationCandidateReady = false;
    Serial.println(F("CPR RAM'e uygulandi. Yeniden baslatmada kalici degildir."));
}

void handleTwist(const char *line)
{
    float linearXMps = 0.0F;
    float angularZRadS = 0.0F;
    char trailing = '\0';

    // sscanf iki float'i ayirir. Sondaki %c, beklenmeyen ucuncu token varsa sonucu
    // 3 yaparak komutu reddetmemizi saglar; boylece yarim/hatalı komut hareket olmaz.
    if (std::sscanf(line, "TWIST %f %f %c", &linearXMps, &angularZRadS, &trailing) != 2 ||
        !std::isfinite(linearXMps) || !std::isfinite(angularZRadS))
    {
        Serial.println(F("HATA: Ornek kullanim: TWIST 0.20 0.00"));
        return;
    }

    const VehicleKinematics::Targets targets =
        VehicleKinematics::calculate(linearXMps, angularZRadS);
    lastLinearXMps = linearXMps;
    lastAngularZRadS = angularZRadS;
    lastTargets = targets;

    if (!targets.valid)
    {
        // Gecersiz Ackermann istegi once mevcut hareketi de durdurur; eski hedefin
        // calismaya devam etmesine izin verilmez.
        MotorControl::stop();
        Serial.println(F("HATA: Ackermann arac v=0 iken wz!=0 donusu yapamaz. STOP."));
        return;
    }

    const MotorControl::SystemState state = MotorControl::getState();
    if (!state.armed)
    {
        // Matematik DISARM iken denenebilir fakat hedef motor katmanina aktarilmaz.
        Serial.println(F("KINEMATIK HESAPLANDI; motor DISARM. STATUS ile sonucu gorun."));
        return;
    }

    MotorControl::setTargets(targets.leftWheelRadS, targets.rightWheelRadS, millis());
    Serial.println(F("TWIST kabul edildi. Guvenlik icin komutu <500 ms aralikla yenileyin."));
}

void handleCommand(const char *line)
{
    // strcmp tam eslesme ister; boylece STOPXYZ gibi belirsiz komutlar calismaz.
    if (std::strcmp(line, "HELP") == 0)
    {
        printHelp();
    }
    else if (std::strcmp(line, "STATUS") == 0)
    {
        printStatus();
    }
    else if (std::strcmp(line, "STOP") == 0)
    {
        MotorControl::stop();
        lastLinearXMps = 0.0F;
        lastAngularZRadS = 0.0F;
        lastTargets = VehicleKinematics::calculate(0.0F, 0.0F);
        Serial.println(F("STOP: hedefler ve PWM sifirlandi."));
    }
    else if (std::strcmp(line, "DISARM") == 0)
    {
        MotorControl::disarm();
        Serial.println(F("DISARM: fiziksel motor cikisi kapali."));
    }
    else if (std::strcmp(line, "ARM") == 0)
    {
        Serial.println(MotorControl::arm()
                           ? F("ARM basarili. Motor hala STOP; simdi TWIST gonderin.")
                           : F("HATA: ARM olmadi. CPR, SIGN ve GAINS eksik olabilir."));
    }
    else if (std::strcmp(line, "CAL START") == 0)
    {
        startCalibration();
    }
    else if (std::strcmp(line, "CAL END") == 0)
    {
        finishCalibration();
    }
    else if (std::strcmp(line, "CAL APPLY") == 0)
    {
        applyCalibration();
    }
    else if (std::strncmp(line, "TWIST ", 6) == 0)
    {
        handleTwist(line);
    }
    else if (std::strncmp(line, "SIGN ", 5) == 0)
    {
        int leftSign = 0;
        int rightSign = 0;
        char trailing = '\0';
        if (std::sscanf(line, "SIGN %d %d %c", &leftSign, &rightSign, &trailing) == 2 &&
            (leftSign == -1 || leftSign == 1) &&
            (rightSign == -1 || rightSign == 1) &&
            MotorControl::setEncoderSigns(static_cast<int8_t>(leftSign),
                                          static_cast<int8_t>(rightSign)))
        {
            Serial.println(F("Encoder isaretleri uygulandi; guvenlik icin DISARM."));
        }
        else
        {
            Serial.println(F("HATA: SIGN degerleri yalniz -1 veya 1 olabilir."));
        }
    }
    else if (std::strncmp(line, "GAINS ", 6) == 0)
    {
        float newKp = 0.0F;
        float newKi = 0.0F;
        float newKd = 0.0F;
        char trailing = '\0';
        if (std::sscanf(line, "GAINS %f %f %f %c", &newKp, &newKi, &newKd, &trailing) == 3 &&
            MotorControl::setPidGains(newKp, newKi, newKd))
        {
            Serial.println(F("PID kazanimlari uygulandi; guvenlik icin DISARM."));
        }
        else
        {
            Serial.println(F("HATA: Kazanimlar >=0 ve en az biri >0 olmali."));
        }
    }
    else
    {
        // Hatalı komut hedefi degistirmez; kullanici STOP'u her zaman ayrica verebilir.
        Serial.println(F("HATA: Bilinmeyen komut. HELP yazin."));
    }
}

void readSerialNonBlocking()
{
    // available() kadar byte okunur; veri yoksa fonksiyon beklemez. Bu sayede seri
    // giris kontrol dongusunun sabit ornekleme zamanini bozmaz.
    while (Serial.available() > 0)
    {
        const char received = static_cast<char>(Serial.read());
        if (received == '\r')
        {
            continue;  // Windows CRLF icindeki CR yok sayilir; LF satiri tamamlar.
        }
        if (received == '\n')
        {
            if (commandOverflow)
            {
                Serial.println(F("HATA: Komut satiri cok uzun; yok sayildi."));
            }
            else if (commandLength > 0)
            {
                commandBuffer[commandLength] = '\0';
                handleCommand(commandBuffer);
            }
            commandLength = 0;
            commandOverflow = false;
            continue;
        }

        // Son byte C string sonlandiricisi '\0' icin ayrilir. Tasan satirin parcasi
        // calistirilmaz; newline gelene kadar tamamı guvenli bicimde atilir.
        if (commandLength < COMMAND_BUFFER_SIZE - 1)
        {
            commandBuffer[commandLength++] = received;
        }
        else
        {
            commandOverflow = true;
        }
    }
}
}  // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD);
    MotorControl::begin();

    // Boot'ta begin() cikislari LOW/PWM=0 yapar; kullanici calibration, gains ve ARM
    // adimlarini tamamlamadan hicbir otomatik sweep veya motor hareketi yapilmaz.
    Serial.println();
    Serial.println(F("ESP32 iki motor low-level egitim sistemi hazir."));
    Serial.println(F("Motorlar DISARM ve STOP. Komutlar icin HELP yazin."));
}

void loop()
{
    // micros() kontrol dt'si icin daha ince, millis() timeout/telemetry icin yeterli
    // cozunurluk sunar. Her ikisi de unsigned tasma-guvenli cikarma ile kullanilir.
    const uint32_t nowMicros = micros();
    const uint32_t nowMillis = millis();

    readSerialNonBlocking();
    MotorControl::update(nowMicros, nowMillis);

    if (nowMillis - lastTelemetryMs >= TELEMETRY_PERIOD_MS)
    {
        lastTelemetryMs = nowMillis;
        printPeriodicTelemetryNonBlocking();
    }
}
