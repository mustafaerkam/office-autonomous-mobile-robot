#include <Arduino.h>

#include <cmath>
#include <cstdio>

#include "commands.h"
#include "motor_control.h"
#include "serial_protocol.h"
#include "vehicle_kinematics.h"

namespace
{
// Serial hizi bilgisayardaki monitor veya gelecekteki ROS 2 bridge ayariyla ayni
// olmalidir. USB seri uzerinde 115200 baud, komut ve dusuk frekansli telemetry icin
// yeterlidir; kontrol dongusunun hizi bundan bagimsiz olarak motor_control'dedir.
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t TELEMETRY_PERIOD_MS = 500;
constexpr float CALIBRATION_TURNS = 10.0F;

// Sabit boyutlu tampon dinamik String tahsisi yapmaz. Uzun veya bozuk bir satir
// newline'a kadar atilir; parcasinin komut olarak calismasi engellenir.
char commandBuffer[PROTOCOL_COMMAND_BUFFER_SIZE] = {};
size_t commandLength = 0U;
bool commandOverflow = false;
bool commandMayBeMachineFormat = false;

uint32_t lastTelemetryMs = 0U;
uint32_t lastMachineSequence = 0U;

// Son Twist degerleri STATUS ile tum donusum zincirini gostermek icin saklanir.
float lastLinearXMps = 0.0F;
float lastAngularZRadS = 0.0F;
VehicleKinematics::Targets lastTargets{0.0F, 0.0F, 0.0F, true};

// CPR kalibrasyonu motor surmeden elle on tur sayim farkini olcmek icin durum tutar.
bool calibrationActive = false;
bool calibrationCandidateReady = false;
int32_t calibrationStartLeft = 0;
int32_t calibrationStartRight = 0;
float candidateLeftCpr = 0.0F;
float candidateRightCpr = 0.0F;

void writeMachineAcknowledgement(const SerialProtocol::Command &command)
{
    if (!command.machineFormat)
    {
        return;
    }
    Serial.printf("V%u ACK %lu %s\n", PROTOCOL_VERSION,
                  static_cast<unsigned long>(command.sequence),
                  SerialProtocol::commandName(command.id));
}

void writeMachineError(const SerialProtocol::Command &command, const uint8_t errorCode)
{
    if (command.machineFormat)
    {
        Serial.printf("V%u ERR %lu %s\n", PROTOCOL_VERSION,
                      static_cast<unsigned long>(command.sequence),
                      SerialProtocol::errorName(errorCode));
        return;
    }

    // Insan terminali eski komut deneyimini koruyan aciklayici bir metin gorur;
    // bridge ise bu dile baglanmadan yukaridaki sabit ERR adini kullanir.
    switch (errorCode)
    {
    case PROTOCOL_ERROR_NOT_ARMED:
        Serial.println(F("KINEMATIK HESAPLANDI; motor DISARM. STATUS ile sonucu gorun."));
        break;
    case PROTOCOL_ERROR_INVALID_KINEMATICS:
        Serial.println(F("HATA: Ackermann arac v=0 iken wz!=0 donusu yapamaz. STOP."));
        break;
    case PROTOCOL_ERROR_FEEDBACK_NOT_READY:
        Serial.println(F("HATA: ARM olmadi. CPR, SIGN ve GAINS eksik olabilir."));
        break;
    case PROTOCOL_ERROR_CALIBRATION_NOT_ACTIVE:
        Serial.println(F("HATA: Once CAL START yazin."));
        break;
    case PROTOCOL_ERROR_CALIBRATION_NOT_READY:
        Serial.println(F("HATA: Uygulanabilir kalibrasyon adayi yok."));
        break;
    default:
        Serial.print(F("HATA: "));
        Serial.println(SerialProtocol::errorName(errorCode));
        break;
    }
}

bool writeMachineTelemetry(const uint32_t sequence, const bool nonBlocking)
{
    const MotorControl::SystemState state = MotorControl::getState();
    char line[256] = {};

    // Alan sirasi protokol V1'in sozlesmesidir:
    // uptime, sequence, armed, feedbackReady, timeout, countlar, hedef/olcum rad/s,
    // sanal direksiyon rad ve PWM'ler. Birimler metin degil bu sabit sirayla tasinir.
    const int length = std::snprintf(
        line, sizeof(line),
        "V%u TEL %lu %lu %u %u %u %ld %ld %.4f %.4f %.4f %.4f %.4f %u %u\n",
        PROTOCOL_VERSION, static_cast<unsigned long>(millis()),
        static_cast<unsigned long>(sequence), state.armed ? 1U : 0U,
        state.feedbackReady ? 1U : 0U, state.commandTimedOut ? 1U : 0U,
        static_cast<long>(state.left.encoderCount), static_cast<long>(state.right.encoderCount),
        state.left.targetRadS, state.right.targetRadS,
        state.left.measuredRadS, state.right.measuredRadS,
        lastTargets.steeringAngleRad, state.left.pwm, state.right.pwm);

    if (length <= 0 || length >= static_cast<int>(sizeof(line)))
    {
        return false;
    }
    if (nonBlocking && Serial.availableForWrite() < length)
    {
        // Periyodik telemetry bilgi amaclidir; UART doluysa kontrol dongusunu
        // bekletmek yerine bu ornek atlanir. STATUS cevabi ise bloklayabilir.
        return false;
    }

    Serial.write(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(length));
    return true;
}

void printHelp()
{
    Serial.println(F("\nINSAN TERMINAL KOMUTLARI:"));
    Serial.println(F("  HELLO(v) | PING(p) | HELP(h) | STATUS(s)"));
    Serial.println(F("  TWIST/t <linear_x[m/s]> <angular_z[rad/s]>"));
    Serial.println(F("  STOP(x) | ARM(a) | DISARM(d)"));
    Serial.println(F("  CAL/c START | END | APPLY"));
    Serial.println(F("  SIGN/n <-1|1> <-1|1>"));
    Serial.println(F("  GAINS/g <Kp> <Ki> <Kd>"));
    Serial.println(F("\nMAKINE BICIMI:"));
    Serial.println(F("  V1 CMD <sequence> <yukaridaki-komut>"));
    Serial.println(F("  Ornek: V1 CMD 42 TWIST 0.30 0.50"));
    Serial.println(F("  Cevap: V1 ACK/ERR ...; telemetry: V1 TEL ..."));
    Serial.println(F("\nGuvenli sira: CAL -> SIGN -> GAINS -> ARM -> TWIST."));
    Serial.println(F("ARM sonrasi TWIST en gec 500 ms araliklarla yenilenmelidir."));
}

void printHumanStatus()
{
    const MotorControl::SystemState state = MotorControl::getState();
    constexpr float DEGREES_PER_RADIAN = 57.2957795131F;

    Serial.println(F("\nCMD:"));
    Serial.printf("  linear_x       = %.4f m/s\n", lastLinearXMps);
    Serial.printf("  angular_z      = %.4f rad/s\n", lastAngularZRadS);
    Serial.println(F("KINEMATICS:"));
    Serial.printf("  valid          = %s\n", lastTargets.valid ? "true" : "false");
    Serial.printf("  left_target    = %.4f rad/s\n", lastTargets.leftWheelRadS);
    Serial.printf("  right_target   = %.4f rad/s\n", lastTargets.rightWheelRadS);
    Serial.printf("  steering       = %.4f rad = %.2f deg (sanal; aktuator yok)\n",
                  lastTargets.steeringAngleRad,
                  lastTargets.steeringAngleRad * DEGREES_PER_RADIAN);
    Serial.println(F("ENCODERS:"));
    Serial.printf("  left_count     = %ld\n", static_cast<long>(state.left.encoderCount));
    Serial.printf("  right_count    = %ld\n", static_cast<long>(state.right.encoderCount));
    Serial.printf("  left_speed     = %.4f rad/s\n", state.left.measuredRadS);
    Serial.printf("  right_speed    = %.4f rad/s\n", state.right.measuredRadS);
    Serial.printf("  CPR L/R        = %.3f / %.3f\n",
                  state.left.countsPerWheelRev, state.right.countsPerWheelRev);
    Serial.printf("  SIGN L/R       = %d / %d\n",
                  state.left.encoderSign, state.right.encoderSign);
    Serial.println(F("CONTROL:"));
    Serial.printf("  left_error     = %.4f rad/s\n", state.left.errorRadS);
    Serial.printf("  right_error    = %.4f rad/s\n", state.right.errorRadS);
    Serial.printf("  left_pwm/dir   = %u / %d\n", state.left.pwm, state.left.direction);
    Serial.printf("  right_pwm/dir  = %u / %d\n", state.right.pwm, state.right.direction);
    Serial.printf("  ready/armed    = %s / %s\n",
                  state.feedbackReady ? "true" : "false", state.armed ? "true" : "false");
    Serial.printf("  cmd_timeout    = %s\n", state.commandTimedOut ? "true" : "false");
}

uint8_t startCalibration(const SerialProtocol::Command &command)
{
    MotorControl::disarm();
    MotorControl::readEncoderCounts(calibrationStartLeft, calibrationStartRight);
    calibrationActive = true;
    calibrationCandidateReady = false;

    if (command.machineFormat)
    {
        Serial.printf("V%u CAL START %lu %ld %ld\n", PROTOCOL_VERSION,
                      static_cast<unsigned long>(command.sequence),
                      static_cast<long>(calibrationStartLeft),
                      static_cast<long>(calibrationStartRight));
    }
    else
    {
        Serial.println(F("CAL basladi. Motorlar DISARM."));
        Serial.printf("Baslangic raw count L/R: %ld / %ld\n",
                      static_cast<long>(calibrationStartLeft),
                      static_cast<long>(calibrationStartRight));
        Serial.println(F("Her tekeri elle TAM 10 tur cevirin, sonra CAL END yazin."));
    }
    return PROTOCOL_ERROR_NONE;
}

uint8_t finishCalibration(const SerialProtocol::Command &command)
{
    if (!calibrationActive)
    {
        return PROTOCOL_ERROR_CALIBRATION_NOT_ACTIVE;
    }

    int32_t endLeft = 0;
    int32_t endRight = 0;
    MotorControl::readEncoderCounts(endLeft, endRight);
    const int32_t deltaLeft = endLeft - calibrationStartLeft;
    const int32_t deltaRight = endRight - calibrationStartRight;
    calibrationActive = false;

    if (deltaLeft == 0 || deltaRight == 0)
    {
        calibrationCandidateReady = false;
        return PROTOCOL_ERROR_CALIBRATION_NOT_READY;
    }

    // abs(delta)/10, secilen 1x decoder ile bir tam teker turundeki effective CPR'i
    // verir. Encoder yonu bu asamada onemsizdir; SIGN komutu ayrica belirlenir.
    candidateLeftCpr = std::fabs(static_cast<float>(deltaLeft)) / CALIBRATION_TURNS;
    candidateRightCpr = std::fabs(static_cast<float>(deltaRight)) / CALIBRATION_TURNS;
    calibrationCandidateReady = true;

    if (command.machineFormat)
    {
        Serial.printf("V%u CAL CANDIDATE %lu %ld %ld %.3f %.3f\n", PROTOCOL_VERSION,
                      static_cast<unsigned long>(command.sequence),
                      static_cast<long>(deltaLeft), static_cast<long>(deltaRight),
                      candidateLeftCpr, candidateRightCpr);
    }
    else
    {
        Serial.printf("Delta count L/R: %ld / %ld\n",
                      static_cast<long>(deltaLeft), static_cast<long>(deltaRight));
        Serial.printf("Aday effective CPR L/R: %.3f / %.3f\n",
                      candidateLeftCpr, candidateRightCpr);
        Serial.println(F("Tur sayisini ve sonucu dogruladiysaniz CAL APPLY yazin."));
    }
    return PROTOCOL_ERROR_NONE;
}

uint8_t applyCalibration(const SerialProtocol::Command &command)
{
    if (!calibrationCandidateReady)
    {
        return PROTOCOL_ERROR_CALIBRATION_NOT_READY;
    }
    if (!MotorControl::setCountsPerWheelRevolution(candidateLeftCpr, candidateRightCpr))
    {
        return PROTOCOL_ERROR_INVALID_PARAMETER;
    }

    calibrationCandidateReady = false;
    if (command.machineFormat)
    {
        Serial.printf("V%u CAL APPLIED %lu %.3f %.3f\n", PROTOCOL_VERSION,
                      static_cast<unsigned long>(command.sequence),
                      candidateLeftCpr, candidateRightCpr);
    }
    else
    {
        Serial.println(F("CPR RAM'e uygulandi. Yeniden baslatmada kalici degildir."));
    }
    return PROTOCOL_ERROR_NONE;
}

uint8_t executeTwist(const SerialProtocol::Command &command)
{
    const VehicleKinematics::Targets targets =
        VehicleKinematics::calculate(command.firstFloat, command.secondFloat);
    lastLinearXMps = command.firstFloat;
    lastAngularZRadS = command.secondFloat;
    lastTargets = targets;

    if (!targets.valid)
    {
        // Geçersiz Ackermann istegi eski hedefi calistirmaya devam ettirmemelidir.
        MotorControl::stop();
        return PROTOCOL_ERROR_INVALID_KINEMATICS;
    }

    if (!MotorControl::getState().armed)
    {
        // DISARM durumunda matematik STATUS ile incelenebilir, fakat hedef motor
        // katmanina aktarilmaz. Makine istemcisi bunu ERR NOT_ARMED olarak gorur.
        return PROTOCOL_ERROR_NOT_ARMED;
    }

    MotorControl::setTargets(targets.leftWheelRadS, targets.rightWheelRadS, millis());
    return PROTOCOL_ERROR_NONE;
}

uint8_t executeCommand(const SerialProtocol::Command &command)
{
    switch (command.id)
    {
    case HELLO:
        Serial.printf("V%u HELLO OAMR_ESP32_LOW_LEVEL\n", PROTOCOL_VERSION);
        return PROTOCOL_ERROR_NONE;

    case PING:
        Serial.printf("V%u PONG %lu\n", PROTOCOL_VERSION,
                      static_cast<unsigned long>(millis()));
        return PROTOCOL_ERROR_NONE;

    case HELP:
        if (!command.machineFormat)
        {
            printHelp();
        }
        return PROTOCOL_ERROR_NONE;

    case STATUS:
        if (command.machineFormat)
        {
            writeMachineTelemetry(command.sequence, false);
        }
        else
        {
            printHumanStatus();
        }
        return PROTOCOL_ERROR_NONE;

    case TWIST:
        return executeTwist(command);

    case STOP:
        MotorControl::stop();
        lastLinearXMps = 0.0F;
        lastAngularZRadS = 0.0F;
        lastTargets = VehicleKinematics::calculate(0.0F, 0.0F);
        if (!command.machineFormat)
        {
            Serial.println(F("STOP: hedefler ve PWM sifirlandi."));
        }
        return PROTOCOL_ERROR_NONE;

    case ARM:
        if (!MotorControl::arm())
        {
            return PROTOCOL_ERROR_FEEDBACK_NOT_READY;
        }
        if (!command.machineFormat)
        {
            Serial.println(F("ARM basarili. Motor hala STOP; simdi TWIST gonderin."));
        }
        return PROTOCOL_ERROR_NONE;

    case DISARM:
        MotorControl::disarm();
        if (!command.machineFormat)
        {
            Serial.println(F("DISARM: fiziksel motor cikisi kapali."));
        }
        return PROTOCOL_ERROR_NONE;

    case CALIBRATION:
        if (command.firstInteger == CALIBRATION_START)
        {
            return startCalibration(command);
        }
        if (command.firstInteger == CALIBRATION_END)
        {
            return finishCalibration(command);
        }
        if (command.firstInteger == CALIBRATION_APPLY)
        {
            return applyCalibration(command);
        }
        return PROTOCOL_ERROR_FORMAT;

    case SET_ENCODER_SIGNS:
        if (!MotorControl::setEncoderSigns(command.firstInteger, command.secondInteger))
        {
            return PROTOCOL_ERROR_INVALID_PARAMETER;
        }
        if (!command.machineFormat)
        {
            Serial.println(F("Encoder isaretleri uygulandi; guvenlik icin DISARM."));
        }
        return PROTOCOL_ERROR_NONE;

    case SET_PID_GAINS:
        if (!MotorControl::setPidGains(command.firstFloat, command.secondFloat,
                                       command.thirdFloat))
        {
            return PROTOCOL_ERROR_INVALID_PARAMETER;
        }
        if (!command.machineFormat)
        {
            Serial.println(F("PID kazanimlari uygulandi; guvenlik icin DISARM."));
        }
        return PROTOCOL_ERROR_NONE;

    default:
        return PROTOCOL_ERROR_UNKNOWN_COMMAND;
    }
}

void dispatchLine(const char *line)
{
    const SerialProtocol::ParseResult parsed = SerialProtocol::parseLine(line);
    if (parsed.errorCode != PROTOCOL_ERROR_NONE)
    {
        writeMachineError(parsed.command, parsed.errorCode);
        return;
    }

    const uint8_t result = executeCommand(parsed.command);
    if (result != PROTOCOL_ERROR_NONE)
    {
        writeMachineError(parsed.command, result);
        return;
    }

    if (parsed.command.machineFormat)
    {
        lastMachineSequence = parsed.command.sequence;
    }
    writeMachineAcknowledgement(parsed.command);
}

void readSerialNonBlocking()
{
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
                SerialProtocol::Command overflowCommand{
                    COMMAND_INVALID, commandMayBeMachineFormat, 0U,
                    0.0F, 0.0F, 0.0F, 0, 0};
                writeMachineError(overflowCommand, PROTOCOL_ERROR_LINE_TOO_LONG);
            }
            else if (commandLength > 0U)
            {
                commandBuffer[commandLength] = '\0';
                dispatchLine(commandBuffer);
            }
            commandLength = 0U;
            commandOverflow = false;
            commandMayBeMachineFormat = false;
            continue;
        }

        if (commandLength == 0U)
        {
            commandMayBeMachineFormat = received == 'V';
        }
        if (commandLength < PROTOCOL_COMMAND_BUFFER_SIZE - 1U)
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

    // Boot'ta MotorControl PWM=0 ve DISARM uygular. Bu duyuru bilgi amaclidir;
    // motor hareketi ancak CPR, SIGN, GAINS, ARM ve gecerli TWIST zinciriyle mumkundur.
    Serial.println();
    Serial.println(F("ESP32 iki motor low-level egitim sistemi hazir."));
    Serial.printf("V%u HELLO OAMR_ESP32_LOW_LEVEL\n", PROTOCOL_VERSION);
    Serial.println(F("Motorlar DISARM ve STOP. Komutlar icin HELP yazin."));
}

void loop()
{
    const uint32_t nowMicros = micros();
    const uint32_t nowMillis = millis();

    readSerialNonBlocking();
    MotorControl::update(nowMicros, nowMillis);

    if (nowMillis - lastTelemetryMs >= TELEMETRY_PERIOD_MS)
    {
        lastTelemetryMs = nowMillis;
        writeMachineTelemetry(lastMachineSequence, true);
    }
}
