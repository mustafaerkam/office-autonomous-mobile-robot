#include "serial_protocol.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace SerialProtocol
{
namespace
{
Command emptyCommand()
{
    return Command{COMMAND_INVALID, false, 0U, 0.0F, 0.0F, 0.0F, 0, 0};
}

ParseResult makeResult(const Command &command, const uint8_t errorCode)
{
    return ParseResult{command, errorCode};
}

// Kisa komut (t) ile uzun alias (TWIST) ayni sayisal kurali kullanir. Parser,
// komut adindan sonraki bolumu alir; %c ile fazladan token olup olmadigini denetler.
bool hasOnlyTwoFiniteFloats(const char *arguments, float &first, float &second)
{
    char trailing = '\0';
    return std::sscanf(arguments, " %f %f %c", &first, &second, &trailing) == 2 &&
           std::isfinite(first) && std::isfinite(second);
}

bool hasOnlyThreeFiniteFloats(const char *arguments, float &first, float &second, float &third)
{
    char trailing = '\0';
    return std::sscanf(arguments, " %f %f %f %c", &first, &second, &third, &trailing) == 3 &&
           std::isfinite(first) && std::isfinite(second) && std::isfinite(third);
}

bool hasOnlyTwoSigns(const char *arguments, int8_t &first, int8_t &second)
{
    int firstSign = 0;
    int secondSign = 0;
    char trailing = '\0';
    if (std::sscanf(arguments, " %d %d %c", &firstSign, &secondSign, &trailing) != 2 ||
        (firstSign != -1 && firstSign != 1) ||
        (secondSign != -1 && secondSign != 1))
    {
        return false;
    }
    first = static_cast<int8_t>(firstSign);
    second = static_cast<int8_t>(secondSign);
    return true;
}

bool parseCalibrationAction(const char *arguments, int8_t &action)
{
    if (std::strcmp(arguments, " START") == 0 || std::strcmp(arguments, " start") == 0)
    {
        action = CALIBRATION_START;
    }
    else if (std::strcmp(arguments, " END") == 0 || std::strcmp(arguments, " end") == 0)
    {
        action = CALIBRATION_END;
    }
    else if (std::strcmp(arguments, " APPLY") == 0 || std::strcmp(arguments, " apply") == 0)
    {
        action = CALIBRATION_APPLY;
    }
    else
    {
        return false;
    }
    return true;
}

// Insan ve makine formatinin ortak komut govdesi ayni fonksiyonda ayrıştırılır.
// Bu sayede "TWIST" terminalden de bridge'den de gelse ayni birim/format kurallari
// uygulanir; iki farkli parser zamanla birbirinden sapmaz.
ParseResult parsePayload(const char *payload, Command command)
{
    if (std::strcmp(payload, "HELLO") == 0)
    {
        command.id = HELLO;
    }
    else if (std::strcmp(payload, "PING") == 0)
    {
        command.id = PING;
    }
    else if (std::strcmp(payload, "HELP") == 0)
    {
        command.id = HELP;
    }
    else if (std::strcmp(payload, "STATUS") == 0)
    {
        command.id = STATUS;
    }
    else if (std::strcmp(payload, "STOP") == 0)
    {
        command.id = STOP;
    }
    else if (std::strcmp(payload, "ARM") == 0)
    {
        command.id = ARM;
    }
    else if (std::strcmp(payload, "DISARM") == 0)
    {
        command.id = DISARM;
    }
    else if (std::strncmp(payload, "TWIST ", 6) == 0)
    {
        if (!hasOnlyTwoFiniteFloats(payload + 5, command.firstFloat, command.secondFloat))
        {
            return makeResult(command, PROTOCOL_ERROR_FORMAT);
        }
        command.id = TWIST;
    }
    else if (std::strncmp(payload, "SIGN ", 5) == 0)
    {
        if (!hasOnlyTwoSigns(payload + 4, command.firstInteger, command.secondInteger))
        {
            return makeResult(command, PROTOCOL_ERROR_INVALID_PARAMETER);
        }
        command.id = SET_ENCODER_SIGNS;
    }
    else if (std::strncmp(payload, "GAINS ", 6) == 0)
    {
        if (!hasOnlyThreeFiniteFloats(payload + 5, command.firstFloat,
                                      command.secondFloat, command.thirdFloat))
        {
            return makeResult(command, PROTOCOL_ERROR_FORMAT);
        }
        command.id = SET_PID_GAINS;
    }
    else if (std::strncmp(payload, "CAL ", 4) == 0)
    {
        if (!parseCalibrationAction(payload + 3, command.firstInteger))
        {
            return makeResult(command, PROTOCOL_ERROR_FORMAT);
        }
        command.id = CALIBRATION;
    }
    // Referanstaki commands.h mantigi: ilk karakter dogrudan seri komuttur.
    // Bu blok, "t 0.30 0.50" gibi kisa komutlari uzun isimli alias'larla ayni
    // Command yapisina donusturur. Boylece iki ayri motor kontrol yolu oluşmaz.
    else if (payload[1] == '\0')
    {
        const char shortCommand = payload[0];
        if (shortCommand == HELLO || shortCommand == PING || shortCommand == HELP ||
            shortCommand == STATUS || shortCommand == STOP || shortCommand == ARM ||
            shortCommand == DISARM)
        {
            command.id = static_cast<uint8_t>(shortCommand);
        }
        else
        {
            return makeResult(command, PROTOCOL_ERROR_UNKNOWN_COMMAND);
        }
    }
    else if (payload[1] == ' ')
    {
        const char shortCommand = payload[0];
        const char *arguments = payload + 1;
        if (shortCommand == TWIST)
        {
            if (!hasOnlyTwoFiniteFloats(arguments, command.firstFloat, command.secondFloat))
            {
                return makeResult(command, PROTOCOL_ERROR_FORMAT);
            }
            command.id = TWIST;
        }
        else if (shortCommand == SET_ENCODER_SIGNS)
        {
            if (!hasOnlyTwoSigns(arguments, command.firstInteger, command.secondInteger))
            {
                return makeResult(command, PROTOCOL_ERROR_INVALID_PARAMETER);
            }
            command.id = SET_ENCODER_SIGNS;
        }
        else if (shortCommand == SET_PID_GAINS)
        {
            if (!hasOnlyThreeFiniteFloats(arguments, command.firstFloat,
                                          command.secondFloat, command.thirdFloat))
            {
                return makeResult(command, PROTOCOL_ERROR_FORMAT);
            }
            command.id = SET_PID_GAINS;
        }
        else if (shortCommand == CALIBRATION)
        {
            if (!parseCalibrationAction(arguments, command.firstInteger))
            {
                return makeResult(command, PROTOCOL_ERROR_FORMAT);
            }
            command.id = CALIBRATION;
        }
        else
        {
            return makeResult(command, PROTOCOL_ERROR_UNKNOWN_COMMAND);
        }
    }
    else
    {
        return makeResult(command, PROTOCOL_ERROR_UNKNOWN_COMMAND);
    }

    return makeResult(command, PROTOCOL_ERROR_NONE);
}
}  // namespace

ParseResult parseLine(const char *line)
{
    Command command = emptyCommand();
    if (line == nullptr || line[0] == '\0')
    {
        return makeResult(command, PROTOCOL_ERROR_FORMAT);
    }

    // Makine formatinda surum ve sira numarasindan sonra ayni insan-okunur komut
    // govdesi gelir: "V1 CMD 42 TWIST 0.30 0.50". %n, ayrıştırılan prefix'in kac
    // karakter oldugunu verir; boylece payload icin yeni bir tampon kopyalanmaz.
    if (line[0] == 'V')
    {
        unsigned int version = 0U;
        unsigned long sequence = 0UL;
        int payloadOffset = 0;
        if (std::sscanf(line, "V%u CMD %lu %n", &version, &sequence, &payloadOffset) != 2 ||
            payloadOffset <= 0 || line[payloadOffset] == '\0')
        {
            return makeResult(command, PROTOCOL_ERROR_FORMAT);
        }

        command.machineFormat = true;
        if (sequence > std::numeric_limits<uint32_t>::max())
        {
            return makeResult(command, PROTOCOL_ERROR_FORMAT);
        }
        command.sequence = static_cast<uint32_t>(sequence);
        if (version != PROTOCOL_VERSION)
        {
            return makeResult(command, PROTOCOL_ERROR_UNSUPPORTED_VERSION);
        }
        return parsePayload(line + payloadOffset, command);
    }

    return parsePayload(line, command);
}

const char *commandName(const uint8_t commandId)
{
    switch (commandId)
    {
    case HELLO: return "HELLO";
    case PING: return "PING";
    case HELP: return "HELP";
    case STATUS: return "STATUS";
    case TWIST: return "TWIST";
    case STOP: return "STOP";
    case ARM: return "ARM";
    case DISARM: return "DISARM";
    case CALIBRATION: return "CAL";
    case SET_ENCODER_SIGNS: return "SIGN";
    case SET_PID_GAINS: return "GAINS";
    default: return "INVALID";
    }
}

const char *errorName(const uint8_t errorCode)
{
    switch (errorCode)
    {
    case PROTOCOL_ERROR_NONE: return "NONE";
    case PROTOCOL_ERROR_FORMAT: return "FORMAT";
    case PROTOCOL_ERROR_UNKNOWN_COMMAND: return "UNKNOWN_COMMAND";
    case PROTOCOL_ERROR_UNSUPPORTED_VERSION: return "UNSUPPORTED_VERSION";
    case PROTOCOL_ERROR_NOT_ARMED: return "NOT_ARMED";
    case PROTOCOL_ERROR_FEEDBACK_NOT_READY: return "FEEDBACK_NOT_READY";
    case PROTOCOL_ERROR_INVALID_KINEMATICS: return "INVALID_KINEMATICS";
    case PROTOCOL_ERROR_INVALID_PARAMETER: return "INVALID_PARAMETER";
    case PROTOCOL_ERROR_CALIBRATION_NOT_ACTIVE: return "CALIBRATION_NOT_ACTIVE";
    case PROTOCOL_ERROR_CALIBRATION_NOT_READY: return "CALIBRATION_NOT_READY";
    case PROTOCOL_ERROR_LINE_TOO_LONG: return "LINE_TOO_LONG";
    default: return "UNKNOWN_ERROR";
    }
}
}  // namespace SerialProtocol
