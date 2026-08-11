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

bool hasOnlyTwoFiniteFloats(const char *text, float &first, float &second)
{
    char trailing = '\0';
    return std::sscanf(text, "TWIST %f %f %c", &first, &second, &trailing) == 2 &&
           std::isfinite(first) && std::isfinite(second);
}

bool hasOnlyThreeFiniteFloats(const char *text, float &first, float &second, float &third)
{
    char trailing = '\0';
    return std::sscanf(text, "GAINS %f %f %f %c", &first, &second, &third, &trailing) == 3 &&
           std::isfinite(first) && std::isfinite(second) && std::isfinite(third);
}

// Insan ve makine formatinin ortak komut govdesi ayni fonksiyonda ayrıştırılır.
// Bu sayede "TWIST" terminalden de bridge'den de gelse ayni birim/format kurallari
// uygulanir; iki farkli parser zamanla birbirinden sapmaz.
ParseResult parsePayload(const char *payload, Command command)
{
    if (std::strcmp(payload, "HELLO") == 0)
    {
        command.id = COMMAND_HELLO;
    }
    else if (std::strcmp(payload, "PING") == 0)
    {
        command.id = COMMAND_PING;
    }
    else if (std::strcmp(payload, "HELP") == 0)
    {
        command.id = COMMAND_HELP;
    }
    else if (std::strcmp(payload, "STATUS") == 0)
    {
        command.id = COMMAND_STATUS;
    }
    else if (std::strcmp(payload, "STOP") == 0)
    {
        command.id = COMMAND_STOP;
    }
    else if (std::strcmp(payload, "ARM") == 0)
    {
        command.id = COMMAND_ARM;
    }
    else if (std::strcmp(payload, "DISARM") == 0)
    {
        command.id = COMMAND_DISARM;
    }
    else if (std::strcmp(payload, "CAL START") == 0)
    {
        command.id = COMMAND_CALIBRATION_START;
    }
    else if (std::strcmp(payload, "CAL END") == 0)
    {
        command.id = COMMAND_CALIBRATION_END;
    }
    else if (std::strcmp(payload, "CAL APPLY") == 0)
    {
        command.id = COMMAND_CALIBRATION_APPLY;
    }
    else if (std::strncmp(payload, "TWIST ", 6) == 0)
    {
        if (!hasOnlyTwoFiniteFloats(payload, command.firstFloat, command.secondFloat))
        {
            return makeResult(command, PROTOCOL_ERROR_FORMAT);
        }
        command.id = COMMAND_TWIST;
    }
    else if (std::strncmp(payload, "SIGN ", 5) == 0)
    {
        int firstSign = 0;
        int secondSign = 0;
        char trailing = '\0';
        if (std::sscanf(payload, "SIGN %d %d %c", &firstSign, &secondSign, &trailing) != 2 ||
            (firstSign != -1 && firstSign != 1) ||
            (secondSign != -1 && secondSign != 1))
        {
            return makeResult(command, PROTOCOL_ERROR_INVALID_PARAMETER);
        }
        command.id = COMMAND_SET_ENCODER_SIGNS;
        command.firstInteger = static_cast<int8_t>(firstSign);
        command.secondInteger = static_cast<int8_t>(secondSign);
    }
    else if (std::strncmp(payload, "GAINS ", 6) == 0)
    {
        if (!hasOnlyThreeFiniteFloats(payload, command.firstFloat,
                                      command.secondFloat, command.thirdFloat))
        {
            return makeResult(command, PROTOCOL_ERROR_FORMAT);
        }
        command.id = COMMAND_SET_PID_GAINS;
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
    case COMMAND_HELLO: return "HELLO";
    case COMMAND_PING: return "PING";
    case COMMAND_HELP: return "HELP";
    case COMMAND_STATUS: return "STATUS";
    case COMMAND_TWIST: return "TWIST";
    case COMMAND_STOP: return "STOP";
    case COMMAND_ARM: return "ARM";
    case COMMAND_DISARM: return "DISARM";
    case COMMAND_CALIBRATION_START: return "CAL_START";
    case COMMAND_CALIBRATION_END: return "CAL_END";
    case COMMAND_CALIBRATION_APPLY: return "CAL_APPLY";
    case COMMAND_SET_ENCODER_SIGNS: return "SIGN";
    case COMMAND_SET_PID_GAINS: return "GAINS";
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
