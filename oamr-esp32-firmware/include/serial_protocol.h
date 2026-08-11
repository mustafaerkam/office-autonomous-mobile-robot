// Bu modul seri satirini yorumlar, fakat motor calistirmaz. Parser'in GPIO/PID'den
// ayri tutulmasi ayni komut sozlesmesinin masaustu testinde de kullanilabilmesini
// ve main.cpp'nin yalniz sistem yoneticisi kalmasini saglar.
#pragma once

#include <stdint.h>

#include "commands.h"

namespace SerialProtocol
{
// Bir Command, dogrulanmis bir komutun donanimdan bagimsiz temsilidir. float alanlari
// TWIST ve GAINS gibi fiziksel sayilari; int alanlari encoder isaretlerini tasir.
struct Command
{
    uint8_t id;
    bool machineFormat;       // "V1 CMD ..." ise true, insan terminal komutuysa false.
    uint32_t sequence;        // Makine komutunun sira numarasi; insan komutunda 0.
    float firstFloat;
    float secondFloat;
    float thirdFloat;
    int8_t firstInteger;
    int8_t secondInteger;
};

// ParseResult, hata olsa bile mesajin makine biciminde olup olmadigini ve varsa
// sequence degerini korur. Boylece host, hatali komutuna da eslesmis ERR cevabi alir.
struct ParseResult
{
    Command command;
    uint8_t errorCode;
};

// Satiri ayrıştırir. Basarida errorCode=PROTOCOL_ERROR_NONE olur; komut uygulama
// karari burada verilmez. Ornegin CPR eksikligi parser degil dispatcher hatasidir.
ParseResult parseLine(const char *line);

// Makine cevaplarinda okunabilir sabit adlar kullanmak icin donusum fonksiyonlari.
const char *commandName(uint8_t commandId);
const char *errorName(uint8_t errorCode);
}  // namespace SerialProtocol
