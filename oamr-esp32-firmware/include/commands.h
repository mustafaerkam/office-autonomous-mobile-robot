// Bilgisayar ile ESP32 arasindaki seri protokolun ortak sabitleri bu dosyada
// toplanir. Burada GPIO, PWM veya PID kodu yoktur: komut katmani donanimin nasil
// suruldugunu degil, bilgisayarin ne istedigini ifade eder.
#pragma once

// Kullanici istegiyle enum class yerine #define kullaniliyor. Sayisal kimlikler
// seri protokolun sabit sozlesmesidir; yeni bir komut eklenirken mevcut degerlerin
// anlami degistirilmez. 0 degeri her zaman gecersiz/bilinmeyen komut icindir.
#define COMMAND_INVALID             0U
#define COMMAND_HELLO               1U
#define COMMAND_PING                2U
#define COMMAND_HELP                3U
#define COMMAND_STATUS              4U
#define COMMAND_TWIST               5U
#define COMMAND_STOP                6U
#define COMMAND_ARM                 7U
#define COMMAND_DISARM              8U
#define COMMAND_CALIBRATION_START   9U
#define COMMAND_CALIBRATION_END     10U
#define COMMAND_CALIBRATION_APPLY   11U
#define COMMAND_SET_ENCODER_SIGNS   12U
#define COMMAND_SET_PID_GAINS       13U

// Sol ve sag teker indexleri seri mesajlarda veya dizi tabanli telemetry'de ayni
// sirayi korur. Bu tanimlar pin numarasi degildir; yalniz mantiksal teker kimligidir.
#define WHEEL_LEFT                  0U
#define WHEEL_RIGHT                 1U

// V1, makine-okunur mesaj biciminin ilk surumudur. Bir host yazilimi baglandiginda
// HELLO cevabindan bu surumu gorur; uyumsuz surumde komut uygulanmaz.
#define PROTOCOL_VERSION             1U
#define PROTOCOL_COMMAND_BUFFER_SIZE 96U

// Parser veya dispatcher hatalari sayisal olarak tanimlanir. Boylesi, ROS 2 bridge'in
// Turkce insan metnini ayiklamak yerine kararlı hata adini kullanmasini saglar.
#define PROTOCOL_ERROR_NONE                    0U
#define PROTOCOL_ERROR_FORMAT                  1U
#define PROTOCOL_ERROR_UNKNOWN_COMMAND         2U
#define PROTOCOL_ERROR_UNSUPPORTED_VERSION     3U
#define PROTOCOL_ERROR_NOT_ARMED               4U
#define PROTOCOL_ERROR_FEEDBACK_NOT_READY      5U
#define PROTOCOL_ERROR_INVALID_KINEMATICS      6U
#define PROTOCOL_ERROR_INVALID_PARAMETER       7U
#define PROTOCOL_ERROR_CALIBRATION_NOT_ACTIVE  8U
#define PROTOCOL_ERROR_CALIBRATION_NOT_READY   9U
#define PROTOCOL_ERROR_LINE_TOO_LONG           10U
