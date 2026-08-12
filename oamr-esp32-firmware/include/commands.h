// Bilgisayar ile ESP32 arasindaki seri protokolun ortak sabitleri bu dosyada
// toplanir. Burada GPIO, PWM veya PID kodu yoktur: komut katmani donanimin nasil
// suruldugunu degil, bilgisayarin ne istedigini ifade eder.
#pragma once

// Bu karakterler PC'nin seri hattan gonderecegi gerçek kisa komutlardir. Ornegin
// bilgisayar "t 0.30 0.50\n" gonderdiginde ilk karakter TWIST olur. Sayisal bir
// ic kimlik katmani yoktur; parser bu karakteri Command.id alanina tasir. Uzun
// komutlar (TWIST, STATUS gibi) ayni komutlarin geriye uyumlu alias'lari olarak kalir.
#define COMMAND_INVALID          0U
#define HELLO                    'v'
#define PING                     'p'
#define HELP                     'h'
#define STATUS                   's'
#define TWIST                    't'
#define STOP                     'x'
#define ARM                      'a'
#define DISARM                   'd'
#define CALIBRATION              'c'
#define SET_ENCODER_SIGNS        'n'
#define SET_PID_GAINS            'g'
// JOG, kalibrasyon ve ARM gerektirmeyen acik-cevrim tezgah komutudur. PWM ve sure
// tavanlari motor_control.cpp icinde sabittir ve sure dolunca cikis kendiliginden
// kesilir. Amaci "motor donuyor mu, encoder sayiyor mu" sorusunu PID devreye
// girmeden yanitlamaktir.
#define JOG                      'j'

// CAL tek karakterli bir komuttur; ikinci kelime hangi kalibrasyon asamasinin
// istendigini belirtir. Ornekler: "c start", "c end" ve "c apply".
#define CALIBRATION_START        1U
#define CALIBRATION_END          2U
#define CALIBRATION_APPLY        3U

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
