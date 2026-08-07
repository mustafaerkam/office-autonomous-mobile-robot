#include <Arduino.h>

// =========================
// PIN TANIMLARI
// =========================

// Encoder
constexpr int ENCODER_A = 16;
constexpr int ENCODER_B = 17;

// L298N
constexpr int MOTOR_IN1 = 18;
constexpr int MOTOR_IN2 = 19;
constexpr int MOTOR_PWM = 25;


// =========================
// PWM AYARLARI
// =========================

constexpr int PWM_CHANNEL = 0;

// L298N bipolar transistor koprusu kullanir ve yavastir.
// 20 kHz'de anahtarlama tamamlanamadigi icin motor hic donmeyebilir.
// Bu surucu icin 1 kHz guvenli araliktir.
constexpr int PWM_FREQUENCY = 1000;

constexpr int PWM_RESOLUTION = 8;      // 8 bit -> 0-255


// =========================
// TARAMA TESTI AYARLARI
// =========================

constexpr int SWEEP_START = 60;    // buradan baslayip
constexpr int SWEEP_END   = 255;   // buraya kadar
constexpr int SWEEP_STEP  = 15;    // bu adimlarla artir
constexpr int SWEEP_HOLD  = 800;   // her kademede bekleme (ms)


// =========================
// ENCODER DEGISKENI
// =========================

volatile long encoderCount = 0;


// =========================
// ENCODER INTERRUPT
// =========================

void IRAM_ATTR encoderISR()
{
    // A fazinin yukselen kenarinda B fazina bak.
    // Boylece yon bilgisi de elde edilir.

    if (digitalRead(ENCODER_B))
    {
        encoderCount++;
    }
    else
    {
        encoderCount--;
    }
}


long readEncoder()
{
    long countCopy;

    noInterrupts();
    countCopy = encoderCount;
    interrupts();

    return countCopy;
}


// =========================
// MOTOR FONKSIYONLARI
// =========================

void motorForward(uint8_t pwm)
{
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);

    ledcWrite(PWM_CHANNEL, pwm);
}


void motorBackward(uint8_t pwm)
{
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);

    ledcWrite(PWM_CHANNEL, pwm);
}


void motorStop()
{
    ledcWrite(PWM_CHANNEL, 0);

    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
}


// =========================
// TARAMA TESTI
// =========================
//
// PWM'i kademe kademe artirir ve her kademede encoder'in
// kac tik ilerledigini yazar. Amac: motorun gercekten
// donmeye basladigi esik PWM degerini bulmak.

void sweepTest(const char *yon, void (*surucu)(uint8_t))
{
    Serial.println();
    Serial.print("--- ");
    Serial.print(yon);
    Serial.println(" tarama ---");
    Serial.println("PWM\tTik\tDurum");

    int ilkHareketPwm = -1;

    for (int pwm = SWEEP_START; pwm <= SWEEP_END; pwm += SWEEP_STEP)
    {
        long baslangic = readEncoder();

        surucu(pwm);
        delay(SWEEP_HOLD);

        long fark = readEncoder() - baslangic;

        Serial.print(pwm);
        Serial.print('\t');
        Serial.print(fark);
        Serial.print('\t');

        if (fark == 0)
        {
            Serial.println("duruyor");
        }
        else
        {
            Serial.println("DONUYOR");

            if (ilkHareketPwm < 0)
            {
                ilkHareketPwm = pwm;
            }
        }
    }

    motorStop();
    delay(500);

    Serial.print(">> ");
    Serial.print(yon);
    Serial.print(" esik PWM: ");

    if (ilkHareketPwm < 0)
    {
        Serial.println("HIC DONMEDI");
    }
    else
    {
        Serial.println(ilkHareketPwm);
    }
}


// =========================
// SETUP
// =========================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("ESP32 motor + encoder TESHIS testi");
    Serial.print("PWM frekansi: ");
    Serial.print(PWM_FREQUENCY);
    Serial.println(" Hz");


    // -------------------------
    // MOTOR PINLERI
    // -------------------------

    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);

    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);


    // -------------------------
    // PWM
    // -------------------------

    ledcSetup(
        PWM_CHANNEL,
        PWM_FREQUENCY,
        PWM_RESOLUTION
    );

    ledcAttachPin(
        MOTOR_PWM,
        PWM_CHANNEL
    );


    // -------------------------
    // ENCODER
    // -------------------------
    //
    // Cogu encoder modulu acik kollektor cikis verir.
    // Dahili pull-up olmadan hat havada kalir ve sayac ya
    // hic artmaz ya da gurultuden rastgele artar.

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_A),
        encoderISR,
        RISING
    );


    // -------------------------
    // TESHIS
    // -------------------------

    Serial.println();
    Serial.println("Once motor durur halde encoder okunuyor...");
    Serial.println("(Simdi mili ELINLE cevir, sayac degismeli)");

    for (int i = 0; i < 5; i++)
    {
        delay(1000);
        Serial.print("  elle cevirme testi, tik: ");
        Serial.println(readEncoder());
    }

    sweepTest("ILERI", motorForward);
    sweepTest("GERI",  motorBackward);

    Serial.println();
    Serial.println("Test bitti. Motor durduruldu.");
}


// =========================
// LOOP
// =========================

void loop()
{
    static unsigned long lastPrintTime = 0;

    if (millis() - lastPrintTime >= 500)
    {
        lastPrintTime = millis();

        Serial.print("Encoder count: ");
        Serial.println(readEncoder());
    }
}
