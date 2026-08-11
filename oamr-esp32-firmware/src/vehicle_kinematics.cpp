#include "vehicle_kinematics.h"

#include "vehicle_config.h"

#include <cmath>

namespace VehicleKinematics
{
namespace
{
// Bu sabit yalniz bu .cpp icinde gerekli oldugu icin anonim namespace'e konur;
// baska dosyalarin isim alanini kirletmez. Hiza "sifir" derken sensor/girdi
// yuvarlama hatalarini da kapsamak icin tam esitlik yerine tolerans kullanilir.
constexpr float NEAR_ZERO = 1.0e-5F;
}

Targets calculate(const float linearXMps, const float angularZRadS)
{
    // Once guvenli stop sonucu hazirlanir. Bir girdi gecersiz cikarsa motor hedefleri
    // sifir kalir; boylece hata tehlikeli bir hareket komutuna donusmez.
    Targets result{0.0F, 0.0F, 0.0F, true};

    // NaN veya sonsuz sayilar karsilastirma ve kontrol hesabini bozabilir. Seri
    // parser normalde bunlari reddeder; bu ikinci kontrol matematigi de korur.
    if (!std::isfinite(linearXMps) || !std::isfinite(angularZRadS))
    {
        result.valid = false;
        return result;
    }

    // Ackermann araci direksiyon kirarak ilerler; lineer hiz yokken kendi merkezi
    // etrafinda donamaz. Diferansiyel surus formulu bu durumu yapabilse de burada
    // fiziksel araca uymayan v=0, wz!=0 komutunu reddediyoruz ve sifira bolmuyoruz.
    if (std::fabs(linearXMps) < NEAR_ZERO)
    {
        if (std::fabs(angularZRadS) >= NEAR_ZERO)
        {
            result.valid = false;
        }
        return result;
    }

    // Arac merkezinin donus yaricapi R=v/w'dir. Sol ve sag teker merkezleri R'nin
    // iki yaninda track_width/2 kadar uzakta oldugundan v_left/right = v +/- wz*L/2
    // olur. Bu fark virajda dis tekerin daha hizli donmesini saglar.
    const float halfTrackM = VehicleConfig::REAR_TRACK_M * 0.5F;
    const float leftLinearMps = linearXMps - (angularZRadS * halfTrackM);
    const float rightLinearMps = linearXMps + (angularZRadS * halfTrackM);

    // v = omega*r oldugu icin metre/saniye cinsinden teker cevresel hizini yaricapa
    // bolerek rad/s cinsinden acisal hiza ceviriyoruz. Yaricap sabit ve sifirdan buyuk.
    result.leftWheelRadS = leftLinearMps / VehicleConfig::WHEEL_RADIUS_M;
    result.rightWheelRadS = rightLinearMps / VehicleConfig::WHEEL_RADIUS_M;

    // Bicycle modelinde delta=atan(L*wz/v). Bu deger yalniz telemetry'dir; projede
    // direksiyon servo/aktuatoru olmadigi icin hicbir GPIO bu degerle surulmez.
    result.steeringAngleRad =
        std::atan((VehicleConfig::WHEELBASE_M * angularZRadS) / linearXMps);
    return result;
}
}  // namespace VehicleKinematics
