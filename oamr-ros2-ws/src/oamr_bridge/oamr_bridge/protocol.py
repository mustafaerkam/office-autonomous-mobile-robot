"""ESP32 low-level firmware'inin V1 seri sozlesmesinin Python karsiligi.

Bu modul bilerek ROS'tan bagimsizdir: ne rclpy ne de mesaj tipleri import edilir.
Boylece protokol hem bridge node'u hem de sade bir CLI/test tarafindan kullanilabilir
ve donanim olmadan birim testi yazilabilir.

Sozlesmenin kaynagi firmware tarafinda su dosyalardir:
  oamr-esp32-firmware/include/commands.h
  oamr-esp32-firmware/src/serial_protocol.cpp
  oamr-esp32-firmware/src/main.cpp   (TEL alan sirasi)
Firmware bu alanlari degistirirse burasi da guncellenmelidir.
"""

from dataclasses import dataclass, field
from typing import Optional

PROTOCOL_VERSION = 1

# Firmware tarafindaki tek karakterli komut kimlikleri (commands.h ile ayni).
# Yalniz bridge'in gercekten gonderdigi komutlar burada tanimlidir. Kalibrasyon
# komutlari (CAL/SIGN/GAINS) elle seri terminalden verilir; ROS tarafina
# tasindiklarinda buraya eklenmelidirler.
CMD_TWIST = "t"
CMD_STOP = "x"
CMD_ARM = "a"
CMD_DISARM = "d"


def encode(sequence: int, payload: str) -> bytes:
    """Bir komut govdesini V1 makine bicimine sarar.

    Makine bicimi ACK/ERR cevaplarini sequence ile eslestirmeyi mumkun kilar;
    insan terminal bicimi bunu yapamadigi icin bridge her zaman bu yolu kullanir.
    """
    return f"V{PROTOCOL_VERSION} CMD {sequence} {payload}\n".encode("ascii")


def encode_twist(sequence: int, linear_x: float, angular_z: float) -> bytes:
    # Dort ondalik, firmware'in %f parser'i icin fazlasiyla yeterli ve satiri kisa tutar.
    return encode(sequence, f"{CMD_TWIST} {linear_x:.4f} {angular_z:.4f}")


def encode_arm(sequence: int) -> bytes:
    return encode(sequence, CMD_ARM)


def encode_disarm(sequence: int) -> bytes:
    return encode(sequence, CMD_DISARM)


def encode_stop(sequence: int) -> bytes:
    return encode(sequence, CMD_STOP)


@dataclass
class Telemetry:
    """V1 TEL satirinin alanlari; sira firmware main.cpp'deki snprintf ile birebir."""

    uptime_ms: int
    sequence: int
    armed: bool
    feedback_ready: bool
    cmd_timeout: bool
    left_count: int
    right_count: int
    left_target_rad_s: float
    right_target_rad_s: float
    left_measured_rad_s: float
    right_measured_rad_s: float
    steering_rad: float
    left_pwm: int
    right_pwm: int


@dataclass
class Response:
    """Firmware'den gelen tek bir satirin ayrıştırılmış hali.

    kind degerleri: ACK, ERR, TEL, HELLO, PONG, CAL, OTHER
    'OTHER', insan-okunur Turkce bilgi satirlarini kapsar; bunlar hata degildir,
    yalnizca makine tarafindan yorumlanmaz.
    """

    kind: str
    raw: str
    sequence: Optional[int] = None
    name: Optional[str] = None
    telemetry: Optional[Telemetry] = None
    values: list = field(default_factory=list)


def _parse_telemetry(parts: list) -> Optional[Telemetry]:
    # parts, "V1 TEL" sonrasindaki 14 alandir. Eksik/bozuk satir sessizce atilir;
    # seri hatta gurultu olabilecegi icin exception firlatmak dogru degildir.
    if len(parts) != 14:
        return None
    try:
        return Telemetry(
            uptime_ms=int(parts[0]),
            sequence=int(parts[1]),
            armed=parts[2] == "1",
            feedback_ready=parts[3] == "1",
            cmd_timeout=parts[4] == "1",
            left_count=int(parts[5]),
            right_count=int(parts[6]),
            left_target_rad_s=float(parts[7]),
            right_target_rad_s=float(parts[8]),
            left_measured_rad_s=float(parts[9]),
            right_measured_rad_s=float(parts[10]),
            steering_rad=float(parts[11]),
            left_pwm=int(parts[12]),
            right_pwm=int(parts[13]),
        )
    except ValueError:
        return None


def parse_line(line: str) -> Optional[Response]:
    """Firmware'den gelen bir satiri Response'a cevirir; bos satirda None doner."""
    line = line.strip()
    if not line:
        return None

    prefix = f"V{PROTOCOL_VERSION} "
    if not line.startswith(prefix):
        # Firmware acilista ve insan komutlarinda duz Turkce satirlar da basar.
        return Response(kind="OTHER", raw=line)

    body = line[len(prefix):].split()
    if not body:
        return Response(kind="OTHER", raw=line)

    tag = body[0]
    rest = body[1:]

    if tag == "ACK" and len(rest) >= 2:
        try:
            return Response(kind="ACK", raw=line, sequence=int(rest[0]), name=rest[1])
        except ValueError:
            return Response(kind="OTHER", raw=line)

    if tag == "ERR" and len(rest) >= 2:
        try:
            return Response(kind="ERR", raw=line, sequence=int(rest[0]), name=rest[1])
        except ValueError:
            return Response(kind="OTHER", raw=line)

    if tag == "TEL":
        telemetry = _parse_telemetry(rest)
        if telemetry is None:
            return Response(kind="OTHER", raw=line)
        return Response(kind="TEL", raw=line, sequence=telemetry.sequence,
                        telemetry=telemetry)

    if tag == "HELLO":
        return Response(kind="HELLO", raw=line, name=rest[0] if rest else None)

    if tag == "PONG":
        return Response(kind="PONG", raw=line, values=rest)

    if tag == "CAL":
        # "V1 CAL START <seq> ...", "V1 CAL CANDIDATE <seq> ...", "V1 CAL APPLIED <seq> ..."
        return Response(kind="CAL", raw=line,
                        name=rest[0] if rest else None, values=rest[1:])

    return Response(kind="OTHER", raw=line)


class LineReader:
    """Seri porttan gelen parcali byte'lari tam satirlara bolen tampon.

    Seri okuma nadiren tam satir sinirinda biter; bu sinif yarim kalan parcayi
    saklar ve yalniz newline gordugunde satir uretir.
    """

    def __init__(self, max_buffer: int = 4096) -> None:
        self._buffer = bytearray()
        self._max_buffer = max_buffer

    def feed(self, data: bytes) -> list:
        """Yeni byte'lari ekler ve tamamlanan satirlarin listesini dondurur."""
        if not data:
            return []
        self._buffer.extend(data)

        # Newline hic gelmezse tampon sinirsiz buyumemelidir; bozuk baglantida
        # eski veriyi atmak, belleği tuketmekten daha guvenlidir.
        if len(self._buffer) > self._max_buffer:
            del self._buffer[:-self._max_buffer]

        lines = []
        while True:
            index = self._buffer.find(b"\n")
            if index < 0:
                break
            chunk = bytes(self._buffer[:index])
            del self._buffer[:index + 1]
            lines.append(chunk.decode("utf-8", errors="replace").strip())
        return lines
