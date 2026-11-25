/*
  Pengukur Arah Angin dengan Sensor Magnetik (HMC5883L) — ESP32
  ==============================================================
  Cara kerja:
  Sebuah baling-baling penunjuk arah (wind vane) dipasang di poros
  vertikal; angin memutar baling-baling sampai panahnya menghadap ke
  arah datangnya angin. Sebuah magnet kecil menempel di poros dan
  ikut berputar. Tepat di bawah poros, sensor magnetometer HMC5883L
  yang TIDAK ikut berputar membaca arah medan magnet dari magnet
  tersebut. Karena magnetnya dekat (beberapa cm), medannya jauh lebih
  kuat dari medan magnet bumi, sehingga sudut yang terbaca adalah
  sudut putar poros — alias arah baling-baling. ESP32 menghitung
  heading 0-360 derajat dari komponen X dan Y medan magnet, lalu
  memetakannya ke 16 arah mata angin (tiap sektor 22,5 derajat) dan
  menampilkannya di LCD I2C 16x2, lengkap dengan derajatnya. Hasil
  yang sama dikirim juga ke Serial Monitor.

  PENTING — ini BUKAN kompas bumi: sensor tidak mengukur arah utara
  magnetis dari medan bumi. Yang diukur adalah arah magnet di poros.
  Supaya angka yang tampil sama dengan arah mata angin yang benar,
  sensor harus dikalibrasi sekali menghadap utara (lihat
  KALIBRASI_UTARA_DEG di bawah): arahkan baling-baling tepat ke
  utara yang diketahui (kompas HP atau patokan yang pasti), baca
  heading di Serial Monitor, lalu isi konstantanya dengan negatif
  dari angka tersebut.

  HMC5883L dibaca MENTAH lewat I2C (Wire), TANPA library eksternal —
  registernya ditulis langsung sesuai datasheet Honeywell HMC5883L:
    - Alamat I2C: 0x1E
    - Register 0x00 (Config A): 0x70 = rata-rata 8 sampel, 15 Hz
    - Register 0x01 (Config B): 0x20 = gain +/-1,3 Ga (1090 LSB/Ga)
    - Register 0x02 (Mode): 0x00 = pengukuran terus-menerus
    - Data mulai register 0x03, 6 byte berurutan:
      X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB (perhatikan: Z
      dibaca sebelum Y — itu urutan register chip ini, bukan salah
      ketik).
  Modul QMC5883L TIDAK cocok dengan sketch ini — alamatnya 0x0D dan
  registernya berbeda. Pakai modul bertuliskan HMC5883L.

  Pemasangan mekanik:
    - Magnet kecil (mis. neodymium 5-10 mm) ditempel di poros, dengan
      kutubnya menghadap ke samping (bukan ke atas/bawah), supaya
      medan di bidang X-Y sensor berputar mengikuti poros.
    - Jarak magnet ke sensor sekitar 2-4 cm dan TETAP: kalau terlalu
      jauh, medan bumi mulai mengganggu; kalau terlalu dekat, sensor
      jenuh (nilai mentah mentok di +/-2048).
    - Sensor dipasang DATAR dan tidak ikut berputar; jauhkan dari
      besi, motor, dan kabel berarus besar.

  Wiring:
    - HMC5883L: VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22.
    - LCD I2C 16x2 alamat 0x27: SDA -> GPIO21, SCL -> GPIO22,
      VCC -> 3V3, GND -> GND (satu bus I2C dengan sensor).
    - ESP32 dicatu USB 5 V; semua GND digabung.

  Video penjelasan (60 detik): link menyusul setelah upload
  Repo: https://github.com/oerban70-cloud/pengukur-arah-angin
*/

#include <Wire.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>

// ---------- Konfigurasi pin ----------
const uint8_t PIN_SDA = 21;  // SDA bus I2C (sensor + LCD)
const uint8_t PIN_SCL = 22;  // SCL bus I2C (sensor + LCD)

// ---------- Konfigurasi LCD ----------
const uint8_t ALAMAT_LCD = 0x27;  // alamat I2C LCD 16x2
const uint8_t KOLOM_LCD  = 16;    // lebar layar (karakter)
const uint8_t BARIS_LCD  = 2;     // tinggi layar (baris)

// ---------- Konfigurasi magnetometer HMC5883L ----------
const uint8_t ALAMAT_MAGNETOMETER = 0x1E;  // alamat I2C HMC5883L
const uint8_t REG_CONFIG_A = 0x00;  // rata-rata sampel + laju data
const uint8_t REG_CONFIG_B = 0x01;  // gain (skala medan)
const uint8_t REG_MODE     = 0x02;  // mode pengukuran
const uint8_t REG_DATA     = 0x03;  // awal blok data 6 byte
const uint8_t REG_STATUS   = 0x09;  // bit 0 = data baru siap (RDY)
const uint8_t NILAI_CONFIG_A = 0x70;  // 8 sampel rata-rata, 15 Hz, normal
const uint8_t NILAI_CONFIG_B = 0x20;  // gain +/-1,3 Ga
const uint8_t NILAI_MODE     = 0x00;  // pengukuran terus-menerus
// Nilai mentah saturasi pada gain ini: di atasnya sensor jenuh,
// artinya magnet terlalu dekat — jauhkan sedikit.
const int16_t BATAS_JENUH = 2048;
// Rata-rata vektor dari 3 kali baca supaya angka tidak bergetar.
const uint8_t JUMLAH_SAMPEL_RATA = 3;

// ---------- Konfigurasi arah mata angin ----------
// 16 arah, tiap sektor 22,5 derajat; indeks 0 = Utara.
// Nama lengkap dipakai di Serial, singkatan dipakai di LCD (16 kolom).
const uint8_t JUMLAH_ARAH = 16;
const float LEBAR_SEKTOR_DEG = 360.0 / JUMLAH_ARAH;  // 22,5 derajat
const char* NAMA_ARAH[JUMLAH_ARAH] = {
  "Utara", "Utara Timur Laut", "Timur Laut", "Timur Timur Laut",
  "Timur", "Timur Tenggara", "Tenggara", "Selatan Tenggara",
  "Selatan", "Selatan Barat Daya", "Barat Daya", "Barat Barat Daya",
  "Barat", "Barat Barat Laut", "Barat Laut", "Utara Barat Laut"
};
const char* SINGKATAN_ARAH[JUMLAH_ARAH] = {
  "U", "UTL", "TL", "TTL", "T", "TTG", "TG", "STG",
  "S", "SBD", "BD", "BBD", "B", "BBL", "BL", "UBL"
};

// ---------- Konfigurasi kalibrasi dan tampilan ----------
// Koreksi supaya heading 0 = utara yang benar. CARA ISI: arahkan
// baling-baling tepat ke utara, baca heading di Serial Monitor,
// lalu isi konstanta ini dengan NEGATIF dari angka tersebut.
// Contoh: Serial menunjukkan 8,5 saat menghadap utara, isi -8,5.
const float KALIBRASI_UTARA_DEG = 0.0;
const unsigned long PERIODE_TAMPIL_MS = 500;   // baca + tampil tiap 0,5 detik
const unsigned long PERIODE_COBA_ULANG_MS = 2000;  // coba sensor tiap 2 detik
const unsigned long BAUD_SERIAL = 115200;

// ---------- State ----------
bool sensorSiap = false;  // false = sensor belum menjawab di I2C
float headingDeg = 0.0;   // heading terakhir (0-360 derajat)
uint8_t indeksArah = 0;   // indeks 0-15 ke NAMA_ARAH
unsigned long waktuTampilTerakhirMs = 0;
unsigned long waktuCobaTerakhirMs = 0;

LiquidCrystal_I2C lcd(ALAMAT_LCD, KOLOM_LCD, BARIS_LCD);

// Tulis satu byte ke satu register HMC5883L. False bila I2C gagal.
bool tulisRegisterMagnetometer(uint8_t reg, uint8_t nilai) {
  Wire.beginTransmission(ALAMAT_MAGNETOMETER);
  Wire.write(reg);
  Wire.write(nilai);
  return Wire.endTransmission() == 0;
}

// Baca satu blok data 6 byte dari HMC5883L (mulai REG_DATA).
// Urutan byte chip ini: X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB.
// Mengisi x, y, z (mentah) dan mengembalikan false bila I2C gagal
// atau data belum siap (bit RDY di register status masih 0).
bool bacaMedanMagnet(int16_t &x, int16_t &y, int16_t &z) {
  // Cek bit RDY dulu: jangan baca data setengah jalan.
  Wire.beginTransmission(ALAMAT_MAGNETOMETER);
  Wire.write(REG_STATUS);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  if (Wire.requestFrom(ALAMAT_MAGNETOMETER, (uint8_t)1) != 1) {
    return false;
  }
  if ((Wire.read() & 0x01) == 0) {
    return false;  // data baru belum siap
  }

  Wire.beginTransmission(ALAMAT_MAGNETOMETER);
  Wire.write(REG_DATA);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  if (Wire.requestFrom(ALAMAT_MAGNETOMETER, (uint8_t)6) != 6) {
    return false;
  }
  uint8_t xMsb = Wire.read();
  uint8_t xLsb = Wire.read();
  uint8_t zMsb = Wire.read();
  uint8_t zLsb = Wire.read();
  uint8_t yMsb = Wire.read();
  uint8_t yLsb = Wire.read();
  x = (int16_t)((xMsb << 8) | xLsb);
  z = (int16_t)((zMsb << 8) | zLsb);
  y = (int16_t)((yMsb << 8) | yLsb);
  return true;
}

// Inisialisasi HMC5883L: cek ada di bus, lalu tulis 3 register
// konfigurasi. False bila sensor tidak menjawab.
bool inisialisasiMagnetometer() {
  Wire.beginTransmission(ALAMAT_MAGNETOMETER);
  if (Wire.endTransmission() != 0) {
    return false;  // tidak ada perangkat di alamat 0x1E
  }
  if (!tulisRegisterMagnetometer(REG_CONFIG_A, NILAI_CONFIG_A)) {
    return false;
  }
  if (!tulisRegisterMagnetometer(REG_CONFIG_B, NILAI_CONFIG_B)) {
    return false;
  }
  if (!tulisRegisterMagnetometer(REG_MODE, NILAI_MODE)) {
    return false;
  }
  return true;
}

// Hitung heading 0-360 derajat dari komponen X dan Y medan magnet.
// atan2 memberi -180..+180, digeser ke 0..360, lalu dikoreksi
// kalibrasi utara dan dilipat kembali ke 0..360.
float hitungHeading(int16_t x, int16_t y) {
  float heading = atan2((float)y, (float)x) * 180.0 / PI;
  if (heading < 0) {
    heading += 360.0;
  }
  heading += KALIBRASI_UTARA_DEG;
  while (heading < 0) {
    heading += 360.0;
  }
  while (heading >= 360.0) {
    heading -= 360.0;
  }
  return heading;
}

// Petakan heading ke indeks 0-15: tiap sektor 22,5 derajat, sektor
// Utara berpusat di 0 (dari 348,75 sampai 11,25 derajat).
uint8_t arahDariHeading(float heading) {
  int indeks = (int)((heading + LEBAR_SEKTOR_DEG / 2.0) / LEBAR_SEKTOR_DEG);
  return (uint8_t)(indeks % JUMLAH_ARAH);
}

// Baca sensor beberapa kali, rata-rata vektor X/Y-nya, lalu hitung
// heading + arah. Mengembalikan false bila baca gagal atau sensor
// jenuh (magnet terlalu dekat) — heading lama dipertahankan.
bool ukurArahAngin() {
  long jumlahX = 0;
  long jumlahY = 0;
  uint8_t berhasil = 0;
  for (uint8_t i = 0; i < JUMLAH_SAMPEL_RATA; i++) {
    int16_t x, y, z;
    if (bacaMedanMagnet(x, y, z)) {
      // Jenuh: nilai mentah mentok, sudutnya tidak bisa dipercaya.
      if (abs(x) >= BATAS_JENUH || abs(y) >= BATAS_JENUH) {
        Serial.println("PERINGATAN: sensor jenuh, jauhkan magnet sedikit.");
        return false;
      }
      jumlahX += x;
      jumlahY += y;
      berhasil++;
    }
    delay(10);
  }
  if (berhasil == 0) {
    return false;
  }
  headingDeg = hitungHeading((int16_t)(jumlahX / berhasil),
                             (int16_t)(jumlahY / berhasil));
  indeksArah = arahDariHeading(headingDeg);
  return true;
}

// Tampilkan arah + derajat di LCD (singkatan supaya muat 16 kolom).
void tampilkanLcd() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Arah: ");
  lcd.print(SINGKATAN_ARAH[indeksArah]);
  lcd.setCursor(0, 1);
  lcd.print(headingDeg, 1);
  lcd.print(" drjt");
}

// Kirim arah lengkap + derajat + nilai mentah ke Serial Monitor.
void kirimSerial() {
  Serial.print("Heading: ");
  Serial.print(headingDeg, 1);
  Serial.print(" drjt  Arah: ");
  Serial.println(NAMA_ARAH[indeksArah]);
}

void setup() {
  Serial.begin(BAUD_SERIAL);
  Wire.begin(PIN_SDA, PIN_SCL);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Pengukur Arah");
  lcd.setCursor(0, 1);
  lcd.print("Angin siap...");

  sensorSiap = inisialisasiMagnetometer();
  if (sensorSiap) {
    Serial.println("HMC5883L siap. Arahkan baling-baling ke utara untuk kalibrasi.");
  } else {
    Serial.println("HMC5883L TIDAK ditemukan di 0x1E. Periksa wiring SDA/SCL.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sensor tidak ada");
    lcd.setCursor(0, 1);
    lcd.print("cek wiring I2C");
  }
  delay(1500);
  if (sensorSiap) {
    tampilkanLcd();
  }
}

void loop() {
  unsigned long sekarang = millis();

  // Sensor belum siap: coba inisialisasi ulang tiap 2 detik.
  if (!sensorSiap) {
    if (sekarang - waktuCobaTerakhirMs >= PERIODE_COBA_ULANG_MS) {
      waktuCobaTerakhirMs = sekarang;
      sensorSiap = inisialisasiMagnetometer();
      if (sensorSiap) {
        Serial.println("HMC5883L ditemukan, mulai mengukur.");
        tampilkanLcd();
      }
    }
    return;
  }

  // Baca + tampilkan tiap 0,5 detik.
  if (sekarang - waktuTampilTerakhirMs >= PERIODE_TAMPIL_MS) {
    waktuTampilTerakhirMs = sekarang;
    if (ukurArahAngin()) {
      tampilkanLcd();
      kirimSerial();
    } else {
      Serial.println("Baca magnetometer gagal, memakai angka terakhir.");
    }
  }
}
