# Pengukur Arah Angin — ESP32 + HMC5883L + LCD

Baling-baling ini tahu angin datang dari mana. Sebuah baling-baling
penunjuk arah (wind vane) diputar angin sampai panahnya menghadap ke
arah datangnya angin; magnet kecil di porosnya ikut berputar, dan
sensor magnetometer HMC5883L yang diam di bawah poros membaca sudut
putar itu. ESP32 menghitung heading 0-360 derajat, memetakannya ke
16 arah mata angin, dan menampilkannya di LCD — lengkap dengan
derajatnya, setiap setengah detik.

Video penjelasan (60 detik):
https://youtube.com/shorts/YIarzz5OOn8

## Komponen

| Bagian | Keterangan |
|---|---|
| ESP32 DevKit | Otak alat |
| Baling-baling penunjuk arah | Panah + ekor di poros vertikal, berputar bebas mengikuti angin |
| Magnet kecil | Neodymium 5-10 mm di poros, kutub menghadap samping |
| HMC5883L | Magnetometer 3 sumbu, alamat I2C 0x1E |
| LCD 16x2 I2C (0x27) | Tampil singkatan arah (mis. BL) + derajat |
| Catu USB 5 V | Adaptor USB / powerbank untuk ESP32 |

## Wiring

| Dari | Ke |
|---|---|
| HMC5883L VCC | ESP32 3V3 |
| HMC5883L GND | GND |
| HMC5883L SDA | ESP32 GPIO21 |
| HMC5883L SCL | ESP32 GPIO22 |
| LCD SDA | ESP32 GPIO21 (satu bus I2C dengan sensor) |
| LCD SCL | ESP32 GPIO22 (satu bus I2C dengan sensor) |
| LCD VCC | ESP32 3V3 |
| LCD GND | GND |
| ESP32 VIN 5V | Catu USB 5 V |
| Semua GND | Digabung |

Skema lengkap: [schematic.png](schematic.png)

## Cara kerja

Yang diukur sensor BUKAN arah utara magnetis bumi — melainkan arah
magnet kecil di poros baling-baling. Karena magnetnya dekat (2-4 cm),
medannya jauh lebih kuat dari medan magnet bumi, sehingga sudut yang
terbaca HMC5883L adalah sudut putar poros, alias arah baling-baling.
Sensor dibaca mentah lewat I2C (Wire), tanpa library eksternal:
register Config A (0x00) diisi 0x70, Config B (0x01) diisi 0x20
(gain +/-1,3 Ga), Mode (0x02) diisi 0x00 (pengukuran terus-menerus),
lalu 6 byte data dibaca mulai register 0x03 dengan urutan X, Z, Y
(perhatikan: Z dibaca sebelum Y — itu urutan register chip ini).

Heading dihitung dengan rumus:

```
heading (derajat) = atan2(Y, X) x 180 / pi   (dinormalisasi ke 0-360)
```

Tiga kali baca dirata-rata dulu (rata-rata vektor X/Y) supaya angka
tidak bergetar. Lalu heading dipetakan ke 16 arah mata angin — tiap
sektor 22,5 derajat, sektor Utara berpusat di 0 derajat (dari 348,75
sampai 11,25 derajat):

| Derajat | Arah | Singkatan |
|---|---|---|
| 348,75 - 11,25 | Utara | U |
| 11,25 - 33,75 | Utara Timur Laut | UTL |
| 33,75 - 56,25 | Timur Laut | TL |
| 56,25 - 78,75 | Timur Timur Laut | TTL |
| 78,75 - 101,25 | Timur | T |
| 101,25 - 123,75 | Timur Tenggara | TTG |
| 123,75 - 146,25 | Tenggara | TG |
| 146,25 - 168,75 | Selatan Tenggara | STG |
| 168,75 - 191,25 | Selatan | S |
| 191,25 - 213,75 | Selatan Barat Daya | SBD |
| 213,75 - 236,25 | Barat Daya | BD |
| 236,25 - 258,75 | Barat Barat Daya | BBD |
| 258,75 - 281,25 | Barat | B |
| 281,25 - 303,75 | Barat Barat Laut | BBL |
| 303,75 - 326,25 | Barat Laut | BL |
| 326,25 - 348,75 | Utara Barat Laut | UBL |

LCD menampilkan singkatan arah di baris pertama ("Arah: BL") dan
derajat di baris kedua ("312,5 drjt"). Nama lengkap arah + derajat
dikirim juga ke Serial Monitor tiap 0,5 detik.

## Kalibrasi utara (wajib sekali sebelum dipakai)

Supaya angka yang tampil sama dengan arah mata angin yang benar,
sensor harus dikalibrasi sekali menghadap utara:

1. Arahkan baling-baling tepat ke utara yang diketahui (kompas HP
   atau patokan yang pasti, mis. arah kiblat yang diketahui).
2. Baca heading di Serial Monitor.
3. Isi konstanta `KALIBRASI_UTARA_DEG` di sketch dengan NEGATIF dari
   angka tersebut. Contoh: Serial menunjukkan 8,5 saat menghadap
   utara, isi `-8,5`.
4. Upload ulang sketch.

Pemasangan mekanik yang perlu diperhatikan:

- Magnet ditempel di poros dengan kutub menghadap ke SAMPING (bukan
  ke atas/bawah), supaya medan di bidang X-Y sensor berputar
  mengikuti poros.
- Jarak magnet ke sensor sekitar 2-4 cm dan TETAP: kalau terlalu
  jauh, medan bumi mulai mengganggu; kalau terlalu dekat, sensor
  jenuh (nilai mentah mentok di +/-2048 — sketch memberi peringatan
  di Serial bila ini terjadi).
- Sensor dipasang DATAR dan tidak ikut berputar; jauhkan dari besi,
  motor, dan kabel berarus besar.
- Modul QMC5883L TIDAK cocok dengan sketch ini — alamatnya 0x0D dan
  registernya berbeda. Pakai modul bertuliskan HMC5883L.

## Angka compile terverifikasi

Sketch `PengukurArahAnginMagnetometer.ino` terverifikasi compile pada
25 Nov 2026 dengan arduino-cli 1.6.0, ESP32 core 4.0.0:

- Ukuran sketch: 318.405 byte (24% dari program storage)
- RAM (global variables): 24.760 byte (7% dari dynamic memory)

Library yang dipakai: LiquidCrystal_I2C 1.1.2 (tersedia di `libraries/`).
HMC5883L dibaca mentah via Wire — tanpa library eksternal.

## Lisensi

MIT — lihat [LICENSE](LICENSE).
