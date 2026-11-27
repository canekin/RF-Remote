# RF REMOTE

ESP32-C3 (0.42" OLED) ve CC1101 Sub-GHz alici-verici modulu kullanilarak gelistirilmis, **acilir acilmaz secili kumandayi otomatik gonderen**, fiziksel buton ve karanlik temali Web arayuzune sahip tasinabilir bir **Sub-GHz RF kumanda kopyalayici / verici** projesidir.

Garaj kapilari, bariyerler, kepenkler, uzaktan kumandali prizler ve sabit kodlu / ham (RAW) OOK RF sinyali kullanan diger tum sistemler icin kullanilabilir.

---

## Ozellikler

- **Acilista Otomatik Yayin (Auto-TX):** Cihaza guc verildigi anda Wi-Fi baslamadan once NVS'de secili olan kumandayi aninda gonderir (~1 sn). Telefona veya Wi-Fi'a baglanmaya gerek kalmadan tak-calistir kullanim saglar.
- **Cift Donanim Verici Motoru:**
  - **ESP32-C3 Hardware RMT TX (`1 us` cozunurluk):** Kesintisiz ve sifira yakin zamanlama sapmasiyla OOK/ASK darbe dizisi uretir.
  - **CC1101 40 kbps Infinite-FIFO Yedegi:** Ayni yayini `FSCTRL0=0x00` kalibrasyonu ile CC1101 FIFO uzerinden de destekler.
- **Bostayken Sifir Dinleme (`SIDLE`):** Cihaz bostayken CC1101 `SIDLE` (`MARC=0x01`) modunda bekler; gereksiz RF paraziti, CPU kesmesi ve guc tuketimi olusturmaz.
- **Istege Bagli 60 sn Dinleme Modu:** Yalnizca Web arayuzunden (veya seri porttan) acildiginda CC1101 `SRX` (`MARC=0x0D`) moduna gecer, 2 cerceveli dogrulama ile sabit kodlu (`12/24/25 bit`) veya ham (`RAW`, max 140 kenar) kumandalari yakalayip kaydeder ve sure bitince tekrar `SIDLE` moduna doner.
- **0.42" OLED (`72x40`) Ekran:**
  - Ust satirda aktif frekans (MHz) ve secili slot (`1/N`) veya dinleme aninda canli RSSI (`dBm`).
  - Orta bolumde kumanda adina gore otomatik boyutlanan (`7x13B` / `6x10`) veya 2 satira sarilan okunakli isim gosterimi.
  - Alt satirda Web IP (`192.168.4.1`) ve toplam kayitli kumanda sayisi (`K:N`).
- **BOOT Tusu Kontrolu (`GPIO 9`):**
  - **Kisa basma:** Secili kumandayi gonderir.
  - **Basili tutma (`>= 600 ms`):** Kayitli kumandalar arasinda bir sonrakine gecer ve acilis/BOOT secimi olarak NVS'ye kaydeder.
- **Siyah-Kirmizi Terminal Web Arayuzu (`192.168.4.1`):**
  - Tek tikla gonderim (`/tx`), acilis/BOOT kumandasi secimi (`/select`), yeniden adlandirma (`/rename`) ve silme (`/del`).
  - **Bruce `.sub` RAW Ice Aktarma:** Flipper / Bruce Sub-GHz `RAW_Data:` verilerini dogrudan yapistirarak ekleme (`/add_raw`).
  - Manuel kod ekleme (`/add`), frekans secimi (`433.92`, `433.42`, `315.00`, `868.35 MHz`), ham dokum (`/dump`) ve canli olay loglari.
- **Kalici NVS Hafizasi:** 20 adede kadar kumandayi, ham darbe dizilerini (`<= 140` kenar) ve son secili kumandayi elektrik kesintisinde korur.

---

## Donanim ve Pin Baglantilari

| Bilesen | Pin / Sinyal | ESP32-C3 GPIO | Aciklama |
| :--- | :--- | :--- | :--- |
| **OLED (`SSD1306 72x40`)** | `SDA` | `GPIO 5` | Donanim I2C veri hatti |
| **OLED (`SSD1306 72x40`)** | `SCL` | `GPIO 6` | Donanim I2C saat hatti |
| **CC1101** | `SCK` | `GPIO 4` | SPI saat |
| **CC1101** | `MISO` | `GPIO 3` | SPI MISO |
| **CC1101** | `MOSI` | `GPIO 7` | SPI MOSI |
| **CC1101** | `CSN` | `GPIO 10` | SPI Chip Select |
| **CC1101** | `GDO0` | `GPIO 1` | RMT TX / RX asenkron veri pini |
| **Durum LED'i** | `LED` | `GPIO 8` | Dahili mavi LED (Active LOW) |
| **BOOT Butonu** | `BTN` | `GPIO 9` | Dahili buton (`INPUT_PULLUP`) |

> **Not:** `CC1101` modulunun 26 MHz kristalli 433 MHz versiyonu ile test edilmistir (`FSCTRL0 = 0x00`).

---

## Kurulum ve Derleme

### Gerekli Kutuphaneler
Arduino IDE / `arduino-cli` uzerinden asagidaki kutuphanelerin kurulu olmasi gerekir:
- `SmartRC-CC1101-Driver-Lib`
- `U8g2`
- `rc-switch`

### Arduino IDE Ayarlari
- **Board:** `ESP32C3 Dev Module`
- **USB CDC On Boot:** `Enabled` *(Seri port loglari ve USB yukleme icin zorunludur)*
- **CPU Frequency:** `160 MHz`
- **Flash Mode:** `QIO`
- **Flash Size:** `4MB (32Mb)`

### `arduino-cli` ile Derleme ve Yukleme
```powershell
arduino-cli compile --fqbn "esp32:esp32:esp32c3:UploadSpeed=921600,CDCOnBoot=cdc,CPUFreq=160,FlashFreq=80,FlashMode=qio,FlashSize=4M,PartitionScheme=default" RF_REMOTE
arduino-cli upload -p COM3 --fqbn "esp32:esp32:esp32c3:UploadSpeed=921600,CDCOnBoot=cdc,CPUFreq=160,FlashFreq=80,FlashMode=qio,FlashSize=4M,PartitionScheme=default" RF_REMOTE
```

---

## Kullanim

### 1. Wi-Fi Web Arayuzu
1. Cihaz acildiginda olusturulan Wi-Fi agina baglanin:
   - **SSID:** `your_ssid`
   - **Sifre:** `your_password`
2. Tarayicidan **`http://192.168.4.1`** adresine gidin.
3. Buradan mevcut kumandalari gonderebilir, isimlerini degistirebilir, **Acilista Sec** butonu ile guce takildiginda otomatik gonderilecek kumandayi belirleyebilir, **Dinleme Modu** ile yeni kumanda ogretebilir veya Bruce `.sub` dosyasindan `RAW_Data:` yapistirabilirsiniz.

### 2. Fiziksel Buton (`BOOT / GPIO 9`)
- **Kisa Basma:** Secili kumandayi hemen yayinlar.
- **Basili Tutma (`>= 0.6 sn`):** Bir sonraki kayitli kumandaya gecer ve ekrani/NVS'yi gunceller.

### 3. Seri Port Komutlari (`115200 baud`)
USB uzerinden seri monitor acildiginda kullanilabilen komutlar:
- `tx1` .. `txN`: Belirtilen slottaki kumandayi gonderir.
- `dump`: Kayitli tum kumandalari ve ham darbe surelerini listeler.
- `regs`: Anlik CC1101 register durumlarini yazdirir.
- `listen`: 60 saniyelik dinleme modunu acar/kapatir.

---

## Depo Yapisi

```text
.
├── RF_REMOTE/
│   └── RF_REMOTE.ino   # ESP32-C3 + CC1101 ana firmware kaynak kodu
├── raw.sub             # Ornek Bruce SubGhz RAW yakalama dosyasi
└── README.md           # Proje dokumantasyonu
```

---

## Yasal Uyari

Bu proje yalnizca **kendi sahip oldugunuz cihazlarin yedeklenmesi, egitim ve arastirma** amaciyla hazirlanmistir. Yetkisiz sistemler uzerinde kullanmayiniz. Bu proje ile yaplilan hic bir eylemden ben sorumlu degilim. Yaptiklarinizdan sadece siz sorumlu olursunuz.
