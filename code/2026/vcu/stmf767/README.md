# MVCU Gömülü Kodu
----------
## Özellikler

- 16 adet farklı işlem koduna sahip veri gönderimi/alımı gerçekleştirilebilmektedir. 12'si rezerve edilmiş olup 4'ü kullanımdadır.
- Kullanımda olan işlem kodları şunlardır:
  1. **Alt Yürür Motorları ve Verileri**
  2. **Robot Kol Motorları ve Verileri**
  3. **Bilim Sistemi Motorları ve Verileri**
  4. **Konum Bilgisi**

- `processID` olarak belirtilen yerde, en sağ bitten başlayarak 1 veya 0 olması, talep edilen verinin olup olmadığını belirtir.
  - Örneğin,
    - `0000 0001` → Sadece Alt Yürür Motorları sürme komutu ve motor verileri geri beslemesi.
    - `0000 0011` → Alt Yürür Motorları ve Robot Kol Motorları sürme komutları ve motor verileri geri beslemeleri.

---

## TCP/IP

### Server
- **IP Adresi:** `192.168.1.2`
- **Port:** `6300`

### Mesaj Paketi Yapısı
```
0-1      Byte-Hex    TimeStamp
2-3      Byte-Hex    Process ID
4-64    Byte-Hex    Payload
```

---

## UDP/IP

### Server
- **IP Adresi:** `Bilgisayarınızın IP adresi`
- **Port:** `6102`

### Client
- **IP Adresi:** `192.168.1.2`
- **Port:** `6101`

### Açıklama
- Server adresi girilmelidir gömülüye eklenmelidir. Server, ilk paketi gönderdikten sonra akış başlatılmalıdır.
- İstenilen veya gönderilen veriler **Process ID** kısmında belirtilemlidir.
- Birden fazla veri gönderildiğinde, istendiğinde, gerekli byte uzunlukları ilerleyen zamanlarda daha kesinleşecektir. Şu anlık geliştirme aşamasındadır.

### Mesaj Paketi Yapısı
```
0-1      Byte-Hex    TimeStamp
2-3      Byte-Hex    Process ID
4-256    Byte-Mixed  Payload
```


### Process ID Anlamlandırması
```
0000 0000 0000 0000
```
- 2 byte veri 16 bitten oluşmaktadır. Bu 16 bitin her biri bir fonksiyonu temsil etmekte olup, 1 veya 0 olmasına bağlı olarak, gelen mesaj paketine ilişkin verinin "Payload" kısmında hangi verilerin barındırdığını belirtir.
- Örnek olarak `0000 0000 0000 0011` verisi iki fonksiyonu belirtir. Gömülüye gelen payload üzerinde pID sonrası gelen ilk veriyi, LSB'den başlayarak 1 veya 0 olmasına bağlı olarak payload ile process id'yi ilişkilendirir.
  - Örneğin `0000 0000 0000 0101` pID gelmesi durumunda payload bölümündeki ilk X byte en sağdaki bitin bağlı olduğu fonksiyona, önceden tanımlanmış sayıda byte sonrasında da sırası ile onun solundaki fonksiyonların, 1 veya 0 olmasına bağlı olarak fonksiyonun girişi ile payload arasında ilişkilendirme yapmaktadır. 

- Şu anki fonksiyon-bit ilişiği aşağıdaki gibidir.

```void (*bitOperations[16])(uint8_t* payload, uint16_t length) = {
    drivetrain_motorCommand, roboticarm_motorCommand, science_motorCommand, gnssData,
    ledState, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL
};
```


### Payload Anlamlandırması
- **Alt Yürür**
```
0        Byte-Char    Açısal Hız Yönü (0 - 1)
1-3      Byte-Char    Açısal Hız      (000-999)
4        Byte-Char    Doğrusal Yön    (0 - 1)
5-8      Byte-Char    Doğrusal Hız    (0000-9999)
```
- Örneğin
  - `101510250` 1-3-1-4 olarak parçalara bölüp inceleyecek olursak; 
    - 1    → Saat yönünde açısal hız komutu veriliyor.
    - 015  → 15 birim açısal hız
    - 1    → İleri yönlü doğrusal hız komutu
    - 0250 → Belirtilen yönde 250 birim doğrusal hız


- **Robot Kol**


---

09.03.2025 test application description:
- SPI Transmits its txbuffer evey second with a timer interrrupt and increases txbuffer value every iteration. LD3 led toggles for every succesful transmission.
  
- TCP and UDP server apllications append a string to the client message and send it back. They also control the LD2 led with commands form client. (LT = LED TOGGLE, LS = LED SET, LR = LED RESET)
  
- UDP Client periodicaly sends the number of server messages recieved during runtime using a timer interrupt.
