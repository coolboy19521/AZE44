# Voltivarius Komandasının Hesabatı

[![WRO - Future Engineers](https://img.shields.io/badge/WRO-Future_Engineers-2e52af)](https://wro-association.org/wp-content/uploads/WRO-2024-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)
[![YouTube - Race & Installing Videos](https://img.shields.io/badge/YouTube-▶️%20Race_&_Installing_Videos-df3e3e?logo=youtube)](https://www.youtube.com/playlist?list=PLiso-udvas0o-0Et_wnVIpQN-5FDyN6-4)

Bizim robotumuz **Raspberry Pi** bir-lövhəli kompüter ilə avtonom sürüş edir. Maneələri və hərəkətini təyin etmək üçün lidar, kamera, giroskop və məsafə sensorlarından istifadə edir. Robotun hərəkət etməsi üçün arxasında bir motor və bu motora qoşulmuş iki təkər vardır. Dönməsi üçünsə robotun qabağındakı servo motorundan və **Ackermann idarəetmə sistemindən** yararlanılır.

<table cellspacing="0" cellpadding="0" style="margin:0; padding:0; border-collapse:collapse;">
  <tr>
    <td align="center" colspan="2">
      <img src="media/right.jpg" height="300"><br>
      <b>Sağ</b>
    </td>
    <td align="center" colspan="2">
      <img src="media/left.jpg" height="300"><br>
      <b>Sol</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="media/up.jpg" height="300"><br>
      <b>Üst</b>
    </td>
    <td align="center">
      <img src="media/front.jpg" height="300"><br>
      <b>Qabaq</b>
    </td>
    <td align="center">
      <img src="media/back.jpg" height="300"><br>
      <b>Arxa</b>
    </td>
    <td align="center">
      <img src="media/down.jpg" height="300"><br>
      <b>Alt</b>
    </td>
  </tr>
</table>

Robotumuz maneə mərhələsi üçün effektiv alqoritmdən istifadə edir. Belə ki, ilk tur bizim robot üçün öyrənmə turudur və biz bu turu yavaş yerinə yetiririk. Bu turda hədəfimiz bütün maneələrin rəngini düzgün görmək və maneələrə toxunmamaqdır. Bu turda maneələrin rəngi yadda saxlanılır. Buna görə də ikinci və üçüncü turlarda maneələrin rənginin oxunmasına ehtiyac qalmır. Beləcə birinci tur üçün `70 saniyə`, ikinci və üçüncü tur üçünsə sadəcə `30-35 saniyə` xərcləyirik.

### Hesabat Mündəricatı

- [Robot elektronikası]()
  - [Materialların siyahısı]()
- [Proqram mühitinin hazırlanması]()
  - [Raspberry Pi konfiqurasiyası]()
  - [Lazımlı paketlərin yüklənilməsi]()
  - [Kodu yoxlama və işlətmə]()
- [Strategiyanın izahı]()
- [Robotun videoları]()

# Robot elektronikası
### Materialların siyahısı

<table>
  <tr>
    <th>Komponentlər</th><th>Şəkil</th>
    <th>Komponentlər</th><th>Şəkil</th>
  </tr>

  <tr>
    <td>Raspberry Pi 5 x 1</td><td><img src="media/image1.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Motor Sürücü x 1</td><td><img src="media/image2.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>RPLidar-A1M8 x 1</td><td><img src="media/image3.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>BNO055 x 1</td><td><img src="media/image4.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>3s Li-Po x 1</td><td><img src="media/image5.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>IMAX B6AC batareya doldurucu x 1</td><td><img src="media/image6.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>MG995 Servo Motor x 1</td><td><img src="media/image7.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Raspberry Pi Kamera Modulu 3 x 1</td><td><img src="media/image8.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Enkoder mühərriki x 1</td><td><img src="media/image9.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Perfboard x 1</td><td><img src="media/image10.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Dəyişdiricini ON-OFF x 1</td><td><img src="media/image11.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>2,54 mm JST-XH Bağlayıcı Kişi x 5</td><td><img src="media/image12.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>2,54 mm JST-XH Konnektor Dişi x 5</td><td><img src="media/image13.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>XT-60 x 1</td><td><img src="media/image14.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>2.54 mm Pin Başlığı Dişi x 1</td><td><img src="media/image15.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Düymə x 1</td><td><img src="media/image16.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Sigorta x 1</td><td><img src="media/image17.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Sigorta Tutucusu x 1</td><td><img src="media/image18.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>10K ohm Rezistor x 1</td><td><img src="media/image19.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>330ohm rezistor x 2</td><td><img src="media/image19.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>GPIO kabeli x 1</td><td><img src="media/image20.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Batareya monitoru x 1</td><td><img src="media/image21.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Bilyalı birləşmə x 4</td><td><img src="media/image22.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>3 mm sərt flanşlı mufta x 4</td><td><img src="media/image23.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>50 mm Standoff Qadın-Kişi x 6</td><td><img src="media/image24.jpg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>20mm Standoff Qadın - Kişi x 4</td><td><img src="media/image25.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>3 mm mil 7 sm x 2</td><td><img src="media/image26.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Li-Po Batareya Test Cihazı Gərginlik Monitoru x 1</td><td><img src="media/image27.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Filament 1 kq x 1</td><td><img src="media/image28.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>M3 Neylon Daxiletmə Kilid Qoz x 2</td><td><img src="media/image29.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Type-c - Type-c Kabel x 1</td><td><img src="media/image30.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Type-c - USB Kabel x 1</td><td><img src="media/image31.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>32 GB Yaddaş Kartı x 1</td><td><img src="media/image32.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Aktiv soyuducu x 1</td><td><img src="media/image33.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Avtomobil Şassisinin Aşağı hissəsi x 1</td><td><img src="media/image34.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Avtomobilin şassisinin yuxarı hissəsi x 1</td><td><img src="media/image35.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Motor qutusu x 1</td><td><img src="media/image36.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Motor qutusu təkəri x 1</td><td><img src="media/image37.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Motor qutusu Təkər 2 x 1</td><td><img src="media/image38.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Sürət qutusu təkəri x 4</td><td><img src="media/image39.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Servo braket x 1</td><td><img src="media/image40.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Təkər Sükan Sistemi x 2</td><td><img src="media/image41.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Ön Təkər Tutacağı x 2</td><td><img src="media/image42.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Kabel tutacağı x 1</td><td><img src="media/image43.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Düymə Sahibi x 1</td><td><img src="media/image44.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>Kamera Tutacağı x 1</td><td><img src="media/image45.png" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>XL-4015 x 2</td><td><img src="media/image46.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
    <td>HCSR-04 x 1</td><td><img src="media/image47.jpeg" style="width:310px;height:170px;object-fit:contain;background:white;"></td>
  </tr>
</table>

### Sistemə baxış
### Elektron Sistemin Arxitekturası

- **Raspberry Pi 5:** Sensor məlumatlarını toplayır, emal edir və aktuator əmrləri yaradır.
- **Raspberry Pi Pico + DRV8833:** DC motor və servoları idarə edir, enkoder məlumatlarını geri ötürür.
- **Enkoderli DC Motor:** İrəli-geri hərəkəti təmin edir və sürət/mövqe məlumatı verir.
- **Servo Motor:** Robotun dönüşlərini idarə edir.

### İş Prinsipi
-	Enerji Li-Po 3S batareyadan təmin olunur.
-	Raspberry Pi 5 işə düşür və ROS qovşaqları aktivləşir.
-	Sensorlardan (lidar, kamera) məlumat alınır.
-	Pi 5 məlumatları emal edir və qərar qəbul edir.
-	Əmrlər Pico vasitəsilə DRV8833-ə ötürülür.
-	Motorlar və servo hərəkəti icra edir.
-	Sistem real vaxt rejimində dövr edir.

<h3>Raspberry Pi 5</h3>

<table style="width:80%; border-collapse:collapse;">
  <tr>
    <td rowspan="4" style="width:400px; text-align:center; vertical-align:middle; padding:0; margin:0;">
      <img src="media/image1.jpeg" style="width:400px; height:210px; display:block; margin:0 auto; border:0;">
    </td>
    <td>USB</td>
    <td>LiDAR sensoru və əlavə cihazların birləşdirilməsi</td>
  </tr>
  <tr>
    <td>CSI Kamera Poru:</td>
    <td>Kamera Modulu 3 üçün qoşulma</td>
  </tr>
  <tr>
    <td>GPIO sancaqları</td>
    <td>Başlat düyməsi, güc açarı, kodlayıcı və digər sensor siqnalları</td>
  </tr>
  <tr>
    <td>İş gərginliyi</td>
    <td>5V DC, minimum 3A</td>
  </tr>
</table>

Raspberry Pi 5 robotun əsas nəzarətçisi kimi istifadə olunan yüksək performanslı bir lövhəli kompüterdir. Sensor məlumatlarını emal edir, aktuatorlara nəzarət siqnalları yaradır və ümumi sistem əməliyyatlarını ROS üzərindən idarə edir.

Robotumuzda rolu:

- RPLiDAR A1M8 və Kamera Modulu 3-dən məlumatların emalı
- Raspberry Pi Pico vasitəsilə motorların və servoların idarəsi
- Başlat düyməsi və güc açarı üçün GPIO istifadə

Güc Tələbləri:

- Li-Po 3S 1300 mAh batareya və gərginlik tənzimləyicisi ilə təchiz olunur
- Periferik cihazlar üçün sabit enerji təmin edir

<h3>Raspberry Pi Pico + DRV8833 Motor Sürücü</h3>

<table style="width:80%; border-collapse:collapse;">
  <tr>
    <td rowspan="5" style="width:300px; text-align:center; vertical-align:middle; padding:0; margin:0;">
      <img src="media/image49.jpeg" style="width:300px; height:210px; display:block; margin:0 auto; border:0;">
    </td>
    <td>RP2040 mikronəzarətçi (Pico)</td>
  </tr>
  <tr>
    <td>Dual H-Bridge motor sürücü (DRV8833)</td>
  </tr>
  <tr>
    <td>Enkoder oxunuşları üçün dəstək</td>
  </tr>
  <tr>
    <td>PWM ilə sürət və istiqamət idarəsi</td>
  </tr>
  <tr>
    <td>Kompakt ölçü və aşağı enerji istehlakı</td>
  </tr>
</table>

Raspberry Pi Pico mikronəzarətçisi DRV8833 motor sürücüsü ilə birlikdə DC mühərrikləri və servoları idarə etmək üçün istifadə olunur. Əsas nəzarətçi olan Raspberry Pi 5-dən gələn əmrləri qəbul edir və aktuator hərəkətlərinə çevirir.

Mühərrik Birləşmələri
-	2 kanal DC mühərrik idarəsi
-	PWM siqnalları ilə sürət və istiqamət nəzarəti
-	Enkoderlərdən sürət və mövqe rəyi

Güc Tələbləri
-	**DRV8833:** 2.7–10.8V giriş
-	**Raspberry Pi Pico:** 3.3V işləmə gərginliyi

<h3>Mühərriklər və Servo Motor</h3>

<h4>Mühərrik</h4>

<table style="width:80%; border-collapse:collapse;">
  <tr>
    <td rowspan="5" style="width:400px; text-align:center; vertical-align:middle; padding:0; margin:0;">
      <img src="media/image51.png" style="width:400px; height:210px; display:block; margin:0 auto; border:0;">
    </td>
    <td>İş gərginliyi</td>
    <td>5–12V DC</td>
  </tr>
  <tr>
    <td>Enkoder gərginliyi</td>
    <td>3.3–5V DC</td>
  </tr>
  <tr>
    <td>Yüksüz cərəyan</td>
    <td>200 mA, nominal: 300 mA, dayanma: 800 mA</td>
  </tr>
  <tr>
    <td>Sürət</td>
    <td>~1000 RPM @ 12V</td>
  </tr>
  <tr>
    <td>Ölçü</td>
    <td>25 × 71 mm, Çəki: 95 q</td>
  </tr>
</table>

<h4>Servo</h4>

<table style="width:80%; border-collapse:collapse;">
  <tr>
    <td rowspan="4" style="width:400px; text-align:center; vertical-align:middle; padding:0; margin:0;">
      <img src="media/image52.png" style="width:400px; height:210px; display:block; margin:0 auto; border:0;">
    </td>
    <td>Fırlanma bucağı</td>
    <td>30°–160°</td>
  </tr>
  <tr>
    <td>Sürət</td>
    <td>0.15 s/60°</td>
  </tr>
  <tr>
    <td>Tork</td>
    <td>13–15 kg·cm</td>
  </tr>
  <tr>
    <td colspan="2">Metal dişli, yüksək davamlılıq</td>
  </tr>
</table>

Əlaqələr və İnteqrasiya
-	**DC mühərriklər:** DRV8833 vasitəsilə idarə olunur
-	**Enkoderlər:** Pico üzərindən Pi 5-ə məlumat göndərir
-	**Servo:** PWM ilə nəzarət

Güc Tələbləri
-	**DC mühərriklər:** 5–12V DC
-	**Servo:** 4.8–6V DC

Nəzarət Siqnalları
-	**DC mühərriklər:** PWM + Enkoder rəyi
-	**Servo:** PWM ilə mövqe bucağı idarəsi

<h3>LiDAR Sensor (RPLIDAR A1M8)</h3>

<table style="width:80%; border-collapse:collapse;">
  <tr>
    <td rowspan="5" style="width:330px; text-align:center; vertical-align:middle; padding:0; margin:0;">
      <img src="media/image3.jpeg" style="width:330px; height:210px; display:block; margin:0 auto; border:0;">
    </td>
    <td>Ölçü</td>
    <td>98.5 × 70 × 60 mm</td>
  </tr>
  <tr>
    <td>Çəki</td>
    <td>170 q</td>
  </tr>
  <tr>
    <td>Tarama diapazonu</td>
    <td>6 m</td>
  </tr>
  <tr>
    <td>Nümunə götürmə sürəti</td>
    <td>8000/s</td>
  </tr>
  <tr>
    <td>Tarama tezliyi</td>
    <td>2–10 Hz (PWM ilə)</td>
  </tr>
</table>

SLAMTEC RPLiDAR A1M8, xəritəçəkmə və naviqasiya üçün nəzərdə tutulmuş 360° 2D lazer skaneridir. Yüngül, etibarlı və mobil robotlar üçün optimallaşdırılmışdır.

Əlaqələr
-	USB interfeysi ilə Raspberry Pi 5-ə qoşulur
-	PWM ilə motor sürəti tənzimlənir

Güc Tələbləri
-	5V DC (USB)
-	Aşağı enerji istehlakı

<h3>Kamera (Raspberry Pi Camera Module v3)</h3>

<table style="width:80%; border-collapse:collapse;">
  <tr>
    <td rowspan="4" style="width:280px; text-align:center; vertical-align:middle; padding:0; margin:0;">
      <img src="media/image53.png" style="width:280px; height:210px; display:block; margin:0 auto; border:0;">
    </td>
    <td>Çözünürlük</td>
    <td>4608 × 2592 (11.9 MP)</td>
  </tr>
  <tr>
    <td>Piksel</td>
    <td>1.4 µm</td>
  </tr>
  <tr>
    <td>Video</td>
    <td>1080p50, 720p100, 480p120</td>
  </tr>
  <tr>
    <td>Ölçü</td>
    <td>25 × 24 × 11.5 mm, Çəki: 3 q</td>
  </tr>
</table>

Sony IMX708 sensorlu 11.9 MP kamera modulu, yüksək keyfiyyətli təsvir və video çəkmə imkanına malikdir. HDR və PDAF avtofokus dəstəkləyir.

Əlaqələr
-	CSI-2 portu ilə Raspberry Pi 5-ə qoşulur

Güc Tələbləri
-	CSI interfeysi üzərindən enerji alır

<h3>Gyroskop (IMU) Sensoru – Adafruit BNO055</h3>

<table style="width:80%; border-collapse:collapse;">
  <tr>
    <td rowspan="4" style="width:310px; text-align:center; vertical-align:middle; padding:0; margin:0;">
      <img src="media/image54.jpeg" style="width:310px; height:210px; display:block; margin:0 auto; border:0;">
    </td>
    <td>9 DOF</td>
    <td>Akselerometr, Giroskop, Maqnitometr</td>
  </tr>
  <tr>
    <td>Çıxış</td>
    <td>Euler bucaqları, Kvaternion, Vektorlar</td>
  </tr>
  <tr>
    <td>Tezlik</td>
    <td>100 Hz-ə qədər</td>
  </tr>
  <tr>
    <td>Ölçü</td>
    <td>20 × 27 × 4 mm, Çəki: 3 q</td>
  </tr>
</table>

Bosch BNO055 əsaslı IMU sensoru daxili sensor birləşməsi ilə dəqiq oriyentasiya məlumatı təqdim edir.

Əlaqələr
-	**Rabitə:** I²C (0x28 və ya 0x29), UART dəstəyi
-	Reset və Interrupt pinləri

Güc Tələbləri
-	**İş gərginliyi:** 3.3V (5V tolerant I²C girişləri ilə)

<h3>Xüsusi PCB Dizaynı</h3>

Bu xüsusi idarəetmə lövhəsi Raspberry Pi 5 və robot sisteminin periferik komponentləri arasında əsas interfeys kimi hazırlanmışdır. O, modullar arasında mütəşəkkil və təhlükəsiz əlaqəni təmin edən sensor birləşmələrini, istifadəçinin qarşılıqlı əlaqə elementlərini (LED, düymə, səs siqnalı) və enerji idarəetmə komponentlərini birləşdirir.

**Bir qatlı perfboard (prototip lövhəsi)** üzərində qurulmuşdur və aşağıdakı əsas komponentləri ehtiva edir:
-	**İki LED (qırmızı və yaşıl):** Vəziyyət və xəbərdarlıq göstəriciləri kimi istifadə olunur.
-	**Bir siqnal:** Səsli xəbərdarlıqlar verir.
-	**Bir təkan düyməsi:** İstifadəçi girişinə nəzarət.
-	**Bir qoruyucu:** Həddindən artıq cərəyandan qorunma.
-	**Bir giroskop sensoru (BNO055 IMU):** Oriyentasiya və hərəkət sensoru.
-	**Bir açar:** Gücü yandırma/söndürmə nəzarəti.
-	**Raspberry Pi 5 GPIO əlaqələri:** Sistemə nəzarət və rabitə üçün.

Perfboardda elektrik xətləri qalın mis naqillərlə, siqnal xətləri isə daha incə naqillərlə, cərəyanı idarə etmək qabiliyyətini təmin etmək və müdaxiləni minimuma endirmək üçün çəkilir.

Bağlantı nöqtələri
-	**Güc Girişi:** 6–14 V xarici enerji təchizatı ilə işləyir .
-	**Raspberry Pi 5 GPIO:** LEDlər, düymələr, səs siqnalları və sensor üçün birbaşa bağlantılar.
-	**Sensor bağlantısı:** BNO055 IMU I²C interfeysi (SCL, SDA) vasitəsilə Raspberry Pi 5-ə qoşulur .
-	**Bağlayıcılar:** JST və pin başlıq bağlayıcıları həm güc, həm də siqnal interfeysləri üçün istifadə olunur.

İstehsal və montaj qeydləri
-	Dövrə əl ilə perfboard (prototip lövhəsi) üzərində yığılır .
-	Elektrik xətləri yüksək cərəyan üçün daha qalın naqillərlə lehimlənir, siqnal xətləri isə daha nazik naqillərdən istifadə edir.
-	Prototip olaraq bu dizayn tez modifikasiyalara və sınaqlara imkan verir.
-	Uzunmüddətli və davamlı istifadə üçün, kütləvi istehsalda lehim maskası və səthi bitirmə ilə FR4 əsaslı PCB- yə keçmək tövsiyə olunur.

<h3>Ultrasonik Sensor (HC-SR04)</h3>

<table style="width:80%; border-collapse:collapse;">
  <tr>
    <td rowspan="7" style="width:275px; text-align:center; vertical-align:middle; padding:0; margin:0;">
      <img src="media/image55.jpg" style="width:300px; height:210px; display:block; margin:0 auto; border:0;">
    </td>
    <td>Ölçmə diapazonu</td>
    <td>2 sm – 400 sm</td>
  </tr>
  <tr>
    <td>Ölçmə dəqiqliyi</td>
    <td>±3 mm</td>
  </tr>
  <tr>
    <td>Ölçmə bucağı</td>
    <td>~15°</td>
  </tr>
  <tr>
    <td>İş gərginliyi</td>
    <td>5V DC</td>
  </tr>
  <tr>
    <td>Ölçülər</td>
    <td>45 × 20 × 15 mm</td>
  </tr>
  <tr>
    <td>Çəki</td>
    <td>9 g</td>
  </tr>
  <tr>
    <td colspan="2">Sürətli cavab müddəti (təxminən 10 µs trigger impulsu)</td>
  </tr>
</table>

HC-SR04, maneələrin aşkarlanması və yaxın məsafələrin ölçülməsi üçün istifadə olunan ucuz və geniş yayılmış ultrasonik məsafə sensorudur. O, səs dalğalarının əks olunma prinsipinə əsaslanır və qısa məsafəli obyekt aşkarlanmasında yüksək dəqiqlik təmin edir.

Əlaqələr
-	**VCC (5V):** Raspberry Pi və ya RRC Lite Controller-in 5V xəttinə qoşulur.
-	**GND:** Ümumi torpaq xəttinə qoşulur.
-	**TRIG (Trigger):** Raspberry Pi GPIO pininə çıxış siqnalı üçün qoşulur.
-	**ECHO:** Raspberry Pi GPIO pininə giriş siqnalı üçün qoşulur.

> Qeyd: ECHO pinindən gələn 5V siqnalı Raspberry Pi GPIO (3.3V tolerant) üçün uyğunlaşdırmaq məqsədilə gərginlik bölücü rezistor istifadə olunmalıdır.

Güc Tələbləri
-	**İş gərginliyi:** 5V DC
-	**Tipik cərəyan istehlakı:** 15 mA

<h3>Batareyanın Xüsusiyyətləri (Gərginlik, Tutum, C Qiyməti)</h3>

<table style="width:80%; border-collapse:collapse;">
  <tr>
    <td rowspan="8" style="width:400px; text-align:center; vertical-align:middle; padding:0; margin:0;">
      <img src="media/image5.jpeg" style="width:400px; height:210px; display:block; margin:0 auto; border:0;">
    </td>
    <td>Gərginlik</td>
    <td>11.1 V (nominal)</td>
  </tr>
  <tr>
    <td>Tutum</td>
    <td>1300 mAh</td>
  </tr>
  <tr>
    <td>Hüceyrə sayı</td>
    <td>3S (3 hüceyrə, seriyalı)</td>
  </tr>
  <tr>
    <td>Boşaltma dərəcəsi</td>
    <td>50C davamlı / 100C pik (maksimum 10 s)</td>
  </tr>
  <tr>
    <td>Çəki</td>
    <td>110 g</td>
  </tr>
  <tr>
    <td>Ölçülər</td>
    <td>74 × 33 × 21 mm</td>
  </tr>
  <tr>
    <td>Boşaltma konnektoru</td>
    <td>XT60 (qara)</td>
  </tr>
  <tr>
    <td>Doldurma konnektoru</td>
    <td>JST-XHR (ağ)</td>
  </tr>
</table>

Layihədə istifadə olunan enerji mənbəyi — yüksək cərəyan tələb edən robototexnika, dron və müdafiə sistemləri üçün nəzərdə tutulmuş JetFire 3S Li-Po batareyasıdır. Bu batareya yüksək boşalma dərəcəsi və etibarlı hüceyrə quruluşu ilə sistemin sabit və uzunmüddətli işləməsini təmin edir.

# Proqram mühitinin hazıranması

### Raspberry Pi konfiqurasiyası

> Aşağıdakı əməliyyatları yerinə yetirə bilmək üçün kompüterinizə ən az `16GB` yaddaşı olan microSD kart taxmalısınız. Raspberry Pi əməliyyat sistemi bu karta yüklənəcək.

Biz Raspberry Pi üçün Ubuntu Server 24.04.3 LTS (64-bit) əməliyyat sistemini işlədirik. Bu əməliyyat sistemini mikrroSD kartına istədiyiniz kimi yükləyə bilərsiniz, ancaq məsləhət görülən üsul rəsmi Raspbery Pi saytından [Raspberry PI Imager tool](https://www.raspberrypi.com/software/) proqramını işlətməkdir. 

![Raspberry PI Imager](media/Imager.PNG)

Raspberry Pi modelinizi seçdikdən sonra "Əməliyyat sistemini seç" (Choose OS) düyməsinə basın. Bundan sonra `Other general-purpose OS` > `Ubuntu` > `Ubuntu Server 24.04.3 LTS (64-bit)` əməliyyat sistemini seçdirin.

![Ubuntu Server 24.04.3 LTS (64-bit)](media/OS.PNG)

> Ubuntu Server 24.04.3 LTS (64-bit) Ubuntunun axırıncı server buraxılışıdır. Buna baxmayaraq bir çox Raspberry Pi modelini dəstəkləyir. Sırf bu buraxılışı işlətmək zəruri deyil, ancaq biz Ubuntu Server 24.04.3 LTS (64-bit) işlətdiyimizə görə sizə də bunu məsləhət görürük.

> Bu layihə üçünn Raspberry Pi OS (Raspbian) arzuolunan kimi işləmir. Bunun səbəbi layihədə `ROS2 Jazzy` paketinin işlədirməsidir. Hansı ki Raspberry Pi OS (Raspbian) tərəfindən dəstəklənmir.

İndi isə əməliyyat sistemini konfiqurasiya etmək üçün `Ctrl` `Shift` `X` və ya `⌘` `Shift` `X` düyməsinə sıxmalısınız. Açılan paneldə `General` hissəsinin aşağısında internetinizin adını, şifrəsini; Raspberry Pi üçün istifadəçi adını və şifrəsini konfiqurasiya edin. Bundan sonra `Services` hissəsinin aşağısında SSH və şifrə autentikasiyasına icazə verin. Beləcə Raspberry Pi alətinizə internetlə də qoşula bilərsiniz. Konfiqurasiya bitdikdən sonra `Next` düyməsini sıxın və yükləmə başlayacaq.

Yükləmə bitdikdən sonra mikroSD kartını Raspberry Pi - yə taxın. Raspberry Pi açıldıqdan sonra ona qoşulun və məsləhət görülür ki, bu iki komandanı verin:

```bash
$ sudo apt update
$ sudo apt upgrade
```

### İstifadə edilmiş paketlərin yüklənilməsi

`Git` proqramını yükləyərək başlayın. Bu paketi digər paketləri yükləmək üçün istifadə edəcəyik:

```bash
$ sudo apt-get install git
```

Raspberry Pi kamera modulunu istifadə etmək üçün (biz bu modulun üçüncü versiyasını işlədirik) `libcamera` və `rpicam-apps` paketləri yüklənməlidir.

Asılılıqları yükləyərək başlayın:

```bash
$ sudo apt install -y libboost-dev
$ sudo apt install -y libgnutls28-dev openssl libtiff5-dev pybind11-dev
$ sudo apt install -y meson cmake
$ sudo apt install -y python3-yaml python3-ply
$ sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev
```

İndi isə paket kodlarını yükləmək üçün:

```bash
$ git clone https://github.com/raspberrypi/libcamera.git
```

Əsas qovluğa keçin:

```bash
$ cd libcamera
```

Paket yüklənməsini konfiqurasiya etmək üçün:

```bash
$ meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
```

Son olaraq `libcamera` paketini yükləmək üçün:

```bash
$ sudo ninja -C build install
```

`libcamera` paketi yükləndiyinə görə artıq `rpicam-apps` paketini yükləyə bilərik.

Yenə asılılıqlardan başlayırıq:

```bash
$ sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
$ sudo apt install -y meson ninja-build
```

Raspberry Pi `home/{istifadəçi adı}` qovluğuna keçin və paketin kodunu yükləyin: 

```bash
$ git clone https://github.com/raspberrypi/rpicam-apps.git
```

Paket qovluğuna keçmək üçün:

```bash
$ cd rpicam-apps
```

Yükləməni konfiqurasiya etmədən əvvəl `libav` paketini yükləyin ki kameradan video çəkmək mümkün olsun:

```bash
$ sudo apt install -y libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavdevice-dev libavfilter-dev
```

İndi isə yükləməni konfiqurasiya edin:

```bash
$ meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=disabled -Denable_qt=disabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
```

Artıq `rpicam-apps` paketini yükləyə bilərsiniz:

```bash
$ meson compile -C build
$ sudo meson install -C build
```

`ldconfig cache` - i yeniləyərək yükləməni bitirə bilərsiniz (bu əməliyyatı yüklədiyimiz paketləri başqa qovluqlarda da işlədə bilmək üçün yerinə yetiririk):

```bash
$ sudo ldconfig
```

Lazımlı paketləri yüklədiyinizə görə artıq `ROS2 Jazzy` inteqrasiyasını yükləyə bilərik.

[`ROS2 Jazzy`](https://docs.ros.org/en/jazzy/index.html) robot inteqrasiyası üçün istifadə olunan paketlər topluluğudur. Aşağıdakı instruksiyalar ilə bu paketlər toplusunu Raspberry Pi sisteminizə yükləyə bilərsiniz:

Raspberry Pi dilini konfiqurasiya edin:

```bash
$ locale 
$ sudo apt update && sudo apt install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8
$ locale 
```

Lazımlı proqramları yükləyə bilmək üçün `apt` paket idarəçisinə mənbələri əlavə edin:

```bash
$ sudo apt install software-properties-common
$ sudo add-apt-repository universe
```

İndi isə `ros2-apt-source` paketini əlavə edin:

```bash
$ sudo apt update && sudo apt install curl -y
$ export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
$ curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
$ sudo dpkg -i /tmp/ros2-apt-source.deb
```

Proqramlama mühiti üçün:

```bash
$ sudo apt update && sudo apt install -y \
  python3-flake8-blind-except \
  python3-flake8-class-newline \
  python3-flake8-deprecated \
  python3-mypy \
  python3-pip \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-mock \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-pytest-runner \
  python3-pytest-timeout \
  ros-dev-tools
```

`ROS2 Jazzy` emalatxanası düzəldin və lazım olan `ROS2 Jazzy` pakelərinin kodunu yükləyin:

```bash
$ mkdir -p ~/ros2_jazzy/src
$ cd ~/ros2_jazzy
$ vcs import --input https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos src
```

Bütün paketlərin son versiyada olduğuna əmin olmaq üçün:

```bash
$ sudo apt upgrade
```

Son olaraq hər şeyi yükləyin:

```bash
$ sudo rosdep init
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
```

`ROS2 Jazzy` sistemini yüklədikdən sonra layihə üçün paketləri yükləyə bilərik.

`ROS2 Jazzy` sistemini yükləmək onu işlətmək üçün bəs etmir. Hər işlətmək istənildikdə `/opt/ros/jazzy/setup.bash` komandası daxil edilməlidir. Əgər bunu etmək istəmirsinizsə bu komandanı `~/.bashrc` faylına əlavə edə bilərsiniz:

```bash
$ echo "source /opt/ros/jazzy/setup.bash">>~/.bashrc
```

Bu əməliyyatı icra etdikdən sonra terminalı yenidən başladın.

`camera_ros` paketi Raspberry Pi sisteminin kamerası ilə əlaqə saxlamaq üçün hazırlannmış `ROS2 Jazzy` paketidir. `camera_ros` paketini yükləmək üçün bu əməliyyatları icra edin:

```bash
$ cd ~/ros2_jazzy/src
$ sudo apt -y install python3-colcon-meson
$ git clone https://github.com/christianrauch/camera_ros.git
$ source /opt/ros/$ROS_DISTRO/setup.bash
$ cd ~/ros2_jazzy/
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
$ colcon build --event-handlers=console_direct+
```

`rplidar_ros` isə Rplidar lidar sistemi ilə əlaqəyə keçmək üçündür. Onu yükləmək üçün:

```bash
$ cd ~/ros2_jazzy/src
$ git clone https://github.com/Slamtec/rplidar_ros.git
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
$ colcon build --symlink-install
```

Paketləri yüklədikdən sonra istifadə edə bilmək üçün `~/ros2_jazzy/install/setup.bash` əməliyyatını icra etməlisiniz. Əgər bunu etmək istəmirsinizsə bu əməliyyatı `~/.bashrc` faylına əlavə edə bilərsiniz:

```bash
$ echo "source ~/ros2_jazzy/install/setup.bash">>~/.bashrc
```

### Kodun işlədilməsi və test edilməsi

Kodlarımızı işlətmək üçün kodunu endirin:

```bash
$ git clone https://github.com/coolboy19521/Voltivarius.git
$ cd Voltivarius/src
```

Əgər bütün fayllara baxsanız bu üç proqramı görəcəksiniz:

```bash
$ ls
future_engineers.py color_selector.py vizualiser.py
```

Bu faylların ilki (`future_engineers.py`) bizim raundlar üçün yazdığımız koddur və Raspberry Pi sistemində işləyir.

Bu faylların ikincisi (`color_selector.py`) rəng tanıma dəyərlərinin kalibrasiyası üçündür. Bu fayl isttifadəçinin kompüterində işlədilməlidir.

Axırıncı fayl (`visualizer.py`) isə robotun gördüklərini anlıq olaraq vizualizasiya etmək üçündür. Bu proqramı da istifadəçi öz kompüterində işlətməlidir.

### Explanation of Strategy

Before explaining the strategy it would be smart to explain how we read the data and how we work on it:

* Lidar values are in radians and inverted by default. Also distances are given in meters. That is why before using lidar data we convert angles to degrees, invert it once more and multiply distances by 100 to work with centimeters (e.g. `(-math.degrees(angle), dis * 100)`).
* Lidar angles are based on the rotation (yaw angle) of the robot. Therefore it is a little bit inconsistent for our usage. To fix this we shift lidar angles by gyro error (e.g. `angle -= gyro_error`).
* It is not stable enough to turn our robot by gyro_error itself without any additional work. That is why we added a PID controller. We were adjusting the PID values while we were testing our robot. PID is also used for wall following (e.g. `self.move(self.gyro_pid.pid(gyro_error))`).
* Our sensor reading (including camera) is not happening synchronously. The main reason for that is the unmatching refresh frequencies of our sensors (lidar is 5-10Hz, camera is almost consistently 30Hz and gyro sensor is stacking all previous values). We are running all reading asynchronously and make cruicial decisions in lidar's callback. Because lidar's callback is responsible for obstacle detection.

Now that you get how we are treating the data, we would like to talk a little bit about obstacle detection with lidar as it is very cruicial for our runs:

* To detect an obstacle we first had to find a way to differentiate the obstacle from other stuff. It is not very straightforward. Because at some aspects lidar's readings of outer and inner walls look just like an obstacle. F.e. look at this: TODO.
* To resolve this issue we first tried to limit the vision angle of the robot so it would not see anything else. But this strategy turned out to be not that 'good'. You see, when you give a large angle it is pointless, as robot still sees inner and outer walles. And when you give a small degree now it is not able to see obstacle itself when very close :|
* After this failing strategy we came up with something more reliable (we still use it): TODO.

After all this is our strategy diagram:

![Diagram of Strategy](media/Diagram_Future_Engineers.png)

**P.S. You can contact us for any help needed or for unclear points. But before contacting us please make sure you did not find anything related on the report.**