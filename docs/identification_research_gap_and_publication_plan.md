# Identification Research Gap And Publication Plan

Tarih: 2026-04-10

Bu notun amacı, mevcut multicopter identification hattında literatürde tam karşılığı olmayan kısmı netleştirmek ve bunu yayınlanabilir bir araştırma planına dönüştürmektir.

## Çıkış Noktası

Bugünkü durumda elimizde:

- literatürle uyumlu bir manevra seti var
- HIL tarafında bu manevraları temiz uçurabiliyoruz
- tek uçuşta tüm ident manevralarını alabiliyoruz
- no-truth modunda bazı parametre ailelerini indirekt çıkarabiliyoruz

Ama şu aileler hâlâ eksik:

- mutlak kütle
- motor time constants
- motor constant
- moment constant
- rotor-level aero katsayıları

Sebep:

- ESC telemetry yok
- thrust stand verisi yok
- loglarda mutlak thrust anchor yok

## Literatürdeki Boşluk

Literatürde çoğu çalışma şu üç yoldan birine gidiyor:

1. thrust stand / bench test
2. simulator truth veya extra sensing
3. transfer function / black-box identification

Bizim asıl hedefimiz farklı:

- sadece standart PX4 onboard logları
- mümkünse ESC telemetry olmadan
- manevra-tabanlı
- herhangi bir multicopter için genellenebilir
- dijital ikiz üretmeye yetecek parametre ailelerini çıkarmaya çalışan bir yöntem

Bu tam haliyle güçlü bir boşluk. Özellikle şu birleşim nadir:

- `ESC telemetry yok`
- `tek-uçuş veya kısa uçuş kampanyası`
- `manevra tabanlı parameter family separation`
- `multicopter-generic estimator`
- `SITL + HITL + real-flight validation chain`

## Olası Bilimsel Katkı

Bu çalışmanın yayınlanabilir katkısı şu eksende kurulabilir:

### Katkı 1: ESC Telemetry'siz İndirekt Multicopter Identification

Ana iddia:

- standart PX4 loglarından, rotor telemetry olmadan, bazı fiziksel parametre aileleri güvenilir biçimde çıkarılabilir

Bunlar:

- collective specific-force gain
- hover command
- eksen bazlı drag
- inertia family
- belki latent actuator delay / effective motor lag

### Katkı 2: Manevra-Family Design

Ana iddia:

- parametre ailelerini ayrıştırmak için minimal ama bilgi içeriği yüksek bir manevra kampanyası tasarlanabilir

Örnek aileler:

- hover / vertical
- roll/pitch/yaw sweep
- x/y/z drag runs
- actuator step

Gerekirse eklenecek yeni manevralar:

- chirp-based collective excitation
- multi-sine roll/pitch excitation
- symmetric ascent-descent mass anchor maneuver
- translational acceleration-deceleration drag maneuver
- reversal-based actuator lag maneuver

### Katkı 3: Digital-Twin-Oriented Validation Pipeline

Ana iddia:

- yöntem sadece parametre estimate etmiyor, aynı zamanda bu estimate'in dijital ikiz kalitesini SITL/HITL/real-flight üçgeninde doğruluyor

Bu çok değerli çünkü çoğu çalışma:

- ya sadece model ident yapıyor
- ya sadece tracking validation yapıyor
- ama kapalı çevrim dijital ikiz tutarlılığını bu kadar sistematik ele almıyor

## Yayınlanabilir Asıl Fikir

En güçlü makale fikri bence şu:

**"ESC telemetry olmadan, standart PX4 loglarından, manevra-tabanlı multicopter digital twin identification"**

Bu başlık altında iki önemli vurgu olur:

1. practical:
   - ekstra donanım gerektirmiyor
   - gerçek saha kullanımına yakın

2. scientific:
   - hangi parametre ailelerinin gerçekten observable olduğunu net ayırıyor
   - eksik kalan aileler için yeni manevralar / yeni indirect anchors öneriyor

## Şu Anki Eksiklere Yönelik Yeni Teknik Geliştirme Fikirleri

### 1. Kütle İçin Yeni Dolaylı Anchor

Sorun:

- normalized thrust var
- mutlak thrust yok
- bu yüzden mass mutlak olarak çözülemiyor

Öneri:

- vertical symmetric excitation manevrası ekle
- yükselme ve alçalma bölgesini birlikte kullan
- thrust command ile world-frame vertical acceleration arasındaki ilişkiyi bias ve hover trim ile birlikte fit et
- aynı anda drag_z ve hover bias ayrıştır

Alternatif:

- known payload-change protocol
- uçuş öncesi ve sonrası bilinen küçük yük değişimi ile mass scaling solve et

İlk çalışan araştırma-adımı:

- `collective_specific_force_gain` zaten mevcut loglardan estimate ediliyor
- buna zayıf bir `thrust_scale` prior anchor eklenince
- `mass = thrust_scale / collective_specific_force_gain` ile mutlak kütle geri kazanılabiliyor

Bu şu anda kodda çalışan ilk research-mode çözüm oldu.

### 2. Motor Lag / Actuator Delay İçin Kapalı-Çevrim Gecikme Fit'i

Sorun:

- ESC rpm yok
- rotor hızları direkt ölçülmüyor

Öneri:

- `motor_step` manevrasını sadece rotor-level değil, body response tabanlı fit et
- giriş: thrust command veya body torque proxy
- çıkış: body rate / vertical acceleration transient
- closed-loop deconvolution ile effective actuator delay tahmin et

Yani ölçmek istediğimiz şey:

- gerçek motor time constant değilse bile
- kontrol açısından eşdeğer actuator lag

Bu zaten pratikte daha değerli olabilir.

Bugünkü ilk implementasyon:

- `effective_actuator_dynamics.collective_up/down`
- mevcut `motor_step` verisinden çıkarılıyor
- daha fazla olay toplamak için yeni `actuator_lag_collective` manevrası paketin sonuna eklendi

### 3. Inertia İçin Daha Güçlü Sweep Tasarımı

Sorun:

- inertia sayısal çıkıyor ama prior ile iyi örtüşmüyor

Öneri:

- tek frekanslı değil, chirp veya multi-sine excitation
- her eksende amplitude scheduling
- cross-axis contamination azaltılmış manevra
- gerekirse offboard reference yerine body-rate shaping

### 4. Observable / Unobservable Parametre Haritası

Bence makalenin güçlü bir katkısı da bu olmalı:

- hangi parametre hangi log alanlarıyla observable
- hangisi yalnız ek anchor ile observable
- hangisi telemetry olmadan güvenilir değil

Bu tek başına çok değerli bir tablo olur.

## Minimum Publishable Roadmap

### Faz 1

- mevcut no-truth estimator'ı stabilize et
- collectve gain + drag + inertia tarafını güvenilir hale getir
- 5 tekrar HIL + 5 tekrar SITL ile varyans tablosu çıkar

### Faz 2

- mass için yeni manevra veya indirect anchor geliştir
- actuator lag için yeni fit geliştir
- telemetry olmadan hangi motor family terimlerinin gerçekten estimate edilebildiğini göster

### Faz 3

- x500 stock SITL üzerinde ground-truth validation
- jMAVSim HIL üzerinde cross-platform validation
- gerçek uçuşta repeatability ve plausibility validation

### Faz 4

- dijital ikiz karşılaştırması:
  - stock model
  - prior model
  - identified model
- trajectory tube / overlap / RMSE / repeatability metrikleri

## Makale İskeleti

1. Giriş
2. Problem tanımı
3. Literatür özeti
4. Manevra-tabanlı identification methodology
5. Observable parameter families
6. Indirect estimation without ESC telemetry
7. SITL/HITL/real-flight validation pipeline
8. Sonuçlar
9. Sınırlamalar
10. Gelecek çalışma

## En Güçlü Tez Cümlesi

Bu çalışmanın ana tezi şu olabilir:

**"Standart PX4 logları ve özel tasarlanmış manevra ailesi kullanılarak, ESC telemetry gerektirmeden multicopter dinamiklerinin kısmi ama dijital ikiz odaklı bir identification'ı yapılabilir; hangi parametre ailelerinin gözlenebilir olduğu ise sistematik olarak ayrıştırılabilir."**

## Pratik Karar

Buradan sonra hedef sadece "literatürde ne var" sorusu olmamalı. Hedef:

- literatürdeki güçlü manevra fikirlerini almak
- eksik kalan parametre aileleri için yeni indirect technique geliştirmek
- bunu repeatability ve digital twin validation ile desteklemek

Yani evet, bu aşamadan sonra yeni metodoloji geliştirmek ve yayın hedefi koymak mantıklı.
