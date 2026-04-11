# HITL-SITL Bridge Calibration Trajectory Selection

Tarih: 2026-04-11

## Amaç

Bu notun amacı, residual bridge calibration için hangi trajectory'nin daha uygun olduğunu
öznel sezgiden çıkarıp daha savunulabilir bir zemine oturtmaktır.

Buradaki hedef:

- tek bir trajectory'yi ezberleyen bir model üretmek değil
- calibration için dengeli bir trajectory seçmek
- sonra `hairpin` gibi daha sert bir hold-out trajectory ile genellemeyi test etmek

## Değerlendirme Ölçütleri

Bir calibration trajectory bizim iş için şu özellikleri taşımalı:

1. Sol/sağ yön değişimlerini içermeli
2. Aşırı keskin tekil terminal dönüşe dayanmamalı
3. Hız ve ivme profili yeterince zengin olmalı
4. Çok kısa olmamalı; birkaç saniyelik rasgele transient'e kalibrasyon kilitlenmemeli
5. Sadece tek yönde dönme veya tek eksen uyarımı ile sınırlı kalmamalı
6. Hold-out trajectory'den tamamen farklı olmayan ama onu ezberlemeye de izin vermeyen bir yapıda olmalı

## Kısa Sayısal Referans

Reference trajectory örneklerinden çıkarılan kaba büyüklükler:

- `hairpin`
  - süre: `27.7 s`
  - ort/max hız: `3.10 / 4.51 m/s`
  - XY ivme p95/max: `5.79 / 12.68 m/s^2`
- `lemniscate`
  - süre: `23.18 s`
  - ort/max hız: `2.80 / 4.15 m/s`
  - XY ivme p95/max: `5.53 / 10.21 m/s^2`
- `circle`
  - süre: `18.9 s`
  - ort/max hız: `3.35 / 4.02 m/s`
  - XY ivme p95/max: `6.73 / 11.66 m/s^2`
- `time_optimal_30s`
  - süre: `11.0 s`
  - ort/max hız: `1.81 / 3.72 m/s`
  - XY ivme p95/max: `6.75 / 9.72 m/s^2`
- `minimum_snap_50s`
  - süre: `23.44 s`
  - ort/max hız: `1.10 / 2.11 m/s`
  - XY ivme p95/max: `2.65 / 8.41 m/s^2`

Bu tablo şunu destekliyor:

- `lemniscate`, `hairpin` kadar keskin değil
- `minimum_snap` kadar düşük uyarımlı değil
- `circle` kadar tek-yönlü değil

## Elde Bulunan 5 Validation Trajectory İçin Kısa Teknik Okuma

### `hairpin`

Artıları:

- güçlü yön değişimi var
- hız ve ivme içeriği zengin
- hold-out test olarak çok değerli

Eksileri:

- tek büyük keskin dönüşe fazla ağırlık veriyor
- terminal frenleme ve dönüş bölgesi residual kalibrasyonu kolayca overfit eder
- simetrik olsa da davranışın baskın kısmı tek bir sert manevraya yükleniyor

Karar:

- ana calibration trajectory olarak riskli
- hold-out/generalization testi olarak çok güçlü

### `lemniscate`

Artıları:

- sola/sağa ardışık geçişler içeriyor
- daha dengeli ve tekrar eden bir yapı sunuyor
- `hairpin` kadar sert terminal tekillik taşımıyor
- hız/ivme seviyesi yeterince uyarıcı ama aşırı mission-specific değil
- kapalı çevrim gövde davranışını okumak için daha nötr bir sahne veriyor

Eksileri:

- tek başına thrust authority ve actuator residual'ı ayırmak için yetmeyebilir
- bütün residual farkları sadece bu yörüngeye yüklemek overfit riski taşır

Karar:

- şu anki en mantıklı birincil calibration trajectory adayı
- ama tek başına bırakılmamalı

### `circle`

Artıları:

- sürekli hareket
- düzgün ve temiz bir eğrilik yapısı

Eksileri:

- tek yönlü dönme baskın
- simetrik sol/sağ authority farkını ayırmak için zayıf
- yaw/centripetal residual'ı tek yönde öğretip genelleme zayıflatabilir

Karar:

- ikinci calibration trajectory olabilir
- tek başına birincil calibration trajectory olmamalı

### `time_optimal_30s`

Artıları:

- agresif transient içerik var

Eksileri:

- çok mission-specific
- yüksek eğrilik ve agresif bölgesel davranışlar residual kalibrasyonu kolayca ezbere iter
- calibration için fazla keskin

Karar:

- calibration için uygun değil
- daha çok stres testi

### `minimum_snap_50s`

Artıları:

- pürüzsüz
- uzun süreli

Eksileri:

- çok düşük uyarım seviyelerine kayabiliyor
- residual parametreleri ayrıştırmak için yeterli authority istemeyebilir
- mission/path-specific kalma riski var

Karar:

- calibration için birincil aday değil

## Şu Anki En Sağlıklı Seçim

Bugünkü veri ve yapı üzerinden en savunulabilir seçim:

- birincil calibration trajectory: `lemniscate`
- hold-out trajectory: `hairpin`

Ama bu şu anlama gelmiyor:

- “lemniscate kesin en doğru trajectory”

Daha doğru ifade şu:

- `lemniscate`, eldeki 5 aday içinde residual kalibrasyon için en dengeli başlangıç noktası
- fakat tek başına yeterli olmayabilir

## Neden `lemniscate` Yine De Mantıklı Başlangıç

`hairpin`'e göre avantajları:

- daha az terminal tekillik
- daha fazla sol/sağ geçiş
- daha düşük overfit riski

`circle`'a göre avantajları:

- iki yönlü gövde çalışması
- tek taraflı dönüş ezberi riski daha düşük

`time_optimal` ve `minimum_snap`'e göre avantajları:

- daha az mission-specific
- residual köprüyü öğretmek için daha genel

## Önerilen Güncel Calibration Paketi

Tek trajectory yerine aşağıdaki paket daha güçlü görünüyor:

1. `ident` manevraları
   - kütle
   - inertia
   - drag
   - effective actuator lag

2. `lemniscate`
   - ana bridge calibration trajectory

3. kısa ek residual profili
   - yalnız onboard loglardan türetilebilen
   - thrust authority / collective response / bidirectional acceleration içeren
   - ek telemetry veya thrust stand gerektirmeyen
   - şu an ilk aday: `bridge_probe_xy`

4. hold-out
   - `hairpin`

Bu yapı şu riski azaltır:

- residual parametrelerin tek bir yörünge geometrisine ezberlenmesi

## Sonuç

Bugünkü bilgiyle:

- `lemniscate` kötü seçim değil
- hatta mevcut adaylar içinde en mantıklı birincil calibration trajectory
- ama tek başına “nihai doğru seçim” diye kilitlenmemeli

Doğru metodoloji:

- `ident + lemniscate + kısa residual manevrası` ile kalibrasyon
- `hairpin` ile hold-out validation

Yani yön şudur:

- `lemniscate` korunur
- ama residual kalibrasyon bir kısa ek manevra ile güçlendirilir
- başarı metriği `RMSE` değil, tube overlap / contact / center distance olur
