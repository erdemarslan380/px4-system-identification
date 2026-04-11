# Identification Methodology And Sources

Bu notun amacı, multicopter system identification hattında hangi tekniği kullandığımızı, bu tekniğin hangi manevraya dayandığını, hangi literatürden beslendiğini ve mevcut HIL "identified-only" estimate sonucunun ne durumda olduğunu kalıcı biçimde kaydetmektir.

Tarih: 2026-04-10

## Amaç

Hedef, herhangi bir multicopter için uygulanabilecek, manevra-tabanlı ve mümkün olduğunca onboard telemetry ile çalışan bir identification hattı kurmaktır. Bu aşamada odak nokta:

- HIL'de alınan ident CSV'lerinden estimate çıkarmak
- truth kullanmadan nelerin gerçekten gözlenebilir olduğunu ayırmak
- literatürde karşılığı olan manevraları ve estimate tekniklerini netleştirmek

## Mevcut Kod Yolu

Ana dosyalar:

- `experimental_validation/identification.py`
- `experimental_validation/build_multicopter_candidate_from_logs.py`
- `experimental_validation/compare_with_sdf.py`
- `experimental_validation/compare_identified_to_reference_model.py`

HIL tek-uçuş ident veri seti:

- `hitl_runs/ident_single_session_full_visible_20260410/raw_pull/identification_logs`

Bu not içindeki güncel no-truth estimate çıktısı:

- `hitl_runs/ident_single_session_full_visible_20260410/estimate_no_truth_literature_baseline/identified_parameters.json`
- `hitl_runs/ident_single_session_full_visible_20260410/estimate_no_truth_literature_baseline/jmavsim_prior_comparison.json`

## Manevra-Teknik Eşlemesi

### 1. Hover / Collective Specific Force Fit

Manevralar:

- `hover_thrust`
- `mass_vertical`

Mevcut teknik:

- `thrust_cmd` ile `g + a_z` arasında oransal fit kuruluyor
- böylece `collective_specific_force_gain_mps2_per_cmd` elde ediliyor
- ayrıca `hover_command_abs_mean` hesaplanıyor

Bugünkü yorum:

- Bu yöntem ESC telemetry istemiyor
- Bu yöntem normalized collective komuttan "indirekt thrust authority" çıkarmak için uygun
- Ancak logta `thrust_n` olmadığı için mutlak kütle doğrudan gözlenebilir değil

Kod karşılığı:

- `experimental_validation/identification.py`

Literatür bağı:

- Robert Niemiec ve ark., "Multirotor electric aerial vehicle model identification with flight data with corrections to physics-based models", CEAS Aeronautical Journal, 2022.
  - https://link.springer.com/article/10.1007/s13272-022-00583-5
- Jonas Eschmann ve ark., "Data-Driven System Identification of Quadrotors Subject to Motor Delays", arXiv, 2024.
  - https://arxiv.org/abs/2404.07837

Bu kaynaklardan aldığımız fikir:

- hover ve düşey eksen verisi, collective thrust kanalını ve thrust authority benzeri büyüklükleri ayırmak için değerlidir
- ek harici donanım olmadan da proprioceptive veriden bazı parametre aileleri dolaylı biçimde çıkarılabilir

### 1A. Prior-Anchored Mass Recovery From Collective Gain

Manevralar:

- `hover_thrust`
- `mass_vertical`

Yeni araştırma modu:

- önce `collective_specific_force_gain_mps2_per_cmd` estimate ediliyor
- ardından dışarıdan verilen zayıf bir thrust-scale anchor ile kütle geri kazanılıyor
- ilişki:
  - `specific_force_gain = thrust_scale / mass`
  - dolayısıyla `mass = thrust_scale / specific_force_gain`

Bu teknik neden eklendi:

- mevcut loglarda `thrust_n` yok
- saf no-truth modunda mutlak mass observable değil
- ama collective gain zaten güvenilir şekilde estimate edilebiliyor
- dolayısıyla airframe family seviyesinde bilinen bir thrust-scale prior ile mass geri kazanımı mümkün hale geliyor

Kod karşılığı:

- `experimental_validation/identification.py`
- `experimental_validation/build_multicopter_candidate_from_logs.py`

Komut örneği:

```bash
python3 experimental_validation/build_multicopter_candidate_from_logs.py \
  --ident-root /home/earsub/px4-system-identification/hitl_runs/ident_single_session_full_visible_20260410/raw_pull/identification_logs \
  --disable-truth-autodetect \
  --mass-anchor-thrust-scale 16.0 \
  --mass-anchor-label jmavsim_prior_collective_anchor \
  --prefer-prior-mass-anchor \
  --out-dir /home/earsub/px4-system-identification/hitl_runs/ident_single_session_full_visible_20260410/estimate_mass_anchor_jmavsim_prior
```

Bugünkü sonuç:

- saf no-truth fallback mass: `1.0 kg`
- prior-anchored mass: `0.806833419495683 kg`
- jMAVSim prior reference: `0.8 kg`
- mass error: yaklaşık `%0.85`

Yorum:

- bu yöntem saf no-truth değildir
- ama thrust stand veya ESC telemetry istemeden, zayıf bir airframe-family anchor ile mutlak mass geri kazanımı sağlar
- bu, yayın için ayrı bir katkı başlığı olabilir

### 2. Axis-Isolated Inertia Identification

Manevralar:

- `roll_sweep`
- `pitch_sweep`
- `yaw_sweep`

Mevcut teknik:

- eksen bazlı moment veya moment proxy ile açısal ivme arasında fit kuruluyor
- `p_dot`, `q_dot`, `r_dot` ve ilgili torque proxy kullanılıyor
- eksen bazlı fit zayıf kalırsa ortak diagonal inertia fit devreye girebiliyor

Bugünkü yorum:

- Bu teknik literatürle uyumlu
- roll/pitch/yaw eksenlerini ayrıştırmak doğru yaklaşım
- ancak HIL no-truth modunda inertia sonuçları henüz jMAVSim prior ile yeterince uyuşmuyor

Kod karşılığı:

- `experimental_validation/identification.py`

Literatür bağı:

- Wang ve ark., "Aerodynamic-Parameter Identification and Attitude Control of Quad-Rotor Model with CIFER and Adaptive LADRC", Chinese Journal of Mechanical Engineering, 2021.
  - https://cjme.springeropen.com/articles/10.1186/s10033-020-00524-5
- Christina Ivler ve ark., "System Identification Guidance For Multirotor Aircraft: Dynamic Scaling and Test Techniques", Vertical Flight Society Forum 75, 2019.
  - https://www.sjsu.edu/researchfoundation/docs/VFS_2019_Ivler.pdf
- "System Identification of Heterogeneous Multirotor Unmanned Aerial Vehicle", Drones, 2022.
  - https://www.mdpi.com/2504-446X/6/10/309

Bu kaynaklardan aldığımız fikir:

- eksen-izole sweep/chirp/multi-sine manevraları küçük multicopter'larda bilgi içeriği yüksek veri üretir
- roll, pitch ve yaw dinamiklerini ayrı uyarmak inertia ve kontrol türevi benzeri büyüklüklerin ayrıştırılmasını kolaylaştırır

### 3. Axis-Specific Drag Identification

Manevralar:

- `drag_x`
- `drag_y`
- `drag_z`

Mevcut teknik:

- ilgili eksendeki hız ile aynı eksendeki drag ivmesi arasında fit kuruluyor
- hem `drag.coefficient` hem de `specific_drag` raporlanıyor

Bugünkü yorum:

- Bu teknik no-truth modunda çalışan en faydalı parçalardan biri
- x/y/z eksenlerinde sıfır olmayan tutarlı `specific_drag` sonuçları üretebiliyor

Kod karşılığı:

- `experimental_validation/identification.py`

Literatür bağı:

- Robert Niemiec ve ark., CEAS Aeronautical Journal, 2022.
  - https://link.springer.com/article/10.1007/s13272-022-00583-5

Bu kaynaktan aldığımız fikir:

- hover dışında ileri uçuş / translational uçuş verisi alınmadan drag ailesi güvenilir biçimde ayrılamaz
- fizik-tabanlı model ile flight-data update kombinasyonu drag ve motor dinamiği için kritik

### 4. Motor Step / Actuator Dynamics Probe

Manevra:

- `motor_step`
- `actuator_lag_collective` (yeni research-mode maneuver)

Mevcut teknik:

- motor lag, rotor hız sınırı, motor constant ve benzeri terimler için step cevabı veya rotor-temelli türetilmiş büyüklükler kullanılmaya çalışılıyor

Bugünkü yorum:

- Bu aile şu an no-truth modunda çökmüş durumda
- HIL ident CSV içinde `esc_*_rpm` alanları dolu değil
- `hover_thrust` içinde `thrust_n` yok
- `observed_max_rot_velocity_radps` benzeri alanlar güvenilir dolmuyor
- sonuç olarak motor model parametreleri sıfır / fallback dönüyor

Ancak yeni research-mode ayrımı:

- klasik `motor_model.*` ailesi hâlâ gözlenebilir değil
- buna ek olarak, kapalı çevrim gövde cevabından `effective_actuator_dynamics` ailesi tanımlandı
- bu aile gerçek rotor telemetry istemiyor
- yalnızca `thrust_cmd -> vertical specific-force response` olaylarını kullanıyor

Bugünkü mevcut `motor_step` verisinden çıkan ilk değerler:

- `collective_up.delay_s = 0.0390 s`
- `collective_up.time_constant_s = 0.0138 s`
- `collective_down.delay_s = 0.0086 s`
- `collective_down.time_constant_s = 0.0204 s`

Yorum:

- yukarı yön için kullanılabilir bir ilk estimate oluştu
- aşağı yön örnek sayısı hâlâ zayıf
- bu yüzden paketin sonuna yeni bir `actuator_lag_collective` manevrası eklendi
- amaç, daha sık ve simetrik z-step dizisiyle daha fazla olay toplamak

Kod karşılığı:

- `experimental_validation/identification.py`

Literatür bağı:

- Jonas Eschmann ve ark., arXiv, 2024.
  - https://arxiv.org/abs/2404.07837
- Robert Niemiec ve ark., CEAS Aeronautical Journal, 2022.
  - https://link.springer.com/article/10.1007/s13272-022-00583-5
- M. Elsamanty ve ark., "Methodology for Identifying Quadrotor Parameters, Attitude Estimation and Control", AIM 2013.
  - https://amekhalifa.github.io/files/conference/2013_meth_ident_AIM13.pdf

Bu kaynaklardan aldığımız fikir:

- motor lag ve actuator dinamiği ayrı bir kimliklendirme ailesi olarak ele alınmalı
- bazı çalışmalarda thrust stand kullanılıyor, bazı çalışmalarda proprioceptive veri ile latent parametre fit ediliyor
- bizim hedefimiz thrust stand olmadan, onboard veriye dayalı dolaylı estimate

## Bugünkü HIL "Identified-Only" Estimate Sonucu

Komut:

```bash
python3 experimental_validation/build_multicopter_candidate_from_logs.py \
  --ident-root /home/earsub/px4-system-identification/hitl_runs/ident_single_session_full_visible_20260410/raw_pull/identification_logs \
  --disable-truth-autodetect \
  --out-dir /home/earsub/px4-system-identification/hitl_runs/ident_single_session_full_visible_20260410/estimate_no_truth_literature_baseline
```

Ardından jMAVSim prior kıyası:

```bash
python3 experimental_validation/compare_identified_to_reference_model.py \
  --identified-json /home/earsub/px4-system-identification/hitl_runs/ident_single_session_full_visible_20260410/estimate_no_truth_literature_baseline/identified_parameters.json \
  --reference-model jmavsim_prior \
  --out /home/earsub/px4-system-identification/hitl_runs/ident_single_session_full_visible_20260410/estimate_no_truth_literature_baseline/jmavsim_prior_comparison.json
```

### Bulduğumuz şeyler

İyi veya kısmen kullanılabilir:

- `collective_specific_force_gain_mps2_per_cmd = 19.830611391879273`
- `hover_command_abs_mean = 0.49025726616590465`
- `specific_drag_x = -0.02411762783539779`
- `specific_drag_y = -0.025551360016937617`
- `specific_drag_z = 0.007503364469129463`
- inertia için sayısal estimate oluşuyor:
  - `ixx = 0.010698129971484362`
  - `iyy = 0.0016035266088383008`
  - `izz = 0.0009553681718594747`

### Henüz bulamadığımız şeyler

Doğrudan bulunamadı veya fallback döndü:

- `mass_kg`
- `time_constant_up_s`
- `time_constant_down_s`
- `max_rot_velocity_radps`
- `motor_constant`
- `moment_constant`
- `rotor_drag_coefficient`
- `rolling_moment_coefficient`
- `rotor_velocity_slowdown_sim`

### Yeni Araştırma-Modu İyileşmesi

Yeni prior-anchored mass modunda:

- `mass_kg = 0.806833419495683`
- jMAVSim prior'a göre mass yüzde hatası: `0.854%`

İlgili çıktı:

- `hitl_runs/ident_single_session_full_visible_20260410/estimate_mass_anchor_jmavsim_prior/identified_parameters.json`
- `hitl_runs/ident_single_session_full_visible_20260410/estimate_mass_anchor_jmavsim_prior/jmavsim_prior_comparison.json`

### Yeni Effective Actuator Dynamics Çıktısı

Saf no-truth estimator artık ayrıca şu alanı da üretiyor:

- `effective_actuator_dynamics`

Örnek çıktı:

- `hitl_runs/ident_single_session_full_visible_20260410/estimate_no_truth_literature_baseline_v3/identified_parameters.json`
- `hitl_runs/ident_single_session_full_visible_20260410/estimate_mass_anchor_jmavsim_prior_v2/identified_parameters.json`

Bu alan, motor constant veya rotor time constant ile aynı şey değildir. Bu alan:

- kapalı çevrim collective komutundan
- araç gövdesinin dikey ivme cevabına
- eşdeğer gecikme ve eşdeğer ilk-derece zaman sabiti fit eder

### jMAVSim Prior Kıyas Özeti

Toplam skor:

- `score = 0.0006531831673845691`

Bu şu anlama geliyor:

- no-truth estimate hattı çalışıyor
- ancak tam dijital ikiz çıkaracak seviyede değil
- en güçlü aile şu anda drag + collective gain tarafı
- inertia ailesi sayısal olarak üretiliyor ama prior ile yeterince yakın değil
- motor modeli ailesi mevcut log şemasıyla gözlenebilir değil

## Neden Böyle?

Bugünkü no-truth HIL loglarında:

- `esc_0_rpm ... esc_3_rpm = NaN`
- `hover_thrust` içinde `thrust_n` yok
- motor maksimum hızına bağlanacak güvenilir rotor-truth alanı yok

Bu yüzden:

- mutlak thrust anchor yok
- normalized command var ama mutlak kuvvet yok
- motor parametre ailesi için doğrudan gözlenebilir değişkenler eksik

Sonuç:

- mevcut manevralar literatürle uyumlu
- ama mevcut log alanları, literatürdeki bazı parametre ailelerini tamamlamak için yetersiz

## Bu Notun Pratik Kararı

Şu aşamada pipeline iki seviyeye ayrılmalıdır:

1. `identified-only / no-truth` seviye
   - collective gain
   - hover command
   - specific drag
   - inertia family

2. `truth-assisted` veya ek-anchor destekli seviye
   - mutlak mass
   - motor dynamics
   - rotor-level coefficients

Yani bundan sonraki geliştirme yönü:

- manevra setini daha literatür uyumlu hale getirmek
- ama aynı anda log şemasını da "gerçek uçuşta alınabilir dolaylı anchor" mantığıyla zenginleştirmek

## Sonraki Teknik Adımlar

- sweep manevralarını gerekirse otomatik chirp veya multi-sine varyantlarıyla güçlendirmek
- motor step manevrasını latent motor delay fit için daha uygun hale getirmek
- mutlak thrust yerine kullanılabilecek dolaylı thrust anchor tasarlamak
- hangi parametrenin gerçekten observable olduğunu estimator raporunda açıkça ayırmak
