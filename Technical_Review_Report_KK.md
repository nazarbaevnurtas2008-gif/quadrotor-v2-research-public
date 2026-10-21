# Quadrotor V2 — соңғы техникалық verification есебі

**Дайындалған күні:** 17 қазан 2026. **Автор:** Нұртас Назарбаев. **Мәртебесі:** сыртқы ғылыми рецензияға ұсынылатын draft; peer-reviewed емес.

## 1. Қайта жүргізу нәтижесі

- Бұрын бекітілген 100 condition, 4 controller, 400 trajectory frozen код пен дәл сол paired-noise арқылы қайталанды. Жаңа random sample алынған жоқ.
- 400/400 сақталған pass/fail шешімі сәйкес келді; 400/400 сандық metrics қайта шығарылды.
- Құжатта бөлек экспортталмаған `max(abs(z)) < 25 m` шарты 400/400 жағдайында расталды.
- Тексерілген метрлік мәндердің ең үлкен абсолюттік айырмасы: **1.72 × 10⁻¹²**. Барлық 400 ішінде ең үлкен `|z|`: **2.4998 м**.

| Контроллер | Max |z|, м | Шектен өткен жағдай |
|---|---:|---:|
| Carefully tuned conventional PID | 2.4612 | 0/100 |
| Performance Auto-PID | 2.4548 | 0/100 |
| Robust Auto-PID | 2.4998 | 0/100 |
| Actuator-aware Integral LQR | 2.2865 | 0/100 |

## 2. Модельдің физикалық сәйкестігі туралы тексеру

- Hover тексеруі: nominal mass **1.50 кг**, `T = mg = 14.715 N`, бір роторға **3.67875 N**, rotor speed шамамен **525.997 rad/s**.
- Rotor command шегі **950 rad/s**, nominal maximum thrust **12 N/rotor**. Алайда `thrust_scale=1.08` өзгерсе, модельдегі нақты achievable thrust теориялық түрде **12.96 N/rotor** бола алады. Сондықтан 12 N — uncertain plant үшін қатаң шек емес, *nominal* шарт.
- Wind горизонталь компоненті +x бағытымен беріледі; жел азимуты кездейсоқ алынбайды. +x бағытына бейімделген сынақ барлық бағыттағы желге орнықтылық дәлелдемейді.
- Физикалық `drag`, `km/kf`, мотор, сенсор noise/delay, thrust scaling параметрлері нақты аппараттан өлшеніп алынбаған, reconstruction assumptions болып қалады.
- Модель 6-DOF қозғалысты есептейді, бірақ тек **altitude + roll** контурлары реттеледі. Позиция x/y, pitch және yaw толық жабық басқару контурында емес.

## 3. Негізгі ғылыми нәтиже өзгерген жоқ

- 100 шарттағы altitude-task pass rate: conventional PID **95%**, Performance Auto-PID **95%**, Robust Auto-PID **93%**, Integral LQR **99%**.
- Үш McNemar paired comparison үшін Holm-adjusted p-value: 1.000, 1.000 және 0.375; **ешқайсысы α=0.05 деңгейінде статистикалық мәнді емес**.
- LQR altitude task бойынша ең көп өткенмен roll IAE, saturation және yaw excursions бойынша маңызды кемшілік көрсетті. Бұл толық flight safety өлшемі емес.

## 4. Нені растадық, нені растамадық

**Расталған:** frozen simulator-дың бірдей input-пен есептерді дәл қайталауы; алгоритм сол нәтиже беруі; бұрын CSV-де болмаған max |z| шарты.

**Расталмаған:** тәуелсіз физикалық модель дәлдігі; нақты ұшуда басқару қауіпсіздігі; hardware-in-the-loop; сыртқы professor review; эксперименттік noise/wind үлестірімдерінің шынайылығы.

## 5. Dr. Ali Nasir-ға жіберу алдындағы міндетті шектеулер

Ғылыми жұмысты “review requested” ретінде ұсынуға болады; “accepted”, “validated by professor”, “peer reviewed”, “real-world robust flight control” деп жазуға болмайды. Профессордан модельдің физикалық жорамалдары, LQR benchmark fairness, actuator behavior, controller comparison және толық ұшу шеңбері туралы пікір сұрау керек.

`FROZEN_FINAL_PROTOCOL.json` өзгертілген жоқ. Pre-execution seed erratum ескі күйінде сақталды. Жаңа review нұсқасында тек деректі тексеру мен шектеулер түсіндірмесі қосылды.

### Replay нұсқаулығы

```bash
python independent_trajectory_replay.py
```

Бұл скрипт бастапқы frozen dataset пен source-ты `/mnt/data/quadrotor_v2_final_analysis` орналасуынан оқиды. Архивтен шығарып басқа машинада орындау үшін `ROOT` айнымалысын бастапқы зерттеу пакетінің орнына бағыттау қажет.
