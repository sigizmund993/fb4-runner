# CLAUDE.md

Инструкции для Claude Code при работе с этим репозиторием.

## Что это

fb4-runner — программа на Raspberry Pi 5 робота FootBot4 (RoboCup SSL): команды стратегии по UDP → UART на STM32, телеметрия STM32 и мяч с бортовой камеры (Hailo) → стратегии.

Соседние репозитории, которых здесь нет: прошивка `SPbUnited/FootBot4_Firmware`, стратегия, teleop. Протобуфы — сабмодуль `ssl_packet_package`.

Подробная документация — [README.md](README.md): протокол UART, коэффициенты, id робота, алгоритм RICOCHET/CATCH_BALL, калибровка, systemd, чейнджлог. Здесь — то, что нужно, чтобы менять код и ничего не сломать.

## Где что запускается

Раннер работает только на роботе: Python 3.13 (`SharedMemory(track=False)`), Hailo, libcamera, STM32 на UART, пользователь root, репозиторий в `/root/fb4-runner` (путь зашит в `run.sh`).

**Не запускай `src/main.py`, `make run` и `make run-stream` на машине разработчика.** Если hostname не вида `fb4-NN`, `telemetry_transceiver` переименовывает машину в `fb4-00` (`hostnamectl`, `/etc/hosts`, перезапуск avahi). Так уже ноутбук однажды занял имя робота.

`make flash-fw` пишет прошивку в STM32, `make test-uart` двигает робота (он должен стоять на подставке). `make install*` ставит пакеты и драйвер. Всё это — только по явной просьбе.

## Команды

```bash
git submodule update --init                # свежий клон: каталог сабмодуля пустой
make -C ssl_packet_package init build      # сгенерировать protopy; повторять после изменения .proto, и на RPi тоже

make run            # детектор мяча (run.sh) + src/main.py
make run-stream     # то же со стримом rtsp://<RPi>:8554/ball
make test-udp       # печатать входящие команды (раннер остановить: systemctl stop fb4-runner)
make test-uart      # тестовые команды в STM32 (раннер остановить)
make calib-record   # [TEAM=blue] [ID=3], на роботе с запущенным детектором
make calib-fit      # [DATA=calib_data/x.csv] [SAVE=1]
make flash-fw       # [BRANCH=..] [RELEASE=..] [ARGS=--dry-run|--force]
make compile-so     # пересобрать libyolo26_post.so из yolo26_post.cpp (на RPi)
```

- `make run*` ждут venv в `./venv`; цели, создающей его, нет. `requirements.txt` неполон: код импортирует `attr` (пакет `attrs`), а `pyzmq` не используется.
- `make test-*` и `make calib-*` берут `venv/bin/python`, если он есть, иначе системный `python3`.
- Модули импортируются от `src/` (`import config`, `from vision.vision import Field`), сгенерированные протобуфы — от корня (`ssl_packet_package.protopy...`). Отсюда `PYTHONPATH=.:src`.

## Проверка изменений

Юнит-тестов, линтера и CI нет (набор тестов заменён двумя диагностиками на роботе — не добавляй тестовый каркас без просьбы). Без робота можно:

- проверить синтаксис и вызвать чистую логику: `protocol/uart.py` и `utils/envfile.py` — только stdlib; `command_receiver`, `tuning`, `ricochet_log` требуют собранных протобуфов;
- `make flash-fw ARGS=--dry-run` (нужна сеть) — только найти и скачать релиз.

Поведение движения проверяется на роботе и разбирается по `logs/ricochet_*.csv`. Явно говори, что на железе изменение не проверено.

## Архитектура

`src/main.py` запускает три процесса, детектор — четвёртая, отдельная программа:

```
run.sh: libcamerasrc → hailonet (10_hours_brainrot.hef) → libyolo26_post.so ─→ /ball_shm (cx, cy)
Vision_Receiver:        SSL vision multicast 224.5.23.2:10006 → Manager().list()
Command_Sender:         UDP :5555 + vision + /ball_shm → UART, 100 Гц
Telemetry_Transceiver:  UART → UDP broadcast :5556; меняет hostname при смене id
```

| Файл в `src/` | Что внутри |
|---|---|
| `config.py` | все константы, с обоснованием в комментариях |
| `command_receiver.py` | разбор `RobotCommand` → `UartCommand`; вся логика RICOCHET/CATCH_BALL |
| `ball_detector.py` | `/ball_shm` → мяч в системе робота; `BallPath` (прямая по кадрам камеры) |
| `telemetry_transceiver.py` | телеметрия, синхронизация robot_id |
| `tuning.py` | коэффициенты из стратегии (`SET_COEFFICIENTS`), `coefficients.json`, CRC |
| `protocol/uart.py` | кадры UART |
| `vision/` | приём SSL vision (`vision_receiver`), `Field`/`Ball`/`Robot`, альфа-бета фильтры |
| `to_real_coords/` | пиксель → метры (fisheye + гомография), запись и подгонка калибровки |
| `ricochet_log.py` | CSV-лог RICOCHET/CATCH_BALL |
| `utils/auxiliary/aux.py` | геометрия (`Point`, `rotate`, `wind_down_angle`, …) — сначала ищи готовое здесь |

### Процессы и состояние

- Каждый процесс получает **копию** объектов. Общее — только `multiprocessing.Value` (`robot_id`, `coefficients_crc`), списки `Manager` в `VisionClient` и `/ball_shm`. Присваивание атрибута в одном процессе другой не увидит.
- `BallDetector` у `command_receiver` и `telemetry_transceiver` свой, оба считают мяч независимо.
- UART-порт открыт дважды: `command_receiver` только пишет, `telemetry_transceiver` только читает.
- `tuning`, `_last_coming`, `_mode_switch` — глобальное состояние модулей процесса `Command_Sender`.
- Чтение из `Manager().list()` пересылает весь кадр через сокет — не добавляй лишних чтений в цикл 100 Гц.
- `Field()` спит 1 с до первого чтения: к этому времени `Vision_Receiver` должен положить в список первый элемент.

### Поведение команд

- Обычные команды сбрасываются в стоп через `CMD_TIMEOUT_S`; целевые (`TARGET_COMMANDS`: GLOBAL_COORDINATES, RICOCHET, CATCH_BALL) живут до замены. Без позы робота из vision дольше `VISION_TIMEOUT_S` они не едут по vision.
- `SET_COEFFICIENTS` — не команда движения: текущая команда продолжается; каждое сообщение заменяет весь набор.
- `COORDINATE_CONTROL`, `CAP_VEL_AND_ACCEL` и `global_angle` не реализованы.
- Цикл `command_receiver` не должен блокироваться.

## Единицы и системы координат

- Система робота: x вперёд, y **влево**, метры. Калибровка камеры (гомография, `calib_fit`) — y **вправо**; переход только в `calib_to_robot_frame`.
- SSL vision отдаёт миллиметры; `Field` переводит в метры.
- Углы в радианах, скорости м/с и рад/с, напряжение кикера — вольты 0..330 (`OldFormat.kicker_setting` — уровень 0..15).
- Интервалы и возраст данных — по `monotonic()`; фильтры vision считают по `t_capture`. Кадр камеры датируется `monotonic() - CAM_DELAY_S`.
- `Field.update()` скармливает фильтрам только новые `t_capture`: повторы обнуляют скорость и держат потерянный мяч «видимым».

## Менять только вместе

| Что меняется | Где ещё |
|---|---|
| Формат кадра UART (`CMD_FMT`, `TEL_FMT`, `SETTINGS_FIELDS` — порядок важен) | прошивка (`src/devices/uart_control`) и teleop; таблица в README. Кадр другой длины не распознаётся, робот стоит |
| Новый коэффициент | `Coefficients` в `robot/control.proto`; для раннера — поле `Tuning` и дефолт в `config.py`, для STM32 — `SETTINGS_FIELDS` и прошивка; таблица в стратегии |
| `tuning.coefficients_crc` | такая же функция в стратегии |
| `.proto` | сабмодуль, `build` на RPi, стратегия |
| Размер кадра 1280×960, `NO_BALL`, имя `ball_shm`, формат shm | `run.sh`, `yolo26_post.cpp`, `ball_detector.py`, `coord_converter.py`, `config.py`; калибровку переснять |
| Модель `.hef` | имена тензоров и страйды в `yolo26_post.cpp` |
| `COLUMNS` в `ricochet_log.py` | порядок значений в `writerow`; README |
| Переменная `.env` | `.env.example` и таблица в README; `run.sh` читает `.env` своей функцией `setting`, не через `envfile.py` |

## Грабли

- Камера идёт NV12 + `videoconvert`: на RGB-буферах прямо из `libcamerasrc` `hailonet` падает с SIGSEGV. Не «оптимизируй» это обратно.
- `10_hours_brainrot.hef` и `libyolo26_post.so` (aarch64) лежат в git, хотя `.gitignore` их исключает. После правки `.cpp` нужно пересобрать `.so` и закоммитить его.
- `packages/` — офлайн-дистрибутивы Hailo и mediamtx (десятки МБ); не читай и не обновляй без нужды.
- `.env.example` отстал от README: есть неиспользуемый `FB4_ROBOT_ID`, нет `FB4_KICKER_FACE_X`, `FB4_CAM_EXPOSURE_US`, `FB4_CAM_GAIN`.
- Цель `make install-mediamtx` сломана (неверные URL); архив есть в `packages/`.
- `flash_firmware.py` и `envfile.py` должны оставаться на одном stdlib: прошивка запускается системным `python3` без venv.
- Раннер стартует раньше Wi-Fi: сетевые ошибки при старте не должны ронять процессы.

## Соглашения

- Комментарии, докстринги, сообщения в консоль и коммиты — на английском; README — на русском.
- Строки до ~120 символов, type hints, `match` и `X | None` допустимы.
- Настраиваемые числа — в `config.py` с единицами в `[..]` и причиной значения (часто ссылка на лог: «ricochet logs 2026-11-14»).
- Изменение поведения идёт одним коммитом с обновлением README: раздел по теме и запись в «Чейнджлог» под датой.
- Коммиты: `feat:` / `fix:` / `perf:` / `refactor:` / `test:` / `docs:` + описание, что и почему.
- Основная ветка — `master`, но она может сильно отставать от рабочей ветки; перед PR проверь `git log master..HEAD`.
