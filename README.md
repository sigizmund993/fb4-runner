# fb4-runner

Программа на Raspberry Pi робота FootBot4: принимает команды стратегии по UDP, переводит их в UART-команды для STM32 ([прошивка](https://github.com/SPbUnited/FootBot4_Firmware)), пересылает телеметрию STM32 и мяч с бортовой камеры обратно стратегии.

```bash
make run          # без видеострима
make run-stream   # со стримом rtsp://<RPi>:8554/ball
make test         # юнит-тесты
make flash-fw     # залить на STM32 релиз прошивки из CI (FB4_FW_RELEASE)
```

Процессы (`src/main.py`): приём SSL vision, `command_receiver` (UDP :5555 → UART, 100 Гц), `telemetry_transceiver` (UART → UDP broadcast :5556).

## Настройки (.env)

Настройки робота лежат в `.env` в корне репозитория. Файл не в git, у каждого робота свой; шаблон с описанием переменных — `.env.example`. Если `.env` нет, `make run`, `make run-stream` и `make flash-fw` создают его копией шаблона (а раннер — при первой записи id); существующий файл не перезаписывается, поэтому переменные, добавленные в шаблон позже, в него сами не попадут — до тех пор действуют значения по умолчанию. Переменная, заданная в окружении (`FB4_ROBOT_TEAM=blue make run`), перекрывает файл.

| Переменная | По умолчанию | Что это |
|---|---|---|
| `FB4_ROBOT_ID` | из hostname `fb4-NN` | id робота 0..15, раннер сам переписывает его (см. ниже) |
| `FB4_ROBOT_TEAM` | `yellow` | `yellow` или `blue`, для поиска робота в SSL vision |
| `FB4_UART_PORT` | `/dev/ttyACM0` | порт STM32 |
| `FB4_RICOCHET_CAM_GAIN` | `5.0` | RICOCHET: поперечная скорость [1/с] на метр смещения мяча |
| `FB4_FW_REPO` | `SPbUnited/FootBot4_Firmware` | репозиторий прошивки |
| `FB4_FW_BRANCH` | `feature/uart_for_rpi` | ветка прошивки |
| `FB4_FW_RELEASE` | `latest` | `latest`, номер релиза ветки (`5` или `v0.0.5`) или полный тег |

Читает файл `src/utils/envfile.py` (только stdlib: `flash_firmware.py` работает без venv). Другой файл можно подсунуть через `FB4_ENV_FILE`.

Прошивка: `make flash-fw` берёт `FB4_FW_BRANCH` и `FB4_FW_RELEASE`; разово можно `make flash-fw RELEASE=5`, `make flash-fw BRANCH=main RELEASE=latest`, `ARGS=--dry-run` — только найти и скачать. Если на STM32 уже этот образ, повторно не пишет.

## UART с STM32

Код: `src/protocol/uart.py`. Парная сторона — `src/devices/uart_control` в прошивке; форматы должны совпадать.

Кадр в обе стороны: `[0xA5][payload][XOR от 0xA5 и payload]`.

| Направление | Формат | Поля | Кадр |
|---|---|---|---|
| RPi → STM | `<5eB??B` | vel_x, vel_y, w, dribbler, kicker [В] (fp16), kicker_mode, angle_mode, beep, robot_id | 16 байт |
| STM → RPi | `<Be??` | robot_id, напряжение кикера [В] (fp16), ball_deep, ball_front | 7 байт |

Если формат меняется, прошивку, раннер и teleop надо обновлять вместе: кадры другой длины не распознаются, и робот стоит.

## Напряжение кикера

Везде вольты на конденсаторе:

- команда: `KickerAndDribbler.kicker_voltage` (uint32) — цель заряда 0..330 В, `0` — не заряжать. Раннер ограничивает её `KICKER_MAX_V` (330) и передаёт в UART как есть; прошивка ограничивает ещё раз;
- `OldFormat.kicker_setting` остался уровнем 0..15: раннер переводит 1..15 в 100..330 В (`kicker_setting_to_voltage`);
- телеметрия: `StrategyTelemetry.kicker_voltage` (float) — измеренное напряжение.

Раньше `KickerAndDribbler` нёс уровень 0..15 в поле `kicker_setting`. Номер и тип поля те же, так что старая стратегия не сломается при разборе, но её 0..15 теперь означают вольты (робот почти не заряжается); после перегенерации proto имя поля в её коде надо поменять на `kicker_voltage` и слать вольты.

## beep

`RobotCommand.beep` (поле 11, вне `oneof`) работает с любым типом команды и уходит в STM32 битом `beep`. Писк длится, пока приходят команды с `beep = true`; когда стратегия замолкает (`CMD_TIMEOUT_S`), он прекращается вместе с движением.

## robot_id и hostname

Id робота согласуется с STM32 по UART и хранится в `.env` (`FB4_ROBOT_ID`) и в hostname RPi (`fb4-NN`, в mDNS — `fb4-NN.local`).

- При старте id берётся из `FB4_ROBOT_ID` (окружение, потом `.env`), если он пуст — из hostname `fb4-NN`. Если и hostname другой, id неизвестен: RPi шлёт `0xFF`, STM32 оставляет свой, а RPi сразу берёт id STM32.
- Если hostname не совпадает с id, RPi переименовывается при старте.
- RPi шлёт свой id в каждой команде. STM32 берёт его сразу и сохраняет в EEPROM, кроме случая, когда id на роботе только что поменяли кнопками.
- `telemetry_transceiver` берёт id STM32, если тот отличается от своего дольше `ROBOT_ID_ADOPT_S` (1 с), записывает его в `.env` (иначе после перезапуска старый id из `.env` вернулся бы на STM32) и меняет hostname (`hostnamectl`, `/etc/hosts`, перезапуск avahi; нужен root). Если `FB4_ROBOT_ID` задан в окружении, а не в `.env`, после перезапуска победит он. Задержка даёт STM32 время взять id RPi при подключении и не переименовывает RPi на каждом нажатии кнопки, пока id перебирают.
- Текущий id делят процессы через `multiprocessing.Value` (`-1` — неизвестен). `command_receiver` подхватывает новый id на лету: шлёт его в UART и ищет в SSL vision робота с этим id. Пока id неизвестен, команды с целью (GLOBAL_COORDINATES, RICOCHET) стоят.
- После смены id стратегия и teleop должны обращаться к новому `fb4-NN.local`.

Ограничение: если поменять id кнопками и сохранить с перезагрузкой STM32, пока раннер не запущен, после подключения победит id RPi.

## Протобуфы

`ssl_packet_package` — сабмодуль (ветка `telemetry_for_feedback`), сгенерированный код в git не лежит:

```bash
make -C ssl_packet_package init build
```

- После изменения `.proto` (например, нового поля `beep`) `build` нужно перезапустить и на RPi.
- venv сабмодуля привязан к своему пути: после переноса каталога скрипты в `venv/bin` ссылаются на старый путь, и `build` падает с `protoc-gen-mypy: program not found`. Помогает `rm -rf ssl_packet_package/venv` и снова `make -C ssl_packet_package init`.
- Версии в `requirements.txt` сабмодуля не закреплены. Свежий `mypy-protobuf` требует protobuf новее установленного (`VersionError: ... gencode 6.x runtime 5.x`), тогда: `ssl_packet_package/venv/bin/pip install mypy-protobuf==3.6.0`.
