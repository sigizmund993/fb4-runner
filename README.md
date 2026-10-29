# fb4-runner

Программа на Raspberry Pi робота FootBot4: принимает команды стратегии по UDP, переводит их в UART-команды для STM32 ([прошивка](https://github.com/SPbUnited/FootBot4_Firmware)), пересылает телеметрию STM32 и мяч с бортовой камеры обратно стратегии.

```bash
make run          # без видеострима
make run-stream   # со стримом rtsp://<RPi>:8554/ball
make test         # юнит-тесты
make flash-fw     # залить на STM32 последний релиз прошивки из CI
```

Процессы (`src/main.py`): приём SSL vision, `command_receiver` (UDP :5555 → UART, 100 Гц), `telemetry_transceiver` (UART → UDP broadcast :5556).

## UART с STM32

Код: `src/protocol/uart.py`. Парная сторона — `src/devices/uart_control` в прошивке; форматы должны совпадать.

Кадр в обе стороны: `[0xA5][payload][XOR от 0xA5 и payload]`.

| Направление | Формат | Поля | Кадр |
|---|---|---|---|
| RPi → STM | `<5eB??B` | vel_x, vel_y, w, dribbler, kicker (fp16), kicker_mode, angle_mode, beep, robot_id | 16 байт |
| STM → RPi | `<Be??` | robot_id, напряжение кикера (fp16), ball_deep, ball_front | 7 байт |

Если формат меняется, прошивку, раннер и teleop надо обновлять вместе: кадры другой длины не распознаются, и робот стоит.

## beep

`RobotCommand.beep` (поле 11, вне `oneof`) работает с любым типом команды и уходит в STM32 битом `beep`. Писк длится, пока приходят команды с `beep = true`; когда стратегия замолкает (`CMD_TIMEOUT_S`), он прекращается вместе с движением.

## robot_id и hostname

Id робота согласуется с STM32 по UART и хранится в hostname RPi (`fb4-NN`, в mDNS — `fb4-NN.local`).

- При старте id берётся из `FB4_ROBOT_ID`, иначе из hostname `fb4-NN`. Если hostname другой, id неизвестен: RPi шлёт `0xFF`, STM32 оставляет свой, а RPi сразу берёт id STM32.
- Если задан `FB4_ROBOT_ID` и hostname с ним не совпадает, RPi переименовывается при старте.
- RPi шлёт свой id в каждой команде. STM32 берёт его сразу и сохраняет в EEPROM, кроме случая, когда id на роботе только что поменяли кнопками.
- `telemetry_transceiver` берёт id STM32, если тот отличается от своего дольше `ROBOT_ID_ADOPT_S` (1 с), и меняет hostname (`hostnamectl`, `/etc/hosts`, перезапуск avahi; нужен root). Задержка даёт STM32 время взять id RPi при подключении и не переименовывает RPi на каждом нажатии кнопки, пока id перебирают.
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
