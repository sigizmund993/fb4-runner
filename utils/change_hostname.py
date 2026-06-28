import ctypes

def change_hostname(new_name):
    try:
        encoded_name = new_name.encode('utf-8')
        libc = ctypes.CDLL(None)
        result = libc.sethostname(encoded_name, len(encoded_name))
        if result == 0:
            print(f"Имя хоста успешно изменено на: {new_name}")
        else:
            print("Ошибка при смене имени хоста.")
    except PermissionError:
        print("Ошибка: Запустите скрипт через sudo!")