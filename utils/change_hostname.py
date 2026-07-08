import os
import subprocess

def update_hostname_dynamic(new_name):
    # Очищаем имя от .local (системное имя должно быть без суффикса)
    clean_name = new_name.replace(".local", "").strip()
    
    if os.geteuid() != 0:
        print("Ошибка: Скрипт должен быть запущен через sudo!")
        return

    try:
        # 1. Меняем имя хоста в системе
        subprocess.run(["hostnamectl", "set-hostname", clean_name], check=True)
        
        # 2. Читаем текущий /etc/hosts
        with open("/etc/hosts", "r") as f:
            lines = f.readlines()
        
        # 3. Фильтруем строки, удаляя старую запись маркера и любые дубли нового имени
        marker = "# AUTOMATIC HOSTNAME"
        new_lines = []
        for line in lines:
            # Пропускаем строку с маркером
            if marker in line:
                continue
            # Пропускаем строки, где случайно упоминается это же имя на 127.0.0.1, чтобы избежать дублей
            if "127.0.0.1" in line and clean_name in line.split():
                continue
            new_lines.append(line)
        
        # 4. Добавляем обновленную строку с маркером в самый конец
        # Прописываем и чистое имя, и имя с .local для корректной работы всех утилит
        new_lines.append(f"127.0.0.1\t{clean_name} {clean_name}.local {marker}\n")
        
        # 5. Перезаписываем файл
        with open("/etc/hosts", "w") as f:
            f.writelines(new_lines)
            
        print(f"Имя успешно изменено на: {clean_name} (и обновлено в /etc/hosts)")
        
    except Exception as e:
        print(f"Ошибка при обновлении: {e}")

# Пример вызова
update_hostname_dynamic("fb4-00.local")
