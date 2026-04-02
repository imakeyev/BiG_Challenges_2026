# Бортовая система для анализа некооперативных космических целей

![Python](https://img.shields.io/badge/Python-3.9%2B-blue?logo=python&logoColor=yellow)
![Libraries](https://img.shields.io/badge/Libraries-OpenCV%20%7C%20Open3D%20%7C%20NumPy-orange)
![Status](https://img.shields.io/badge/Status-In%20Development-brightgreen)

**Это репозиторий нашего проекта, содержащий в себе код проекта и примеры данных.**
**Графики, скриншоты и видео вы можете найти в каталоге data.**

![20260321-0601-20 4771239](https://github.com/user-attachments/assets/53cbcef5-2d26-4a40-8ec2-e107125f46be)



---

## 🚀 Ключевые возможности
*   **Создание симуляции:** Подгрузка 3D-модели в Blender, установка жесткого освещения и движения по траектории

    <img width="1045" height="344" alt="image" src="https://github.com/user-attachments/assets/8b7aea76-364d-464a-a8e9-5fcd5df57556" />

*   **Создание облаков точек:** Загрузка данных с датчиков и создание облаков точек с лидара и камеры

    <img width="407" height="332" alt="image" src="https://github.com/user-attachments/assets/e4dcf17a-0016-4526-bd75-b93c6995eec7" />

*   **Слияние данных:** Объединение плотного облака точек со стереокамеры и точного, разреженного облака с лидара для получения плотного меша из одного фрейма.

    <img width="104" height="230" alt="image" src="https://github.com/user-attachments/assets/10d87175-1c99-4e70-b005-5059b3a0903b" />

*   **4D-реконструкция:** Реконструкция полной 3D-модели некооперативного объекта.

    ![Recording 2026-03-21 111027](https://github.com/user-attachments/assets/d6382952-594f-47ab-bffc-8b15bcc6ec8a)


*   **Отслеживание 6-DoF позы** Определение траектории передвижения и вращения некооперативного объекта и отображение его в предпросмотре.

    ![20260321-0559-44 3828999](https://github.com/user-attachments/assets/9109a373-886d-42f4-8ba0-7690aeeeb9a2)


  

## 🛠️ Технологический стек

*   **Python 3.9+**
*   **OpenCV:** для обработки 2D-изображений,калибровки камер и сшивание моделей.
*   **Open3D:** для всех операций с 3D-данными (облака точек, визуализация, регистрация).
*   **NumPy & SciPy:** для высокопроизводительных вычислений и работы с математикой.
*   **Blender:** в качестве бэкенда для симуляции.

## ⚙️ Установка и запуск

1.  **Клонируйте репозиторий:**
    ```bash
    git clone https://github.com/ваш_логин/ваш_репозиторий.git
    cd ваш_репозиторий
    ```

2.  **Создайте виртуальное окружение и установите зависимости:**
    ```bash
    # Создаем и активируем окружение
    python -m venv venv
    source venv/bin/activate  # для Linux/macOS
    # .\venv\Scripts\activate   # для Windows

    # Устанавливаем библиотеки
    pip install -r requirements.txt
    ```

3.  **Сгенерируйте конфигурацию:**
    ```bash
    # Для режима симуляции (по умолчанию)
    python src/create_config.py --mode sim

    # Для работы с реальными камерами (запустит модуль калибровки)
    python src/create_config.py --mode real
    ```
    
4.  **Запустите основной модуль (пример):**
    ```bash
    python src/main.py --data_path data/set_01
    ```

## 📊 Результаты работы
  Примеры данных находятся в каталоге data

