import cv2
import numpy as np

def create_bad_pixel_mask(gray_image, glare_thresh=250, shadow_thresh=10):
    """Создает маску для пикселей, которые слишком яркие (блики) или слишком темные (тени)."""
    
    # Маска для бликов: все, что > glare_thresh, становится 0 (плохой), остальное - 255 (хороший).
    _, glare_mask = cv2.threshold(gray_image, glare_thresh, 255, cv2.THRESH_BINARY_INV)
    
    # Маска для теней: все, что < shadow_thresh, становится 0 (плохой), остальное - 255 (хороший).
    _, shadow_mask = cv2.threshold(gray_image, shadow_thresh, 255, cv2.THRESH_BINARY)
    
    # Объединяем две маски. Пиксель хороший, только если он не блик И не тень.
    final_mask = cv2.bitwise_and(glare_mask, shadow_mask)
    
    return final_mask

def preprocess_stereo_pair(left_img, right_img):
    """
    Основная функция предобработки.
    Применяет маскирование к стереопаре.
    """
    left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
    
    # Создаем маски для каждого изображения
    left_mask = create_bad_pixel_mask(left_gray)
    right_mask = create_bad_pixel_mask(right_gray)
    
    # Итоговая маска - это общая "хорошая" область для обоих кадров.
    # Это гарантирует, что мы не ищем соответствие в области, которая на другом кадре засвечена.
    combined_mask = cv2.bitwise_and(left_mask, right_mask)
    
    # Применяем маску (закрашиваем "плохие" пиксели черным)
    left_masked = cv2.bitwise_and(left_gray, left_gray, mask=combined_mask)
    right_masked = cv2.bitwise_and(right_gray, right_gray, mask=combined_mask)
    
    return left_masked, right_masked
