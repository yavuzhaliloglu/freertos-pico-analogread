#ifndef MUTEX_H
#define MUTEX_H

#include <stdint.h>

/**
 * @brief Proje genelinde kullanılacak tüm mutex'leri oluşturur ve başlatır.
 * * @return uint8_t 1 başarılı, 0 başarısız.
 */
uint8_t setMutexes();

/**
 * @brief VRMS eşik değerini thread-safe (iş parçacığı güvenli) bir şekilde okur.
 * * @return uint16_t VRMS eşik değeri.
 */
uint16_t getVRMSThresholdValue();

/**
 * @brief VRMS eşik değerini thread-safe bir şekilde ayarlar.
 * * @param value Yeni eşik değeri.
 */
void setVRMSThresholdValue(uint16_t value);

/**
 * @brief Eşik değerinin daha önce ayarlanıp ayarlanmadığı bayrağını okur.
 * * @return uint8_t Bayrak durumu (1 veya 0).
 */
uint8_t getThresholdSetBeforeFlag();

/**
 * @brief Eşik değerinin ayarlandığına dair bayrağı günceller.
 * * @param value Yeni bayrak durumu.
 */
void setThresholdSetBeforeFlag(uint8_t value);

#endif // MUTEX_H