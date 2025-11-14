#include "unity.h"
#include "mock_stm32f4xx_hal_adc.h"
#include "mock_stm32f4xx_hal_tim.h"
#include "mock_stm32f4xx_hal_uart.h"
#include "main.h"  // Inclure le fichier qui contient StartServoControlTas et autres fonctions

// Fonction d'initialisation avant chaque test
void setUp(void) {
    // Init des mocks
    mock_hal_adc_Init_Expect();
    mock_hal_tim_Init_Expect();
    mock_uart_Init_Expect();
}

// Fonction d'initialisation apres chaque test
void tearDown(void) {
    // Rien a faire pour l'instant
}

// Test de la fonction StartServoControlTas
void test_ServoControlTask(void) {
    uint32_t mock_adc_value = 2048;  // Valeur simulee pour l'ADC
    uint8_t expected_angle = 125;    // Valeur attendue apres le mapping de l'ADC

    // Definir les attentes pour les mocks
    HAL_ADC_Start_ExpectAndReturn(HAL_OK);
    HAL_ADC_PollForConversion_ExpectAndReturn(HAL_OK);
    HAL_ADC_GetValue_ExpectAndReturn(mock_adc_value);
    HAL_TIM_PWM_Start_ExpectAndReturn(HAL_OK);
    SetServoAngle_ExpectAndReturn(HAL_OK);

    // Lancer la fonction a tester
    StartServoControlTas(NULL);  // Passer NULL car ce parametre est utilise pour FreeRTOS

    // Verifier que les appels ont ete effectues correctement et dans le bon ordre
    TEST_ASSERT_EQUAL_UINT8(expected_angle, map(mock_adc_value, 0, 4095, 0, 250));
    // On peut ajouter d'autres assertions si necessaire
}

// Fonction main pour Unity
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_ServoControlTask);
    return UNITY_END();
}