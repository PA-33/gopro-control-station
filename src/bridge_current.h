#pragma once

// === API en deux temps ====================================================
// 1) bridgeCurrentInit()        : ADC2 + calibration usine (lectures synchrones
//    bridgeCurrentReadM*Raw() utilisables immédiatement). À appeler AVANT le
//    Wi-Fi pour pouvoir faire la calibration des butées en lectures fiables.
// 2) bridgeCurrentStartMonitor(): démarre la tâche FreeRTOS périodique de
//    protection surcourant. À appeler après le Wi-Fi.
void bridgeCurrentInit();
void bridgeCurrentStartMonitor();

// Lecture synchrone d'une sonde, avec retry anti-Wi-Fi.
// Renvoie le raw ADC (0..4095) ou -1 si la lecture a échoué.
int  bridgeCurrentReadM1Raw();
int  bridgeCurrentReadM2Raw();

// Réarme la protection surcourant pour le moteur idx (1 ou 2).
// À appeler après une intervention manuelle (m1on / m2on).
void bridgeCurrentClearStall(int idx);

// Active / désactive la détection de stall (les lectures continuent).
// À mettre à false pendant la calibration des butées, où le courant spike
// volontairement à chaque end-stop : sans ça, la tâche de fond couperait
// le moteur en plein milieu de la calibration.
void bridgeCurrentSetStallEnabled(bool en);
