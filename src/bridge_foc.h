#pragma once

void bridgeFocSetup();
void bridgeFocLoop();

bool bridgeFocGetButtonHoldContinuous();
void bridgeFocSetButtonHoldContinuous(bool hold);

// Pas pour le mode pas-a-pas (degres par appui), borne a [0.5, 45].
float bridgeFocGetStepDeg();
void  bridgeFocSetStepDeg(float deg);

// Vitesse pour le mode continu (degres par seconde), borne a [5, 500].
float bridgeFocGetHoldDegPerSec();
void  bridgeFocSetHoldDegPerSec(float dps);

// Coupe / réarme un moteur individuellement (utilisé par la protection
// surcourant et les commandes série m1off/m1on/m2off/m2on).
// idx = 1 (M1) ou 2 (M2). Au réarmement le target est resynchronisé sur
// l'angle courant pour éviter un retour brutal vers l'ancienne consigne.
void bridgeFocDisableMotor(int idx);
void bridgeFocEnableMotor(int idx);

// Renvoie true si le moteur est actuellement actif dans la boucle de
// contrôle (pas désactivé par off, m*off, stall ou fault driver).
bool bridgeFocIsMotorEnabled(int idx);

// Callback appelé au début (active=true) et à la fin (active=false) d'une
// calibration. Permet à bridge_main de mettre à jour les LEDs pendant la
// calibration bloquante.
typedef void (*BridgeFocCalCallback)(bool active);
void bridgeFocSetCalCallback(BridgeFocCalCallback cb);
