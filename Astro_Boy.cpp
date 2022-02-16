
#include "mbed.h"
#include <math.h>

//Moteur 1
PwmOut E1(D4);     //P2_2
DigitalOut M1(D3); //P0_5

//Moteur 2
PwmOut E2(D5);     //P2_3
DigitalOut M2(D6); //P2_4

Serial pc(USBTX, USBRX); //pour lire les donnees des capteurs sur un terminal PC

// capteur infrarouge regle pour détecter un obstacle a 15 cm -> low = 0 = OBSTACLE / high = 1 = TOUT VA BIEN
DigitalIn Infra1(D7); //P0_0
DigitalIn InfrC5(D9); //P0_1

// Reseau de capteurs de reflectance
// capteur 4  P0_4
// capteur C2 P0_6
// capteur 11 P0_7
// capteur C4 P0_8
// capteur C5 P0_9
// capteur 5  P0_23
// Distance C4-C2 : 2.3cm
// Distance 5-11 : 0.8cm
// Distance 5-4 : 1.5cm
// Distance 11-4 : 0.7cm
// Distance C4-C5 : 0.5cm


// Ligne blanche : largeur de 15mm

// Arrivee : tasseau de bois de 20 cm de long (barre finale) et situe perpendiculairement
// a la fin de la piste. Cette barre est posee à ses extremites sur deux parallelepipedes de
// bois qui la maintiennent a 4 cm du sol

// Indicateur de priorite a droite : portion de piste perpendiculaire a la piste normale, placee
// du cote droit de la piste a environ 15 cm avant l�intersection. La longueur de l'indicateur de
// priorite a droite est d'environ de 5 cm

Timer timer;
float coeffVitesseCalibre = 0.145/0.245;

void tournerRoueD(float vitesse){
    float vitesseCalibre = coeffVitesseCalibre*vitesse;
    if (vitesseCalibre > 1)
        vitesseCalibre = 1;
    E1.write(vitesseCalibre);
}

void tournerRoueG(float vitesse){
    E2.write(vitesse);
}

void allerToutDroit(float vitesse){
    tournerRoueD(vitesse);
    tournerRoueG(vitesse);
}

int main(){

/*
	tout à droite (à côté de VIN) du réseau de capteurs de reflectance (en position vers le sol)

		C1 = P0.4  = capteur 4
		C2 = P0.6  = capteur C2
		C3 = P0.7  = capteur 11
		C4 = P0.8  = capteur C4
		C5 = P0.9  = capteur C5
		C6 = P0.23 = capteur 5

	tout à droite
*/
	//valeurs de debut et de fin pour les lectures de timer pour les capteurs de reflectance
	int debut;
	int flagC1, flagC2, flagC3, flagC4, flagC5, flagC6;
	int C1,C2,C3,C4,C5,C6;

	LPC_PINCON->PINMODE0 = 0xAC500; //met la pin P0.4 en open drain
	LPC_PINCON->PINMODE1=0x8000;    //met la pin P0.23 en open drain

	timer.start();

	E1.period(0.01);
	E2.period(0.01);
	E1.pulsewidth(0.001);
	E2.pulsewidth(0.001);
	M1.write(0); // roue tourne "vers l'avant"
	M2.write(0);
	allerToutDroit(0.25); // vitesse maximale

	while(1){

		LPC_GPIO0->FIODIR |= (1<<4) | (1<<6) | (1<<7) | (1<<8) | (1<<9) | (1<<23); // met P0.4, P0.6, P0.7, P0.8, P0.9, P0.23 en sorties
		LPC_GPIO0->FIOPIN |= (1<<4) | (1<<6) | (1<<7) | (1<<8) | (1<<9) | (1<<23); // met P0.4, P0.6, P0.7, P0.8, P0.9, P0.23 a 1
		wait_us(10);
		LPC_GPIO0->FIODIR &= ~(1<<4) | ~(1<<6) | ~(1<<7) | ~(1<<8) | ~(1<<9) | ~(1<<23); // met P0.4, P0.6, P0.7, P0.8, P0.9, P0.23 0
		LPC_GPIO0->FIOPIN &= ~(1<<4) | ~(1<<6) | ~(1<<7) | ~(1<<8) | ~(1<<9) | ~(1<<23); // met P0.4, P0.6, P0.7, P0.8, P0.9, P0.23 en entrées

		debut = timer.read_us();
		while((timer.read_us() - debut) < 2000){
			if((LPC_GPIO0->FIOPIN & (1<<4)) == 0){
				if (flagC1){
					C1 = timer.read_us() - debut;
					//On lit les valeurs des capteurs pour ensuite coder les limites de detection
					//la ligne blanche versus le sol gris / noir
					flagC1 = 0;
				}
			}
			if((LPC_GPIO0->FIOPIN & (1<<6)) == 0){
				if (flagC2){
					C2 = timer.read_us() - debut;
					flagC2 = 0;
				}
			}

			if((LPC_GPIO0->FIOPIN & (1<<7)) == 0){
				if (flagC3){
					C3 = timer.read_us() - debut;
					flagC3 = 0;
				}
			}
			if((LPC_GPIO0->FIOPIN & (1<<8)) == 0){
				if (flagC4){
					C4 = timer.read_us() - debut;
					flagC4 = 0;
				}
			}
			if((LPC_GPIO0->FIOPIN & (1<<9)) == 0){
				if (flagC5){
					C5 = timer.read_us() - debut;
					flagC5 = 0;
				}
			}
			if((LPC_GPIO0->FIOPIN & (1<<23)) == 0){
				if (flagC6){
					C6 = timer.read_us() - debut;
					flagC6 = 0;
				}
			}
		}
		pc.printf("Time capteur 4  = %d \n\r", C1);
		pc.printf("Time capteur C2 = %d \n\r", C2);
		pc.printf("Time capteur 11 = %d \n\r", C3);
		pc.printf("Time capteur C4 = %d \n\r", C4);
		pc.printf("Time capteur C5 = %d \n\r", C5);
		pc.printf("Time capteur 5  = %d \n\r", C6);

		//wait inutile en réalité si on en fait déjà un en fin de boucle
		wait_ms(10);

		// Detection d'un obstable declenche arret total du robot
		if(Infra1 == 0 || InfrC5 == 0) {
			tournerRoueD(0);
			tournerRoueG(0);
		}

		// Sinon on suit la ligne
		else {
			//allerToutDroit(0.245);
			// 2 capteurs du mileu sur ligne blanche
			if ((C3 < 500) && (C4 < 500)){
				allerToutDroit(0.245);
			}
			// 2 capteurs des extrêmités sur sol noir
			else if ((C1 > 500) && (C6> 500)){
				allerToutDroit(0.245);
			}
			// presque dépassement de la ligne à Droite
			else if ((C1 < 700) && (C2 < 700)){
				// virage à droite
				tournerRoueD(0.16);
				tournerRoueG(0.3);
			}
			// presque dépassement de la ligne à Gauche
			else if ((C6< 700) && (C5 < 1000)){
				// virage à gauche
				tournerRoueD(0.17);
				tournerRoueG(0.1);
			}

			// dépassement de la ligne à Droite
			else if (C1 < 500){
				// "brusque" virage à droite
				tournerRoueD(0.17);
				tournerRoueG(0.4);
				}
			// dépassement de la ligne à Gauche
			else if (C6< 500){
				// "brusque" virage à gauche
				tournerRoueD(0.42);
				tournerRoueG(0.15);
			}
			else if (C5 < 500){
				tournerRoueD(0.17);
				tournerRoueG(0.15);
			}
			else if(C2<500){
				tournerRoueD(0.17);
				tournerRoueG(0.35);
			}
			else{
				allerToutDroit(0.245);
			}

			wait(0.1);

		}
	}
}
			//10% = T10 = 350 ms
			//20% = T20 = 90 ms
			//30% = T30 = 50 ms
			//40% = T40 = 43 ms
			//50% = T50 = 39 ms1u
			//60% = T60 = 36 ms
			//70% = T70 = 34 ms
			//80% = T80 = 33 ms
			//90% = T90 = 33 ms
			//100% = T100 = 32 ms
			//1 - rampe

