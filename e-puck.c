/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

typedef struct {
  const char *nom;
  double a, b, c, d;
} Ensemble;

Ensemble loin = {"loin", 0, 0, 30, 100};
Ensemble moyen = {"moyen", 100, 150, 220, 320};
Ensemble proche = {"proche", 230, 400, 5000, 5000};
Ensemble obstacle = {"obstacle", 100, 130, 5000, 5000};

Ensemble droite = {"droite", -90, -90, -60, 0};
Ensemble tout_droit = {"tout droit", -15, 0, 0, 15};
Ensemble gauche = {"gauche", 0, 0, 90, 90};

Ensemble lente = {"lente", 0.4, 0.8, 1.0, 1.4};
Ensemble moyenne = {"moyenne", 0.8, 1.5, 2.0, 2.5};
Ensemble rapide = {"rapide", 1.5, 2.5, 3.0, 3.0};

double fuzzifier(Ensemble e, double mesure) {
  double valeur;
  
  if (mesure < e.a) {
    valeur = 0;
  }
  else if (mesure < e.b) {
    valeur = (mesure - e.a) / (e.b - e.a);
  }
  else if (mesure <= e.c) {
    valeur = 1;
  }
  else if (mesure <= e.d) {
    valeur = (e.d - mesure) / (e.d - e.c);
  }
  else {
    valeur = 0;
  }

  return valeur;
}

double defuzzifier(Ensemble regles[], double poids[], int nb, double debut, double fin, double pas) {
  double haut = 0;
  double bas = 0;
  double x;
  int k;

  for (x = debut; x <= fin; x += pas) {
    double mu = 0;
    
    for (k = 0; k < nb; k++) {
      double m = fuzzifier(regles[k], x);
      
      if (m > poids[k]) {
        m = poids[k];
      }
      // On garde le max pour ce x
      if (m > mu) {
        mu = m;
      }
    }
    haut += x * mu;
    bas += mu;
  }
  // somme(x * mu) / somme(mu)
  return haut / bas;
}

int main(int argc, char *argv[]) {
  /* define variables */
  WbDeviceTag distance_sensor[8], left_motor, right_motor, left_position_sensor, right_position_sensor;
  int i;
  double speed[2];
  double sensors_value[8];
  
  int time_step;
  /* initialize Webots */
  wb_robot_init();

  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    printf("e-puck2 robot\n");
    time_step = 64;
  } else {  // original e-puck
    printf("e-puck robot\n");
    time_step = 256;
  }

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* get a handler to the position sensors and enable them. */
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, time_step);
  wb_position_sensor_enable(right_position_sensor, time_step);

  for (i = 0; i < 8; i++) {
    char device_name[4];

    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i], time_step);
  }
  
  Ensemble regles_angle[5] = {droite, tout_droit, gauche, droite, gauche};
  Ensemble regles_vitesse[4] = {lente, lente, rapide, moyenne};

  /* main loop */
  while (wb_robot_step(time_step) != -1) {
    /* get sensors values */
    for (i = 0; i < 8; i++)
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);

    double cote = sensors_value[2];
    double arriere = sensors_value[3];
    double devant = sensors_value[1];

    double cote_loin = fuzzifier(loin, cote);
    double cote_moyen = fuzzifier(moyen, cote);
    double cote_proche = fuzzifier(proche, cote);
    double arriere_proche = fuzzifier(proche, arriere);
    double obstacle_devant = fuzzifier(obstacle, devant);

    double poids_angle[5] = {cote_loin, cote_moyen, cote_proche, arriere_proche, obstacle_devant};
    double poids_vitesse[4] = {obstacle_devant, cote_proche, cote_moyen, cote_loin};

    double angle = defuzzifier(regles_angle, poids_angle, 5, -90, 90, 1);
    double vitesse = defuzzifier(regles_vitesse, poids_vitesse, 4, 0, 3, 0.05);

    double rapport = angle / 90.0;
    
    speed[0] = vitesse * (1.0 - rapport);
    speed[1] = vitesse * (1.0 + rapport);

    /* printf("cote=%.0f arriere=%.0f devant=%.0f | angle=%.1f vitesse=%.2f\n", cote, arriere, devant, angle, vitesse); */ 

    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }

  wb_robot_cleanup();

  return 0;
}
