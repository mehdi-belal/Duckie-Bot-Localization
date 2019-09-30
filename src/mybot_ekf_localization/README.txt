################################################

    ##   ESAME: INTELIGENT MOBILE ROBOT   ##

Progetto di esame
- Studente: Belal Mehdi
- Coordinatore del Lavoro: Prof. Paolo Valigi


IMPLEMENTAZIONE DI UN PACKAGE ROS PER LA LOCALIZZAZIONE DI UN ROBOT 
DIFFERENZIALE TRAMITE ALGORITMO EKF, BASATA SU SENSORE LASER. 
################################################
################################################


------------------------------------------------



CONTENUTO DEL PACKAGE:

- lauch
 \
  - mapping.lauch - lauch ROS per la creazione della mappa
  - localization.lauch: launch ROS per caricare l'ambiente di localizzazione


- maps
 \
  - mymap.pgm - formato pgm della mappa nota
  - mymap.yaml - formato yaml della mappa nota


- src
 \
  - main.cpp - classe principale per esecuzione
  - ekf.cpp - classe con codice del nodo di localizzazione
  - ekf.hpp - header
  - conditional_pdf
   \
    - nonlinearanalyticconditionalgaussianmobile.cpp - classe che rappresenta modello condizionale per la 
                                                       rappresentazione non lineare del motion model


- include
 \
  - conditional_pdf
   \
    - nonlinearanalyticconditionalgaussianmobile.h



- rviz
 \
  - amcl1.rviz - configurazione per rviz durante la localizzazione
  - amcl2.rviz - configurazione per rviz durante il mapping


- CMakeLists.txt


- package.xml
