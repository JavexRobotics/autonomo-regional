/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       andres                                                    */
/*    Created:      16/10/2026, 19:26:50                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

const double WHEEL_DIAMETER = 4.0;
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
const double TRACK_WIDTH = 15.5;
const double RELATIVE_DISTANCE_ERROR = 0.4445;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;
controller Controller1;


// define your global instances of motors and other devices here

// Motores del lado izquierdo (puertos 1-4)
motor MotorL1(PORT1, false); 
motor MotorL2(PORT2, true);
motor MotorL3(PORT9, false);
motor MotorL4(PORT10, true);
motor_group Left(MotorL1, MotorL2, MotorL3, MotorL4);

// Motores del lado derecho (puertos 7-10)
motor MotorR1(PORT11, true);
motor MotorR2(PORT12, false);
motor MotorR3(PORT19, true);
motor MotorR4(PORT20, false);
motor_group Right(MotorR1, MotorR2, MotorR3, MotorR4);

motor Elevacion1(PORT5, true);
motor Elevacion2(PORT4, false);
motor Elevacion3(PORT17, false);
motor Elevacion4(PORT15, true);
motor_group Elevacion(Elevacion1, Elevacion2, Elevacion3, Elevacion4);

motor GarraGiro(PORT16, true);
motor GarraAbertura(PORT18, true);

// // Create a Distance Sensor instance in Port 1
// distance distanceSensor = distance(PORT3);
// vision::signature Vision1__REDBOX = vision::signature(
//     1,      // signature id
//     10121,  // uMin
//     10757,  // uMax
//     10439,  // uMean
//     -1657,  // vMin
//     -1223,  // vMax
//     -1440,  // vMean
//     2.5,    // range
//     1 );    // type


// // Create the Vision Sensor instance
// vision Vision1 = vision(
//     PORT1,                  // Smart Port
//     50,                     // brightness
//     Vision1__REDBOX
// );     // Color Code

inertial inercial(PORT14);

bool estadoGarra = true; // true = abierta, false = cerrada

void abrirOCerrarGarra() {
    if (estadoGarra) {
        GarraAbertura.spin(directionType::fwd, 100, velocityUnits::pct);
        task::sleep(500);
        GarraAbertura.stop();
    } else {
        GarraAbertura.spin(directionType::rev, 100, velocityUnits::pct);
        task::sleep(500);
        GarraAbertura.stop();
    }
    estadoGarra = !estadoGarra; // Cambiar el estado de la garra
}

void resetEncoders() {
    MotorL1.setPosition(0, rotationUnits::deg);
    MotorL2.setPosition(0, rotationUnits::deg);
    MotorL3.setPosition(0, rotationUnits::deg);
    MotorL4.setPosition(0, rotationUnits::deg);
    MotorR1.setPosition(0, rotationUnits::deg);
    MotorR2.setPosition(0, rotationUnits::deg);
    MotorR3.setPosition(0, rotationUnits::deg);
    MotorR4.setPosition(0, rotationUnits::deg);
}

void setLeftMotors(double speed) {
    MotorL1.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorL2.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorL3.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorL4.spin(directionType::fwd, speed, velocityUnits::pct);

}

void setRightMotors(double speed) {
    MotorR1.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorR2.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorR3.spin(directionType::fwd, speed, velocityUnits::pct);
    MotorR4.spin(directionType::fwd, speed, velocityUnits::pct);
}

void stopAllMotors() {
    MotorL1.stop();
    MotorL2.stop();
    MotorL3.stop();
    MotorL4.stop();
    MotorR1.stop();
    MotorR2.stop();
    MotorR3.stop();
    MotorR4.stop();

    //Piezas actualmente no declaradas que son del antiguo robot, se comentan para evitar errores de compilación
    //Recolector.stop();
    //Rampa.stop();
    //Garra.stop();
}

void moveDistance(double distanceInInches, double speed) {
    resetEncoders();

    //calcular la distancia en pulgadas tomando en cuenta un margen de error 
    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    //esto calcula cuantas rotaciones de las ruedas se necesitan para recorrer la distancia deseada
    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360; 
    
    setLeftMotors(speed);
    setRightMotors(speed);
    //TODO: en este caso se usa el primer motor para medir la rotacion realizada pero tenemos que cambiarlo por el motor mas estable
    while (fabs(MotorL1.position(rotationUnits::deg)) < targetRotations &&
           fabs(MotorR1.position(rotationUnits::deg)) < targetRotations) {
        task::sleep(10);
    }
    stopAllMotors();
}

void rotateOnAxisN(double angleInDegrees, double speed) {
    resetEncoders();

    double targetRotations = (angleInDegrees / 360) * (TRACK_WIDTH * M_PI / WHEEL_DIAMETER) * 360;

    setLeftMotors((angleInDegrees > 0) ? speed : -speed);
    setRightMotors((angleInDegrees > 0) ? -speed : speed);
    
    // Piezas actualmente no declaradas que son del antiguo robot, se comentan para evitar errores de compilación
    // brazo.open();

    while ( (fabs(MotorL1.position(rotationUnits::deg)) < targetRotations) && (fabs(MotorR1.position(rotationUnits::deg)) < targetRotations) ) {
        task::sleep(10);
    }
    stopAllMotors();
    
}

double calcularMargenGiroDerecha(double current){
    //90 grados hizo aprox 360
    double margenError = 360;
    return ((current * current) / margenError) /2;


}

double calcularMargenGiroIzquierda(double current){
    //90 grados hizo aprox 360
    double margenError = 360;
    return (((current * current) / margenError) /2) + 3;


}

void girarDerecha(int grados){
    resetEncoders();
    while(inercial.rotation(rotationUnits::deg) < grados - 43){
        setLeftMotors(50);
        setRightMotors(-50);
        
    }
    stopAllMotors();
}

void girarIzquierda(int grados){
    resetEncoders();
    while(inercial.rotation(rotationUnits::deg) > -(grados - 43)){
        setLeftMotors(-50);
        setRightMotors(50);
        
    }
    stopAllMotors();
}
int main() {

    inercial.calibrate();

    while (inercial.isCalibrating()) {
    wait(20, msec);
    }
    inercial.resetHeading();
    inercial.resetRotation();

    girarIzquierda(360*2);

    // moveDistance(24, 70);
    // task::sleep(1000);

    // rotateOnAxisN(calcularMargenGiroDerecha(90), 70);
    // //rotateOnAxisN(1,-70);
    //     task::sleep(1000);

    // moveDistance(12, 70);
    //     task::sleep(1000);

    // rotateOnAxisN(calcularMargenGiroIzquierda(90), -70);
    // //rotateOnAxisN(1,70);


    while (true){
        task::sleep(20);
       

        
    }

}
