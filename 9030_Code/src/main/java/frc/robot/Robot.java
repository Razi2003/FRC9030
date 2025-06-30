/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class Robot extends TimedRobot {
  
  private XboxController RController;
  private Servo handServo = new Servo(1);

  private DigitalInput limitSwitch_handUP;
  private DigitalInput limitSwitch_handDOWN;
  private DigitalInput limitSwitch_cranUP;
  private DigitalInput limitSwitch_cranDOWN;


  private VictorSPX TankLeftMotor1 = new VictorSPX(1);
  private VictorSPX TankLeftMotor2 = new VictorSPX(4);
  private VictorSPX TankRightMotor1 = new VictorSPX(2);
  private VictorSPX TankRightMotor2 = new VictorSPX(3);

  private SparkMax cranMotor;
  private SparkMax handMotor;
  private SparkMax ballCollectorMotor1;
  private SparkMax ballCollectorMotor2;
  private SparkMax tubeCollectorMotor;

  double position = 0.01;

  @Override
  public void robotInit() {

    cranMotor = new SparkMax(5, MotorType.kBrushed);   // spark motor
    handMotor = new SparkMax(9, MotorType.kBrushless);  
    ballCollectorMotor1 = new SparkMax(8, MotorType.kBrushless);
    ballCollectorMotor2 = new SparkMax(15, MotorType.kBrushless);
    tubeCollectorMotor = new SparkMax(3, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
  
    //m_leftMotor.restoreFactoryDefaults()

    RController = new XboxController(0);

    limitSwitch_handUP = new DigitalInput(1);
    limitSwitch_handDOWN = new DigitalInput(0);
    limitSwitch_cranUP = new DigitalInput(2);
    limitSwitch_cranDOWN = new DigitalInput(3);

    handServo.set(position);
    //m_rightStick = new Joystick(1);
  }

  public void remoteDrive() {
    double speed = 0;
    // Getting joystick values for drive
    if (RController.getRightTriggerAxis() > 0.03){
      speed = -1 * RController.getRightTriggerAxis();
    }
    else if (RController.getLeftTriggerAxis() > 0.03){
      speed = RController.getLeftTriggerAxis();
    }
    else {
      speed = 0;
    }
    // double speed2 = RController.getLeftTriggerAxis();
    double rotation = -RController.getLeftX() / 3;


    // Drive calculations
    double leftSpeed = speed + rotation;
    double rightSpeed = speed - rotation;

    TankLeftMotor1.set(ControlMode.PercentOutput, -leftSpeed);
    TankLeftMotor2.set(ControlMode.PercentOutput, -leftSpeed);
    TankRightMotor1.set(ControlMode.PercentOutput, rightSpeed);
    TankRightMotor2.set(ControlMode.PercentOutput, rightSpeed);

  }

  public void cranDrive() { 
    // y -1 olanda saga firlanir motor arxadan baxanda
    double cranRemote = RController.getPOV();
    if(!limitSwitch_cranUP.get() && !limitSwitch_cranDOWN.get()) {

      if (cranRemote == -1){
        cranMotor.stopMotor();
      }
      else if (cranRemote == 180){  // dala
        cranMotor.set(0.50);
      }
      else if (cranRemote == 0){
        cranMotor.set(-0.50);
      }
    }

    else if (!limitSwitch_cranUP.get() && limitSwitch_cranDOWN.get()){

      if (cranRemote == -1) {
        cranMotor.stopMotor();   }
      else if (cranRemote == 180){
        cranMotor.stopMotor();  }
      else if (cranRemote == 0) {
        cranMotor.set(-0.50);  }
      }

      else if (limitSwitch_cranUP.get() && !limitSwitch_cranDOWN.get()){

        if (cranRemote == 0) {
          cranMotor.stopMotor();   }
        else if (cranRemote == -1) {
          cranMotor.stopMotor();   }
        else if (cranRemote == 180) {
          cranMotor.set(0.50);   }
      }
  
  }

  public void handDrive(double handSpeed) {
    // double stableSpeed = 0;
    // hand speed positive does down
    if(!limitSwitch_handUP.get() && !limitSwitch_handDOWN.get()) {

      if (RController.getRightX() < 0.15 && RController.getRightX() > -0.15){
        handMotor.stopMotor(); 
        // handMotor.set(stableSpeed); 
      }
      else if (RController.getRightX() > 0.15){
        handMotor.set(-1 * handSpeed);  }
      else if (RController.getRightX() < -0.15){
        handMotor.set(handSpeed);  }
    }

    else if (!limitSwitch_handUP.get() && limitSwitch_handDOWN.get()){

      if (RController.getRightX() > 0.15) {
        handMotor.stopMotor();  
        // handMotor.set(stableSpeed); 
      }
      else if (RController.getRightX() < 0.15 && RController.getRightX() > -0.15){
        handMotor.stopMotor();
        // handMotor.set(stableSpeed); 
      }
      else if (RController.getRightX() < -0.15) {
        handMotor.set(handSpeed);  }
      }

      else if (limitSwitch_handUP.get() && !limitSwitch_handDOWN.get()){

        if (RController.getRightX() < -0.15) {
          handMotor.stopMotor(); 
          // handMotor.set(stableSpeed); 
        }
        else if (RController.getRightX() < 0.15 && RController.getRightX() > -0.15) {
          handMotor.stopMotor();
          // handMotor.set(stableSpeed); 
        }
        else if (RController.getRightX() > 0.15) {
          handMotor.set(-1 * handSpeed);   }
      }

      
    // Controlling the servo hand
    System.out.println(position);
    if(RController.getYButton()){   // servo up
      if(position <= 1.1 && position > 0.01){
        position = position - 0.01;
        handServo.set(position);
      }
    }
    if(RController.getXButton()){  // servo down
      if(position >= 0 && position < 0.98){
        position = position + 0.01;
        handServo.set(position);
      }    
    }
  }
  
  public void neoConrol() {

    double handRemote = RController.getPOV();

    if(RController.getAButton()){
      ballCollectorMotor1.set(0.6);
      ballCollectorMotor2.set(-0.6);
    }
    else if (RController.getBButton()){
      ballCollectorMotor1.set(-0.6);
      ballCollectorMotor2.set(0.6);
    }
    else{
      ballCollectorMotor1.stopMotor();
      ballCollectorMotor2.stopMotor();
    }

    if(handRemote == 90){
      tubeCollectorMotor.set(0.5);
    }
    else if (handRemote == 270) {
      tubeCollectorMotor.set(-0.5);
    }
    else if (handRemote == -1) {
      tubeCollectorMotor.stopMotor();
    }
  }

  @Override
  public void teleopPeriodic() {
    //Timer.delay(0.5);
    // boolean switch1_ispressed = limitSwitch_hand1.get(); //default true
    
    // Joystick values
    // double bodyControl = RController.getRightY();   // body control
    
    // System.out.println(switch1_ispressed); 
    remoteDrive();
    cranDrive();
    handDrive(0.35);
    neoConrol();

    // This part must change before deploy !!!!
    
  }




void goUp(String movementPartName){
  // positive speed yuxari qaldirir hand
  // positive asagi salir crane
  if (movementPartName == "hand"){
    if (!limitSwitch_handDOWN.get() && limitSwitch_handUP.get()){
      handMotor.stopMotor();
    }
    else {
      handMotor.set(0.35);
    }
  }
  if (movementPartName == "cran") {
    if (!limitSwitch_cranDOWN.get() && limitSwitch_cranUP.get()){
      cranMotor.stopMotor();
    }
    else {
      cranMotor.set(-0.5);
    }
  }
}

void goDown(String movementPartName) {
  // negative speed asagi salir hand
  // negative yuxari qaldirir crane
  if (movementPartName == "hand"){
    if (limitSwitch_handDOWN.get() && !limitSwitch_handUP.get()){
      handMotor.stopMotor();
    }
    else {
      handMotor.set(-0.35);
    }
  }
  if (movementPartName == "cran") {
    if (limitSwitch_cranDOWN.get() && !limitSwitch_cranUP.get()){
      cranMotor.stopMotor();
    }
    else {
      cranMotor.set(0.5);
    }
  }
}

@Override
public void autonomousInit(){

 // making all default state
 handServo.set(0.01);

}

@Override
public void autonomousPeriodic() {

}

@Override
public void testInit(){
  Timer.delay(1);
  System.out.println("HAND UP Pin: 1");
  System.out.println("HAND DOWN Pin: 0");
  System.out.println("LIFT UP Pin: 2");
  System.out.println("LIFT DOWN Pin: 3");
  int cnt = 0;
  while (!limitSwitch_handUP.get()){
    if (cnt == 1){
      System.out.println("Please press HAND UP Limit Switch..");
    }
    cnt = cnt + 1;  
  }
  System.out.println("OK");
  cnt = 0;
  while (!limitSwitch_handDOWN.get()) {
    if (cnt == 1){
      System.out.println("Please press HAND DOWN Limit Switch..");
    }
    cnt = cnt + 1;
  }
  System.out.println("OK");
  cnt = 0;
  while (!limitSwitch_cranUP.get()){
    if (cnt == 1){
      System.out.println("Please press LIFT UP Limit Switch..");
    }
    cnt = cnt + 1;
  }
  System.out.println("OK");
  cnt = 0;
  while (!limitSwitch_cranDOWN.get()){
    if(cnt == 1){
      System.out.println("Please press LIFT DOWN Limit Switch..");
    }
    cnt = cnt + 1;
  }
  
}


@Override
public void testPeriodic() {
  
  // Task 1: Testing Cran control and cran motors
  // Task 2: Writing Servo control with limitSwitches
  // Task 3: Converting joystick to logitech


  System.out.println(limitSwitch_cranDOWN.get());

  if(RController.getAButton()){
    ballCollectorMotor1.set(0.6);
    ballCollectorMotor2.set(-0.6);
  }
  else{
    ballCollectorMotor1.stopMotor();
    ballCollectorMotor2.stopMotor();
  }

}
}
