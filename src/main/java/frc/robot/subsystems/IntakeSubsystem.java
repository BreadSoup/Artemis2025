// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


/* dont remember why they're here might need them later?
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.ErrorCode;
import com.fasterxml.jackson.databind.ser.impl.FailingSerializer;

*/

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;



public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private final VictorSPX intakeMotor;
  private final TalonSRX drawBridge;
  //private final FeedbackDevice drawBridgeEncoder;
  
  PIDController DBPID; // I dont think this does anything in this context as the talon has its own pid
  //private ErrorCode config_kP;????? idk where this came from??? prob will remove later
  //annotating in slang is crazy work bud
  //its peak ezra trust the process

  public IntakeSubsystem() {
    //TODO remove comment hell and cleanup file as whole this is bad
    intakeMotor = new VictorSPX(IntakeConstants.intakeCanID);
    drawBridge = new TalonSRX(IntakeConstants.drawBridgeCanID);
    //drawBridgeEncoder = new CTRE_MagEncoder_Relative(FeedbackDevice.CTRE_MagEncoder_Relative);

    //PIDController pid = new PIDController(IntakeConstatns.DBkD, IntakeConstatns.DBkI, IntakeConstatns.DBkD);
    //pid.enableContinuousInput(0, 1);

    //not sure what slot IDX is but it's 0 in mundell's code
    //from offical docs: 
    /** How long to wait for receipt when setting a param. Many setters take an
    optional timeout that API will wait for. 
    This is benefical for initial setup (before movement), though typically not
    desired when changing parameters concurrently with robot operation (gain scheduling for
    example). from random pdf on the Talon*/
    drawBridge.configFactoryDefault();

    drawBridge.setInverted(IntakeConstants.drawBridgeInverted);
    drawBridge.config_kP(0, IntakeConstants.DBkP);
    drawBridge.config_kI(0, IntakeConstants.DBkI);
    drawBridge.config_kD(0, IntakeConstants.DBkD);
    drawBridge.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    drawBridge.setSelectedSensorPosition(0);
    drawBridge.setSensorPhase(false);

    //TODO find out the minimal requirments for a pid on the talon motor
    //DrawbridgeMotor = new TalonSRX(Constants.DRAWBRIDGE_MOTOR);
    //drawBridge.configFactoryDefault();
    //DrawbridgeMotor.setInverted(Constants.DRAWBRIDGE_INVERTED);
    //DrawbridgeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    //DrawbridgeMotor.setSensorPhase(Constants.DRAWBRIDGE_SENSORPHASE);
    drawBridge.configAllowableClosedloopError(0, IntakeConstants.DRAWBRIDGE_ALLOWABLE_ERROR, 0);
    drawBridge.config_kF(0, 0);
    //DrawbridgeMotor.config_kP(0, Constants.DRAWBRIDGE_kP);
    //DrawbridgeMotor.config_kI(0, Constants.DRAWBRIDGE_kI);
    //DrawbridgeMotor.config_kD(0, Constants.DRAWBRIDGE_kD);
    drawBridge.config_IntegralZone(0, IntakeConstants.DRAWBRIDGE_INTEGRAL_ZONE);
    drawBridge.configMaxIntegralAccumulator(0, IntakeConstants.DRAWBRIDGE_MAXINTEGRAL);
    drawBridge.configClosedLoopPeakOutput(0, 1);
    drawBridge.setSelectedSensorPosition(0);
    drawBridge.set(ControlMode.Position,0);
  }

  public void runIntake(){
    intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.intakeRunSpeed);
    drawBridge.set(ControlMode.Position, IntakeConstants.loweredIntakeValue);
  }
  
  public void purgeIntake(){
    intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.intakePurgeSpeed);
  }

  public void stopIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 0);

    // Example code has this but I feel as they should *not* be the same method for edge cases
    // are these methods???? TODO ask mundell or someone if these are methods or some other wacky name
  }

  public void lowerIntake(){
    drawBridge.set(ControlMode.Position, IntakeConstants.loweredIntakeValue);
    SmartDashboard.putBoolean("Intake/Lower", true);
  }

  public void raiseIntake(){
    drawBridge.set(ControlMode.Position, IntakeConstants.raisedIntakeValue);
  }

  public void fullStopIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
    drawBridge.set(ControlMode.PercentOutput, 0);
  }

  public void printDrawBridgePos(){
    System.out.print(drawBridge.getSelectedSensorPosition());
  }

  public void ResetIntake(){
    drawBridge.set(ControlMode.PercentOutput, -0.15);
  }

  public void TareDrawBridge(){
    drawBridge.setSelectedSensorPosition(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //drawBridge.set(DBPID.calculate((g)))
    SmartDashboard.putNumber("Intake/TargetPos", drawBridge.getSelectedSensorPosition());

  }
}
