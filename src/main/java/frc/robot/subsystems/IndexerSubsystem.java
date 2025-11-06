// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IndexerConstants; 

public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new Indexer. */
  private final VictorSPX indexerMotor;
  private final DigitalInput frontSensor;
  private final DigitalInput backSensor;

  public IndexerSubsystem() { //godbless sparkmax changing all their namespaces and making me spend 2 hours troubleshooting
    //Indexer Motor Config
    indexerMotor = new VictorSPX(IndexerConstants.indexerCanID);

    indexerMotor.setInverted(IndexerConstants.indexerInverted);
    indexerMotor.config_kP(0, IndexerConstants.IkP);
    indexerMotor.config_kI(0, IndexerConstants.IkI);
    indexerMotor.config_kD(0, IndexerConstants.IkD);
    indexerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    indexerMotor.setSelectedSensorPosition(0);
    indexerMotor.setSensorPhase(false);
    //Sensors
    frontSensor = new DigitalInput(IndexerConstants.frontSensorChannel);
    backSensor = new DigitalInput(IndexerConstants.backSensorChannel);
  }

  /*Artemis has 2 sensors for the indexer which can hold 2 balls
   * the back sensor checks for when the first ball is loaded
   * the front sensor checks for when the second ball is loaded
  */
  public void runIndexer(){
    //example code has really weird logic for this and needs to be checked
    //I think this this is a simplified version unsure as I cant test ATM

    if (!backSensor.get()){ //if laser is broken stop
      indexerMotor.set(ControlMode.PercentOutput, 0);
    }
    else if(frontSensor.get()){ //if bumpers switches are pressed stop indexer
      //!wouldnt this in theory stop the indexer when it momentarly detects a ball before going to the back?
      indexerMotor.set(ControlMode.PercentOutput, 0);
    }
    else{ //otherwise run
      indexerMotor.set(ControlMode.PercentOutput, IndexerConstants.indexerRunSpeed);
    }
  }

  public void purgeIndexer(){
    indexerMotor.set(ControlMode.PercentOutput, IndexerConstants.indexerPurgeSpeed);
  }

  public void stopIndexer(){
    indexerMotor.set(ControlMode.PercentOutput, 0);
    //?indexerMotor.stopMotor(); could be used instead I think I like how that looks more tbh will have to test if it works same as set(0)
  }

  public void ShootIndexer(){
    indexerMotor.set(ControlMode.PercentOutput, IndexerConstants.indexerShootSpeed);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Indexer/FrontBeam", frontSensor.get());
    SmartDashboard.putBoolean("Indexer/BackBeam", backSensor.get());
    SmartDashboard.putNumber("Indexer/TargetSpeed", indexerMotor.getMotorOutputPercent());
  }
}
