// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final TalonFX shooterMotor;
  public int targetFlywheelRPM = 0;
  boolean isShooterEnabled;
  double atSpeedTimer;

  public ShooterSubsystem() {
    shooterMotor = new TalonFX(ShooterConstants.shooterCanID);
    reiniTalonFX(shooterMotor);
    setTalonFXPidGains(shooterMotor);
  }

  public void setFlywheelTargetRPM(int rpm){
    targetFlywheelRPM = rpm;
    SmartDashboard.putNumber("Flywheel Target RPM", rpm);
  }

  public double getFlywheelRPM(){
    return talonFXUnitsToRPM(shooterMotor.getVelocity().getValueAsDouble());
  }

  public void setFLywheelDistanceRPM(double ty){
    //I have no idea what this is doing
    //Looks like its for the limelight to guess the speed it needs to go at based on pos?
    //Wont be used until (or if) I get a limelight
    targetFlywheelRPM = (int) Math.round(4560 + -57 * ty + 0.394 * Math.pow(ty, 2) + -0.45 * Math.pow(ty, 3) + 0.0533 * Math.pow(ty, 4) + 0.00456 * Math.pow(ty, 5) + -.00009 * Math.pow(ty, 6));
  }
  
  public void setShooterState(Boolean state){
    isShooterEnabled = state;
  }

  public void setShooter(){
    targetFlywheelRPM = 3500;
    setShooterState(true);
    SmartDashboard.putNumber("Flywheel Target RPM", targetFlywheelRPM);
  }

  public void disableShooter(){
    targetFlywheelRPM = 0;
    setShooterState(false);
    SmartDashboard.putNumber("Flywheel Target RPM", targetFlywheelRPM);
  }

  public static double rpmToTalonFXUnits(double rpm){
    return rpm * 2048 / 600;
  }

  public static double talonFXUnitsToRPM(double talonFXUnit){
    return (talonFXUnit / 2048) * 600;
  }

  public boolean isShooterAtSpeed(){
    // If shooter is above the RPM tolerance then  begin
    if (Math.abs(getFlywheelRPM() - targetFlywheelRPM) < ShooterConstants.shooterRPMTolerance){
      // From referance code:
      // Must be at the target RPM for a certain amount of loops in a row before saying
      // it's safe to fire. 
      //? Not really sure what exactly this means or does
      return RobotController.getFPGATime() > (atSpeedTimer + ShooterConstants.ShooterStableRPMTime);
    } else {
      atSpeedTimer = RobotController.getFPGATime();
      return false;
    }
  }

  TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

  private void reiniTalonFX(TalonFX talonFX){
    //https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/feature-replacements-guide.html?utm_source=chatgpt.com
    //talonFX.configFactoryDefault(); no longer a function???? cant find alternative in docs
    talonFXConfig = new TalonFXConfiguration(); //? should be factory settings by default right?
    talonFX.setNeutralMode(NeutralModeValue.Coast);
    talonFXConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;
    talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.shooterCurrentLimit;
    // SupplyCurrentThreshold has no alternative in phoenix6, ignoring for now
    talonFXConfig.Slot0.kP = ShooterConstants.shooterkP;
    talonFXConfig.Slot0.kI = ShooterConstants.shooterkI;
    talonFXConfig.Slot0.kD = ShooterConstants.shooterkD;
    talonFXConfig.Slot0.kS = ShooterConstants.shooterkS;
    talonFXConfig.MotorOutput.PeakForwardDutyCycle = 1;
    talonFXConfig.MotorOutput.PeakReverseDutyCycle = -1;

    talonFX.getConfigurator().apply(talonFXConfig);
  }

  //not too sure why this would ever be needed as pid values are set when init
  private void setTalonFXPidGains(TalonFX talonFX) {
    //?kF replaced with FeedForward(?)
    //https://docs.wpilib.org/en/latest/docs/software/advanced-controls/controllers/index.html
    talonFXConfig.Slot0.kP = ShooterConstants.shooterkP;
    talonFXConfig.Slot0.kI = ShooterConstants.shooterkI;
    talonFXConfig.Slot0.kD = ShooterConstants.shooterkD;

    talonFX.getConfigurator().apply(talonFXConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isShooterEnabled){
      shooterMotor.set(rpmToTalonFXUnits(targetFlywheelRPM));
      /*while(!isShooterAtSpeed()){
        SmartDashboard.putNumber("Flywheel Actual RPM", shooterMotor.getVelocity().getValueAsDouble());
        //RobotContainer.setControllerRumble(0.5);
        //controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.5);
      }*/
      
      //controller.setRumble(GenericHID.RumbleType.kRightRumble, 1);
      //driverXbox.
      //XboxController.setRumble(GenericHID.RumbleType.kRightRumble, 1);
    }
    else {
      shooterMotor.disable();
    }
    SmartDashboard.putNumber("Flywheel Actual RPM", getFlywheelRPM());
    //controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
  }
}
