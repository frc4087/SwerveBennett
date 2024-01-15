// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.util.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

public class SwerveWheelModuleSubsystem extends SubsystemBase {
  double P = -0.00675;//0.000072;//0.000081;
  double I = 0.00001;//0.000007;
  double D = 0;//0.0000065;
  //hard code in the actual values once we find them off of smartdashboard

  private CANSparkMax angleMotor;
  private CANSparkMax speedMotor;
  private PIDController pidController;
  private CANCoder angleEncoder;
  //private boolean calibrateMode;
  private double encoderOffset;
  private String motorName;
  //private SimpleWidget speedLim;

  public SwerveWheelModuleSubsystem(int angleMotorChannel, int speedMotorChannel, int angleEncoderChannel,
          String motorName, double offset) {
      // We're using TalonFX motors on CAN.
      this.angleMotor = new CANSparkMax(angleMotorChannel,  MotorType.kBrushless);
      this.speedMotor = new CANSparkMax(speedMotorChannel,  MotorType.kBrushless);
      this.angleEncoder = new CANCoder(angleEncoderChannel); // CANCoder Encoder //TODO: CHECK THIS RANGE IS 0-360
      this.speedMotor.setIdleMode(IdleMode.kCoast);
      this.motorName = motorName;
      this.pidController = new PIDController(P, I, D); // This is the PID constant,
      // we're not using any
      // Integral/Derivative control but increasing the P value will make
      // the motors more aggressive to changing to angles.
      
      

      //angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

      // pidController.setTolerance(20); //sets tolerance, shouldn't be needed.

      pidController.enableContinuousInput(0, 360); // This makes the PID controller
      // understand the fact that for
      // our setup, 360 degrees is the same as 0 since the wheel loops.

      SendableRegistry.addChild(this, angleMotor);
      SendableRegistry.addChild(this, speedMotor);
      SendableRegistry.addChild(this, angleEncoder);
      SendableRegistry.addLW(this, "Swerve Wheel Module");
      //create smartDashboard widget for encoder offset

      encoderOffset = offset;
      //UNCOMMENT TO DO AUTO CALIBRATION
      // double startAngle = angleEncoder.getAbsolutePosition();
      // encoderOffset = MathUtil.mod(startAngle - offset, 360);
      // SmartDashboard.putNumber("Encoder Offset " + motorName, encoderOffset);

      //speedLim = Shuffleboard.getTab("Preferences").addPersistent("Speed Lim", 0.5)
      //.withWidget(BuiltInWidgets.kNumberSlider);;
      
      // resetSensor();
  }
  public void setAngle(double angle, double currentEncoderValue)
  {
      SmartDashboard.putNumber("Difference " + motorName, MathUtil.getCyclicalDistance(currentEncoderValue, angle, 360));
      angle = MathUtil.mod(angle, 360);

      
      
      double pidOut = -pidController.calculate(currentEncoderValue, angle);
    
      
      angleMotor.set(pidOut);
      SmartDashboard.putNumber("PID Out " + motorName, pidOut);

      
  }

  public void setSpeed(double speed)
  {
      speedMotor.set(speed/5.5);
  }

  
  public void drive(double speed, double angle) {
      double currentEncoderValue = getPosition();
      
      // SmartDashboard.putNumber("Distance " + motorName, getDistance());
      SmartDashboard.putNumber("Rotation " + motorName, getPositionRad());
      SmartDashboard.putNumber("angle reading", angle);
      SmartDashboard.putNumber("Cyclical Distance", MathUtil.getCyclicalDistance(currentEncoderValue, angle, 360));
   

      if(MathUtil.getCyclicalDistance(currentEncoderValue, angle, 360) > 90){
        setSpeed(-speed);
        setAngle(angle+180, currentEncoderValue);
      }else{
        setSpeed(speed);
        setAngle(angle, currentEncoderValue);
      }
    
  }




 
  // this method outputs position of the encoder to the smartDashBoard, useful for
  // calibrating the encoder offsets
  public double getPosition() {
      return MathUtil.mod(angleEncoder.getAbsolutePosition() - encoderOffset, 360); 
  }

  public double getPositionRad() {
      return MathUtil.mod(angleEncoder.getAbsolutePosition() - encoderOffset, 360) * Math.PI / 180;
  }

  // public double getDistance() {
  //     if (motorName.equals("BR") || motorName.equals("FR")) {
  //         return -(speedMotor.getSelectedSensorPosition() * Constants.WHEEL_CIRCUMFERENCE)/(2048 * Constants.L2_RATIO);
  //     }
  //     return (speedMotor.getSelectedSensorPosition() * Constants.WHEEL_CIRCUMFERENCE)/(2048 * Constants.L2_RATIO);
  // }

  public void stop() {
      speedMotor.set(0);
      angleMotor.set(0);
  }

  @Override
  public void periodic() {
      //calibrateMode = calibrateState.getEntry().getBoolean(false);
      P = SmartDashboard.getNumber("P", P);
      I = SmartDashboard.getNumber("I", I);
      D = SmartDashboard.getNumber("D", D);
      pidController.setPID(P, I, D);
    
      SmartDashboard.putNumber("Encoder " + motorName, getPosition());

  }

  public void coast(){
      speedMotor.setIdleMode(IdleMode.kCoast);
  }

  public void brake(){
      speedMotor.setIdleMode(IdleMode.kBrake);
  }

//   public void resetSensor()
//   {
//       speedMotor.getSelectedSensorPosition(0);
//   }

  
//   public SwerveModulePosition getSwerveModulePosition()
//   {
//       return new SwerveModulePosition(getDistance(), new Rotation2d(getPositionRad()));
//   }
}
