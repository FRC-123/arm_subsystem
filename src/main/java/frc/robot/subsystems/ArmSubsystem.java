package frc.robot.subsystems;

import static com.revrobotics.CANSparkMax.SoftLimitDirection.kForward;
import static com.revrobotics.CANSparkMax.SoftLimitDirection.kReverse;
import static com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyOpen;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

/**
 * Subsystem for wrist mechanism
 */
public class ArmSubsystem extends SubsystemBase {

  private static final int SMART_MOTION_SLOT = 0;
  private static final double GRAVITY_FF = 0.01;
  private static final double GEARBOX_RATIO = 64.0;
  private static final double CHAIN_RAIO = 1.0;

  // limits in degrees rotation
  private static final float LIMIT_BOTTOM = -50.0f;
  private static final float LIMIT_TOP = 100.0f;

  private final CANSparkMax armMotor;
  private final SparkMaxPIDController pidController;
  private final RelativeEncoder armEncoder;

  private Double targetPosition = null;

  public ArmSubsystem() {
    armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_CANID, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    
    // Get the motor relative encoder
    armEncoder = armMotor.getEncoder();
    armEncoder.setPositionConversionFactor(360/CHAIN_RAIO/GEARBOX_RATIO);
    armEncoder.setVelocityConversionFactor(1);
    armEncoder.setPosition(0);
    pidController = armMotor.getPIDController();
    pidController.setFeedbackDevice(armEncoder);

    // Configure closed-loop control
    double kP = 0.0032075*60;  // .0025; 
    double kI = 0.0;
    double kD = 0.00028705*60; 
    double kIz = 0.1; 
    double kFF = 0;
    double kMaxOutput = .6;
    double kMinOutput = -.6;
    double allowedErr = 0.125; // Error in rotations, not radians

    // Smart Motion Coefficients
    double maxVel = 1500; // rpm
    double maxAcc = 1000;
    double minVel = 0;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput); 

    pidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
    pidController.setSmartMotionMinOutputVelocity(minVel, SMART_MOTION_SLOT);
    pidController.setSmartMotionMaxAccel(maxAcc, SMART_MOTION_SLOT);
    pidController.setSmartMotionAllowedClosedLoopError(allowedErr, SMART_MOTION_SLOT);

    // Voltage compensation and current limits
    armMotor.enableVoltageCompensation(12);
    armMotor.setSmartCurrentLimit(20);
 
    // Configure soft limits
    armMotor.setSoftLimit(kForward, LIMIT_TOP);
    armMotor.setSoftLimit(kReverse, LIMIT_BOTTOM);
    armMotor.enableSoftLimit(kForward, true);
    armMotor.enableSoftLimit(kReverse, true);

    // Disable limit switches, we don't have any
    armMotor.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
    armMotor.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
    
    // Brake mode helps hold arm in place - but not for closed loop
    armMotor.setIdleMode(IdleMode.kCoast);
 
    // Save settings to motor flash, so they persist between power cycles
    armMotor.burnFlash();
    
  }

  @Override
  public void periodic() {
    if (targetPosition != null) {
      // Calculate feed forward based on angle to counteract gravity
      //   double cosineScalar = Math.cos(getArmPosition());
      //   double feedForward = GRAVITY_FF * cosineScalar;
        SmartDashboard.putNumber("Arm Setpoint",targetPosition);
      pidController.setReference(targetPosition, 
          ControlType.kPosition ); // feedForward, ArbFFUnits.kPercentOut);
    }

    // SmartDashboard.putNumber("Arm Setpoint",targetPosition);
    SmartDashboard.putNumber("Arm Position Raw", armEncoder.getPosition());
    SmartDashboard.putNumber("Arm Output", armMotor.getAppliedOutput());
  }

  /**
   * Moves the wrist using duty cycle. There is no motor safety, so calling this will continue to move until another
   * method is called.
   * @param speed duty cycle [-1,1]
   */
  public void moveArm(double speed){
    targetPosition =  null;
    armMotor.set(speed);
  }

  /**
   * Moves the elevator to a position. Zero is horizontal, up is positive. There is no motor safety, so calling this
   * will continue to move to this position, and hold it until another method is called.
   * @param radians position in radians
   */
  public void moveToPosition(double degrees) {
    // Set the target position, but move in execute() so feed forward keeps updating
    targetPosition = degrees;
  }

  /**
   * Gets the arm position
   * @return position in degrees
   */
  public double getArmPosition() {
    return (armEncoder.getPosition());
  }

  /**
   * Stop the arm
   */
  public void stop() {
    // targetPosition = null;
    armMotor.stopMotor();
    targetPosition = armEncoder.getPosition();
  }

}