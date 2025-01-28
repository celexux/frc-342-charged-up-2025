// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

import static frc.robot.Constants.LiftConstants.*;

import java.util.List;

public class LiftSystem extends SubsystemBase implements Testable {

  private final SparkMax motorOne;
  private final SparkMax motorTwo;

  private final SparkMaxConfig motorOneConfig;
  private final SparkMaxConfig motorTwoConfig;

  private final SparkClosedLoopController pControllerOne;
  private final SparkClosedLoopController pControllerTwo;

  private final DigitalInput limitUp;
  private final DigitalInput limitDown;
  
  private final DutyCycleEncoder armEncoder;
  private final RelativeEncoder motorOneEncoder;
  private final RelativeEncoder motorTwoEncoder;

  private double posSetpointOne = 0;
  private double posSetpointTwo = 0;
  private double velSetpoint = 0;

  /** Creates a new LiftSystem. */
  public LiftSystem() {
    motorOne = new SparkMax(MOTOR_LEFT, MotorType.kBrushless);
    motorTwo = new SparkMax(MOTOR_RIGHT, MotorType.kBrushless);

    motorOneConfig = new SparkMaxConfig();
    motorTwoConfig = new SparkMaxConfig();

    motorOneConfig
      .smartCurrentLimit(CURRENT_LIMIT);
    motorOneConfig.closedLoop
      .pidf(P_VALUE, 0, D_VALUE, FF_VALUE, ClosedLoopSlot.kSlot1)
      .pidf(0.01275, 0, 0.00625, 1.0, ClosedLoopSlot.kSlot2);
    motorTwoConfig
      .inverted(true)
      .smartCurrentLimit(CURRENT_LIMIT);
      
    motorTwoConfig.closedLoop
      .pidf(P_VALUE, 0, D_VALUE, FF_VALUE, ClosedLoopSlot.kSlot1)
      .pidf(0.01275, 0, 0.00625, 1.0, ClosedLoopSlot.kSlot2);

    armEncoder = new DutyCycleEncoder(ARM_ENCODER_PORT);
    motorOneEncoder = motorOne.getEncoder();
    motorTwoEncoder = motorTwo.getEncoder();
  
    pControllerOne = motorOne.getClosedLoopController();
    pControllerTwo = motorTwo.getClosedLoopController();

    motorOne.configure(motorOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorTwo.configure(motorTwoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Setting default values for PID
    

    //Limit Switches
    limitUp = new DigitalInput(LIMIT_SWITCH_UP);
    limitDown = new DigitalInput(LIMIT_SWITCH_DOWN);
  }

  /**
   * @param xboxController Operator
   * @return Command that uses obtained values from the limit switch to lift the arm within a specified range.
   */
  public Command liftArms(XboxController xboxController){
    
    return runEnd(
      () -> {
        
        double setPoint = -xboxController.getLeftY() * MAX_SPEED; // Percent output 
        System.out.println("set point: " + Double.toString(setPoint)+ " lU: " + limitUp.get() + " lD: " + limitDown.get());
        if(!limitUp.get() && (xboxController.getLeftY() > 0)) { 
          System.out.println("up");
          // When the upward limit switch is triggered and the operator attempts to move upward, it will not move upward.
          pControllerOne.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
          pControllerTwo.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        } else if(!limitDown.get() && (xboxController.getLeftY() < 0)) { 
          System.out.println("down");
          // When the downward limit switch is triggered and the operator attempts to move downward, it will not move downward.
          pControllerOne.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
          pControllerTwo.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        } else {
          System.out.println("moving to position");
          pControllerOne.setReference(setPoint, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
          pControllerTwo.setReference(setPoint, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
          
        }
      },

      () -> {
        System.out.println("Motor was sent to 0");;
        pControllerOne.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        pControllerTwo.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
      }
    );
  }

  /**
   * @param Absolute position to lift to
   * @return Command that uses PID to lift the gripper to the specified position
   */
  public Command liftArmsToPosition(double desiredPosition){
    System.out.println("lift arms to position");
    double clampedPos = MathUtil.clamp(desiredPosition, MAX_POSITION, MIN_POSITION);

    return runEnd(
    //Runs repeatedly until the end
    () -> {
      //If position is within the range of desired position, stop there
      if ((getPosition() < clampedPos + LiftConstants.TOLERANCE) && (getPosition() > clampedPos - LiftConstants.TOLERANCE)){
        pControllerOne.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        pControllerTwo.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
      }
      //If the current position is lower than the desired position, move the arm up
      else if(clampedPos - getPosition() < 0){
        pControllerOne.setReference(LiftConstants.MAX_SPEED, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        pControllerTwo.setReference(LiftConstants.MAX_SPEED,ControlType.kVelocity, ClosedLoopSlot.kSlot1);
      }
      //If higher than desired position, move the arm down
      else {
        pControllerOne.setReference(-LiftConstants.MAX_SPEED, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        pControllerTwo.setReference(-LiftConstants.MAX_SPEED, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
      }
    }, 
    //Runs when command ends
    () -> {
        pControllerOne.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        pControllerTwo.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
      }
    ).until(
      () -> {
        return (getPosition() < clampedPos + LiftConstants.TOLERANCE) && (getPosition() > clampedPos - LiftConstants.TOLERANCE);
      }
    );
  }

  /**
   * @return absolute position from 0 to 1
   */
  public double getPosition(){
    //System.out.print("got position: ");
    double position = armEncoder.get();
    //System.out.println(position);
    return position;
  }

  public double getMotorOnePos(){
    System.out.println("got motor one");
    return motorOneEncoder.getPosition();
  }

  public double getMotorTwoPos(){
    System.out.println("got motor two");
    return motorTwoEncoder.getPosition();
  }

  public void setMotorPIDVelocity(double referenceValue){
    System.out.println("set pid velocity");
    velSetpoint = referenceValue;
    pControllerOne.setReference(referenceValue, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    pControllerTwo.setReference(referenceValue, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  public void setMotorPIDPosition(double referenceValueOne, double referenceValueTwo){
    System.out.println("set pid position");
    posSetpointOne = referenceValueOne;
    posSetpointTwo = referenceValueTwo;

    pControllerOne.setReference(referenceValueOne, ControlType.kPosition,  ClosedLoopSlot.kSlot2);
    pControllerTwo.setReference(referenceValueTwo, ControlType.kPosition,  ClosedLoopSlot.kSlot2);
  }

  /**
   * @param mode If false, set the motor's idle mode to coast. If true, set the idle mode to brake.
   */
  public void setBrakeMode(boolean mode){
    System.out.println("Set brake mode: " + mode);
    SparkMaxConfig config = new SparkMaxConfig();
    // ternary operator: `(condition) ? (value if true) : (value if false)`
    config.idleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
    motorOne.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    motorTwo.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Through-bore encoder position", this::getPosition, null);
  
    builder.addDoubleProperty("Motor one encoder velocity", motorOneEncoder::getVelocity, null);
    builder.addDoubleProperty("Motor two encoder velocity", motorTwoEncoder::getVelocity, null);
    
    builder.addDoubleProperty("Motor one encoder position", motorOneEncoder::getPosition, null);
    builder.addDoubleProperty("Motor two encoder position", motorTwoEncoder::getPosition, null);

    builder.addBooleanProperty("Up Limit Switch", () -> !limitDown.get(), null);
    builder.addBooleanProperty("Down Limit Switch", () -> !limitUp.get(), null);

    builder.addDoubleProperty("Velocity setpoint", () -> velSetpoint, null);
    builder.addDoubleProperty("Left position setpoint", () -> posSetpointOne, null);
    builder.addDoubleProperty("Right position setpoint", () -> posSetpointTwo, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public List<Connection> hardwareConnections(){
    return List.of(
      Connection.fromSparkMax(motorOne),
      Connection.fromSparkMax(motorTwo)
    );
  }

  @Override
  public Command testRoutine() {
    return Commands.sequence();
  }
}
