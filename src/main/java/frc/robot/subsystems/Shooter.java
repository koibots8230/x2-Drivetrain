package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


@Logged
public class Shooter extends SubsystemBase {
  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;
  private SparkClosedLoopController motorController;
  private Voltage voltage;
  private AngularVelocity velocity;
  private Current current;
  private AngularVelocity setpoint;

  public Shooter() {
    motor = new SparkFlex(ShooterConstants.MOTOR_ID, MotorType.kBrushless);
    
    motorConfig = new SparkFlexConfig();
    motorConfig.closedLoop.p(ShooterConstants.KP);
    motorConfig.closedLoop.feedForward.kV(ShooterConstants.KV);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit((int) ShooterConstants.CURRENT_LIMIT_AMPS);
    motorConfig.inverted(true);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    motorController = motor.getClosedLoopController();

    voltage = Volts.of(0);
    velocity = RPM.of(0);
    current = Amps.of(0);
    setpoint = RPM.of(0);
  }

  @Override
  public void periodic() {
    voltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    velocity = RPM.of(motor.getEncoder().getVelocity());
    current = Amps.of(motor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter/VelocityRPM", velocity.in(RPM));
    SmartDashboard.putNumber("Shooter/SetpointRPM", setpoint.in(RPM));
  }

  @Override
  public void simulationPeriodic() {
    velocity = setpoint;
  }

  private void shoot(AngularVelocity velocity) {
    motorController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    setpoint = velocity;
  }

  public boolean atSpeed() {
    return velocity.in(RPM) > setpoint.in(RPM) - 20.0;
  }

  public void setupLiveTuning() {
    SmartDashboard.putNumber("KP", ShooterConstants.KP);
    SmartDashboard.putNumber("KV", ShooterConstants.KV);
  }

  public void updateLiveTuning() {
    motorConfig.closedLoop.p(SmartDashboard.getNumber("Shooter/KP", ShooterConstants.KP));
    motorConfig.closedLoop.feedForward.kV(SmartDashboard.getNumber("Shooter/KV", ShooterConstants.KV));
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorController = motor.getClosedLoopController();
  }

  public Command setVelocityCommand(AngularVelocity velocity) {
    return Commands.runOnce(() -> shoot(velocity), this);
  }
}