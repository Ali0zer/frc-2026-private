package frc.robot.subsystems.shooter.rollers.io;

import org.littletonrobotics.junction.AutoLog;

/**
 * The main superclass interface for shooter motor (hardware) interaction IO classes.
 */
public interface RollerIO {
	/** The inputs class for the Roller Subsystem. */
	@AutoLog
	public class RollerIOInputs {
		public boolean mainMotorConnected = true;
		public double appliedVoltageMain = 0.0;
		public double rpmMain = 0.0;
		public double positionRevsMain = 0.0;
		public double supplyCurrentAmpsMain = 0.0;
		public double statorCurrentAmpsMain = 0.0;
		public double temperatureCelsiusMain = 0.0;

		public boolean follower1MotorConnected = true;
		public double supplyCurrentAmpsFollower1 = 0.0;
		public double temperatureCelsiusFollower1 = 0.0;

		public boolean follower2MotorConnected = true;
		public double supplyCurrentAmpsFollower2 = 0.0;
		public double temperatureCelsiusFollower2 = 0.0;

		public boolean follower3MotorConnected = true;
		public double supplyCurrentAmpsFollower3 = 0.0;
		public double temperatureCelsiusFollower3 = 0.0;
	}

	/**
	 * Updates the given roller inputs.
	 *
	 * @param inputs The inputs.
	 */
	public default void updateInputs(RollerIOInputs inputs) {}

	/**
	 * Sets the voltage of the motors.
	 *
	 * @param volts The voltage.
	 */
	public default void runVolts(double volts) {}

	/**
	 * Sets the velocity of the rollers.
	 *
	 * @param rpm The target velocity in RPM.
	 */
	public default void runVelocity(double rpm) {}

	/**
	 * Zeroes the motor encoders to read 0.
	 */
	public default void zeroEncoders() {}

	/**
	 * Stops the rollers by setting the voltage across the motors to 0.
	 */
	public default void stop() {
		runVelocity(0);
	}
}
