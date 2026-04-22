package frc.robot.subsystems.shooter.rollers;

import edu.wpi.first.math.system.plant.DCMotor;

// TODO

/**
 * The constants required for the roller subsystem.
 */
public class RollerConstants {
	// Motor IDs
	public static final int kMainMotorID = 53;
	public static final int kFollowerMotor1ID = 54;
	public static final int kFollowerMotor2ID = 55;
	public static final int kFollowerMotor3ID = 56;
	public static final String kMotorCANBus = ""; // Main RIO bus

	// Control constants (PID-SVA)
	// Units: Volts, RPM
	public static final double kP = 0.001, kD = 0, kS = 0.0, kV = 0.0034339;
	public static final double kPSim = 1e-6 / 60, kDSim = 0, kSSim = 0.0, kVSim = 0.0033151;
	public static final boolean kIsFOC = false;

	public static final double kMaxTemperature = 80;

	public static final double kSysIdVoltageRampRate = 0.6;
	public static final double kSysIdVoltageStep = 3;
	public static final double kSysIdTimeout = 5;

	// Soft-max, not the hardware limit RPM
	public static final double kMaxAllowedRPM = 4500;

	// Gearbox reduction
	public static final double kGearboxReduction = 20.0 / 12.0; // A 1:1 ratio

	// Current limits for the motors
	public static final double kMotorSupplyLimitAmps = 50.0;
	public static final double kMotorStatorLimitAmps = 90.0;

	// Inversions
	public static final boolean kMotorInverted = true;
	public static final boolean kFollower1OpposesMain = true;
	public static final boolean kFollower2OpposesMain = false;
	public static final boolean kFollower3OpposesMain = false;

	// Simulation data
	public static final DCMotor kGearbox = DCMotor.getKrakenX60(4);
	public static final double kMomentOfInertia = 0.000367211;
}
