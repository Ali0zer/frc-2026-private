package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The constants required for the indexer subsystem.
 */
public class IndexerConstants {
	// Motor IDs
	public static final int kMotorID = 29;
	public static final String kMotorCANBus = "";

	// Control parameters
	public static final double kForwardVoltage = 4.5;
	public static final double kReverseVoltage = -3.5;

	// Gearbox reduction
	public static final double kGearboxReduction = 18.0/12.0.; // TODO

	// Current limit for the motor
	public static final double kMotorSupplyLimitAmps = 30;
	public static final double kMotorStatorLimitAmps = 45;

	public static final boolean kIsFOC = false;

	// Inversions
	public static final boolean kMotorInverted = true;

	// Simulation data
	public static final DCMotor kGearbox = DCMotor.getNEO(1);
	public static final double kMomentOfInertia = 0.000175;
}
