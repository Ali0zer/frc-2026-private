package frc.robot.subsystems.shooter.feeder.io;

import static frc.robot.subsystems.shooter.feeder.FeederConstants.kIsFOC;
import static frc.robot.subsystems.shooter.feeder.FeederConstants.kMotorCANBus;
import static frc.robot.subsystems.shooter.feeder.FeederConstants.kMotorID;
import static frc.robot.subsystems.shooter.feeder.FeederConstants.kMotorInverted;
import static frc.robot.subsystems.shooter.feeder.FeederConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.shooter.feeder.FeederConstants.kMotorSupplyLimitAmps;

import com.bobcats.lib.subsystem.rollers.io.GenericRollerIOTalonFX;

/**
 * The IO hardware implementation for feeder hardware interacions with the TalonFX.
 */
public class FeederIOKraken extends GenericRollerIOTalonFX implements FeederIO {
	/**
	 * Constructs a new FeederIOKraken.
	 */
	public FeederIOKraken() {
		super(kMotorID, -1, kMotorInverted, false, kMotorSupplyLimitAmps, kMotorStatorLimitAmps, kMotorCANBus, "", 1.0,
				-1.0, kIsFOC);
	}
}
