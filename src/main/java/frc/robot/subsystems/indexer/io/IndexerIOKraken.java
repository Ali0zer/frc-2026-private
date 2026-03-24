package frc.robot.subsystems.indexer.io;

import static frc.robot.subsystems.indexer.IndexerConstants.kIsFOC;
import static frc.robot.subsystems.indexer.IndexerConstants.kMotorCANBus;
import static frc.robot.subsystems.indexer.IndexerConstants.kMotorID;
import static frc.robot.subsystems.indexer.IndexerConstants.kMotorInverted;
import static frc.robot.subsystems.indexer.IndexerConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.indexer.IndexerConstants.kMotorSupplyLimitAmps;

import com.bobcats.lib.subsystem.rollers.io.GenericRollerIOTalonFX;

/**
 * The IO hardware implementation for indexer hardware interacions with the TalonFX.
 */
public class IndexerIOKraken extends GenericRollerIOTalonFX implements IndexerIO {
	/**
	 * Constructs a new IndexerIOKraken.
	 */
	public IndexerIOKraken() {
		super(kMotorID, -1, kMotorInverted, false, kMotorSupplyLimitAmps, kMotorStatorLimitAmps, kMotorCANBus, "", 1.0,
				-1.0, kIsFOC);
	}
}
