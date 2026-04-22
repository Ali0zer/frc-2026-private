package frc.robot.subsystems.superstructure;

import com.bobcats.lib.ascope.rotational.PivotAscopeDisplay;
import com.bobcats.lib.utils.Tracer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;

/**
 * A class to relay changes in moving mechanisms to AdvantageScope for displaying.
 */
public class SuperstructureVisualizer extends SubsystemBase {
	private final PivotAscopeDisplay m_intakeDisplay;

	private static final Pose3d kIntakePivotBaselineOffset = new Pose3d(-0.27, 0.476, 0.18, Rotation3d.kZero);

	/**
	 * Constructs a new SuperstructureVisualizer.
	 *
	 * @param hood   The hood mechanism subsystem.
	 * @param intake The intake mechanism subsystem.
	 */
	public SuperstructureVisualizer(Intake intake) {
		// Intake arm display
		m_intakeDisplay = new PivotAscopeDisplay("Intake", () -> kIntakePivotBaselineOffset, 0,
				() -> new Rotation3d(0, Math.toRadians(intake.getIntakeAngle()), 0));

		setName("SuperstructureVisualizer");
	}

	@Override
	public void periodic() {
		Tracer.start("SuperstructureVisualizerPeriodic");
		// Update && log all displays
		m_intakeDisplay.update();
		Tracer.finish("SuperstructureVisualizerPeriodic");
	}
}
