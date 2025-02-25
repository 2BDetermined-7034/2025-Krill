package frc.robot.subsystems;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;

public class VisionPoseMeasurement {
	private final double timeStamp;
	private final EstimatedRobotPose robotPose;
	private final Vector<N3> stdDevs;

	public VisionPoseMeasurement(double timeStamp, EstimatedRobotPose robotPose, Vector<N3> stdDevs) {
		this.timeStamp = timeStamp;
		this.robotPose = robotPose;
		this.stdDevs = stdDevs;
	}

	public double getTimeStamp() {
		return timeStamp;
	}

	public Pose2d getRobotPose() {
		return robotPose.estimatedPose.toPose2d();
	}

	public Vector<N3> getStdDevs() {
		return stdDevs;
	}

}