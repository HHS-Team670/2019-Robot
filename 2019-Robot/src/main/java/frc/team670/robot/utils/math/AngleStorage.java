package frc.team670.robot.utils.math;


public class AngleStorage {
		private double angle;

		public AngleStorage(double angle) {
			setAngle(angle);
		}

		public void setAngle(double angle) {
			this.angle = angle;
		}

		public double getAngle() {
			return angle;
		}
}
