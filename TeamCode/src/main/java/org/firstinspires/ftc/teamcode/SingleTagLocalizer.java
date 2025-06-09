package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Simple AprilTag Detection", group = "Test")
public class SingleTagLocalizer extends LinearOpMode {

    private Limelight3A limelight;
    static class Pose2d {
        private final double x, y;
        public Pose2d(double x, double y) { this.x = x; this.y = y; }
        public double getX() { return x; }
        public double getY() { return y; }
    }

    // Map of AprilTag IDs to their known field positions (in inches or your chosen unit)
    private Map<Integer, Pose2d> aprilTagFieldPositions = new HashMap<>();

    // Robot's estimated global position
    private double robotX = 0;
    private double robotY = 0;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Initialize known AprilTag positions on the field (fill in your actual tag positions)
        aprilTagFieldPositions.put(1, new Pose2d(24, 48));  // Tag 1 at (24,48)
        aprilTagFieldPositions.put(2, new Pose2d(72, 24));  // Tag 2 at (72,24)
        // Add more tags...

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials.isEmpty()) {
                    telemetry.addData("AprilTags", "No tags detected");
                } else {
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        int tagId = fiducial.getFiducialId();

                        telemetry.addData("Tag ID", tagId);
                        telemetry.addData("Angle X (deg)", fiducial.getTargetXDegrees());
                        telemetry.addData("Angle Y (deg)", fiducial.getTargetYDegrees());

                        if (aprilTagFieldPositions.containsKey(tagId)) {
                            Pose2d tagGlobalPose = aprilTagFieldPositions.get(tagId);

                            // Get angles from Limelight in radians
                            double angleX = Math.toRadians(fiducial.getTargetXDegrees());
                            double angleY = Math.toRadians(fiducial.getTargetYDegrees());

                            double distanceToTag = 24; //inches to tag (estimate)

                            // Calculate position
                            double relX = distanceToTag * Math.cos(angleY) * Math.sin(angleX);
                            double relY = distanceToTag * Math.sin(angleY);

                            // Compute estimate global position
                            robotX = tagGlobalPose.getX() - relX;
                            robotY = tagGlobalPose.getY() - relY;

                            telemetry.addData("Robot X", robotX);
                            telemetry.addData("Robot Y", robotY);
                        } else {
                            telemetry.addData("Robot Position", "Unknown tag ID " + tagId);
                        }
                    }
                }
            } else {
                telemetry.addData("AprilTags", "No valid data");
            }

            telemetry.update();
            sleep(50);
        }

        limelight.stop();
    }
}
