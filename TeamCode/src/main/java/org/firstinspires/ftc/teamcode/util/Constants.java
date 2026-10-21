package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.zones.CompositeZone;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

public class Constants {
    public static final double FIELD_WIDTH = 141.5;
    public static final double HALF_FIELD_WIDTH = FIELD_WIDTH / 2;

    private static final double HALF_HIVE_FRAME_SUPPORT_LENGTH = 40 / 2.0;
    private static final double HIVE_FRAME_SUPPORT_WIDTH = 3;
    private static final double HALF_HIVE_SPACE_BETWEEN_SUPPORTS = 48 / 2.0;

    public static final CompositeZone HIVE_SUPPORT_ZONE = new CompositeZone(
            new PolygonZone(
                    new Point(
                            HALF_FIELD_WIDTH - HALF_HIVE_SPACE_BETWEEN_SUPPORTS - HIVE_FRAME_SUPPORT_WIDTH,
                            HALF_FIELD_WIDTH - HALF_HIVE_FRAME_SUPPORT_LENGTH
                    ),
                    new Point(
                            HALF_FIELD_WIDTH - HALF_HIVE_SPACE_BETWEEN_SUPPORTS,
                            HALF_FIELD_WIDTH - HALF_HIVE_FRAME_SUPPORT_LENGTH
                    ),
                    new Point(
                            HALF_FIELD_WIDTH - HALF_HIVE_SPACE_BETWEEN_SUPPORTS,
                            HALF_FIELD_WIDTH + HALF_HIVE_FRAME_SUPPORT_LENGTH
                    ),
                    new Point(
                            HALF_FIELD_WIDTH - HALF_HIVE_SPACE_BETWEEN_SUPPORTS - HIVE_FRAME_SUPPORT_WIDTH,
                            HALF_FIELD_WIDTH + HALF_HIVE_FRAME_SUPPORT_LENGTH
                    )
            ),
            new PolygonZone(
                    new Point(
                            HALF_FIELD_WIDTH + HALF_HIVE_SPACE_BETWEEN_SUPPORTS + HIVE_FRAME_SUPPORT_WIDTH,
                            HALF_FIELD_WIDTH - HALF_HIVE_FRAME_SUPPORT_LENGTH
                    ),
                    new Point(
                            HALF_FIELD_WIDTH + HALF_HIVE_SPACE_BETWEEN_SUPPORTS,
                            HALF_FIELD_WIDTH - HALF_HIVE_FRAME_SUPPORT_LENGTH
                    ),
                    new Point(
                            HALF_FIELD_WIDTH + HALF_HIVE_SPACE_BETWEEN_SUPPORTS,
                            HALF_FIELD_WIDTH + HALF_HIVE_FRAME_SUPPORT_LENGTH
                    ),
                    new Point(
                            HALF_FIELD_WIDTH + HALF_HIVE_SPACE_BETWEEN_SUPPORTS + HIVE_FRAME_SUPPORT_WIDTH,
                            HALF_FIELD_WIDTH + HALF_HIVE_FRAME_SUPPORT_LENGTH
                    )
            )
    );

    public static final double RIGHT_GOAL_Y = 55;
    public static final double LEFT_GOAL_Y = FIELD_WIDTH - 55;
    public static final Pose RIGHT_GOAL = new Pose(58, RIGHT_GOAL_Y);
    public static final Pose LEFT_GOAL = new Pose(58, LEFT_GOAL_Y);
    public static final PolygonZone RIGHT_SCORING_ZONE = new PolygonZone(
            new Point(0, 0),
            new Point(FIELD_WIDTH, 0),
            new Point(FIELD_WIDTH, RIGHT_GOAL_Y),
            new Point(0, RIGHT_GOAL_Y)
    );
    public static final PolygonZone LEFT_SCORING_ZONE = new PolygonZone(
            new Point(0, FIELD_WIDTH),
            new Point(FIELD_WIDTH, FIELD_WIDTH),
            new Point(FIELD_WIDTH, LEFT_GOAL_Y),
            new Point(0, LEFT_GOAL_Y)
    );

    public static final double SHOOT_TIME = 1000;
    public static final double FLOWER_INTAKE_TIME = 2000;
    public static final double GARDEN_INTAKE_TIME = 800;

    // Limelight / HiveVision Camera Calibration Constants
    public static final double LIMELIGHT_X_OFFSET = 6.0;   // inches forward from robot center
    public static final double LIMELIGHT_Y_OFFSET = 0.0;   // inches left from robot center
    public static final double LIMELIGHT_Z_OFFSET = 10.0;  // inches above ground to camera lens
    public static final double LIMELIGHT_PITCH_DEG = -20.0; // camera pitch angle in degrees (negative = tilted down)
    public static final double LIMELIGHT_YAW_DEG = 0.0;     // camera yaw offset in degrees relative to robot forward

    public static final double BALL_RADIUS = 1.5;           // ball center height above ground in inches
}
