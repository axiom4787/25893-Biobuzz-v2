package org.firstinspires.ftc.teamcode.util;

import com.skeletonarmy.marrow.zones.CompositeZone;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

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

    // center of the hive's front face
    // adjusted a bit down because there is less valid scoring area in the top triangle than in the bottom rectangle
    public static final Position RIGHT_GOAL = new Position(DistanceUnit.INCH,
            58.8976, 55.1654, 58.3307, 0);
    public static final Position LEFT_GOAL = new Position(DistanceUnit.INCH,
            RIGHT_GOAL.x, HALF_FIELD_WIDTH-RIGHT_GOAL.y, RIGHT_GOAL.z, 0);

    // TODO: NEED HEAVY TUNING
    public static final double SHOOT_TIME = 1000;
    public static final double FLOWER_INTAKE_TIME = 2000;
    public static final double GARDEN_INTAKE_TIME = 800;
}
