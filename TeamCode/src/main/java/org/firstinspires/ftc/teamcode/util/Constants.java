package org.firstinspires.ftc.teamcode.util;

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
}
