// RoutesToRun.java
package com.example.sharedconstants;

public enum RoutesToRun {
    DO_NOTHING,
    NET_SCORE_1_SPECIMEN_PRELOAD,
    OBS_SCORE_1_PRELOAD,

    // Pickup the samples on their short side
    NET_SCORE_2_SPECIMEN_PRELOAD_AND_1_SAMPLE,
    NET_SCORE_3_SPECIMEN_PRELOAD_AND_2_SAMPLES,
    NET_SCORE_4_SPECIMEN_PRELOAD_AND_3_SAMPLES,
    NET_SCORE_5_SPECIMEN_PRELOAD_AND_3_SAMPLES_AND_1_HUMAN_PLAYER_SAMPLE,
    NET_SCORE_6_SPECIMEN_PRELOAD_AND_3_SAMPLES_AND_2_HUMAN_PLAYER_SAMPLE,
    NET_SCORE_5_SAMPLE_PRELOAD,

    OBS_SCORE_4_PRELOAD_PUSH_TWO_AND_PICKUP_AT_TRIANGLE,
    OBS_SCORE_4_PRELOAD_PUSH_ALL_AND_PICKUP_AT_TRIANGLE,
    OBS_SCORE_5_PRELOAD_PUSH_ALL_AND_PICKUP_AT_TRIANGLE,
    OBS_SCORE_5_LEAVE_PRELOAD_PUSH_AND_PICKUP_AT_TRIANGLE,

    OBS_SCORE_5_PRELOAD_GROUND_PICKUP_AND_DUMP_AND_PICKUP_AT_TRIANGLE
}
