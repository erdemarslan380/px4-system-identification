/**
 * Enable RC pot trajectory/profile selection
 *
 * When enabled, one AUX channel selects either trajectory id (POSITION/TRAJECTORY)
 * or identification profile (IDENTIFICATION mode).
 *
 * @boolean
 */
PARAM_DEFINE_INT32(TRJ_RC_SEL_EN, 0);

/**
 * RC AUX channel used for trajectory or identification-profile selection
 *
 * @min 1
 * @max 6
 */
PARAM_DEFINE_INT32(TRJ_RC_SEL_CH, 6);

/**
 * Maximum trajectory id reachable by the RC selector
 *
 * The AUX range is quantized into values [0, TRJ_RC_MAX_ID].
 *
 * @min 0
 * @max 255
 */
PARAM_DEFINE_INT32(TRJ_RC_MAX_ID, 3);

/**
 * Identification profile
 *
 * @value 0 hover_thrust
 * @value 1 roll_sweep
 * @value 2 pitch_sweep
 * @value 3 yaw_sweep
 * @value 4 drag_x
 * @value 5 drag_y
 * @value 6 drag_z
 * @value 7 mass_vertical
 * @value 8 motor_step
 */
PARAM_DEFINE_INT32(TRJ_IDENT_PROF, 0);
