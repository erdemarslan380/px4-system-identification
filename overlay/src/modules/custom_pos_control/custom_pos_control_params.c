/**
 * Enable custom position forwarding
 *
 * 0 keeps the module passive.
 * 1 publishes offboard setpoints from trajectory_reader.
 *
 * @boolean
 */
PARAM_DEFINE_INT32(CST_POS_CTRL_EN, 0);

/**
 * Active forwarding mode
 *
 * @value 0 NO_CTRL
 * @value 4 PX4_DEFAULT
 * @value 6 SYSID
 */
PARAM_DEFINE_INT32(CST_POS_CTRL_TYP, 0);

/**
 * Enable RC pot controller selection
 *
 * The selected AUX channel is split into two slots:
 * PX4_DEFAULT and SYSID.
 *
 * @boolean
 */
PARAM_DEFINE_INT32(CST_RC_SEL_EN, 0);

/**
 * RC AUX channel used for controller selection
 *
 * @min 1
 * @max 6
 */
PARAM_DEFINE_INT32(CST_RC_CTRL_CH, 5);
