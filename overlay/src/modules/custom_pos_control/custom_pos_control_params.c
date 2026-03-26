// custom_pos_control_params.c

/**
 * Enable Custom Position Control
 * @boolean
 */

PARAM_DEFINE_INT32(CST_POS_CTRL_EN, 0);

/**
 * Custom Position Controller Type
 * 
 * Switch between controllers
 * 
 * @value 0 NO_CTRL
 * @value 1 DFBC
 * @value 2 MPC
 * @value 3 CMPC
 * @value 4 PX4_DEFAULT
 * @value 5 INDI
 * @value 6 SYSID
 */
PARAM_DEFINE_INT32(CST_POS_CTRL_TYP, 0);

/**
 * Enable RC pot controller selection
 *
 * When enabled, the selected AUX channel is quantized into controller slots:
 * PX4_DEFAULT, DFBC, MPC, CMPC, INDI, SYSID.
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
