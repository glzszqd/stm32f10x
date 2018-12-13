#ifndef _HAL_BLDC_H_
#define _HAL_BLDC_H_

typedef enum{
	BLDC_PORT0 = 0,
	BLDC_PORT1,
	BLDC_PORT_MAX_NUM
}hal_bldc_port_t;

typedef struct {
    PHASE_ERROR1 = 0,
	PHASE_UV,
	PHASE_UW,
	PHASE_VW,
	PHASE_VU,
	PHASE_WU,
	PHASE_WV,
	PHASE_ERROR2
} hal_bldc_phase_t;

typedef struct
{
	ROTATE_CLOCKWISE,
	ROTATE_ANTICLOCKWISE
}hal_bldc_rotate_t;

typedef struct
{
	MODE_STOP,
	MODE_START,
	MODE_RUN,
	MODE_BREAK
}hal_bldc_mode_t;

typedef struct {
    uint16_t  duty;  /* the pwm duty */
    uint16_t  freq;        /* the pwm freq */
	hal_bldc_phase_t phase;	/* the bldc phase */
	hal_bldc_rotate_t rotate;	/*the bldc rotate*/
	hal_bldc_mode_t mode;	/*the bldc mode*/
} bldc_config_t;

typedef struct {
    uint8_t      port;    /* bldc port */
    bldc_config_t config;  /* bldc config */
    void        *priv;    /* priv data */
} bldc_dev_t;

int8_t  hal_bldc_init(bldc_dev_t *bldc);
int8_t  hal_bldc_finalize(bldc_dev_t *bldc);
int8_t  hal_bldc_start(bldc_dev_t *bldc);
int8_t  hal_bldc_stop(bldc_dev_t *bldc);
int8_t  hal_bldc_update_pwm(bldc_dev_t *bldc);
int8_t  hal_bldc_update_phase(bldc_dev_t *bldc);

#endif