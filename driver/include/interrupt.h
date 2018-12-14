

#ifndef HAL_INTERRUPT_H
#define HAL_INTERRUPT_H

typedef void (*hal_interrupt_t)(void *para);

typedef struct {
    hal_interrupt_t fun;
    void         *para;
} hal_interrupt_desc_t;


/**
 * Interrupt vector init
 *
 * @return  0 : on success, -1 : if an error occurred with any step
 */
int32_t hal_interrupt_init(void);

/**
 * Mask specified interrupt vector
 *
 *
 * @param[in] vec specified interrupt vector
 *
 * @return  0 : on success, -1 : if an error occurred with any step
 */
int32_t hal_interrupt_mask(int32_t vec);

/**
 * Unmask specified interrupt vector
 *
 *
 * @param[in] vec specified interrupt vector
 *
 * @return  0 : on success, -1 : if an error occurred with any step
 */
int32_t hal_interrupt_umask(int32_t vec);

/**
 * Install specified interrupt vector
 *
 *
 * @param[in] vec specified interrupt vector
 *
 * @return  0 : on success, -1 : if an error occurred with any step
 */
int32_t hal_interrupt_install(int32_t vec, hal_interrupt_t handler,void *para);


extern hal_interrupt_desc_t interrupt_desc[];

static __INLINE void hal_interrupt_callback(int32_t vec)
{
	if(vec >= VECTOR_MAX || vec < 0)
		return;
	
	interrupt_desc[vec].fun(interrupt_desc[vec].para);
}
#endif

