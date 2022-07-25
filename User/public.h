/*
 * public.h
 *
 *  Created on: Aug 28, 2020
 *      Author: bobkov
 */

/*
 * public.h
 *
 *  Created on: 25 авг. 2020 г.
 *      Author: PC
 */

#ifndef PUBLIC_H_
#define PUBLIC_H_

  /** @addtogroup PUBLIC_PublicIncludes
  * @{
  */
#include <stdint.h>
/**
  * @}
  */

typedef enum {S_DISABLE = 0, S_ENABLE = !S_DISABLE} State;

/** @addtogroup PUBLIC_PublicMacros
* @{
*/
#define ARRAY_SIZE(array) ((sizeof((array)))/(sizeof((array)[0])))
#define ARRAY_LAST_IND(array) (ARRAY_SIZE(array)-1)
#define ARRAY_LAST_EL(array) (array[(ARRAY_LAST_IND(array))])

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#ifdef  USE_FULL_ASSERT
#define ASSERT(expr) ((expr) ? (void)0U : ASSERT_FAIL((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void ASSERT_FAIL(uint8_t* file, uint32_t line);
#elif	USE_TEST_ASSERT
extern void ASSERT_SPY(uint8_t* file, uint32_t line);
#define ASSERT(expr) ((expr) ? (void)0U : ASSERT_SPY((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
#else
  #define ASSERT(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */
/**
* @}
*/



#endif /* PUBLIC_H_ */
