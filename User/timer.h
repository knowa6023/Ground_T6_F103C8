/*
 * timer.h
 *
 *  Created on: Aug 28, 2020
 *      Author: bobkov
 */

/*
 * timer.h
 *
 *  Created on: 25 авг. 2020 г.
 *      Author: PC
 */

#ifndef TIMER_H_
#define TIMER_H_
/*******************************************************************************
* File Name          : timers.h
* Author             :
* Version            : V2.1.0
* Date               : V1.0.0 16-07-2012,
                     : V1.0.1 09-01-2014 (выполнение callback вынесено в отдельную функцию, разделены байты статуса, флагов и настроек)
                     : V1.0.2 22-07-2014 - исправлена ошибка, приводящая к повторному постоянному выполнению callback функции у таймеров однократного выполнения
                     : V1.0.3 25-02-2016 - не выставлялся флаг остановки таймера при вызове функции Stop
                     : V1.0.3a 19-04-2016 - из-за того, что теперь выставляется флаг остановки таймера при вызове функции Stop немедленно выполняется callback, связанный с этим таймером. Не помню, зачем я выставляю флаг стоп при остановке таймера.
                     : V2.0.0 28-02-2019 - Принципиальное изменение архитектуры. Теперь очередь таймеров, это связный список
                     : V2.1.0 03-03-2020 - Тестирование функционала с использованием Google Test, исправление мелких глюков
* Description        :
*******************************************************************************/

/** @addtogroup Timer_Driver
  * @{
  */


  /** @addtogroup Timer_Includes
  * @{
  */
#include "public.h"
  /**
  * @}
  */

  /** @addtogroup Timer_Types
  * @{
  */
typedef struct{
  uint8_t isAutoReload : 1;
  uint8_t unused       : 7;
}TimerConfig;

typedef enum{
  TIMER_STOP = 0,
  TIMER_END = 1,
  TIMER_RUN = 2
}TimerExecState;

typedef struct{
  TimerExecState exState        : 4;
  uint8_t        callbackFlag   : 1;
  uint8_t        unused         : 3;
}TimerState;

typedef enum{
  TMR_RELOAD_YES = 1,
  TMR_RELOAD_NO = 0
}AutoreloadSetup;
  /**
  * @}
  */

  /** @addtogroup Timer_Types
  * @{
  */
typedef struct Timer{
  uint32_t count;
  uint32_t reloadValue;
  void (*CallBackFunction)(void);
  TimerState state;
  TimerConfig config;
  struct Timer *nextTimer;
}Timer;
  /**
  * @}
  */

  /** @addtogroup Timer_Constants
  * @{
  */

  /**
  * @}
  */

  /** @addtogroup Timer_PublicMacros
  * @{
  */
#define NO_CALLBACK    0
  /**
  * @}
  */

  /** @addtogroup Timer_PublicFunction
  * @{
  */


/*!Инициализировать таймер*/
void TMR_Init( void );
/*!Декрементирует всей таймеры на время, прошедшее с последнего вызова*/
void TMR_Tick( void );
/*!Добавить таймер*/
void TMR_Add( Timer* timer, void ( *callback )( void ), AutoreloadSetup autoreload );
/*!Запустить таймер*/
void TMR_Start( Timer* timer, uint32_t delay );
/*!Сделать таймер автоперезагружаемым*/
void TMR_SetAutoreload( Timer* timer, uint8_t autoreload );
/*!Остановить таймер*/
void TMR_Stop( Timer* timer );
/*!Перезапустить таймер*/
void TMR_Restart( Timer* timer );
/*!Перезапустить таймер с новым временем*/
void TMR_RestartWithNewDelay( Timer* timer, uint32_t delay );
/*!Очистить таймер*/
void TMR_Clear( Timer* timer );
/*!Получить оставшееся до конца счета время*/
uint32_t TMR_GetLeftTime( const Timer* timer );
/*!Получить время, прошедшее с момента запуска таймера*/
uint32_t TMR_GetElapsedTime( const Timer* timer );
/*!Блокирующая задержка*/
void TMR_BlockingDelay( uint32_t time );
/*!Получить статус таймера*/
TimerExecState TMR_GetStatus( const Timer* timer );
/*!Выполнить все callback. (Не вызывать эту функцию в прерываниях, только в основном коде)*/
void TMR_ExecuteCallbacks( void );
/*!Получить время, прошедшее с момента вызова функции TMR_Init(), в тиках таймера*/
uint32_t TMR_GetTimeFromStart( void );
/*!Установить время, прошедшее с момента вызова функции TMR_Init(), в тиках таймера*/
void TMR_SetTimeFromStart( uint32_t time );
/*!Запомнить текущее значение счетчика времени, относительно которого будет в дальнейшем работать функция TMR_WaitUntil*/
void TMR_StartWait( void );
/*!Ждать time тиков таймера, прошедших с момента последнего вызова TMR_StartWait*/
uint8_t TMR_WaitUntil( uint32_t time );
/*!Хардовый ресет контроллера*/
void TMR_ResetMCU( void );
  /**
  * @}
  */


/**
  * @}
  */





#endif /* TIMER_H_ */
