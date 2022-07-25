/*
 * timer.c
 *
 *  Created on: Aug 28, 2020
 *      Author: bobkov
 */

/*
 * timer.c
 *
 *  Created on: 25 авг. 2020 г.
 *      Author: PC
 */

/*******************************************************************************
 * File Name          : timers.c
 * Author             : Блохин Константин Евгеньевич
 * Version            : V2.0.2
 * Date               : 28-02-2019
 * Description        : Работа с софтовыми таймерами на основе одного хардового
 *******************************************************************************/
/*
 v2.0.1 - Добавлены функции TMR_StartWait и TMR_WaitUntil для реализации неблокирующей задержки. Ограничение - в один момент времени может осуществляться одно ожидание
 v2.0.2 - Добавлен ресет контроллера через вотчдог
 v2.1.0 - Добавлена функция TMR_SetTimeFromStart, исправлены мелкие глюки, вынесены аппаратно-зависимые функции в отдельный файл, требующий своей реализации
 на каждой целевой платформе, таймеры протестированы с использованием фреймворка Google tests
 */

/*
 1. Инициализация
 1.1 Вызвать TMR_Init где-нибудь до бесконечного цикла
 1.2 Создать переменную типа Timer. Переменная должна располагаться в глобальной памяти (не в стеке!)
 1.3 Вызвать функцию TMR_Add с необходимыми параметрами. Функция проинициализирует переменную типа Timer, переданную в качестве параметра
 Нельзя вызывать функцию TMR_Add бесконечное количество раз, да еще с одинаковыми параметрами, это может привести к непредсказуемуму поведению
 В основе таймеров лежит односвязный список. Каждый раз при вызове функции Add в списке появляется новый узел, прикрепленный ссылкой к предыдущему.
 Если прикрепить к предыдущему таймеру ссылку на себя же (вызвать два раза функцию Add с одним и тем же таймером) - это приведет к бесконечному циклу
 и зависанию.
 1.4 Запустить таймер функцией TMR_Start( Timer*, uint32_t time ). По истечении времени, если был определен коллбэк (ненулевым значением), он сработает

 2. Блокирующая задержка
 Т.к. значения таймеров не изменяются в прерывании, а пересчитываются в контексте main при вызове функции TMR_ExecuteCallbacks, на них нельзя
 реализовать блокирующую задержку. Для этих целей служит функция TMR_BlockingDelay

 3. Ожидающая задержка.
 Иногда нужно дождать одного из двух событий, одно из которых должно быть связано с окончанием счета таймера
 Для этого есть пара функций TMR_StartWait и TMR_WaitUntil.
 Сначала вызывается TMR_StartWait, функция запоминает текущее значение счетчика времени
 Функция TMR_WaitUntil выполняет ожидание на +time тиков, относительно того значения, которое запомнила функция TMR_StartWait.
 К примеру:
 TMR_StartWait();
 while ( event ){
 if ( TMR_WaitUntil(1000) ){
 break;
 }
 }
 Здесь идет ожидание события event, но цикл прерывается, если раньше закончится время ожидания TMR_WaitUntil
 Есть, однако, ограничения, связанные с применением этих функций. Нельзя одновременно в нескольких местах программы
 использовать эти функции. Под одновременностью я подразумеваю следующую ситуацию:
 while(1){
 TMR_StartWait();
 if ( TMR_WaitUntil(1000) ){
 i = 1;
 }

 TMR_StartWait();
 if ( TMR_WaitUntil(2000) {
 i = 2;
 }
 }

 Функцию TMR_WaitUntil не является блокирующей. Поэтому ни одно из условий i=1 или i=2 никогда не будет выполнено.
 В примере до этого блок while(event) осуществлял задержку выполнения всей программы на срок либо до наступления event, либо до окончания ожидания,
 поэтому в подобных блоках, идущих друг за другом, противоречий не будет.
 */

/** @addtogroup Timer_driver
 * @{
 */

/** @addtogroup Timer_Includes
 * @{
 */
#include "timer.h"
#include "stdio.h"
/**
 * @}
 */

/** @addtogroup Timer_PrivateTypes
 * @{
 */

/**
 * @}
 */

/** @addtogroup Timer_PrivateVariables
 * @{
 */
/*!Время, прошедшее с момента вызова TMR_Init(). Чтобы время считалось правильно, прерывание, в котором вызывается TMR_Tick() должно быть настроено на 1мс*/
static volatile uint32_t msFromStart = 0;
static uint32_t oldTime = 0;
static Timer* headTimer = 0;
/**
 * @}
 */

/** @addtogroup Timer_PrivateConstants
 * @{
 */

/**
 * @}
 */

/** @addtogroup Timer_PrivateMacros
 * @{
 */
//Спасибо Semtech за timeServer
/*!
 * safely execute call back
 */
#define exec_cb( _callback_ )           \
  do {                                  \
      if( _callback_ == 0 )             \
      {                                 \
        ;                       \
      }                                 \
      else                              \
      {                                 \
        _callback_();                   \
      }                                 \
  } while(0);
/*Для отладки
 #define exec_cb( _callback_ )           \
  do {                                  \
      if( _callback_ == 0 )             \
      {                                 \
        while(1);                       \
      }                                 \
      else                              \
      {                                 \
        _callback_();                   \
      }                                 \
  } while(0);
 */
/**
 * @}
 */

/** @addtogroup Timer_PrivateFunctionPrototypes
 * @{
 */
/*!Платформенно-зависимые функции*/
extern void eTMR_InitWatchdog(void);
extern void eTMR_ClearWatchdog(void);
extern void eTMR_DisableInterrupt(void);
extern void eTMR_EnableInterrupt(void);

/*!Вызывается в прерывании, настроенном на 1мс*/
void TMR_IRQ(void);

static void _decrementAllTimers(uint32_t decrement);
static void _checkElapsedTimers(void);
static void _executeAllCallbacks(void);
static void _init_watchdog(void);
/**
 * @}
 */

/** @addtogroup Timer_PrivateFunctions
 * @{
 */
static void _decrementAllTimers(uint32_t decrement) {
	Timer *currentTimer = headTimer;
	while (currentTimer != 0) {
		if (currentTimer->count && currentTimer->state.exState == TIMER_RUN) {
			if (currentTimer->count > decrement) {
				currentTimer->count -= decrement;
			} else {
				currentTimer->count = 0;
			}
		}
		currentTimer = currentTimer->nextTimer;
	}
}

/**
 * @brief  Проверяем, какие таймеры закончились, выставляем флаги, перезапускаем если необходимо
 * @param  None
 * @retval None
 */
static void _checkElapsedTimers(void) {
	Timer *currentTimer = headTimer;
	while (currentTimer != 0) {
		if (currentTimer->count == 0
				&& currentTimer->state.exState == TIMER_RUN) {
			if (currentTimer->config.isAutoReload) {
				TMR_Restart(currentTimer);
			} else {
				currentTimer->state.exState = TIMER_END;
			}
			currentTimer->state.callbackFlag = 1;
		}
		currentTimer = currentTimer->nextTimer;
	}
}

static void _executeAllCallbacks(void) {

#ifdef WATCHDOG
	eTMR_ClearWatchdog();
  #endif
	Timer *currentTimer = headTimer;

	while (currentTimer != 0) {
#ifdef WATCHDOG
	  eTMR_ClearWatchdog();
    #endif
		if (currentTimer->state.callbackFlag) {
			currentTimer->state.callbackFlag = 0;
			exec_cb(currentTimer->CallBackFunction);
		}
		currentTimer = currentTimer->nextTimer;
	}
}

static void _init_watchdog(void) {

	eTMR_InitWatchdog();

}
/**
 * @}
 */

void TMR_IRQ(void) {
	msFromStart++;
	TMR_Tick(); //Можно здесь (если таймеров мало), а можно в контексте main. Например в TMR_ExecuteCallbacks
}
/** @addtogroup Timer_PublicFunction
 * @{
 */
/**
 * @brief  Инициализируем таймер
 * @param  TIM заполненная структура типа Timer_InitTypeDef
 * @retval None
 */
void TMR_Init(void) {
	msFromStart = 0;
	headTimer = 0;
	oldTime = 0;
#ifdef WATCHDOG
  _init_watchdog();
#endif
}

/**
 * @brief  Декрементировать все активные таймеры на время, прошедшее с последнего вызова
 * @param  None
 * @retval None
 */
void TMR_Tick(void) {
	//__disable_interrupt();
	uint32_t newTime = msFromStart;
	//__enable_interrupt();
	uint32_t delta = 0;
	if (newTime < oldTime) { //произошло переполнение. 49 суток, но мало ли...
		delta = 0xFFFFFFFF - oldTime + newTime;
	} else {
		delta = newTime - oldTime;
	}
	if (delta == 0)
		return;

	oldTime = newTime;

	_decrementAllTimers(delta);
}

/**
 * @brief  Добавляет таймер.
 * @param  timer - структура таймера, которая добавится к общему списку
 * @param  callbackFunction функция, которая выполнится после того, как таймер закончит счет
 * @retval None
 */
void TMR_Add(Timer *timer, void (*callback)(void), AutoreloadSetup autoreload) {
	timer->CallBackFunction = callback;
	timer->count = 0;
	timer->reloadValue = 0;
	timer->state.exState = TIMER_STOP;
	timer->state.callbackFlag = 0;
	if (autoreload) {
		timer->config.isAutoReload = 1;
	} else {
		timer->config.isAutoReload = 0;
	}

	//Еще нет ни одного таймера
	if (headTimer == 0) {
		timer->nextTimer = 0;
		headTimer = timer;
		return;
	}

	//Проходим список от начала и до конца, проверяя таймеры на повтор
	Timer *currentTimer = headTimer;
	while (currentTimer->nextTimer != 0) {
		if (currentTimer != timer) {
			currentTimer = currentTimer->nextTimer;
		} else {
			return; //Ошибка, в списке таймеров такой таймер уже есть
		}
	}

	//Цепляем новый таймер в начало списка (можно в конец, если воспользоваться переменной currentTimer)
	timer->nextTimer = headTimer;
	headTimer = timer;
}

/**
 * @brief  Устанавливает параметр, с которым происходит вызов коллбека.
 * @param  timer - структура таймера, над которым производится операция
 * @param  context параметр вызова коллбека
 * @retval None
 */
/*void TMR_SetContext( Timer* timer, void* context ){
 timer->context = context;
 }
 */

/**
 * @brief  Устанавливает флаг автоперезагружаемости таймера
 * @param  timer - структура таймера, над которым производится операция
 * @param  autoreload 0 - таймер не автоперезагружаемый, любое другое значение - перезагружаемый
 * @retval None
 */
void TMR_SetAutoreload(Timer *timer, uint8_t autoreload) {
	if (autoreload) {
		timer->config.isAutoReload = 1;
	} else {
		timer->config.isAutoReload = 0;
	}
}

/**
 * @brief  Запустить таймер не трогая счетчик. Если таймер остановлен - продолжит выполнение, если уже запущен - ничего не произойдет.
 * @param  timer - таймер, над которым проводится операция
 * @param  delay - время таймера в мс
 * @retval event текущий статус таймера
 */
void TMR_Start(Timer *timer, uint32_t delay) {
	if (timer->state.exState == TIMER_STOP
			|| timer->state.exState == TIMER_END) {
		timer->state.exState = TIMER_RUN;
		timer->reloadValue = delay;
		timer->count = delay;
	}
}

/**
 * @brief  Остановить таймер
 * @param  timer - таймер, над которым проводится операция
 * @retval none
 */
void TMR_Stop(Timer *timer) {
	timer->state.exState = TIMER_STOP;
}

/**
 * @brief  Перезапустить таймер, обнулив счетчик. Если таймер запущен выполнение начнется с начала
 * @param  timer - таймер, над которым проводится операция
 * @retval none
 */
void TMR_Restart(Timer *timer) {
	timer->count = timer->reloadValue;
	timer->state.exState = TIMER_RUN;
}

/**
 * @brief  Перезапустить таймер с новым значением задержки
 * @param  timer - таймер, над которым проводится операция
 * @retval none
 */
void TMR_RestartWithNewDelay(Timer *timer, uint32_t delay) {
	timer->reloadValue = delay;
	TMR_Restart(timer);
}

/**
 * @brief  Сбросить текущий счетчик таймера до первоначального значения
 * @param  timer - таймер, над которым проводится операция
 * @retval none
 */
void TMR_Clear(Timer *timer) {
	timer->count = timer->reloadValue;
}

/**
 * @brief  Выполнить все функции обратного вызова
 * @param  None
 * @retval Статус
 */
void TMR_ExecuteCallbacks(void) {
	//Один из вариантов размещения этой функции. Можно в упрощенном виде (без рассчета delta) вызывать в прерывании
	//что экономит время нахождения в прерывании, но зато все таймеры всегда имеют актуальное время.
	//Либо здесь, где эта функция будет выполняться столько раз, сколько успеет в главном цикле в контексте main.
	//TMR_Tick();
	_checkElapsedTimers();
	_executeAllCallbacks();
}

/**
 * @brief  Блокирующая задержка
 * @param  None
 * @retval None
 */
void TMR_BlockingDelay(uint32_t time) {
	uint32_t endTime = msFromStart + time + 1;
	while (msFromStart < endTime) {
		if (msFromStart % 100) {
#ifdef WATCHDOG
    	eTMR_ClearWatchdog();
      #endif
		}
	}
}

/**
 * @brief  Получить оставшееся до истечения таймера время
 * @param  TIM заполненная структура типа Timer_InitTypeDef
 * @retval none
 */
uint32_t TMR_GetLeftTime(const Timer *timer) {
	return timer->count;
}

uint32_t TMR_GetElapsedTime(const Timer *timer) {
	return (timer->reloadValue - timer->count);
}

/**
 * @brief  Получить статус таймера
 * @param  TIM заполненная структура типа Timer_InitTypeDef
 * @retval Статус
 */
TimerExecState TMR_GetStatus(const Timer *timer) {
	return timer->state.exState;
}
/**
 * @}
 */

/**
 * @brief  Получить время с момента запуска таймеров
 * @param  none
 * @retval Время с момента запуска таймеров в мс
 */
uint32_t TMR_GetTimeFromStart(void) {
	return msFromStart;
}

static uint32_t startTime = 0;
/**
 * @brief  Запомнить новое время, с которого начинается ожидание
 * @param  none
 * @retval none
 */
void TMR_StartWait(void) {
#ifdef WATCHDOG
	eTMR_ClearWatchdog();
  #endif
	startTime = TMR_GetTimeFromStart();
}

/*!Установить время, прошедшее с момента вызова функции TMR_Init(), в тиках таймера*/
void TMR_SetTimeFromStart( uint32_t time ){
	msFromStart = time;
}

/**
 * @brief  Неблокирующие ожидание с момента вызова TMR_StartWait до startTime + time
 * @param  none
 * @retval 1 - если ожидание закончено, 0 - в противном случае
 */
uint8_t TMR_WaitUntil(uint32_t time) {
	uint32_t currentTime = TMR_GetTimeFromStart();
	uint32_t elapsedTime;
	//На случай переполнения (ну а вдруг)
	if (currentTime < startTime) {
		elapsedTime = 0xFFFFFFFF - startTime + currentTime;
	} else {
		elapsedTime = currentTime - startTime;
	}

	if (elapsedTime % 100 == 0) {
#ifdef WATCHDOG
	  eTMR_ClearWatchdog();
    #endif
	}

	if (elapsedTime >= time) {
		return 1;
	}
	return 0;
}
/**
 * @}
 */

void TMR_ResetMCU(void) {
	eTMR_DisableInterrupt( );
	_init_watchdog();
	while (1)
		;
}
/**
 * @}
 */

