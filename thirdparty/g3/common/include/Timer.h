#ifndef __CALLBACK_TIMER_H__
#define __CALLBACK_TIMER_H__

// forward declaration
struct TTimer;

/**********************************************************************************************************************/
/** Event handler which make the timers running
 **********************************************************************************************************************/
void Timer_EventHandler(void);

/**********************************************************************************************************************/
/** Use this function to register a timer. Timers are one-shot (not repetitive)
 **********************************************************************************************************************/
void Timer_Register(
  struct TTimer *pTimer,
  uint32_t u32TenthsSeconds
  );

/**********************************************************************************************************************/
/** Use this function to unregister a timer. If the timer is not registered this function does nothing
 **********************************************************************************************************************/
void Timer_Unregister(
  struct TTimer *pTimer
  );

/**********************************************************************************************************************/
/** Resets (unregisters) all the timers
 **********************************************************************************************************************/
void Timer_ResetAll(void);

/**********************************************************************************************************************/
/** Returns true if the timer is registered; false otherwise
 **********************************************************************************************************************/
bool Timer_IsRegistered(struct TTimer *pTimer);

/**********************************************************************************************************************/
/** Returns true if the time value from the parameter is in the past
 **********************************************************************************************************************/
bool Timer_IsPast(int32_t i32TimeValue);

/**********************************************************************************************************************/
/** Returns true if the time value from the parameter is in the past
 **********************************************************************************************************************/
bool Timer_IsPast10Seconds(int32_t i32TimeValue);

/**********************************************************************************************************************/
/** Returns true if the time2 is older than time1
 **********************************************************************************************************************/
bool Timer_IsPastCmp(int32_t i32Time1, int32_t i32Time2);

/**********************************************************************************************************************/
/** Returns the uptime in tenths of seconds as a signed integer
 **********************************************************************************************************************/
int32_t Timer_SignedSysGetUpTimeTenthsSeconds(void);

/**********************************************************************************************************************/
/** Returns the uptime in 10 seconds steps as a signed integer
 **********************************************************************************************************************/
int32_t Timer_SignedSysGetUpTime10Seconds(void);

/**********************************************************************************************************************/
/** Defines the format of the callback function
 **********************************************************************************************************************/
typedef void (*TimerExpired_Callback)(struct TTimer *pTimer);

/**********************************************************************************************************************/
/** Defines the internal representation of a timer
 **********************************************************************************************************************/
struct TTimer {
  struct TTimer *m_pNextTimer; // pointer to next timer (private member)
  TimerExpired_Callback m_fnctCallback; // callback to be called when timer expired (public member)
  void *m_pUserData;

  int32_t m_i32ExpirationTime; // ticks / milliseconds (private member)
};
#endif
