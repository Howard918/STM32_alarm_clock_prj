/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


enum CLOCK_MODE{
	NORMAL_STATE,
	TIME_SETTING,
	ALARM_TIME_SETTING,
	MUSIC_SELECT,
	ALARM_TRIGGERED_STATE
};

enum CLOCK_MANIPULATE{
	NO_KEY,
	UP,
	DOWN,
	RIGHT,
	LEFT,
	SEL,
	DBL_SEL,
	LONG_SEL
};

struct clock_state{
	enum CLOCK_MODE mode;
	volatile enum CLOCK_MANIPULATE key;
	int music_num;
	int edit_pos;
	int alarm_enabled;
	int song_note_index;
	uint32_t note_end_time;
	int volume;
	uint32_t display_timeout_timer;
};

struct clock_state current_state;

uint32_t last_key_time = 0;

// 편집 위치 정의
enum EDIT_POS {
    POS_AMPM = 0,
    POS_HOUR,
    POS_MIN,
    POS_SEC,
	POS_ALARM_ONOFF
};
// 블링킹 제어용 변수
int blink_state = 0; // 0:보임, 1:숨김
int blink_timer = 0;
int blink_force = 0;

typedef struct {
  int8_t hours;
  int8_t minutes;
  int8_t seconds;
}TimeTypeDef;

TimeTypeDef ctime;  // current time
TimeTypeDef stime; // setting time
TimeTypeDef atime;  // alarm time

typedef struct {
  int8_t music_num;
  char music_title[16];
}MusicTypeDef;

MusicTypeDef alarm_music[] =
{
  {0,"School Bell"},
  {1,"For Elise"},
  {2,"Ode to Joy"},
};


/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_12     ((uint32_t)0x08100000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_13     ((uint32_t)0x08104000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_14     ((uint32_t)0x08108000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_15     ((uint32_t)0x0810C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_16     ((uint32_t)0x08110000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_17     ((uint32_t)0x08120000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_18     ((uint32_t)0x08140000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_19     ((uint32_t)0x08160000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_20     ((uint32_t)0x08180000) /* Base @ of Sector 8, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_21     ((uint32_t)0x081A0000) /* Base @ of Sector 9, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_22     ((uint32_t)0x081C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_23     ((uint32_t)0x081E0000) /* Base @ of Sector 11, 128 Kbytes */

#define MAGIC_NUM 0xdeadbeef

typedef struct {
  uint32_t magic_num;
  TimeTypeDef setting_time;
  TimeTypeDef alarm_time;
  int8_t alarm_music_num;
  int8_t alarm_enabled;
  int8_t volume;
}NVitemTypeDef;

#define nv_items  ((NVitemTypeDef *) ADDR_FLASH_SECTOR_23)

NVitemTypeDef default_nvitem =
{
  MAGIC_NUM,
  {0,0,0},
  {0,0,0},
  0,
  0,
  5
};
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define LONG_PRESS_MS 3000
#define DBL_CLICK_GAP_MS 220

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Process_Music_Select(void);
void Save_And_Display_Message(void);
void Process_Alarm_Triggered(void);
void Process_Alarm_Playback(void);
void Adjust_Volume(enum CLOCK_MANIPULATE direction);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 음계 주파수 (Hz)
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_GS4 415 // G#4 / Ab4
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554 // C#5
#define NOTE_D5  587
#define NOTE_DS5 622 // D#5
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_GS5 831 // G#5
#define NOTE_A5  880
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_E6  1319
#define REST      0   // 쉼표

// Note structure for melodies
typedef struct {
    uint16_t frequency;
    uint16_t duration_ms;
} Note;

// Song 1: School Bell (Rhythmically Corrected & Staccato)
const Note school_bell[] = {
    {NOTE_G4, 200}, {REST, 2}, {NOTE_G4, 200}, {REST, 2}, {NOTE_A4, 200}, {REST, 2}, {NOTE_A4, 200}, {REST, 2},
    {NOTE_G4, 200}, {REST, 2}, {NOTE_G4, 200}, {REST, 2}, {NOTE_E4, 400}, {REST, 300},
    {NOTE_G4, 200}, {REST, 2}, {NOTE_G4, 200}, {REST, 2}, {NOTE_E4, 200}, {REST, 2}, {NOTE_E4, 200}, {REST, 2},
    {NOTE_D4, 400}, {REST, 300},
	{NOTE_G4, 200}, {REST, 2}, {NOTE_G4, 200}, {REST, 2}, {NOTE_A4, 200}, {REST, 2}, {NOTE_A4, 200}, {REST, 2},
	{NOTE_G4, 200}, {REST, 2}, {NOTE_G4, 200}, {REST, 2}, {NOTE_E4, 400}, {REST, 300},
	{NOTE_G4, 200}, {REST, 2}, {NOTE_E4, 200}, {REST, 2}, {NOTE_D4, 200}, {REST, 2}, {NOTE_E4, 200}, {REST, 2},
	{NOTE_C4, 400}, {REST, 500},
    {REST, 0} // End of song marker
};

// Song 2: For Elise (Rhythmically Corrected & Staccato)
const Note for_elise[] = {
    {NOTE_E5, 150}, {REST, 2}, {NOTE_DS5, 150}, {REST, 2}, {NOTE_E5, 150}, {REST, 2}, {NOTE_DS5, 150}, {REST, 2}, {NOTE_E5, 150}, {REST, 2}, {NOTE_B4, 150}, {REST, 2}, {NOTE_D5, 150}, {REST, 2}, {NOTE_C5, 150}, {REST, 2},
    {NOTE_A4, 300}, {REST, 150}, {NOTE_C4, 150}, {REST, 2}, {NOTE_E4, 150}, {REST, 2}, {NOTE_A4, 150}, {REST, 2}, {NOTE_B4, 300}, {REST, 150},
    {NOTE_E4, 150}, {REST, 2}, {NOTE_C5, 150}, {REST, 2}, {NOTE_B4, 150}, {REST, 2}, {NOTE_A4, 300},
    {REST, 0} // End of song marker
};

// Song 3: Super Mario Bros Intro (Rhythmically Corrected & Staccato)
const Note ode_to_joy[] = {
    // 미 미 파 솔 / 솔 파 미 레 / 도 도 레 미 / 미(점) 레 레
    {NOTE_E4, 250}, {REST, 10}, {NOTE_E4, 250}, {REST, 10}, {NOTE_F4, 250}, {REST, 10}, {NOTE_G4, 250}, {REST, 10},
    {NOTE_G4, 250}, {REST, 10}, {NOTE_F4, 250}, {REST, 10}, {NOTE_E4, 250}, {REST, 10}, {NOTE_D4, 250}, {REST, 10},
    {NOTE_C4, 250}, {REST, 10}, {NOTE_C4, 250}, {REST, 10}, {NOTE_D4, 250}, {REST, 10}, {NOTE_E4, 250}, {REST, 10},
    {NOTE_E4, 375}, {REST, 10}, {NOTE_D4, 125}, {REST, 10}, {NOTE_D4, 500}, {REST, 250},

    // 미 미 파 솔 / 솔 파 미 레 / 도 도 레 미 / 레(점) 도 도
    {NOTE_E4, 250}, {REST, 10}, {NOTE_E4, 250}, {REST, 10}, {NOTE_F4, 250}, {REST, 10}, {NOTE_G4, 250}, {REST, 10},
    {NOTE_G4, 250}, {REST, 10}, {NOTE_F4, 250}, {REST, 10}, {NOTE_E4, 250}, {REST, 10}, {NOTE_D4, 250}, {REST, 10},
    {NOTE_C4, 250}, {REST, 10}, {NOTE_C4, 250}, {REST, 10}, {NOTE_D4, 250}, {REST, 10}, {NOTE_E4, 250}, {REST, 10},
    {NOTE_D4, 375}, {REST, 10}, {NOTE_C4, 125}, {REST, 10}, {NOTE_C4, 500}, {REST, 500},

    {REST, 0} // End of song marker
};



// Array of song pointers
const Note* songs[] = {
    school_bell,
    for_elise,
    ode_to_joy
};

// Buzzer control functions (templates)
#define BUZZER_TIMER_CLK 1000000UL // Using 1MHz clock after prescaler
// TIM_HandleTypeDef htim3; // This is now declared in main.h via CubeMX

// Call this to start playing a note
void Buzzer_PlayNote(uint16_t frequency)
{
    if (frequency == 0 || current_state.volume == 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); // Off
        return;
    }
    // Calculate timer period for the desired frequency, based on 1MHz clock
    uint32_t period = BUZZER_TIMER_CLK / frequency;
    if (period > 65535) period = 65535; // Clamp to max 16-bit value

    // Calculate compare value based on volume (0-10)
    // We'll scale the duty cycle from 0% to 5% (max volume is now quieter)
    uint32_t compare_value = (period * current_state.volume) / 200; // (period / 20) * (volume / 10)

    __HAL_TIM_SET_AUTORELOAD(&htim3, period);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare_value);
}

// Call this to stop the buzzer
void Buzzer_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}

// Call this to start the buzzer PWM
void Buzzer_Start(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

//================ UART / tim Variance ==========================
volatile int timer_count = 0;
volatile int g_task_flag_1ms = 0;
volatile int g_task_flag_100ms = 0;
char rcv_byte;
char uart_buf[30];

//================ JoyStick ADC Variance ========================
volatile uint16_t g_adc_x_value = 0;
volatile uint16_t g_adc_y_value = 0;

static volatile uint32_t g_button_press_time = 0;      // Timestamp of the last button press
static volatile uint32_t g_button_last_release_time = 0; // Timestamp of the last button release
static volatile int g_button_click_count = 0;          // Number of clicks detected
static volatile int g_new_release_event = 0;           // Flag for a new button release event

//================== User Function ========================
void Time_Increment(TimeTypeDef *time)
{
    time->seconds++;

    if(time->seconds >= 60)
    {
        time->seconds = 0;
        time->minutes++;

        if(time->minutes >= 60)
        {
            time->minutes = 0;
            time->hours++;

            if(time->hours >= 24)
            {
                time->hours = 0;
            }
        }
    }
}

// blink_pos: 깜빡일 위치 (0~3), -1이면 깜빡임 없음
void lcd_time_display_ex(TimeTypeDef *time, int blink_pos)
{
    LCD_SetCursor(0, 1);

    // 임시 변수
    int h = time->hours;
    char ampm_str[3] = "AM";

    if(h >= 12) {
        strcpy(ampm_str, "PM");
        if (h > 12) h -= 12;
    }
    if (h == 0) h = 12; // 0시는 12 AM으로 표시

    // 1. AM/PM 출력 (blink_pos == 0 이고 blink_state == 1 이면 공백 출력)
    if (blink_pos == POS_AMPM && blink_state == 1) LCD_Print("  ");
    else LCD_Print(ampm_str);

    LCD_Print(" ");

    // 2. 시(Hour) 출력
    if (blink_pos == POS_HOUR && blink_state == 1) LCD_Print("  ");
    else LCD_Printf("%02d", h);

    LCD_Print(":");

    // 3. 분(Minute) 출력
    if (blink_pos == POS_MIN && blink_state == 1) LCD_Print("  ");
    else LCD_Printf("%02d", time->minutes);

    LCD_Print(":");

    // 4. 초(Second) 출력
    if (blink_pos == POS_SEC && blink_state == 1) LCD_Print("  ");
    else LCD_Printf("%02d", time->seconds);

	if (current_state.mode == NORMAL_STATE) {
		if (current_state.alarm_enabled) {
			LCD_Printf(" AL%d", current_state.music_num + 1);
		} else {
			LCD_Print("   ");
		}
	} else if (current_state.mode == ALARM_TIME_SETTING) {
		LCD_SetCursor(12, 1);
		if (blink_pos == POS_ALARM_ONOFF && blink_state == 1) {
			LCD_Print("   ");
		} else {
			if (current_state.alarm_enabled) {
				LCD_Print(" ON");
			} else {
				LCD_Print("OFF");
			}
		}
	}

}

void display_Click_Indicator(void)
{
    char bar[4] = "   ";

    // Long press animation (while button is physically pressed)
    if (g_button_press_time > 0) {
        uint32_t press_duration = HAL_GetTick() - g_button_press_time;
        // Animation fills from the right over 2.5 seconds
        if (press_duration > 833) bar[2] = 0b11111111;
        if (press_duration > 1666) bar[1] = 0b11111111;
        if (press_duration >= 2500) bar[0] = 0b11111111;
    }
    // If button is not pressed, the "   " will clear the area.

    LCD_SetCursor(13, 0); // Draw at columns 13, 14, 15
    LCD_Print(bar);
}

void display_Handler(void)
{
    #define BLINK_SUPPRESS_MS 500 // Suppress blinking for 500ms after key activity

    // Suppress blinking for a short period after joystick activity in setting modes
    if (current_state.mode == TIME_SETTING || current_state.mode == ALARM_TIME_SETTING) {
        if (HAL_GetTick() - last_key_time < BLINK_SUPPRESS_MS) {
            blink_state = 0; // Keep display on
            blink_force = 1; // Prevent blink timer from running
        }
    }

    // Blink timer processing
    if(blink_force == 0){
		blink_timer++;
		if(blink_timer > 2) { // Assuming 100ms cycle, this is 300ms.
			blink_state = !blink_state;
			blink_timer = 0;
		}
    }

    blink_force = 0; // Reset force flag every cycle

    LCD_SetCursor(0, 0);

    switch (current_state.mode)
    {
    case NORMAL_STATE:
    	#define VOLUME_DISPLAY_TIMEOUT 2000 // 2 seconds
		// If the volume has been adjusted recently, show the volume bar instead of the clock title
		if (current_state.display_timeout_timer > 0 && (HAL_GetTick() - current_state.display_timeout_timer < VOLUME_DISPLAY_TIMEOUT))
		{
			// Show volume bar
			char vol_bar[17];
			int i;
			for(i = 0; i < 10; i++)
			{
				if(i < current_state.volume) vol_bar[i] = 0b11111111; // Solid block character
				else vol_bar[i] = '-';
			}
			vol_bar[10] = '\0';

			LCD_SetCursor(0, 0);
			LCD_Printf("Volume: %-10s", ""); // Clear line
			LCD_SetCursor(0, 1);
			LCD_Printf("[%s] %2d", vol_bar, current_state.volume);
		}
		else
		{
			// Show normal clock
			current_state.display_timeout_timer = 0; // Ensure it's off
			LCD_Print("H's Clock     ");
			// 일반 모드에서는 깜빡임 없음 (-1)
			lcd_time_display_ex(&ctime, -1);
		}
        break;

    case TIME_SETTING:
        LCD_Print("Time Setting  ");
        // 현재 edit_pos 위치를 깜빡이게 함
        lcd_time_display_ex(&stime, current_state.edit_pos);
        break;

    case ALARM_TIME_SETTING:
        LCD_Print("Alarm Setting ");
        // 알람 설정 시에도 동일하게 커서 사용 가능
        lcd_time_display_ex(&atime, current_state.edit_pos);
        break;
    case MUSIC_SELECT:
		LCD_Print("Select Music:");
		LCD_SetCursor(0,1);
		LCD_Printf("> %-14s", alarm_music[current_state.music_num].music_title);
		break;
    case ALARM_TRIGGERED_STATE:
		if (blink_state) {
			LCD_SetCursor(0, 0);
			LCD_Print("   Alarm!!!     ");
		} else {
			LCD_SetCursor(0, 0);
			LCD_Print("                ");
		}
		// Also show the current time
		lcd_time_display_ex(&ctime, -1);
		break;
    }
	display_Click_Indicator();
}

void lcd_time_display(TimeTypeDef *time)
{

    LCD_SetCursor(0, 1);
  if(time->hours>=12)
  {
    LCD_Printf("PM");
    time->hours = time->hours - 12;
  }
  else
  {
    LCD_Printf("AM");
  }
  LCD_Printf(" %02d:%02d:%02d",time->hours,time->minutes,time->seconds);

}


// 조이스틱 값을 방향키로 변환 (with auto-repeat)
typedef struct {
    enum CLOCK_MANIPULATE last_key;
    uint32_t first_press_time;
    uint32_t last_repeat_time;
    int is_repeating;
} JoystickRepeat_t;

static JoystickRepeat_t joy_repeat_state = {NO_KEY, 0, 0, 0};

void Process_Joystick_With_Repeat(void)
{
    #define REPEAT_DELAY_MS 500  // Time before repeat starts
    #define REPEAT_RATE_MS  150  // Repeat rate after it starts

    enum CLOCK_MANIPULATE current_key = NO_KEY;
    uint32_t now = HAL_GetTick();

    // If a key from another source (like a button) is being processed, do nothing.
    // However, we still check for joystick release to reset the state.
    if (current_state.key != NO_KEY) {
        if (g_adc_y_value < 2730 && g_adc_y_value > 1365 && g_adc_x_value < 2730 && g_adc_x_value > 1365) {
            joy_repeat_state.last_key = NO_KEY;
            joy_repeat_state.is_repeating = 0;
        }
        return;
    }

    // 1. Detect current hardware key state
    if (g_adc_y_value > 2730) current_key = UP;
    else if (g_adc_y_value < 1365) current_key = DOWN;
    else if (g_adc_x_value > 2730) current_key = RIGHT;
    else if (g_adc_x_value < 1365) current_key = LEFT;

    // 2. Process state changes and repeat logic
    if (current_key != NO_KEY) {
        if (joy_repeat_state.last_key != current_key) {
            // New key pressed: fire once immediately
            current_state.key = current_key;
            joy_repeat_state.last_key = current_key;
            joy_repeat_state.first_press_time = now;
            joy_repeat_state.last_repeat_time = now;
            joy_repeat_state.is_repeating = 0;
        } else {
            // Key is being held
            if (!joy_repeat_state.is_repeating) {
                if (now - joy_repeat_state.first_press_time > REPEAT_DELAY_MS) {
                    joy_repeat_state.is_repeating = 1;
                    joy_repeat_state.last_repeat_time = now;
                    current_state.key = current_key; // Start repeating
                }
            }
            else { // Already repeating
                if (now - joy_repeat_state.last_repeat_time > REPEAT_RATE_MS) {
                    joy_repeat_state.last_repeat_time = now;
                    current_state.key = current_key; // Fire repeat event
                }
            }
        }
    } else { // No key is physically pressed: reset state
        joy_repeat_state.last_key = NO_KEY;
        joy_repeat_state.is_repeating = 0;
    }
}

//============= Timer, GPIO, UART, ADC Callback ==================

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM2)
  {
	  // 1ms tick
	  g_task_flag_1ms = 1;
	  timer_count++;

	  // 100ms tick
	  if ((timer_count % 100) == 0) // Now 100 counts for 100ms
	  {
		  g_task_flag_100ms = 1;
	  }

	  // 1-second tick
	  if (timer_count >= 1000) // Now 1000 counts for 1s
	  {
		  Time_Increment(&ctime);
		  timer_count = 0;

          // Check for alarm condition
          if (current_state.mode == NORMAL_STATE && current_state.alarm_enabled)
          {
              if (ctime.hours == atime.hours && ctime.minutes == atime.minutes && ctime.seconds == atime.seconds)
              {
                  current_state.mode = ALARM_TRIGGERED_STATE;
                  // Start playing the music from the beginning
                  current_state.song_note_index = 0;
                  current_state.note_end_time = HAL_GetTick();
                  Buzzer_Start();
              }
          }
	  }
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == USER_Btn_Pin)
	{
		// This ISR now simply flags a release event after a press.
		if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
		{
			// Button is pressed down. Record the initial time if not already pressed.
			if (g_button_press_time == 0) {
				g_button_press_time = HAL_GetTick();
			}
		}
		else
		{
			// Button is released. Only flag a release if a press was recorded.
			if (g_button_press_time > 0) {
				g_button_last_release_time = HAL_GetTick();
				g_new_release_event = 1; // Signal that a full press-release cycle occurred.
			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
	    /* Transmit one byte with 100 ms timeout */
//	HAL_UART_Transmit(&huart3, &rcv_byte, 1, 100);
	HAL_UART_Receive_IT(&huart3, &rcv_byte, 1);
	/* Receive one byte in interrupt mode */
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc == &hadc1)
	{
		g_adc_x_value = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Start_IT(&hadc2);
	}
	else if(hadc == &hadc2)
	{
		g_adc_y_value = HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Start_IT(&hadc1);

	}
}


//==================== Process State ==========================

// Hybrid Button Processing - Shared Variables & Processing Function
// These are shared between the GPIO ISR and the periodic processing task.
// 'volatile' ensures the compiler always reads them from memory, as they can change unexpectedly.

/**
  * @brief Processes time-based button events (long press, double click timeouts).
  * @note  This function is designed to be called periodically by a fast timer (e.g., 10ms).
  *        It checks the state variables set by the HAL_GPIO_EXTI_Callback.
  */
void Process_Button_Events(void)
{
    uint32_t now = HAL_GetTick();

    // If an event is already pending, wait for it to be processed.
    if (current_state.key != NO_KEY) {
        return;
    }

    // --- Time-based Logic ---

    // Part 1: Real-time long press detection (while button is held down)
    if (g_button_press_time > 0 && !g_new_release_event)
    {
        if (now - g_button_press_time >= LONG_PRESS_MS)
        {
            printf("LC\r\n");
            current_state.key = LONG_SEL;
            g_button_press_time = 0;  // Consume the event to prevent re-triggering.
            g_button_click_count = 0; // A long press is not a click.
        }
    }

    // Part 2: Process a completed press-and-release cycle for short clicks.
    if (g_new_release_event)
    {
        // Check if the press was already handled as a long press above.
        if (g_button_press_time > 0)
        {
            // If press_time is still valid, it means it was not a long press.
            g_button_click_count++;
        }
        // Consume the event by resetting the state variables.
        g_button_press_time = 0;
        g_new_release_event = 0;
    }

    // Part 3: Process counted clicks after the double-click timeout window.
    if (g_button_click_count > 0 && (now - g_button_last_release_time > DBL_CLICK_GAP_MS))
    {
        if (g_button_click_count == 1) {
        	printf("C\r\n");
            current_state.key = SEL; // It was a single click.
        } else { // 2 or more clicks
        	printf("DC\r\n");
            current_state.key = DBL_SEL; // It was a double click.
        }
        g_button_click_count = 0; // Consume the click events.
    }
}


void Process_Normal_State(void)
{
    // Input is now handled in the main loop.

    if (current_state.key == LONG_SEL)
    {
		current_state.mode = ALARM_TIME_SETTING;
		current_state.edit_pos = POS_HOUR; // Start editing at hour
		LCD_Clear();
    }
    else if (current_state.key == DBL_SEL)
    {
    	current_state.mode = MUSIC_SELECT;
    	LCD_Clear();
    }

	// Consume the key, whether it was handled or not, to prevent getting stuck.
	current_state.key = NO_KEY;
}

void Process_Time_Setting(void)
{
    // 키 입력이 없으면 종료
    if (current_state.key == NO_KEY) return;

    switch (current_state.key)
    {
    // 1. 좌우 이동 (커서 변경)
    case RIGHT:
        current_state.edit_pos++;
        if (current_state.edit_pos > POS_SEC) current_state.edit_pos = POS_AMPM;
        blink_timer = 0; blink_state = 1; // Reset blink to hide, to start blinking immediately
        break;

    case LEFT:
        current_state.edit_pos--;
        if (current_state.edit_pos < POS_AMPM) current_state.edit_pos = POS_SEC;
        blink_timer = 0; blink_state = 1; // Reset blink to hide, to start blinking immediately
        break;

    // 2. 상하 이동 (값 변경)
    case UP:
        last_key_time = HAL_GetTick(); // Suppress blinking
        switch(current_state.edit_pos) {
            case POS_AMPM: // 12시간 더하면 AM/PM 토글됨
                stime.hours = (stime.hours + 12) % 24;
                break;
            case POS_HOUR:
                stime.hours++;
                if(stime.hours >= 24) stime.hours = 0;
                break;
            case POS_MIN:
                stime.minutes++;
                if(stime.minutes >= 60) stime.minutes = 0;
                break;
            case POS_SEC:
                stime.seconds++;
                if(stime.seconds >= 60) stime.seconds = 0;
                break;
        }
        break;

    case DOWN:
        last_key_time = HAL_GetTick(); // Suppress blinking
        switch(current_state.edit_pos) {
            case POS_AMPM:
                stime.hours = (stime.hours + 12) % 24;
                break;
            case POS_HOUR:
                stime.hours--;
                if(stime.hours < 0) stime.hours = 23;
                break;
            case POS_MIN:
                stime.minutes--;
                if(stime.minutes < 0) stime.minutes = 59;
                break;
            case POS_SEC:
                stime.seconds--;
                if(stime.seconds < 0) stime.seconds = 59;
                break;
        }
        break;

    // 3. 설정 완료 (SEL 버튼)
    case SEL:
    case DBL_SEL:
    case LONG_SEL:
        ctime = stime; // 설정값 적용
        Save_And_Display_Message();
        break;
    }

    // 키 처리 후 초기화 (연속 입력 방지용, 혹은 딜레이 필요)
    current_state.key = NO_KEY;
}

void Process_Alarm_Setting(void)
{
    // 키 입력이 없으면 종료
    if (current_state.key == NO_KEY) return;

    switch (current_state.key)
    {
    // 1. 좌우 이동 (커서 변경)
    case RIGHT:
        current_state.edit_pos++;
        if (current_state.edit_pos > POS_ALARM_ONOFF) current_state.edit_pos = POS_AMPM;
        blink_timer = 0; blink_state = 1; // Reset blink to hide, to start blinking immediately
        break;

    case LEFT:
        current_state.edit_pos--;
        if (current_state.edit_pos < POS_AMPM) current_state.edit_pos = POS_ALARM_ONOFF;
        blink_timer = 0; blink_state = 1; // Reset blink to hide, to start blinking immediately
        break;

    // 2. 상하 이동 (값 변경)
    case UP:
        last_key_time = HAL_GetTick(); // Suppress blinking
        if(current_state.edit_pos == POS_ALARM_ONOFF) {
			current_state.alarm_enabled = !current_state.alarm_enabled;
		}
		else
		{
			switch(current_state.edit_pos) {
				case POS_AMPM: // 12시간 더하면 AM/PM 토글됨
					atime.hours = (atime.hours + 12) % 24;
					break;
				case POS_HOUR:
					atime.hours++;
					if(atime.hours >= 24) atime.hours = 0;
					break;
				case POS_MIN:
					atime.minutes++;
					if(atime.minutes >= 60) atime.minutes = 0;
					break;
				case POS_SEC:
					atime.seconds++;
					if(atime.seconds >= 60) atime.seconds = 0;
					break;
			}
		}
        break;

    case DOWN:
        last_key_time = HAL_GetTick(); // Suppress blinking
		if(current_state.edit_pos == POS_ALARM_ONOFF) {
			current_state.alarm_enabled = !current_state.alarm_enabled;
		}
		else
		{
			switch(current_state.edit_pos) {
				case POS_AMPM:
					atime.hours = (atime.hours + 12) % 24;
					break;
				case POS_HOUR:
					atime.hours--;
					if(atime.hours < 0) atime.hours = 23;
					break;
				case POS_MIN:
					atime.minutes--;
					if(atime.minutes < 0) atime.minutes = 59;
					break;
				case POS_SEC:
					atime.seconds--;
					if(atime.seconds < 0) atime.seconds = 59;
					break;
			}
		}
        break;

    // 3. 설정 완료 (SEL 버튼)
    case SEL:
    case DBL_SEL:
    case LONG_SEL:
    	Save_And_Display_Message();
        break;
    }

    // 키 처리 후 초기화 (연속 입력 방지용, 혹은 딜레이 필요)
    current_state.key = NO_KEY;
}

void Process_Music_Select(void)
{
    if (current_state.key == NO_KEY) return;

    switch (current_state.key)
    {
        case UP:
            current_state.music_num++;
            if (current_state.music_num >= 3) // 3 songs total
            {
                current_state.music_num = 0;
            }
            break;
        case DOWN:
            current_state.music_num--;
            if (current_state.music_num < 0)
            {
                current_state.music_num = 2; // Loop back to the last song
            }
            break;
        case SEL:
        case DBL_SEL:
        case LONG_SEL:
        	Save_And_Display_Message();
            break;
    }
    // 키 처리 후 초기화 (연속 입력 방지용, 혹은 딜레이 필요)
    current_state.key = NO_KEY;
}

void Adjust_Volume(enum CLOCK_MANIPULATE direction)
{
    if (direction == UP)
    {
        current_state.volume++;
        if (current_state.volume > 10) current_state.volume = 10;
    }
    else // DOWN
    {
        current_state.volume--;
        if (current_state.volume < 0) current_state.volume = 0;
    }
    // Set the display timeout timer to now, to show the volume bar
    current_state.display_timeout_timer = HAL_GetTick();
}

void Process_Alarm_Triggered(void)
{
    // Any key press stops the alarm
    if (current_state.key == SEL)
    {
        Buzzer_Stop(); // Stop the sound
        current_state.mode = NORMAL_STATE; // Go back to normal
        current_state.key = NO_KEY; // Consume the key
        LCD_Clear();
    }
}

void Process_Alarm_Playback(void)
{
    uint32_t now = HAL_GetTick();

    // Time to play the next note?
    if (now >= current_state.note_end_time)
    {
        const Note* current_song = songs[current_state.music_num];
        Note current_note = current_song[current_state.song_note_index];

        // Play the note
        Buzzer_PlayNote(current_note.frequency);

        // Set the end time for this note
        current_state.note_end_time = now + current_note.duration_ms;

        // Move to the next note
        current_state.song_note_index++;

        // End of song? Check for the {REST, 0} marker which indicates the end.
        if (current_song[current_state.song_note_index].duration_ms == 0 && current_song[current_state.song_note_index].frequency == REST)
        {
            // Loop back to the beginning to repeat the song
            current_state.song_note_index = 0;
        }
    }
}

void Save_And_Display_Message(void)
{
	// Show "Saving..." message first
	LCD_Clear();
	LCD_SetCursor(0, 0);
	LCD_Print("Saving...");

	// Perform the blocking save operation.
	// The message will be visible on the LCD while the MCU is busy here.
	Save_Settings_To_Flash();

	// Done, go back to normal state
	LCD_Clear();
	current_state.mode = NORMAL_STATE;
}


//===================== printf Description ==============================

#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  Load_Settings_From_Flash();

  LCD_Init(&hi2c1);
  LCD_Backlight(1);
  LCD_SetCursor(0, 0);
  LCD_Print("H's Clock");
  HAL_TIM_Base_Init(&htim2);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart3, &rcv_byte, 1);
  HAL_ADC_Start_IT(&hadc1);

  current_state.key = NO_KEY;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  int needs_display_update = 0;

	  // --- 1ms Task ---
	  // High-frequency tasks like input processing
	  if (g_task_flag_1ms)
	  {
		  g_task_flag_1ms = 0;

		  // 1. First, process the current state's periodic tasks
		  if (current_state.mode == ALARM_TRIGGERED_STATE)
		  {
			  Process_Alarm_Playback();
		  }

		  // 2. Process physical inputs
		  Process_Button_Events();
		  Process_Joystick_With_Repeat();

		  // Handle low-priority joystick inputs for NORMAL_STATE
		  if (current_state.mode == NORMAL_STATE && (current_state.key == UP || current_state.key == DOWN))
		  {
			  Adjust_Volume(current_state.key);
			  current_state.key = NO_KEY; // Consume the key
			  needs_display_update = 1;
		  }

		  // 3. If a key event was generated, process it based on the current state
		  if (current_state.key != NO_KEY)
		  {
			   switch (current_state.mode)
			   {
				   case NORMAL_STATE:
					   Process_Normal_State();
					   break;
				   case TIME_SETTING:
					   Process_Time_Setting();
					   break;
				   case ALARM_TIME_SETTING:
					  Process_Alarm_Setting();
					   break;
				   case MUSIC_SELECT:
					  Process_Music_Select();
					   break;
				   case ALARM_TRIGGERED_STATE:
					  Process_Alarm_Triggered();
					   break;
			   }
			   // An action was taken based on input, so update the display immediately.
			   needs_display_update = 1;
		  }
	  }

	  // --- 100ms Task ---
	  // Low-frequency tasks like regular display updates
	  if (g_task_flag_100ms)
	  {
		  g_task_flag_100ms = 0;
		  needs_display_update = 1; // Schedule a regular display update.
	  }

	  // --- Display Update ---
	  // Consolidate all display updates here.
	  if (needs_display_update)
	  {
		  display_Handler();
		  needs_display_update = 0; // Consume the flag
	  }

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83; // For 1ms tick (84MHz / (83+1) = 1MHz counter clock)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;  // For 1ms tick (1MHz / (999+1) = 1kHz interrupt)
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Flash에 설정값 저장 (반드시 설정을 마친 후 호출)
void Save_Settings_To_Flash(void)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    // 1. 데이터 준비
    NVitemTypeDef data_to_save;
    data_to_save.magic_num = MAGIC_NUM;
    // Process_Time_Setting에서 ctime = stime으로 설정했으므로,
    // 현재 설정된 시간이 ctime에 들어있음.
    data_to_save.setting_time = ctime;
    data_to_save.alarm_time = atime;
    data_to_save.alarm_music_num = current_state.music_num;
	data_to_save.alarm_enabled = current_state.alarm_enabled;
	data_to_save.volume = current_state.volume;


	// 인터럽트 비활성화
	__disable_irq();

    HAL_FLASH_Unlock();

    // 2. 섹터 지우기 (Sector 23)
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR_23;
    EraseInitStruct.NbSectors = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    {
        HAL_FLASH_Lock();
		__enable_irq(); // 인터럽트 다시 활성화
        return;
    }

    // 3. 데이터 쓰기
    uint32_t address = ADDR_FLASH_SECTOR_23;
    uint32_t *pData = (uint32_t *)&data_to_save;
    int size = sizeof(NVitemTypeDef) / 4;
    if (sizeof(NVitemTypeDef) % 4 != 0) size++;

    for (int i = 0; i < size; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + (i*4), pData[i]) != HAL_OK)
        {
             break;
        }
    }

    HAL_FLASH_Lock();
	__enable_irq(); // 인터럽트 다시 활성화
}

// 부팅 시 Flash에서 데이터 불러오기
void Load_Settings_From_Flash(void)
{
    if (nv_items->magic_num == MAGIC_NUM)
    {
        // 저장된 데이터가 있음
        ctime = nv_items->setting_time; // 저장된 시간으로 복구
        atime = nv_items->alarm_time;
        current_state.music_num = nv_items->alarm_music_num;
		current_state.alarm_enabled = nv_items->alarm_enabled;
		current_state.volume = nv_items->volume;

        current_state.mode = NORMAL_STATE;
    }
    else
    {
        // 초기 데이터 (공장 초기화 상태)
        ctime.hours = 12; ctime.minutes = 0; ctime.seconds = 0;
        atime.hours = 6; atime.minutes = 0; atime.seconds = 0;
        current_state.music_num = 0;
		current_state.alarm_enabled = 0;
		current_state.volume = 5; // Default volume

        stime = ctime;
		current_state.mode = TIME_SETTING;
		current_state.edit_pos = POS_HOUR;
    }
    // 설정용 변수 초기화
    current_state.display_timeout_timer = 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
