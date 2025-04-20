/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Referee_System.h
  * @brief          : The header file of Referee_System.c
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REFEREE_INFO_H
#define REFEREE_INFO_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* Exported defines ----------------------------------------------------------*/

#define REFEREE_RXFRAME_LENGTH  136   //frame_header 5bytes , cmd_id 2bytes , data_max 127bytes , crc16 2bytes = 136bytes 

/**
 * @brief Referee Communication protocol format
 */
#define FrameHeader_Length    5U   /*!< the length of frame header */
#define CMDID_Length          2U   /*!< the length of CMD ID */
#define CRC16_Length          2U   /*!< the length of CRC ID */

/**
 * @brief Cmd id
 */
#define GAME_STATUS_ID                    0x0001U  /* game status data */
#define GAME_RESULT_ID                    0x0002U  /* game result data */
#define GAME_ROBOT_HP_ID                  0x0003U  /* robot HP data */
#define EVENET_DATA_ID                    0x0101U  /* site event data */
#define REFEREE_WARNING_ID                0x0104U  /* referee warning data */
#define DART_INFO_ID                      0x0105U  /* dart shoot data */
#define ROBOT_STATUS_ID                   0x0201U  /* robot status data */
#define POWER_HEAT_DATA_ID                0x0202U  /* real power heat data */
#define ROBOT_POS_ID                      0x0203U  /* robot position data */
#define BUFF_ID                           0x0204U  /* robot buff data */
#define HURT_DATA_ID                      0x0206U  /* robot hurt data */
#define SHOOT_DATA_ID                     0x0207U  /* real robo t shoot data */
#define PROJECTILE_ALLOWANCE_ID           0x0208U  /* bullet remain data */
#define RFID_STATUS_ID                    0x0209U  /* RFID status data */
#define DART_CLIENT_CMD_ID                0x020AU  /* DART Client cmd data */
#define GROUND_ROBOT_POSITION_ID          0x020BU  /* ground robot position */
#define RADAR_MARAKING_DATA_ID            0x020CU  /* Radar marking progress*/
#define SENTRY_INFO_ID                    0x020DU  /* SENTRY make autonomous decisions*/
#define RADAR_INFO_ID                     0x020EU  /* RADAR make autonomous decisions*/
#define ROBOT_INTERACTION_DATA_ID         0x0301U  /* robot interactive data */
#define MAP_COMMAND_ID                    0x0303U  /* mini map interactive data */
#define MAP_ROBOT_DATA_ID                 0x0305U  /* mini map receive data */
#define MAP_DATA_ID                       0x0307U  /* mini map Auto Robot path */
#define CUSTOM_INFO_ID                    0x0308U  /* mini map robot  path */
/**
 * @brief Robot id
 */
#define ROBOT_RED_HERO_ID                 0x0001U
#define ROBOT_RED_ENGINEER_ID             0x0002U
#define ROBOT_RED_3_INFANTEY_ID           0x0003U
#define ROBOT_RED_4_INFANTEY_ID           0x0004U
#define ROBOT_RED_5_INFANTEY_ID           0x0005U
#define ROBOT_RED_AERIAL_INFANTEY_ID      0x0006U
#define ROBOT_RED_SENTEY_INFANTEY_ID      0x0007U
#define ROBOT_RED_DART_INFANTEY_ID        0x0008U
#define ROBOT_RED_RADAR_INFANTEY_ID       0x0009U
#define ROBOT_RED_OUTPOST_INFANTEY_ID     0x0010U
#define ROBOT_RED_BASE_INFANTEY_ID        0x0011U

#define ROBOT_BLUE_HERO_ID                0x0101U
#define ROBOT_BLUE_ENGINEER_ID            0x0102U
#define ROBOT_BLUE_3_INFANTEY_ID          0x0103U
#define ROBOT_BLUE_4_INFANTEY_ID          0x0104U
#define ROBOT_BLUE_5_INFANTEY_ID          0x0105U
#define ROBOT_BLUE_AERIAL_INFANTEY_ID     0x0106U
#define ROBOT_BLUE_SENTEY_INFANTEY_ID     0x0107U
#define ROBOT_BLUE_DART_INFANTEY_ID       0x0108U
#define ROBOT_BLUE_RADAR_INFANTEY_ID      0x0109U
#define ROBOT_BLUE_OUTPOST_INFANTEY_ID    0x0110U
#define ROBOT_BLUE_BASE_INFANTEY_ID       0x0111U

/**
 * @brief client id
 */
#define CLIENT_RED_HERO_ID                0x0101U
#define CLIENT_RED_ENGINEER_ID            0x0102U
#define CLIENT_RED_3_INFANTEY_ID          0x0103U
#define CLIENT_RED_4_INFANTEY_ID          0x0104U
#define CLIENT_RED_5_INFANTEY_ID          0x0105U
#define CLIENT_RED_AERIAL_INFANTEY_ID     0x0106U

#define CLIENT_BLUE_HERO_ID               0x0165U
#define CLIENT_BLUE_ENGINEER_ID           0x0166U
#define CLIENT_BLUE_3_INFANTEY_ID         0x0167U
#define CLIENT_BLUE_4_INFANTEY_ID         0x0168U
#define CLIENT_BLUE_5_INFANTEY_ID         0x0169U
#define CLIENT_BLUE_AERIAL_INFANTEY_ID    0x016AU

/* Exported types ------------------------------------------------------------*/
/* cancel byte alignment */
#pragma  pack(1)

/**
 * @brief typedef structure that contains the information of frame header
 */
typedef struct 
{
  uint8_t  SOF;           /*!< Data frame start byte, fixed value is 0xA5 */
  uint16_t Data_Length;   /*!< the length of data in the data frame */
  uint8_t  Seq;           /*!< package serial number */
  uint8_t  CRC8;          /*!< Frame header CRC8 checksum */
}FrameHeader_TypeDef;


/**
 * @brief typedef structure that contains the information of game status, id: 0x0001U
 */
typedef struct          
{
  /**
   * @brief the type of game,
            1:RMUC,
            2:RMUT,
            3:RMUA,
            4:RMUL,3v3,
            5:RMUL,1v1,
   */
	 uint8_t game_type : 4;        
	 uint8_t game_progress : 4;	    /*!< the progress of game */
	 uint16_t stage_remain_time;	    /*!< remain time of real progress */
   uint64_t SyncTimeStamp;         /*!< unix time */
       
}game_status_t;

/**
 * @brief typedef structure that contains the information of game result, id: 0x0002U
 */
typedef struct
{
  /**
   * @brief the result of game
            0:draw
            1:Red wins
            2:Blue wins
   */
 uint8_t winner;
}game_result_t;

/**
 * @brief typedef structure that contains the information of robot HP data, id: 0x0003U
 */
typedef struct
{
  uint16_t red_1_robot_HP;   /*!< Red Hero HP */
  uint16_t red_2_robot_HP;   /*!< Red Engineer HP */
  uint16_t red_3_robot_HP;   /*!< Red 3 Infantry HP */
  uint16_t red_4_robot_HP;   /*!< Red 4 Infantry HP */
  uint16_t red_reserved;          
  uint16_t red_7_robot_HP;   /*!< Red Sentry HP */
  uint16_t red_outpost_HP;   /*!< Red Outpost HP */
  uint16_t red_base_HP;      /*!< Red Base HP */

  uint16_t blue_1_robot_HP;   /*!< Blue Hero HP */
  uint16_t blue_2_robot_HP;   /*!< Blue Engineer HP */
  uint16_t blue_3_robot_HP;   /*!< Blue 3 Infantry HP */
  uint16_t blue_4_robot_HP;   /*!< Blue 4 Infantry HP */
  uint16_t blue_reserved;  
  uint16_t blue_7_robot_HP;   /*!< Blue Sentry HP */
  uint16_t blue_outpost_HP;   /*!< Blue Outpost HP */
  uint16_t blue_base_HP;      /*!< Blue Base HP */
}game_robot_HP_t;

/**
 * @brief typedef structure that contains the information of site event data, id: 0x0101U
 */
typedef union
{
    /**
     * @brief the event of site
	            bits 0-2:
              bit 0:  Status of the supply zone that does not overlap with the exchange zone, 1 for occupied
              bit 1:  Status of the supply zone that overlaps with the exchange zone, 1 for occupied
              bit 2:  Status of the supply zone, 1 for occupied (applicable only to RMUL)
	
	            bits 3-5: Status of the energy mechanism:
              bit 3:    Status of the small energy mechanism, 1 for activated
              bit 4:    Status of the large energy mechanism, 1 for activated
              bit 5-6:  Status of the central highland, 1 for occupied by own side, 2 for occupied by the opponent
              bit 7-8:  Status of the trapezoidal highland, 1 for occupied
              bit 9-17: Time of the opponent's last dart hit on the own side's outpost or base (0-420, default is 0 at the start)
	            bit 18-20:  Specific target of the opponent's last dart hit on the own side's outpost or base, default is 0 at the start, 
	                        1 for hitting the outpost, 2 for hitting the fixed target on the base, 3 for hitting the random fixed target on the base, 
	                        4 for hitting the random moving target on the base
	            bit 21-22: Status of the central gain point, 0 for unoccupied, 1 for occupied by own side, 
	                       2 for occupied by the opponent, 3 for occupied by both sides (applicable only to RMUL)               
              bit 23-31: Reserved
                        
    */
    uint32_t  event_data;
}event_data_t;

/**
 * @brief typedef structure that contains the warning  of Referee , id: 0x0104U
 */
typedef struct 
{ 
	/**
   * @brief the type of game,
            1: Double Yellow Card,
            2: Yellow Card,
            3:RMUA,
            4:RMUL,3v3,
            5:RMUL,1v1,
   */
  uint8_t level; 
  uint8_t offending_robot_id; 
  uint8_t count; 
}referee_warning_t; 

/**
 * @brief typedef structure that contains the information of dart, id: 0x0105U
 */
typedef  struct
{
 
 uint8_t dart_remaining_time;/* The remaining time for our side's dart launcher, in seconds.*/
 
 uint16_t dart_info;
  /**
   * @brief dart_info
   *       bit 0-2 The most recent target hit by our side's dart is defaulted to 0 at the start, 
	                 where 1 indicates hitting the outpost, 2 indicates hitting the base's fixed target, 
	                 3 indicates hitting the base's random fixed target, and 4 indicates hitting the base's random moving target.
   *       bit 3-5 The cumulative hit count of the opponent's recently hit target (defaulting to 0 at the start, with a maximum of 4)
   *       bit 6-7 The currently selected target for the dart (defaulting to 0 at the start or when not selected/selecting the outpost, 
	                 1 for selecting the base's fixed target, 2 for selecting the base's random fixed target, 3 for selecting the base's random moving target)
   *       bit 8-15 Reserved
   */
}dart_info_t;
/**
 * @brief typedef structure that contains the information of robot status, id: 0x0201U
 */
typedef struct
{
  /**
   * @brief robot id
   *             0: robot none
   *             1: red hero
   *             2: red engineer
   *         3/4/5: red infantry
   *             6: red aerial
   *             7: red sentry
   *             8: red dart
   *             9: red radar station
   *           101: blue hero
   *           102: blue engineer
   *   103/104/105: blue infantry
   *           106: blue aerial
   *           107: blue sentry
   *           108: blue dart
   *           109: blue radar station
   */
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t current_HP;
  uint16_t maximum_HP;

  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_barrel_heat_limit;
  uint16_t chassis_power_limit;

  uint8_t mains_power_gimbal_output : 1;
  uint8_t mains_power_chassis_output : 1;
  uint8_t mains_power_shooter_output : 1;
} robot_status_t;

/**
 * @brief typedef structure that contains the information of power heat data, id: 0x0202U
 */
typedef struct
{
  uint16_t buffer_energy;
  uint16_t shooter_17mm_1_barrel_heat;
  uint16_t shooter_17mm_2_barrel_heat;
  uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

/**
 * @brief typedef structure that contains the information of robot position data, id: 0x0203U
 */
typedef struct
{
  float x;      /* position x coordinate, unit: m */
  float y;      /* position y coordinate, unit: m */
  float angle;  /* Position muzzle, unit: degrees */
} robot_pos_t;

/**
 * @brief typedef structure that contains the information of robot buff data, id: 0x0204U
 */
typedef struct
{
   uint8_t recovery_buff;       /* Robot healing gain (percentage, value of 10 represents 10% of the maximum healing volume per second) */
   uint8_t cooling_buff;        /* Robot shooting heat cooling rate (direct value, a value of 5 indicates 5 times cooling; the fixed heat cooling gain provided by the fortress gain point is temporarily not applicable) */
   uint8_t defence_buff;        /* Robot defense gain (percentage, a value of 50 represents a 50% defense gain) */
   uint8_t vulnerability_buff;  /* Robot negative defense gain (percentage, a value of 30 represents -30% defense gain) */
   uint16_t attack_buff;        /* Robot attack gain (percentage, a value of 50 represents 50% attack gain) */
	
	 /**
	 * @brief remaining_energy
	 *	bit0-4: Feedback on the remaining energy value of the robot, identifying the proportion of the remaining energy value of the robot in hexadecimal, only
	 *	Feedback when the remaining energy of the robot is less than 50%, and default feedback for the rest is 0x32.
	 *	bit0: When the remaining energy is greater than 50%, it is 1, and in other cases, it is 0
	 *	bit 1: When the remaining energy is greater than 30%, it is 1, and in other cases, it is 0
	 *	bit2: When the remaining energy is greater than 15%, it is 1, and in other cases, it is 0
	 *	bit3: When the remaining energy is greater than 5%, it is 1, and in other cases it is 0 Bit4: when the remaining energy is greater than 1%
	 *	When is 1, in other cases it is 0
	 */
	 uint8_t remaining_energy;
}buff_t;
/**
 * @brief typedef structure that contains the information of robot hurt, id: 0x0206U
 */
typedef struct
{
 uint8_t armor_id : 4; /* hurt armor id */
  /**
   * @brief hurt type
   *        0: armor hurt
   *        1: module offline
   *        2: over fire rate
   *        3: over fire heat
   *        4: over chassis power
   *        5: armor bump
   */
 uint8_t HP_deduction_reason : 4;
}hurt_data_t;

/**
 * @brief typedef structure that contains the information of real shoot data, id: 0x0207U
 */
typedef  struct
{
  uint8_t bullet_type;  /* 1:17mm 2:42mm */
  uint8_t shooter_number; 
	/* 
	     1:First 17mm Shoot 
	     2:Second 17mm Shoot 
	     3:42mm Shoot
	*/
  uint8_t launching_frequency;  /* Hz */
  float initial_speed;   /* m/s */
}shoot_data_t;

/**
 * @brief typedef structure that contains the information of bullet remaining number, id: 0x0208U
 */
typedef  struct
{
  uint16_t projectile_allowance_17mm; 
  uint16_t projectile_allowance_42mm;  
  uint16_t remaining_gold_coin; 
}projectile_allowance_t;;

/**
 * @brief typedef structure that contains the information of RFID status, id: 0x0209U
 */
typedef struct
{
 uint32_t rfid_status;
}rfid_status_t;

/**
 * @brief typedef structure that contains the information of dart client data, id: 0x020AU
 */
typedef  struct
{
 uint8_t dart_launch_opening_status;
 uint8_t reserved;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
}dart_client_cmd_t;

/**
 * @brief typedef structure that contains the information of robot position in mimi map, id: 0x020BU
 */
typedef struct
{
  float hero_x;
  float hero_y;
  float engineer_x;
  float engineer_y;
  float standard_3_x;
  float standard_3_y;
  float standard_4_x;
  float standard_4_y;
}ground_robot_position_t;

/**
 * @brief typedef structure that contains the information of robot mark, id: 0x020C
 */
typedef struct
{
  uint8_t mark_progress; 
}radar_mark_data_t;

/**
 * @brief typedef structure that contains the information of robot mark, id: 0x020D
 */
typedef  struct
{
  uint32_t sentry_info; 
  uint16_t sentry_info_2; 
} sentry_info_t;

/**
 * @brief typedef structure that contains the information of radar, id: 0x020E
 */
typedef  struct
{
  uint8_t radar_info;
}radar_info_t;
/**
 * @brief typedef structure that contains the information of custom controller interactive, id: 0x0301U
 */
typedef struct{ 
	uint16_t data_cmd_id;
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[113];
}robot_interaction_data_t;
/**
 * @brief typedef structure that contains the information of client transmit data, id: 0x0303U
 */
typedef struct
{
  /**
   * @brief target position coordinate, is 0 when transmit target robot id
   */
  float target_position_x;
  float target_position_y;
  uint8_t cmd_keyboard;
  uint16_t target_robot_id;   
  uint16_t cmd_source;
	
}map_command_t;
/**
 * @brief typedef structure that contains the information of client receive data, id: 0x0305U
 */
typedef struct
{
	uint16_t hero_position_x;
	uint16_t hero_position_y;
	uint16_t engineer_position_x;
	uint16_t engineer_position_y;
	uint16_t infantry_3_position_x;
	uint16_t infantry_3_position_y;
	uint16_t infantry_4_position_x;
	uint16_t infantry_4_position_y;
	uint16_t infantry_5_position_x;
	uint16_t infantry_5_position_y;
	uint16_t sentry_position_x;
	uint16_t sentry_position_y;
}map_robot_data_t;
/**
 * @brief typedef structure that contains the information of sentry path, id: 0x0307U
 */
typedef struct
{
  /**
   * @brief  sentry status
   *         1: attack on target point
   *         2: defend on target point
   *         3: move to target point
   */
  uint8_t intention;
  uint16_t start_position_x;
  uint16_t start_position_y;
  int8_t delta_x[49];
  int8_t delta_y[49];
	uint16_t sender_id;
}map_data_t;
/**
 * @brief typedef structure that contains the information of sentry path, id: 0x0308U
 */
typedef  struct
{
	
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];

}custom_info_t;
/**
 * @brief typedef structure that contains the information of Referee
 */
typedef struct 
{
  uint8_t Index;
  uint16_t DataLength;
  
#ifdef GAME_STATUS_ID
  game_status_t game_status;
#endif

#ifdef GAME_RESULT_ID
  game_result_t game_result;
#endif

#ifdef  GAME_ROBOT_HP_ID
	game_robot_HP_t game_robot_HP;
#endif	
	
#ifdef EVENET_DATA_ID
  event_data_t event_data;
#endif

#ifdef REFEREE_WARNING_ID
    referee_warning_t referee_warning;
#endif

#ifdef DART_INFO_ID
  dart_info_t dart_info;
#endif

#ifdef ROBOT_STATUS_ID
  robot_status_t robot_status;
#endif

#ifdef POWER_HEAT_DATA_ID
  power_heat_data_t power_heat_data;
#endif

#ifdef ROBOT_POS_ID
  robot_pos_t  robot_pos;
#endif

#ifdef BUFF_ID
  buff_t  buff;
#endif

#ifdef HURT_DATA_ID
  hurt_data_t  hurt_data;
#endif

#ifdef SHOOT_DATA_ID
  shoot_data_t  shoot_data;
#endif

#ifdef PROJECTILE_ALLOWANCE_ID
  projectile_allowance_t projectile_allowance;
#endif

#ifdef  RFID_STATUS_ID
    rfid_status_t rfid_status;
#endif

#ifdef DART_CLIENT_CMD_ID
    dart_client_cmd_t  dart_client_cmd;
#endif

#ifdef GROUND_ROBOT_POSITION_ID
    ground_robot_position_t  ground_robot_position;
#endif 

#ifdef RADAR_MARAKING_DATA_ID
    radar_mark_data_t  radar_mark_data;
#endif

#ifdef SENTRY_INFO_ID
    sentry_info_t  sentry_info;
#endif

#ifdef RADAR_INFO_ID
    radar_info_t  radar_info;
#endif

#ifdef ROBOT_INTERACTION_DATA_ID
    robot_interaction_data_t robot_interaction_data;
#endif

#ifdef MAP_COMMAND_ID
    map_command_t map_command;
#endif

#ifdef MAP_ROBOT_DATA_ID
    map_robot_data_t  map_robot_data;
#endif

#ifdef MAP_DATA_ID
    map_data_t map_data;
#endif

#ifdef CUSTOM_INFO_ID
    custom_info_t custom_info;
#endif
}Referee_System_Info_TypeDef;

/* restore byte alignment */
#pragma  pack()

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief Referee_RxDMA MultiBuffer
 */
extern uint8_t Referee_System_Info_MultiRx_Buf[2][REFEREE_RXFRAME_LENGTH];
/**
 * @brief Referee structure variable
 */
extern Referee_System_Info_TypeDef Referee_Info;
/* Exported functions prototypes ---------------------------------------------*/
extern void Referee_System_Frame_Update(uint8_t *Buff);

#endif //REFEREE_INFO_H
