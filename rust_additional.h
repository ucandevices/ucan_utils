#ifndef RUST_ADDITIONAL
#define RUST_ADDITIONAL

#define FDCAN1
// #define USE_HAL_FDCAN_REGISTER_CALLBACKS 1
#define __IO
typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

typedef uint32_t FDCAN_GlobalTypeDef;


typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;


/**
  * @brief  HAL Status structures definition
  */
typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

/**
  * @brief  HAL Lock structures definition
  */
typedef enum
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U
} HAL_LockTypeDef;

#endif