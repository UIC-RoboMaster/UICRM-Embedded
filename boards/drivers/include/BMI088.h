/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#pragma once
#include "bsp_error_handler.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_thread.h"
#include "main.h"

#define BMI088_ACC_CHIP_ID 0x00  // the register is  " Who am I "
#define BMI088_ACC_CHIP_ID_VALUE 0x1E

#define BMI088_ACC_ERR_REG 0x02
#define BMI088_ACCEL_CONGIF_ERROR_SHFITS 0x2
#define BMI088_ACCEL_CONGIF_ERROR (1 << BMI088_ACCEL_CONGIF_ERROR_SHFITS)
#define BMI088_FATAL_ERROR_SHFITS 0x0
#define BMI088_FATAL_ERROR (1 << BMI088_FATAL_ERROR)

#define BMI088_ACC_STATUS 0x03
#define BMI088_ACCEL_DRDY_SHFITS 0x7
#define BMI088_ACCEL_DRDY (1 << BMI088_ACCEL_DRDY_SHFITS)

#define BMI088_ACCEL_XOUT_L 0x12
#define BMI088_ACCEL_XOUT_M 0x13
#define BMI088_ACCEL_YOUT_L 0x14
#define BMI088_ACCEL_YOUT_M 0x15
#define BMI088_ACCEL_ZOUT_L 0x16
#define BMI088_ACCEL_ZOUT_M 0x17

#define BMI088_SENSORTIME_DATA_L 0x18
#define BMI088_SENSORTIME_DATA_M 0x19
#define BMI088_SENSORTIME_DATA_H 0x1A

#define BMI088_ACC_INT_STAT_1 0x1D
#define BMI088_ACCEL_DRDY_INTERRUPT_SHFITS 0x7
#define BMI088_ACCEL_DRDY_INTERRUPT (1 << BMI088_ACCEL_DRDY_INTERRUPT_SHFITS)

#define BMI088_TEMP_M 0x22

#define BMI088_TEMP_L 0x23

#define BMI088_ACC_CONF 0x40
#define BMI088_ACC_CONF_MUST_Set 0x80
#define BMI088_ACC_BWP_SHFITS 0x4
#define BMI088_ACC_OSR4 (0x0 << BMI088_ACC_BWP_SHFITS)
#define BMI088_ACC_OSR2 (0x1 << BMI088_ACC_BWP_SHFITS)
#define BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS)

#define BMI088_ACC_ODR_SHFITS 0x0
#define BMI088_ACC_12_5_HZ (0x5 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_25_HZ (0x6 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_50_HZ (0x7 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_100_HZ (0x8 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_200_HZ (0x9 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_400_HZ (0xA << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)

#define BMI088_ACC_RANGE 0x41

#define BMI088_ACC_RANGE_SHFITS 0x0
#define BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_6G (0x1 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_12G (0x2 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_24G (0x3 << BMI088_ACC_RANGE_SHFITS)

#define BMI088_INT1_IO_CTRL 0x53
#define BMI088_ACC_INT1_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS)
#define BMI088_ACC_INT1_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT1_GPIO_OD (0x1 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT1_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)
#define BMI088_ACC_INT1_GPIO_HIGH (0x1 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)

#define BMI088_INT2_IO_CTRL 0x54
#define BMI088_ACC_INT2_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT2_IO_ENABLE (0x1 << BMI088_ACC_INT2_IO_ENABLE_SHFITS)
#define BMI088_ACC_INT2_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT2_GPIO_PP (0x0 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT2_GPIO_OD (0x1 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT2_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT2_GPIO_LOW (0x0 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)
#define BMI088_ACC_INT2_GPIO_HIGH (0x1 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)

#define BMI088_INT_MAP_DATA 0x58
#define BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS 0x6
#define BMI088_ACC_INT2_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS)
#define BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS 0x2
#define BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)

#define BMI088_ACC_SELF_TEST 0x6D
#define BMI088_ACC_SELF_TEST_OFF 0x00
#define BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL 0x0D
#define BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL 0x09

#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_PWR_SUSPEND_MODE 0x03
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00

#define BMI088_ACC_PWR_CTRL 0x7D
#define BMI088_ACC_ENABLE_ACC_OFF 0x00
#define BMI088_ACC_ENABLE_ACC_ON 0x04

#define BMI088_ACC_SOFTRESET 0x7E
#define BMI088_ACC_SOFTRESET_VALUE 0xB6

#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F

#define BMI088_GYRO_X_L 0x02
#define BMI088_GYRO_X_H 0x03
#define BMI088_GYRO_Y_L 0x04
#define BMI088_GYRO_Y_H 0x05
#define BMI088_GYRO_Z_L 0x06
#define BMI088_GYRO_Z_H 0x07

#define BMI088_GYRO_INT_STAT_1 0x0A
#define BMI088_GYRO_DYDR_SHFITS 0x7
#define BMI088_GYRO_DYDR (0x1 << BMI088_GYRO_DYDR_SHFITS)

#define BMI088_GYRO_RANGE 0x0F
#define BMI088_GYRO_RANGE_SHFITS 0x0
#define BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_1000 (0x1 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_500 (0x2 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_250 (0x3 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_125 (0x4 << BMI088_GYRO_RANGE_SHFITS)

#define BMI088_GYRO_BANDWIDTH 0x10
// the first num means Output data  rate, the second num means bandwidth
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80
#define BMI088_GYRO_2000_532_HZ 0x00
#define BMI088_GYRO_2000_230_HZ 0x01
#define BMI088_GYRO_1000_116_HZ 0x02
#define BMI088_GYRO_400_47_HZ 0x03
#define BMI088_GYRO_200_23_HZ 0x04
#define BMI088_GYRO_100_12_HZ 0x05
#define BMI088_GYRO_200_64_HZ 0x06
#define BMI088_GYRO_100_32_HZ 0x07

#define BMI088_GYRO_LPM1 0x11
#define BMI088_GYRO_NORMAL_MODE 0x00
#define BMI088_GYRO_SUSPEND_MODE 0x80
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20

#define BMI088_GYRO_SOFTRESET 0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6

#define BMI088_GYRO_CTRL 0x15
#define BMI088_DRDY_OFF 0x00
#define BMI088_DRDY_ON 0x80

#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_GYRO_INT4_GPIO_MODE_SHFITS 0x3
#define BMI088_GYRO_INT4_GPIO_PP (0x0 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT4_GPIO_OD (0x1 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT4_GPIO_LVL_SHFITS 0x2
#define BMI088_GYRO_INT4_GPIO_LOW (0x0 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS)
#define BMI088_GYRO_INT4_GPIO_HIGH (0x1 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS)
#define BMI088_GYRO_INT3_GPIO_MODE_SHFITS 0x1
#define BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT3_GPIO_OD (0x1 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT3_GPIO_LVL_SHFITS 0x0
#define BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)
#define BMI088_GYRO_INT3_GPIO_HIGH (0x1 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)

#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18

#define BMI088_GYRO_DRDY_IO_OFF 0x00
#define BMI088_GYRO_DRDY_IO_INT3 0x01
#define BMI088_GYRO_DRDY_IO_INT4 0x80
#define BMI088_GYRO_DRDY_IO_BOTH (BMI088_GYRO_DRDY_IO_INT3 | BMI088_GYRO_DRDY_IO_INT4)

#define BMI088_GYRO_SELF_TEST 0x3C
#define BMI088_GYRO_RATE_OK_SHFITS 0x4
#define BMI088_GYRO_RATE_OK (0x1 << BMI088_GYRO_RATE_OK_SHFITS)
#define BMI088_GYRO_BIST_FAIL_SHFITS 0x2
#define BMI088_GYRO_BIST_FAIL (0x1 << BMI088_GYRO_BIST_FAIL_SHFITS)
#define BMI088_GYRO_BIST_RDY_SHFITS 0x1
#define BMI088_GYRO_BIST_RDY (0x1 << BMI088_GYRO_BIST_RDY_SHFITS)
#define BMI088_GYRO_TRIG_BIST_SHFITS 0x0
#define BMI088_GYRO_TRIG_BIST (0x1 << BMI088_GYRO_TRIG_BIST_SHFITS)

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150

#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
// #define BMI088_ACCEL_RANGE_6G
// #define BMI088_ACCEL_RANGE_12G
// #define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
// #define BMI088_GYRO_RANGE_1000
// #define BMI088_GYRO_RANGE_500
// #define BMI088_GYRO_RANGE_250
// #define BMI088_GYRO_RANGE_125

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

#define BMI088_GYRO_RX_BUF_DATA_OFFSET 1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

#define BMI088_SPI_DMA_GYRO_LENGHT 8
#define BMI088_SPI_DMA_ACCEL_LENGHT 9
#define BMI088_SPI_DMA_ACCEL_TEMP_LENGHT 4

#define BMI088_IMU_DR_SHFITS 0
#define BMI088_IMU_SPI_SHFITS 1
#define BMI088_IMU_UPDATE_SHFITS 2
#define BMI088_IMU_NOTIFY_SHFITS 3

namespace imu {

    enum {
        BMI088_NO_ERROR = 0x00,
        BMI088_ACC_PWR_CTRL_ERROR = 0x01,
        BMI088_ACC_PWR_CONF_ERROR = 0x02,
        BMI088_ACC_CONF_ERROR = 0x03,
        BMI088_ACC_SELF_TEST_ERROR = 0x04,
        BMI088_ACC_RANGE_ERROR = 0x05,
        BMI088_INT1_IO_CTRL_ERROR = 0x06,
        BMI088_INT_MAP_DATA_ERROR = 0x07,
        BMI088_GYRO_RANGE_ERROR = 0x08,
        BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
        BMI088_GYRO_LPM1_ERROR = 0x0A,
        BMI088_GYRO_CTRL_ERROR = 0x0B,
        BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
        BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

        BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
        BMI088_SELF_TEST_GYRO_ERROR = 0x40,
        BMI088_NO_SENSOR = 0xFF,
    };

    /**
     * @brief BMI088 初始化结构体
     */
    /**
     * @brief BMI088 init structure
     */
    struct BMI088_init_t {
        bsp::SPIMaster* spi_master;
        bsp::GPIO* CS_ACCEL;
        bsp::GPIO* CS_GYRO;
        bsp::GPIT* INT_ACCEL = nullptr;
        bsp::GPIT* INT_GYRO = nullptr;
        bool is_DMA = true;
    };

    /**
     * @brief BMI088 回调函数
     * @details BMI088 读取完成后的回调函数
     * @note 由于BMI088的读取是通过DMA或者中断完成的，所以需要回调函数来通知主程序读取完成
     */
    /**
     * @brief BMI088 callback function
     * @details BMI088 callback function after reading
     * @note Because the reading of BMI088 is completed through DMA or interrupt, a callback
     * function is needed to notify the main program that the reading is completed
     */
    typedef void (*BMI088_callback_t)();

    /**
     * @brief BMI088 陀螺仪和加速度计
     * @details BMI088是一款由BOSCH公司生产的高性能陀螺仪和加速度计
     */
    /**
     * @brief BMI088 gyroscope and accelerometer
     * @details BMI088 is a high-performance gyroscope and accelerometer produced by BOSCH
     */
    class BMI088 {
      public:
        /**
         * @brief 构造函数
         * @param init 初始化结构体
         */
        /**
         * @brief constructor
         * @param init init structure
         */
        BMI088(BMI088_init_t init);
        /**
         * @brief 构造函数
         * @param spi_master SPI总线
         * @param CS_ACCEL 加速度计片选引脚
         * @param CS_GYRO 陀螺仪片选引脚
         * @param INT_ACCEL 加速度计中断引脚
         * @param INT_GYRO 陀螺仪中断引脚
         */
        /**
         * @brief constructor
         * @param spi_master SPI bus
         * @param CS_ACCEL accelerometer chip select pin
         * @param CS_GYRO gyroscope chip select pin
         * @param INT_ACCEL accelerometer interrupt pin
         * @param INT_GYRO gyroscope interrupt pin
         */
        BMI088(bsp::SPIMaster* spi_master, bsp::GPIO* CS_ACCEL, bsp::GPIO* CS_GYRO,
               bsp::GPIT* INT_ACCEL = nullptr, bsp::GPIT* INT_GYRO = nullptr, bool is_DMA = true);
        /**
         * @brief 注册回调函数
         * @param callback 回调函数
         */
        /**
         * @brief register callback function
         * @param callback callback function
         */
        void RegisterCallback(BMI088_callback_t callback);
        /**
         * @brief 判断是否初始化完成
         * @return true为初始化完成，false为初始化未完成
         */
        /**
         * @brief Determine whether the initialization is complete
         * @return true for initialization completed, false for initialization not completed
         */
        bool IsReady();

        volatile float gyro_[3];     /**< 陀螺仪数据 */
        volatile float accel_[3];    /**< 加速度计数据 */
        volatile float temperature_; /**< 温度 */
        volatile float time_;        /**< 时间戳 */

      protected:
        /**
         * @brief 在阻塞模式下读取陀螺仪和加速度计数据
         */
        /**
         * @brief Read gyroscope and accelerometer data in blocking mode
         */
        void Read();
        /**
         * @brief 在中断或者DMA模式下读取陀螺仪数据
         */
        /**
         * @brief Read gyroscope data in interrupt or DMA mode
         */
        void Read_IT();
        void temperature_read_over(uint8_t* rx_buf);
        void accel_read_over(uint8_t* rx_buf);
        void gyro_read_over(uint8_t* rx_buf);

        volatile uint8_t gyro_update_flag = 0;
        volatile uint8_t accel_update_flag = 0;
        volatile uint8_t accel_temp_update_flag = 0;
        volatile uint8_t mag_update_flag = 0;
        volatile uint8_t bmi088_start_flag = 0;

        uint8_t gyro_dma_rx_buf[BMI088_SPI_DMA_GYRO_LENGHT];
        uint8_t gyro_dma_tx_buf[BMI088_SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF,
                                                               0xFF, 0xFF, 0xFF, 0xFF};

        uint8_t accel_dma_rx_buf[BMI088_SPI_DMA_ACCEL_LENGHT];
        uint8_t accel_dma_tx_buf[BMI088_SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF,
                                                                 0xFF, 0xFF, 0xFF, 0xFF};

        uint8_t accel_temp_dma_rx_buf[BMI088_SPI_DMA_ACCEL_TEMP_LENGHT];
        uint8_t accel_temp_dma_tx_buf[BMI088_SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};

        virtual void RxCompleteCallback();

      private:
        bsp::SPIMaster* spi_master_;
        bsp::SPIDevice* spi_device_accel_ = nullptr;
        bsp::SPIDevice* spi_device_gyro_ = nullptr;

        bsp::GPIT* gpit_accel_ = nullptr;
        bsp::GPIT* gpit_gyro_ = nullptr;

        bool dma_ = true;

        BMI088_callback_t callback_ = []() {};

        // Only one BMI088 object can be created
        static void GyroCallbackWrapper(void* args);

        static void AccelCallbackWrapper(void* args);

        static void AccelSPICallbackWrapper(void* args);

        static void GyroSPICallbackWrapper(void* args);

        /**
         * @brief 初始化BMI088
         * @return BMI088_NO_ERROR为初始化成功，其他为初始化失败
         */
        uint8_t Init();

        uint8_t bmi088_accel_init();
        uint8_t bmi088_gyro_init();

        void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data);
        void BMI088_accel_read_single_reg(uint8_t reg, uint8_t* data);
        void BMI088_accel_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len);

        void BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data);
        void BMI088_gyro_read_single_reg(uint8_t reg, uint8_t* data);
        void BMI088_gyro_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len);

        void imu_cmd_spi();

        bsp::EventThread* callback_thread_ = nullptr;

        const osThreadAttr_t callback_thread_attr_ = {.name = "bmi088UpdateTask",
                                                      .attr_bits = osThreadDetached,
                                                      .cb_mem = nullptr,
                                                      .cb_size = 0,
                                                      .stack_mem = nullptr,
                                                      .stack_size = 128 * 4,
                                                      .priority = (osPriority_t)osPriorityRealtime,
                                                      .tz_module = 0,
                                                      .reserved = 0};

        static void callback_thread_func_(void* arg);
    };

}  // namespace imu