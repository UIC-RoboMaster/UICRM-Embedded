
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

#include "bsp_can.h"

#include "bsp_error_handler.h"
#include "cmsis_os.h"

namespace bsp {

    std::unordered_map<FDCAN_HandleTypeDef*, CAN*> CAN::ptr_map;

    /**
     * @brief find instantiated can line
     *
     * @param hfdcan  HAL can handle
     *
     * @return can instance if found, otherwise NULL
     */
    CAN* CAN::FindInstance(FDCAN_HandleTypeDef* hfdcan) {
        const auto it = ptr_map.find(hfdcan);
        if (it == ptr_map.end())
            return nullptr;

        return it->second;
    }

    /**
     * @brief check if any associated CAN instance is instantiated or not
     *
     * @param hfdcan  HAL can handle
     *
     * @return true if found, otherwise false
     */
    bool CAN::HandleExists(FDCAN_HandleTypeDef* hfdcan) {
        return FindInstance(hfdcan) != nullptr;
    }

    /**
     * @brief callback handler for CAN rx feedback data
     *
     * @param hfdcan  HAL can handle
     */
    void CAN::RxFIFO0MessagePendingCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
        UNUSED(RxFifo0ITs);
        CAN* can = FindInstance(hfdcan);
        if (!can)
            return;
        can->RxCallback();
    }

    CAN::CAN(FDCAN_HandleTypeDef* hfdcan, bool is_master, uint8_t ext_id_suffix)
        : hfdcan_(hfdcan), ext_id_suffix_(ext_id_suffix) {
        RM_ASSERT_FALSE(HandleExists(hfdcan), "Repeated CAN initialization");
        ConfigureFilter(is_master);
        // activate rx interrupt
        RM_ASSERT_HAL_OK(HAL_FDCAN_RegisterRxFifo0Callback(hfdcan, RxFIFO0MessagePendingCallback),
                         "Cannot register CAN rx callback");
        RM_ASSERT_HAL_OK(HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0),
                         "Cannot activate CAN rx message pending notification");
        RM_ASSERT_HAL_OK(HAL_FDCAN_Start(hfdcan), "Cannot start CAN");

        // save can instance as global pointer
        ptr_map[hfdcan] = this;
    }

    int CAN::RegisterRxCallback(uint32_t std_id, can_rx_callback_t callback, void* args) {
        // int callback_id = std_id - start_id_;

        if (callback_count_ >= MAX_CAN_DEVICES)
            return -1;

        rx_args_[callback_count_] = args;
        rx_callbacks_[callback_count_] = callback;
        id_to_index_[std_id] = callback_count_;
        callback_count_++;

        return 0;
    }

    int CAN::RegisterRxExtendCallback(uint32_t ext_id_suffix, can_rx_ext_callback_t callback,
                                      void* args) {
        if (ext_callback_count_ >= MAX_CAN_DEVICES)
            return -1;

        rx_ext_args_[callback_count_] = args;
        rx_ext_callbacks_[callback_count_] = callback;
        ext_to_index_[ext_id_suffix] = ext_callback_count_;
        ext_callback_count_++;

        return 0;
    }

    int CAN::Transmit(uint16_t id, const uint8_t data[], uint32_t length) {
        RM_EXPECT_TRUE(IS_FDCAN_DLC(length), "CAN tx data length exceeds limit");
        if (!IS_FDCAN_DLC(length))
            return -1;

        FDCAN_TxHeaderTypeDef header = {.Identifier = id,
                                        .IdType = FDCAN_STANDARD_ID,
                                        .TxFrameType = FDCAN_DATA_FRAME,
                                        .DataLength = length << 16,
                                        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
                                        .BitRateSwitch = FDCAN_BRS_OFF,
                                        .FDFormat = FDCAN_CLASSIC_CAN,
                                        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
                                        .MessageMarker = 0x00};

        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &header, (uint8_t*)data) != HAL_OK)
            return -1;

        // poll for can transmission to complete
        //        while (HAL_CAN_IsTxMessagePending(hfdcan_, mailbox))
        //            ;

        return length;
    }

    int CAN::TransmitExtend(uint32_t id, const uint8_t data[], uint32_t length) {
        RM_EXPECT_TRUE(IS_FDCAN_DLC(length), "CAN tx data length exceeds limit");
        if (!IS_FDCAN_DLC(length))
            return -1;

        FDCAN_TxHeaderTypeDef header = {.Identifier = id,
                                        .IdType = FDCAN_EXTENDED_ID,
                                        .TxFrameType = FDCAN_DATA_FRAME,
                                        .DataLength = length << 16,
                                        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
                                        .BitRateSwitch = FDCAN_BRS_OFF,
                                        .FDFormat = FDCAN_CLASSIC_CAN,
                                        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
                                        .MessageMarker = 0x00};

        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &header, (uint8_t*)data) != HAL_OK)
            return -1;

        // poll for can transmission to complete
        //        while (HAL_CAN_IsTxMessagePending(hfdcan_, mailbox))
        //            ;

        return length;
    }

    void CAN::RxCallback() {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[MAX_CAN_DATA_SIZE];
        HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &header, data);
        if (header.IdType == FDCAN_EXTENDED_ID) {
            RxExtendCallback(header, data);
            return;
        }
        uint16_t callback_id = header.Identifier;
        const auto it = id_to_index_.find(callback_id);
        if (it == id_to_index_.end())
            return;
        callback_id = it->second;
        // find corresponding callback
        if (rx_callbacks_[callback_id])
            rx_callbacks_[callback_id](data, rx_args_[callback_id]);
    }

    void CAN::RxExtendCallback(FDCAN_RxHeaderTypeDef header, uint8_t* data) {
        uint32_t extId = header.Identifier;
        // use the first ext_id_suffix_ bits to identify the devices
        uint32_t identifier = extId & ((1 << ext_id_suffix_) - 1);
        const auto it = ext_to_index_.find(identifier);
        if (it == ext_to_index_.end())
            return;
        identifier = it->second;
        // find corresponding callback
        if (rx_ext_callbacks_[identifier])
            rx_ext_callbacks_[identifier](data, extId, rx_ext_args_[identifier]);
    }

    void CAN::ConfigureFilter(bool is_master) {
        UNUSED(is_master);
        FDCAN_FilterTypeDef fdcan_filter;

        fdcan_filter.IdType = FDCAN_STANDARD_ID;
        fdcan_filter.FilterIndex = 0;
        fdcan_filter.FilterType = FDCAN_FILTER_RANGE;
        fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        fdcan_filter.FilterID1 = 0x0000;
        fdcan_filter.FilterID2 = 0x0000;
        RM_EXPECT_HAL_OK(HAL_FDCAN_ConfigFilter(hfdcan_, &fdcan_filter) != HAL_OK,
                         "CAN filter configuration failed.");

        HAL_FDCAN_ConfigFifoWatermark(hfdcan_, FDCAN_CFG_RX_FIFO0, 1);
    }

} /* namespace bsp */