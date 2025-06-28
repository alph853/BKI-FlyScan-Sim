/**
 * @file slam_node.hpp
 * @brief SLAM node for autonomous drone nodes with lifecycle management and bond-based monitoring.
 *
 * This header file contains the SLAM node for all autonomous drone nodes. It provides
 * lifecycle management, bond-based monitoring, state management, and automatic registration
 * with the LifeMonitor. It also provides the SLAM functionality for the autonomous drone.
 *
 * @author UAV team@/flyscan
 * @date 2025/06/17
 * @version 1.0
 */

#pragma once

#include "flyscan_common/types.hpp"
#include "flyscan_common/enums.hpp"
#include "flyscan_common/constants.hpp"

#include "flyscan_core/base_node.hpp"
#include "flyscan_vision/vio_node.hpp"

namespace flyscan {

namespace vision {

using ImageMsg          = flyscan::common::vision_types::ImageMsg;
using ImuMsg            = flyscan::common::vision_types::ImuMsg;
using OdometryMsg       = flyscan::common::vision_types::OdometryMsg;

using NodeType          = flyscan::common::NodeType;
using BaseNode          = flyscan::core::BaseNode;
using OperationStatus   = flyscan::common::OperationStatus;

class SlamNode : public BaseNode
{
public:
    SlamNode(const std::string& node_name, 
            const NodeType& node_type, 
            const std::vector<std::string>& capabilities);
    ~SlamNode();

private:
    // void OnImageCallback(const ImageMsg::ConstSharedPtr& msg);

};

}  // namespace vision

}  // namespace flyscan
