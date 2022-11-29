#pragma once

// #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
// #define SPDLOG_DEBUG_ON
// #define SPDLOG_TRACE_ON
// #include "spdlog/spdlog.h"

// #include <BaseClientRpc.h>
// #include <SessionManager.h>

// #include <RouterClient.h>
// #include <TransportClientTcp.h>
// #include <TransportClientUdp.h>
// #include <BaseCyclicClientRpc.h>
// #include <ActuatorConfig.pb.h>
// #include <ActuatorConfigClientRpc.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
// #include <Eigen/Dense>

#include <iostream>
#include <math.h>
#include <chrono>
#include <random>
#include <sys/wait.h>
#include <sys/types.h>
#include <algorithm>

// #include <google/protobuf/util/json_util.h>

#define DEG2RAD(angle) (static_cast<double>(angle)*M_PI/180.0)
#define RAD2DEG(angle) (static_cast<double>(angle)/M_PI*180.0)
#define ROUND0(val) (abs(val) < 1e-5 ? 0 : val)

// #define CARTESIAN_DIMS 6

// namespace k_api = Kinova::Api;
