#pragma once

#include <deque>
#include <fstream>
#include <iostream>

#include <linalg.h>

#include <ssl-vision/messages_robocup_ssl_detection.pb.h>
#include <ssl-vision/messages_robocup_ssl_geometry.pb.h>
#include <ssl-vision/messages_robocup_ssl_wrapper.pb.h>

#include <math/median_filter.h>
#include <network/udp_client.h>
#include <network/udp_server.h>
#include <setting.h>
#include <state/world.h>