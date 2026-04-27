#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h> // For sleep
#include "rclcpp/rclcpp.hpp"
#include "global_robot_localization/global_robot_localization.hpp"
#include "global_robot_localization/action/localize_in_map.hpp"

using namespace std::chrono_literals;

TEST(GlobalLocFunctionalTest, TestNodeBringup) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<global_robot_localization::GlobalRobotLocalization>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    // give the node some time to come up
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // while (rclcpp::ok() && !node->is_complete()) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    executor.cancel();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    executor.remove_node(node);
    rclcpp::shutdown();
}

TEST(GlobalLocFunctionalTest, TestNodeAction) {
    // bring up the node and send it an action request.
    // this request should fail (generally), because the
    // map provider is not up (generally).
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<global_robot_localization::GlobalRobotLocalization>();
    auto client_node = std::make_shared<rclcpp::Node>("action_client");

    auto actionClient = rclcpp_action::create_client<global_robot_localization::action::LocalizeInMap>(client_node, "LocalizeInMap");

    using ActionT = global_robot_localization::action::LocalizeInMap;
    using GoalHandleT = rclcpp_action::ClientGoalHandle<ActionT>;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(client_node);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    bool client_up = actionClient->wait_for_action_server(5s);

    if (client_up) std::cout << "server up!" << std::endl;

    ActionT::Goal goal;
    rclcpp_action::Client<ActionT>::SendGoalOptions options;

    // send the message to the node
    auto goal_future = actionClient->async_send_goal(goal, options);
    goal_future.wait_for(5s);

    bool goal_response;

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
        goal_response = false;
        std::cout << "Goal rejected" << std::endl;
    } else {
        goal_response = true;
        std::cout << "Goal accepted" << std::endl;
    }

    ASSERT_FALSE(goal_response);


    executor.cancel();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    executor.remove_node(node);
    executor.remove_node(client_node);
    rclcpp::shutdown();

    ASSERT_FALSE(goal_response);
    
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}