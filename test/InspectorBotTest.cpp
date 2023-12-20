// Copyright 2023 Tarun Trilokesh

/**
 * @file InspectorBotTest.cpp
 * @brief Test file for the InspectorBot class.
 * 
 * Implements tests for the InspectorBot class using the Google Test framework.
 * This file demonstrates how to write test cases for methods in the InspectorBot class.
 *
 * @author Tarun Trilokesh
 * @date 12/20/2023
 * @version 1.0
 */

#include <gtest/gtest.h>
#include <string>
#include <geometry_msgs/msg/pose.hpp>
#include "rclcpp/rclcpp.hpp"

// Dummy InspectorBot class
class InspectorBot {
public:
    InspectorBot() {
        // Initialize position and orientation
        current_position.position.x = 0.0;
        current_position.position.y = 0.0;
        current_position.position.z = 0.0;
        current_position.orientation.x = 0.0;
        current_position.orientation.y = 0.0;
        current_position.orientation.z = 0.0;
        current_position.orientation.w = 1.0;
        orientation = 0.0;
        goal_x = 0.0;
        goal_y = 0.0;
        locationReached = false;
    }

    void goToLocation() {
        current_position.position.x = goal_x;
        current_position.position.y = goal_y;
        locationReached = true;
    }

    void continueInspection() {
        state = "Inspecting";
    }

    void rotateBot() {
        orientation = 1.0;
    }

    void setLoc(float x, float y) {
        goal_x = x;
        goal_y = y;
    }

    float getLocx() const {
        return goal_x;
    }

    float getLocy() const {
        return goal_y;
    }

    geometry_msgs::msg::Pose getCurrentPosition() const {
        return current_position;
    }

    std::string getState() const {
        return state;
    }

    void setState(const std::string& new_state) {
        state = new_state;
    }

    double getOrientation() const {
        return orientation;
    }

    void setOrientation(double new_orientation) {
        orientation = new_orientation;
    }

    bool hasReachedLocation() const {
        return locationReached;
    }

private:
    geometry_msgs::msg::Pose current_position;
    std::string state;
    double orientation;
    float goal_x, goal_y;
    bool locationReached;
};

/**
 * @class InspectorBotTest
 * @brief Test Fixture for testing the InspectorBot class.
 */
class InspectorBotTest : public ::testing::Test {
protected:
    std::shared_ptr<InspectorBot> inspector_bot;

    void SetUp() override {
        inspector_bot = std::make_shared<InspectorBot>();
    }

    void TearDown() override {
        // Cleanup if necessary
    }
};

TEST_F(InspectorBotTest, GoToLocationTest) {
    inspector_bot->setLoc(5.0, 5.0);
    inspector_bot->goToLocation();
    EXPECT_TRUE(inspector_bot->hasReachedLocation());
}

TEST_F(InspectorBotTest, ContinueInspectionTest) {
    inspector_bot->setState("Paused");
    inspector_bot->continueInspection();
    EXPECT_EQ("Inspecting", inspector_bot->getState());
}

TEST_F(InspectorBotTest, RotateBotTest) {
    inspector_bot->setOrientation(0);
    inspector_bot->rotateBot();
    EXPECT_NE(0, inspector_bot->getOrientation());
}

TEST_F(InspectorBotTest, SetLocTest) {
    float x = 10.0, y = 20.0;
    inspector_bot->setLoc(x, y);
    EXPECT_EQ(x, inspector_bot->getLocx());
    EXPECT_EQ(y, inspector_bot->getLocy());
}

TEST_F(InspectorBotTest, GetLocxTest) {
    float expected_x = 10.0;
    inspector_bot->setLoc(expected_x, 0.0);
    EXPECT_EQ(expected_x, inspector_bot->getLocx());
}

TEST_F(InspectorBotTest, GetLocyTest) {
    float expected_y = 20.0;
    inspector_bot->setLoc(0.0, expected_y);
    EXPECT_EQ(expected_y, inspector_bot->getLocy());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
