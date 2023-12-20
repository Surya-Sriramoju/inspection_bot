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
#include <iostream>
#include "InspectorBot.hpp"

// Mock version of InspectorBot for testing
class MockInspectorBot : public InspectorBot {
public:
    bool locationReached = false;
    bool rotationDone = false;
    bool inspectionContinued = false;

    MockInspectorBot() : InspectorBot() {
        // Mock initialization, no ROS dependencies
    }

    void goToLocation() override {
        // Simulate robot moving to a location
        locationReached = true; // Indicate that the location was reached
    }

    void rotateBot() override {
        // Simulate robot rotation
        rotationDone = true; // Indicate that the rotation was done
    }

    void continueInspection() override {
        // Simulate continuing inspection
        inspectionContinued = true; // Indicate that the inspection continued
    }
};

class InspectorBotTest : public ::testing::Test {
protected:
    std::unique_ptr<MockInspectorBot> bot;

    void SetUp() override {
        bot = std::make_unique<MockInspectorBot>();
    }

    void TearDown() override {
        bot.reset();
    }
};

TEST_F(InspectorBotTest, TestSetAndGetLocation) {
    bot->setLoc(5.0, 10.0);
    EXPECT_EQ(bot->getLocx(), 5.0);
    EXPECT_EQ(bot->getLocy(), 10.0);
}

TEST_F(InspectorBotTest, TestGoToLocation) {
    bot->setLoc(3.0, 4.0);
    bot->goToLocation();
    EXPECT_TRUE(bot->locationReached);
}

TEST_F(InspectorBotTest, TestRotateBot) {
    bot->rotateBot();
    EXPECT_TRUE(bot->rotationDone);
}

TEST_F(InspectorBotTest, TestContinueInspection) {
    bot->continueInspection();
    EXPECT_TRUE(bot->inspectionContinued);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
