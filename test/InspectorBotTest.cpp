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
#include "InspectorBot.hpp"  // Include your InspectorBot header

class InspectorBotTest : public ::testing::Test {
protected:
    InspectorBot* bot;

    void SetUp() override {
        bot = new InspectorBot();
    }

    void TearDown() override {
        delete bot;
    }
};

TEST_F(InspectorBotTest, DummyTest1) {
    EXPECT_TRUE(true); // Always true, test will always pass
}

TEST_F(InspectorBotTest, DummyTest2) {
    EXPECT_EQ(1, 1); // Always equal, test will always pass
}

TEST_F(InspectorBotTest, DummyTest3) {
    EXPECT_NE(1, 0); // Always not equal, test will always pass
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
