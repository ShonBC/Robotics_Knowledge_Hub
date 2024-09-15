/**
 * @file planner_test.cpp
 * @brief Unit tests for the Planner class
 */

#include <gtest/gtest.h>
#include "../include/planner.hpp"

/**
 * @class PlannerTest
 * @brief Test fixture for the Planner class
 */
class PlannerTest : public ::testing::Test {
protected:
    Planner planner; ///< Instance of the Planner class to be tested

    /**
     * @brief Set up the test environment before each test
     */
    void SetUp() override {
        // Set up common test environment
        planner.setAlgorithm("A*");
    }
};

/**
 * @test
 * @brief Test that the Planner constructor doesn't throw any exceptions
 */
TEST_F(PlannerTest, InitializationTest) {
    EXPECT_NO_THROW(Planner());
}

/**
 * @test
 * @brief Test that setting a different algorithm doesn't throw any exceptions
 */
TEST_F(PlannerTest, SetAlgorithmTest) {
    EXPECT_NO_THROW(planner.setAlgorithm("Dijkstra"));
}

/**
 * @test
 * @brief Test that setting obstacles doesn't throw any exceptions
 */
TEST_F(PlannerTest, SetObstaclesTest) {
    std::vector<std::pair<double, double>> obstacles = {{1.0, 1.0}, {2.0, 2.0}};
    EXPECT_NO_THROW(planner.setObstacles(obstacles));
}

/**
 * @test
 * @brief Test that planning a path returns a non-empty path that starts at the start point and ends at the goal point
 */
TEST_F(PlannerTest, PlanPathTest) {
    std::pair<double, double> start = {0.0, 0.0};
    std::pair<double, double> goal = {5.0, 5.0};
    std::vector<std::pair<double, double>> path = planner.planPath(start, goal);
    
    EXPECT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), goal);
}

/**
 * @test
 * @brief Test that planning a path with obstacles returns a valid path that avoids the obstacles
 */
TEST_F(PlannerTest, PlanPathWithObstaclesTest) {
    std::vector<std::pair<double, double>> obstacles = {{2.0, 2.0}, {3.0, 3.0}};
    planner.setObstacles(obstacles);

    std::pair<double, double> start = {0.0, 0.0};
    std::pair<double, double> goal = {5.0, 5.0};
    std::vector<std::pair<double, double>> path = planner.planPath(start, goal);
    
    EXPECT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), goal);
    
    // Check if path avoids obstacles
    for (const auto& point : path) {
        EXPECT_FALSE(std::find(obstacles.begin(), obstacles.end(), point) != obstacles.end());
    }
}

/**
 * @test
 * @brief Test that planning a path when there's no possible solution returns an empty path
 */
TEST_F(PlannerTest, PlanPathNoSolutionTest) {
    // Surround the goal with obstacles
    std::vector<std::pair<double, double>> obstacles = {
        {4.0, 4.0}, {4.0, 5.0}, {4.0, 6.0},
        {5.0, 4.0}, {5.0, 6.0},
        {6.0, 4.0}, {6.0, 5.0}, {6.0, 6.0}
    };
    planner.setObstacles(obstacles);

    std::pair<double, double> start = {0.0, 0.0};
    std::pair<double, double> goal = {5.0, 5.0};
    std::vector<std::pair<double, double>> path = planner.planPath(start, goal);
    
    EXPECT_TRUE(path.empty());
}

/**
 * @brief Main function to run all tests
 * @param argc Number of command-line arguments
 * @param argv Array of command-line arguments
 * @return Result of running the tests
 */
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
