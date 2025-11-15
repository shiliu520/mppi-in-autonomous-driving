import crcpp
from commonroad.common.file_reader import CommonRoadFileReader

scenario, _ = CommonRoadFileReader("../tests/scenarios/DEU_test_safe_distance.xml").open()
scenario_id = 123
crcpp.register_scenario(
    scenario_id, str(scenario.scenario_id), 0, "DEU", scenario.lanelet_network, scenario.obstacles, []
)
final_time_step = max([obs.prediction.final_time_step for obs in scenario.dynamic_obstacles])

with open("safe_distance_rule.log", "w") as f:
    for time_step in range(final_time_step + 1):
        for obs1 in scenario.dynamic_obstacles:
            for obs2 in scenario.dynamic_obstacles:
                if obs1.obstacle_id == obs2.obstacle_id:
                    continue
                if crcpp.in_same_lane_boolean_evaluation(scenario_id, time_step, obs1.obstacle_id, obs2.obstacle_id):
                    f.write("@{} in_same_lane ({},{})\n".format(time_step, obs1.obstacle_id, obs2.obstacle_id))
                if crcpp.safe_distance_boolean_evaluation(scenario_id, time_step, obs1.obstacle_id, obs2.obstacle_id):
                    f.write(
                        "@{} keeps_safe_distance_prec ({},{})\n".format(time_step, obs1.obstacle_id, obs2.obstacle_id)
                    )
                if crcpp.in_front_of_boolean_evaluation(scenario_id, time_step, obs1.obstacle_id, obs2.obstacle_id):
                    f.write("@{} in_front_of ({},{})\n".format(time_step, obs1.obstacle_id, obs2.obstacle_id))

crcpp.remove_scenario(123)
