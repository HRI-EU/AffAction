import os
from lerobot.robots import make_robot_from_config
from amazingik_robot import AmazingIKRobotConfig, AmazingIKRobot




def main(args=None):
    
    config = AmazingIKRobotConfig(
        build_directory=os.path.expandvars("$HOME/localdisk/AttentiveSupport/build"),
        robo_computer_ip="192.168.1.101",
        id="franka_amazingik_sim",
        debug_level=0,
    )

    robot = AmazingIKRobot(config)
    robot.connect()


    try:
        for i in range(1000):
            obs = robot.get_observation()
            print(obs["observation.state"].shape)           # (7,)
            print(obs["observation.images.camera_0"].shape) # (224, 224, 3)
    
            curr_wrench = robot.backend.sim.getEndEffectorWrench()
            
            action = robot.send_action(
                {
                    "action.wrench.fx": curr_wrench[0] + 0.001,
                    "action.wrench.fy": curr_wrench[1],
                    "action.wrench.fz": curr_wrench[2],
                    "action.wrench.tx": curr_wrench[3],
                    "action.wrench.ty": curr_wrench[4],
                    "action.wrench.tz": curr_wrench[5],
                }
            )
            print(obs["observation.state"])

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user (Ctrl+C)")

    finally:
        print("[INFO] Disconnecting robot...")
        robot.disconnect()

    

if __name__ == "__main__":
    main()
