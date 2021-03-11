import time
import random
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from gazebo_connection import GazeboConnection
import sys
import random
from collections import deque
import numpy as np
import pylab
from drqn import DRQNAgent
from ddqn import DoubleDQNAgent
from dqn import DQNAgent
from quantized_models import DQNAgent_quant, DoubleDQNAgent_quant

EPISODES = 300

class RobotEnvironment:

    def __init__(self):
        # before initialising the environment
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # stablishes connection with simulator
        self.gazebo = GazeboConnection()

        self.prev_pitch = 0
        self.prev_time = 0
        self.prev_accel = 0

        random.seed(42)


    def check_topic_publishers_connection(self):
        
        rate = rospy.Rate(1) # 10hz
        while(self.cmd_vel_pub.get_num_connections() == 0):
            rospy.loginfo("No subscribers to cmd_vel yet so we wait and try again")
            rate.sleep()
        rospy.loginfo("cmd_vel publisher connected")
        
    def reset_cmd_vel_commands(self):
        # We send an empty null Twist
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0

        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd_vel)

    def takeoff_sequence(self, seconds_taking_off=0.1):
        # Before taking off be sure that cmd_vel value there is is null to avoid drifts        
        rospy.loginfo( "Reset sequence start")
        self.reset_cmd_vel_commands()
        time.sleep(seconds_taking_off)
        rospy.loginfo( "Reset sequence completed")


    def take_observation (self):
        pitch_data = None
        while pitch_data is None:
            try:
                raw_pitch_data = rospy.wait_for_message('/curr_pitch', Float32, timeout=5)
                pitch_data = raw_pitch_data.data
            except:
                rospy.loginfo("Current pitch_data not ready yet, retrying for getting pitch_data")

        accel_data = None
        while accel_data is None:
            try:
                raw_accel_data = rospy.wait_for_message('/imu', Imu, timeout=5)
                accel_data = raw_accel_data.linear_acceleration.x
            except:
                rospy.loginfo("Current accel_data not ready yet, retrying for getting accel_data")

        clock_data = None
        while clock_data is None:
            try:
                raw_clock_data = rospy.wait_for_message('/clock', Clock, timeout=5)
                clock_data = raw_clock_data.clock.secs + raw_clock_data.clock.nsecs / float(1000000000)
            except:
                rospy.loginfo("Current clock_data not ready yet, retrying for getting clock_data")

        return pitch_data, accel_data, clock_data

    # Resets the state of the environment and returns an initial observation.
    def reset(self):
        
        # 1st: resets the simulation to initial values
        self.gazebo.reset_sim()

        # 2nd: Unpauses simulation
        self.gazebo.unpause_sim()

        # 3rd: resets the robot to initial conditions
        self.check_topic_publishers_connection()
        self.takeoff_sequence()

        # 4th: takes an observation of the initial condition of the robot
        observation = self.take_observation()
        pitch_data, accel_data, clock_data = observation

        self.prev_accel = accel_data
        self.prev_pitch = pitch_data
        self.prev_time = clock_data

        # 5th: pauses simulation
        self.gazebo.pause_sim()

        return

    def process_data(self, observation):
        done = False
        reward = 1

        pitch_data, accel_data, clock_data = observation

        dt = clock_data - self.prev_time # units in seconds
        dp = pitch_data - self.prev_pitch
        da = accel_data - self.prev_accel

        angular_velocity = dp / dt
        linear_velocity = da * dt

        if abs(pitch_data) > 0.4:
            done = True
            reward = 0

        self.prev_accel = accel_data
        self.prev_pitch = pitch_data
        self.prev_time = clock_data

        state = [pitch_data, angular_velocity, linear_velocity, accel_data]

        return state, reward, done

    def step(self, action, running_step=3):
        # Given the action selected by the learning algorithm,
        # we perform the corresponding movement of the robot
        actions = {
            1: 0.1,
            0: -0.1,
            -1: 0,
        }
        if not action in actions:
            raise Exception("Choose actions 0 or 1")
        
        # 1st, we decide which velocity command corresponds
        cmd_vel = Twist()
        cmd_vel.linear.x = actions[action]

        # Then we send the command to the robot and let it go
        # for running_step seconds
        self.gazebo.unpause_sim()
        self.cmd_vel_pub.publish(cmd_vel)
        time.sleep(running_step)
        observation = self.take_observation()
        self.gazebo.pause_sim()

        # finally we get an evaluation based on what happened in the sim
        state, reward, done = self.process_data(observation)

        return state, reward, done, {}

    def train_drqn(self):
        # Number of past state to use
        number_of_past_state = 4

        # get size of state and action from environment
        state_size = 4
        expanded_state_size = state_size * number_of_past_state
        action_size = 2

        agent = DRQNAgent(expanded_state_size, action_size)

        scores, episodes = [], []

        for episode in range(1, EPISODES+1):
            done = False
            score = 0
            # run reinforcement learning for every episode
            self.reset()
            state, _, _, _ = self.step(-1)

            # expand the state with past states and initialize
            expanded_state = np.zeros(expanded_state_size)
            expanded_next_state = np.zeros(expanded_state_size)
            for h in range(state_size):
                expanded_state[(h + 1) * number_of_past_state - 1] = state[h]

            # reshape states for LSTM input without embedding layer
            reshaped_state = np.zeros((1, expanded_state_size, 2))
            for i in range(expanded_state_size):
                for j in range(2):
                    reshaped_state[0, i, j] = expanded_state[i]

            rospy.loginfo("Episode %d: starting", episode)

            while not done:
                action = agent.get_action(reshaped_state)
                next_state, reward, done, info, = self.step(action) 

                rospy.loginfo("Episode %d: action: %d pitch: %f", episode, action, next_state[0])

                # update the expanded next state with next state values
                for h in range(state_size):
                    expanded_next_state[(h + 1) * number_of_past_state - 1] = next_state[h]

                # reshape expanded next state for LSTM input without embedding layer
                reshaped_next_state = np.zeros((1, expanded_state_size, 2))
                for i in range(expanded_state_size):
                    for j in range(2):
                        reshaped_next_state[0, i, j] = expanded_next_state[i]

                # if an action make the episode end, then gives penalty of -100
                reward = reward if not done or score == 499 else -100

                # save the sample <s, a, r, s'> to the replay memory
                agent.append_sample(
                    reshaped_state, action, reward, reshaped_next_state, done
                )

                # every time step do the training
                agent.train_model()
                score += reward
                reshaped_state = reshaped_next_state

                # Shifting past state elements to the left by one
                expanded_next_state = np.roll(expanded_next_state, -1)

                if done or score >= 500:
                    # every episode update the target model to be same with model
                    agent.update_target_model()

                    # every episode, plot the play time
                    score = score if score == 500 else score + 100
                    scores.append(score)
                    episodes.append(episode)
                    pylab.plot(episodes, scores, "b")
                    pylab.savefig("./cartpole_drqn.png")
                    print(
                        "episode:",
                        episode,
                        "  score:",
                        score,
                        "  memory length:",
                        len(agent.memory),
                        "  epsilon:",
                        agent.epsilon,
                    )
                    break

                # if the mean of scores of last 10 episode is bigger than 490
                # stop training
                # revised to exit cleanly on Jupiter notebook
                if np.mean(scores[-min(10, len(scores)) :]) > 490:
                    sys.exit()
                    break

            # save the model
            if episode % 20 == 0:
                agent.model.save_weights("./cartpole_drqn.h5")


            rospy.loginfo("Episode %d: completed", episode)

    def predict_drqn(self):
        # Number of past state to use
        number_of_past_state = 4

        # get size of state and action from environment
        state_size = 4
        expanded_state_size = state_size * number_of_past_state
        action_size = 2

        agent = DRQNAgent(expanded_state_size, action_size, load_model=True)

        done = False
        score = 0
        # run reinforcement learning for every episode
        self.reset()
        state, _, _, _, = self.step(-1) 

        # expand the state with past states and initialize
        expanded_state = np.zeros(expanded_state_size)
        expanded_next_state = np.zeros(expanded_state_size)
        for h in range(state_size):
            expanded_state[(h + 1) * number_of_past_state - 1] = state[h]

        # reshape states for LSTM input without embedding layer
        reshaped_state = np.zeros((1, expanded_state_size, 2))
        for i in range(expanded_state_size):
            for j in range(2):
                reshaped_state[0, i, j] = expanded_state[i]

        while not done:
            action = agent.get_action(reshaped_state)
            next_state, reward, done, info, = self.step(action) 

            # update the expanded next state with next state values
            for h in range(state_size):
                expanded_next_state[(h + 1) * number_of_past_state - 1] = next_state[h]

            # reshape expanded next state for LSTM input without embedding layer
            reshaped_next_state = np.zeros((1, expanded_state_size, 2))
            for i in range(expanded_state_size):
                for j in range(2):
                    reshaped_next_state[0, i, j] = expanded_next_state[i]

            # if an action make the episode end, then gives penalty of -100
            reward = reward if not done or score == 499 else -100

            # save the sample <s, a, r, s'> to the replay memory
            agent.append_sample(
                reshaped_state, action, reward, reshaped_next_state, done
            )

            # every time step do the training
            score += reward
            reshaped_state = reshaped_next_state

            # Shifting past state elements to the left by one
            expanded_next_state = np.roll(expanded_next_state, -1)

            if done or score >= 500:
                print("score:",score,)
                break

    def train_ddqn(self):
        # get size of state and action from environment
        state_size = 4
        action_size = 2

        agent = DoubleDQNAgent(state_size, action_size)

        scores, episodes = [], []

        for episode in range(1, EPISODES+1):
            # run reinforcement learning for every episode
            done = False
            score = 0

            self.reset()
            state, _, _, _ = self.step(-1)
            state = np.reshape(state, [1, state_size])

            rospy.loginfo("Episode %d: starting", episode)

            while not done:
                action = agent.get_action(state)
                next_state, reward, done, _, = self.step(action) 

                rospy.loginfo("Episode %d: action: %d pitch: %f", episode, action, next_state[0])

                next_state = np.reshape(next_state, [1, state_size])
                # if an action make the episode end, then gives penalty of -100
                reward = reward if not done or score == 499 else -100

                # save the sample <s, a, r, s'> to the replay memory
                agent.append_sample(state, action, reward, next_state, done)
                # every time step do the training
                agent.train_model()
                score += reward
                state = next_state

                if done or score >= 500:
                    # every episode update the target model to be same with model
                    agent.update_target_model()

                    # every episode, plot the play time
                    score = score if score == 500 else score + 100
                    scores.append(score)
                    episodes.append(episode)
                    pylab.plot(episodes, scores, "b")
                    pylab.savefig("./cartpole_ddqn.png")
                    print(
                        "episode:",
                        episode,
                        " score:",
                        score,
                        " memory length:",
                        len(agent.memory),
                        " epsilon:",
                        agent.epsilon,
                    )
                    break

                    # if the mean of scores of last 10 episode is bigger than 490
                    # stop training
                    if np.mean(scores[-min(10, len(scores)) :]) > 490:
                        break

            # save the model
            if episode % 20 == 0:
                agent.model.save_weights("./cartpole_ddqn.h5")

            rospy.loginfo("Episode %d: completed", episode)

    def predict_ddqn(self):
        # get size of state and action from environment
        state_size = 4
        action_size = 2

        agent = DoubleDQNAgent(state_size, action_size, load_model=True)

        done = False
        score = 0

        self.reset()
        state, _, _, _ = self.step(-1)
        state = np.reshape(state, [1, state_size])

        while not done:
            # get action for the current state and go one step in environment
            action = agent.get_action(state)
            next_state, reward, done, info = self.step(action)
            next_state = np.reshape(next_state, [1, state_size])

            score += reward
            state = next_state

            if done or score >= 500:
                print("score:",score)
                break

    def train_dqn(self):
        # get size of state and action from environment
        state_size = 4
        action_size = 2

        agent = DQNAgent(state_size, action_size)

        scores, episodes = [], []

        for episode in range(1, EPISODES+1):
            # run reinforcement learning for every episode
            done = False
            score = 0

            self.reset()
            state, _, _, _ = self.step(-1)
            state = np.reshape(state, [1, state_size])

            rospy.loginfo("Episode %d: starting", episode)

            while not done:
                action = agent.get_action(state)
                next_state, reward, done, _, = self.step(action) 

                rospy.loginfo("Episode %d: action: %d pitch: %f", episode, action, next_state[0])

                next_state = np.reshape(next_state, [1, state_size])
                # if an action make the episode end, then gives penalty of -100
                reward = reward if not done or score == 499 else -100

                # save the sample <s, a, r, s'> to the replay memory
                agent.append_sample(state, action, reward, next_state, done)
                # every time step do the training
                agent.train_model()
                score += reward
                state = next_state

                if done or score >= 500:
                    # every episode update the target model to be same with model
                    agent.update_target_model()

                    # every episode, plot the play time
                    score = score if score == 500 else score + 100
                    scores.append(score)
                    episodes.append(episode)
                    pylab.plot(episodes, scores, "b")
                    pylab.savefig("./cartpole_dqn.png")
                    print(
                        "episode:",
                        episode,
                        " score:",
                        score,
                        " memory length:",
                        len(agent.memory),
                        " epsilon:",
                        agent.epsilon,
                    )
                    break

                    # if the mean of scores of last 10 episode is bigger than 490
                    # stop training
                    if np.mean(scores[-min(10, len(scores)) :]) > 490:
                        break

            # save the model
            if episode % 20 == 0:
                agent.model.save_weights("./cartpole_dqn.h5")

            rospy.loginfo("Episode %d: completed", episode)

    def predict_dqn(self):
        # get size of state and action from environment
        state_size = 4
        action_size = 2

        agent = DQNAgent(state_size, action_size, load_model=True)

        done = False
        score = 0

        self.reset()
        state, _, _, _ = self.step(-1)
        state = np.reshape(state, [1, state_size])

        while not done:
            # get action for the current state and go one step in environment
            action = agent.get_action(state)
            next_state, reward, done, info = self.step(action)
            next_state = np.reshape(next_state, [1, state_size])

            score += reward
            state = next_state

            if done or score >= 500:
                print("score:",score)
                break

    def predict_dqn_quant(self):
        # get size of state and action from environment
        state_size = 4
        action_size = 2

        agent = DQNAgent_quant()

        done = False
        score = 0

        self.reset()
        state, _, _, _ = self.step(-1)
        state = np.reshape(state, [1, state_size])

        while not done:
            # get action for the current state and go one step in environment
            action = agent.get_action(state)
            next_state, reward, done, info = self.step(action)
            next_state = np.reshape(next_state, [1, state_size])

            score += reward
            state = next_state

            if done or score >= 500:
                print("score:",score)
                break

    def predict_ddqn_quant(self):
        # get size of state and action from environment
        state_size = 4
        action_size = 2

        agent = DoubleDQNAgent_quant()

        done = False
        score = 0

        self.reset()
        state, _, _, _ = self.step(-1)
        state = np.reshape(state, [1, state_size])

        while not done:
            # get action for the current state and go one step in environment
            action = agent.get_action(state)
            next_state, reward, done, info = self.step(action)
            next_state = np.reshape(next_state, [1, state_size])

            score += reward
            state = next_state

            if done or score >= 500:
                print("score:",score)
                break


if __name__ == "__main__":
    '''Initializes and cleanup ros node'''
    rospy.init_node('robot_environment_node', anonymous=True)
    env = RobotEnvironment()
    try:
        env.train_ddqn()
    except KeyboardInterrupt:
        print("Shutting down ROS ")
        sys.exit()
    

