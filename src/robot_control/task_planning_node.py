import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import String
from ai_planner import update_task_queue_with_ai
from heuristic_task_planner import update_task_queue_with_heuristic
import time
import matplotlib.pyplot as plt

class RobotState(Enum):
    IDLING = 1
    TRAVELING = 2
    AVOIDANCE = 3
    SERVING = 4

class TaskPlanningNode(Node):
    def __init__(self):
        super().__init__('task_planning')

        # State machine initialization
        self.state = RobotState.IDLING
        self.task_queue = []
        self.going_home = False
        self.current_task = None
        self.holding = None

        # Timer to periodically check for new tasks when idle
        self.idle_timer = self.create_timer(5.0, self.query_task_plan)

        # List to track people
        self.people = []

        # Task completion tracking
        self.task_completion_times = []  # List to store task completion times
        self.task_start_times = {}       # Dictionary to store start times for tasks

        # Publisher for task
        self.task_publisher = self.create_publisher(String, 'task', 10)

        # Subscribers
        self.status_subscriber = self.create_subscription(String, 'robot_status', self.status_callback, 10)
        self.customer_status_subscriber = self.create_subscription(String, 'customer_status', self.customer_status_callback, 10)
        self.holding_info_subscriber = self.create_subscription(String, 'holding_info', self.holding_callback, 10)

        self.get_logger().info('Task Planning Node has been started.')

    def customer_status_callback(self, msg):
        # Parse the comma-separated string
        data = msg.data.split(',')
        person_id = int(data[0])
        table = data[1]
        state = data[2]
        arrival_time = float(data[3])
        food_ready = data[4].lower() == 'true'

        # Check if the person is already being tracked
        existing_person = next((p for p in self.people if p['id'] == person_id), None)

        if existing_person:
            if state == 'leaving':
                # Remove the person if they're leaving
                self.people.remove(existing_person)
            else:
                # Update the person's information
                existing_person.update({
                    'table': table,
                    'state': state,
                    'arrival_time': arrival_time,
                    'food_ready': food_ready
                })
        else:
            # Add a new person to the tracking list
            if state != 'leaving':  # Ignore 'leaving' if not already tracked
                new_person = {
                    'id': person_id,
                    'table': table,
                    'state': state,
                    'arrival_time': arrival_time,
                    'food_ready': food_ready
                }
                self.people.append(new_person)

    def status_callback(self, msg):
        new_status = msg.data
        if new_status == 'performing_task' and self.state == RobotState.IDLING:
            self.transition_state(RobotState.TRAVELING)
        elif new_status == 'obstacle_detected' and self.state == RobotState.TRAVELING:
            self.transition_state(RobotState.AVOIDANCE)
        elif new_status == 'obstacle_cleared' and self.state == RobotState.AVOIDANCE:
            self.transition_state(RobotState.TRAVELING)
        elif new_status == 'reached_destination' and self.state == RobotState.TRAVELING:
            if self.going_home:
                self.going_home = False
                self.current_task = None  # Clear the current task
                self.transition_state(RobotState.IDLING)
            else:
                self.transition_state(RobotState.SERVING)
                self.record_task_completion()  # Record task completion time
        elif new_status == 'serving_complete' and self.state == RobotState.SERVING:
            self.current_task = None  # Clear the current task
            self.transition_state(RobotState.IDLING)
            self.query_task_plan()  # Query new tasks after serving is complete

    def holding_callback(self, msg):
        self.holding = msg.data

    def transition_state(self, new_state):
        self.get_logger().info(f'Transitioning from {self.state.name} to {new_state.name}')
        self.state = new_state
        if new_state != RobotState.SERVING:
            self.update_task_plan()

    def query_task_plan(self):
        self.get_logger().info('PING')
        active_people = [
            person for person in self.people
            if person['state'] in ['waiting_to_order', 'waiting_to_eat', 'waiting_to_pay']
        ]

        if active_people:
            self.update_task_plan()

    def execute_next_task(self):
        if not self.task_queue:
            active_people = [
                person for person in self.people 
                if person['state'] not in ['moving_to_table', 'leaving', 'moving_to_center']
            ]
            if not active_people:
                self.get_logger().info('No tasks in the queue to execute. Sending robot home.')
                self.going_home = True
                self.current_task = {'target': 'home', 'person_id': None}
                self.publish_task('home')
            else:
                self.get_logger().info('No tasks in queue but active people remain. Not sending robot home.')
        else:
            self.current_task = self.task_queue.pop(0)  # Set the current task
            self.task_start_times[self.current_task['person_id']] = time.time()  # Record task start time
            self.publish_task(self.current_task['target'], self.current_task['person_id'])

    def publish_task(self, destination, id=None):
        if id:
            task_message = f"{destination},{id}"
        else:
            task_message = f"{destination}"
        msg = String()
        msg.data = task_message
        self.task_publisher.publish(msg)

    def update_task_plan(self):
        current_time = time.time()
        try:
            print('1', flush=True)
            #self.task_queue = update_task_queue_with_ai(current_time, self.task_queue, self.people, self.current_task, self.holding)
            self.task_queue = update_task_queue_with_heuristic(current_time, self.task_queue, self.people, self.current_task, self.holding)
        except Exception as e:
            pass
        print(self.task_queue, flush=True)
        self.execute_next_task()

    def record_task_completion(self):
        if self.current_task and self.current_task['person_id'] in self.task_start_times:
            start_time = self.task_start_times.pop(self.current_task['person_id'])
            completion_time = time.time() - start_time
            self.task_completion_times.append(completion_time)

    def plot_average_task_completion_time(self):
        if not self.task_completion_times:
            self.get_logger().info("No task completion times to plot.")
            return

        average_times = []
        cumulative_sum = 0
        for i, time_taken in enumerate(self.task_completion_times, start=1):
            cumulative_sum += time_taken
            average_times.append(cumulative_sum / i)

        plt.figure()
        plt.plot(range(1, len(average_times) + 1), average_times, marker='o')
        plt.title('Average Task Completion Time Over Time')
        plt.xlabel('Number of Tasks Completed')
        plt.ylabel('Average Completion Time (seconds)')
        plt.grid()
        plt.savefig('average_task_completion_time2.png')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_average_task_completion_time()  # Generate the plot on shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
