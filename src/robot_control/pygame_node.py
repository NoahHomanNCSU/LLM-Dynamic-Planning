#!/usr/bin/env python3

from matplotlib import pyplot as plt
import rclpy
from rclpy.node import Node
import pygame
import random
import time
from std_msgs.msg import String
import threading

#Stop

class PyGameNode(Node):
    def __init__(self):
        super().__init__('pygame_node')
        pygame.init()
        self.screen = pygame.display.set_mode((1280, 720))
        pygame.display.set_caption('Restaurant Simulation')
        self.clock = pygame.time.Clock()
        self.running = True

        # Publishers and Subscribers
        self.customer_status_publisher = self.create_publisher(String, 'customer_status', 10)
        self.holding_info_publisher = self.create_publisher(String, 'holding_info', 10)
        self.robot_status_publisher = self.create_publisher(String, 'robot_status', 10)
        self.task_subscriber = self.create_subscription(String, 'task', self.task_callback, 10)


        # Rectangle and table dimensions
        self.rect_width = 100
        self.rect_height = 50
        self.box_width = 50
        self.box_height = 50

        # Robot data
        self.robot_x = 0
        self.robot_y = 0
        self.home = 0
        self.robot_target = None
        self.robot_task = None
        self.robot_holding_food = None
        self.robot_state = 'idle'  # idle, traveling, serving


        # Tracks unique ids for each customer
        self.id_counter = 1

        # Static positions for various elements
        center_x = (self.screen.get_width() - self.rect_width) // 2
        center_y = (self.screen.get_height() - self.rect_height) // 2
        self.bathroom_x = center_x
        self.bathroom_y = 0
        self.entrance_x = center_x - 450
        self.entrance_y = self.screen.get_height() - self.rect_height

        # Table positions and occupancy
        self.tables = {
            'table1': {'pos': (center_x - 200, center_y + 100), 'occupied': False},
            'table2': {'pos': (center_x - 200, center_y - 100), 'occupied': False},
            'table3': {'pos': (center_x - 50, center_y), 'occupied': False},
            'table4': {'pos': (center_x + 100, center_y + 100), 'occupied': False},
            'table5': {'pos': (center_x + 100, center_y - 100), 'occupied': False},
            'table6': {'pos': (center_x + 250, center_y), 'occupied': False},
            'table7': {'pos': (center_x + 400, center_y + 100), 'occupied': False},
            'table8': {'pos': (center_x + 400, center_y - 100), 'occupied': False}
        }

        # Colors
        self.table_color = (0, 255, 0)
        self.robot_color = (255, 105, 180)
        self.home_color = (255, 0, 0)
        self.bathroom_color = (0, 0, 255)
        self.entrance_color = (175, 125, 0)
        self.person_color = (255, 105, 180)

        # People list
        self.people = []  # List to hold each person
        self.bathroom_in_use = False
        self.final_satisfactions = []
        
        # Spawn and bathroom timer setup
        self.last_spawn_time = time.time()
        self.last_bathroom_check = time.time()
        self.update_spawn_interval()

    def update_spawn_interval(self):
        occupied_tables = sum(1 for table in self.tables.values() if table['occupied'])
        # Set a new interval between 5 and 15, scaled by occupied tables
        self.spawn_interval = random.randint(5, 15) * max(occupied_tables, 1) * 2

    def publish_customer_status(self, person):
        msg = String()
        msg.data = f"{person['id']},{person['table']},{person['state']},{person['arrival_time']},{person['food_ready']}"
        self.customer_status_publisher.publish(msg)

    def task_callback(self, msg):
        data = msg.data.split(',')
        target = data[0]
        if len(data) > 1:
            task_id = int(data[1])
            self.robot_task = {'target': target, 'id': task_id}
        else:
            self.robot_task = {'target': target}
        if target == 'home':
            self.robot_target = (self.home, self.home)
        elif target in self.tables:
            self.robot_target = self.tables[target]['pos']

        self.robot_state = 'traveling'
        self.get_logger().info(f"Robot is traveling to {target} for task {self.robot_task}")
        self.publish_robot_status('performing_task')



    def publish_robot_status(self, status):
        msg = String()
        msg.data = status
        self.robot_status_publisher.publish(msg)

    def spawn_people(self):
        party_size = random.randint(1, 4)
        unoccupied_tables = [(name, info) for name, info in self.tables.items() if not info['occupied']]
        
        if unoccupied_tables:
            table_name, table_info = random.choice(unoccupied_tables)
            table_info['occupied'] = True  # Mark table as occupied
            target = table_info['pos']

            print(f"Party of size {party_size} being seated at {table_name}", flush=True)
            # Spawn each person with a vertical offset
            for i in range(party_size):
                person = {
                    'id': self.id_counter,
                    'x': self.entrance_x + (self.rect_width // 2),      # Entrance x position
                    'y': self.entrance_y - i * 30,                      # Offset each person by 30 pixels vertically
                    'target': (target[0] + 30 * i, target[1]),          # Final target position at the table
                    'state': 'moving_to_center',                        # Start with moving_to_center state
                    'food_prep_time': None,                             # Set after ordering
                    'eat_time': None,                                   # Set after getting food
                    'remaining_time': None,                             # Time left when leaving for bathroom
                    'arrival_time': time.time(),                        # Time of arrival
                    'party_position': i + 1,                            # Position in party (1 to 4)
                    'table': table_name,                                # Track the assigned table name
                    'food_ready': False,
                    'satisfaction': 100,
                    'satisfaction_delta': random.uniform(0.0002, 0.0005)  

                }
                self.id_counter += 1
                self.people.append(person)
                self.publish_customer_status(person)

    def send_person_to_bathroom(self):
        # Choose a random person at a table to go to the bathroom
        table_people = [p for p in self.people if p['state'] in ['waiting_to_eat', 'eating']]
        if table_people and not self.bathroom_in_use:
            person = random.choice(table_people)
            self.bathroom_in_use = True

            # Save the current state and set remaining_time
            person['previous_state'] = person['state']
            if person['state'] == 'waiting_to_eat':
                # Ensure `eat_time` exists and is valid
                if person.get('eat_time') is None:
                    person['eat_time'] = time.time() + 10  # Default `eat_time` if missing
                person['remaining_time'] = max(0, person['eat_time'] - time.time())
            elif person['state'] == 'eating':
                # Ensure `leave_time` exists and is valid
                if person.get('leave_time') is None:
                    person['leave_time'] = time.time() + 10  # Default `leave_time` if missing
                person['remaining_time'] = max(0, person['leave_time'] - time.time())

            # Set `leave_time` explicitly for the bathroom stay
            person['leave_time'] = time.time() + random.randint(5, 10)

            # Set state to moving to bathroom
            person['state'] = 'moving_to_bathroom'
            person['target'] = (self.bathroom_x + (self.rect_width // 2), self.bathroom_y + (self.rect_height // 2))
            self.get_logger().info(f"Person {person['party_position']} at {person['table']} is heading to the bathroom.")



    def move_people(self):
        for person in self.people:
            previous_state = person['state']

            person['satisfaction'] -= person['satisfaction_delta']
            if person['satisfaction'] < 0:
                person['satisfaction'] = 0

            if person['state'] == 'moving_to_center':
                # Move vertically toward the center of the screen
                center_y = self.screen.get_height() // 2
                if abs(person['y'] - center_y) < 2:
                    person['state'] = 'moving_to_table'
                else:
                    person['y'] -= 2 if person['y'] > center_y else -2

            elif person['state'] == 'moving_to_table':
                # Move towards assigned table
                target_x, target_y = person['target']
                if abs(person['x'] - target_x) < 2 and abs(person['y'] - target_y) < 2:
                    person['state'] = 'waiting_to_order'
                    print(f"Person {person['party_position']} is now waiting to order at {person['table']}", flush=True)
                    self.publish_customer_status(person)
                else:
                    person['x'] += 2 if person['x'] < target_x else -2
                    person['y'] += 2 if person['y'] < target_y else -2

            elif person['state'] == 'waiting_to_order':
                # Wait for robot to serve before progressing to ordering
                pass

            elif person['state'] == 'waiting_to_eat':
                # Wait for robot to serve food before progressing to eating
                pass

            elif person['state'] == 'waiting_to_pay':
                # Wait for robot to process payment before leaving
                pass

            elif person['state'] == 'leaving':
                # Move towards the exit
                target_x, target_y = person['target']
                if abs(person['x'] - target_x) < 3 and abs(person['y'] - target_y) < 3:
                    if target_y == self.screen.get_height() // 2:
                        # Move toward the entrance
                        person['target'] = (self.entrance_x + (self.rect_width // 2), self.entrance_y)
                    else:
                        # Reached the entrance; despawn the person
                        table_name = person['table']
                        self.final_satisfactions.append(person['satisfaction'])
                        self.people.remove(person)  # Remove the person from the list
                        # Check if there are still people at the table
                        remaining_people_at_table = any(p['table'] == table_name for p in self.people)
                        if not remaining_people_at_table:
                            self.tables[table_name]['occupied'] = False  # Set occupancy to false only if no one else is at the table
                        print(f"Person {person['party_position']} at {table_name} has left the restaurant", flush=True)
                        continue  # Avoid further processing for this person
                else:
                    person['x'] += 2 if person['x'] < target_x else -2
                    person['y'] += 2 if person['y'] < target_y else -2

            elif person['state'] == 'moving_to_bathroom':
                # Move toward the bathroom location
                target_x, target_y = person['target']
                if abs(person['x'] - target_x) < 2 and abs(person['y'] - target_y) < 2:
                    # Arrived at the bathroom, start bathroom timer
                    person['state'] = 'bathroom'
                    person['bathroom_start_time'] = time.time()
                    self.get_logger().info(f"Person {person['party_position']} arrived at the bathroom.")
                else:
                    # Continue moving toward the bathroom
                    person['x'] += 2 if person['x'] < target_x else -2
                    person['y'] += 2 if person['y'] < target_y else -2

            elif person['state'] == 'bathroom':
                # Stay in the bathroom for the duration of `leave_time`
                if time.time() >= person['leave_time']:
                    # Set state to returning to table
                    person['state'] = 'returning_to_table'
                    pos = self.tables[person['table']]['pos']
                    person['target'] = (pos[0] + (30 * (person['party_position'] - 1)), pos[1])
                    self.bathroom_in_use = False
                    self.get_logger().info(f"Person {person['party_position']} is returning to {person['table']} from the bathroom.")

            elif person['state'] == 'returning_to_table':
                # Move toward the table position
                target_x, target_y = person['target']
                if abs(person['x'] - target_x) < 2 and abs(person['y'] - target_y) < 2:
                    # Arrived back at the table, resume previous state
                    person['state'] = person['previous_state']
                    if person['state'] == 'waiting_to_eat':
                        person['eat_time'] = time.time() + person['remaining_time']
                    elif person['state'] == 'eating':
                        person['leave_time'] = time.time() + person['remaining_time']
                    self.get_logger().info(f"Person {person['party_position']} has resumed {person['state']} at {person['table']}.")
                else:
                    # Continue moving toward the table
                    person['x'] += 2 if person['x'] < target_x else -2
                    person['y'] += 2 if person['y'] < target_y else -2

            if person['state'] != previous_state:
                self.publish_customer_status(person)


    def move_robot(self):
        if self.robot_target is None or self.robot_state != 'traveling':
            return

        target_x, target_y = self.robot_target
        if abs(self.robot_x - target_x) < 3 and abs(self.robot_y - target_y) < 3:
            # Handle logic for reaching a target
            if self.robot_task['target'] == 'home' and 'id' in self.robot_task:
                self.pick_up_food(self.robot_task['id'])
            elif self.robot_task['target'] in self.tables:
                # Deliver food to the table
                self.serve_table(self.robot_task['target'])

            # Reset the robot's task after serving or picking up food
            self.robot_state = 'idle'
            self.robot_target = None
            self.publish_robot_status('reached_destination')
        else:
            # Continue moving toward the target
            if self.robot_x < target_x:
                self.robot_x += 3
            elif self.robot_x > target_x:
                self.robot_x -= 3
            if self.robot_y < target_y:
                self.robot_y += 3
            elif self.robot_y > target_y:
                self.robot_y -= 3



    def serve_table(self, table_name):
        self.robot_state = 'serving'
        table_people = [p for p in self.people if p['table'] == table_name]

        for person in table_people:
            if person['state'] == 'waiting_to_order':
                person['state'] = 'waiting_to_eat'
                person['food_ready'] = True  # Simulate food being prepared immediately
            elif person['state'] == 'waiting_to_eat' and person['id'] == self.robot_holding_food:
                person['state'] = 'eating'
                person['food_ready'] = False  # Ensure food_ready is set to False
                # Random eating duration between 5 and 10 seconds
                eat_duration = random.randint(5, 10)
                # Start eat_timer to transition to waiting_to_pay
                def transition_to_waiting_to_pay():
                    person['state'] = 'waiting_to_pay'
                    self.get_logger().info(f"Person {person['id']} at {person['table']} is now waiting to pay.")
                    self.publish_customer_status(person)

                threading.Timer(eat_duration, transition_to_waiting_to_pay).start()
            elif person['state'] == 'waiting_to_pay':
                person['state'] = 'leaving'
                person['target'] = (self.entrance_x + (self.rect_width // 2), self.screen.get_height() // 2)

            # Publish updated state for each person at the table
            self.publish_customer_status(person)

        def complete_serving():
            self.robot_holding_food = None
            self.robot_target = None  # Clear the target
            self.robot_state = 'traveling'  # Transition back to traveling
            self.get_logger().info(f"Finished serving table {table_name}.")
            
            msg = String()
            msg.data = "None"
            self.holding_info_publisher.publish(msg)  # Publish "None" correctly
            self.publish_robot_status('serving_complete')

        # Start the timer thread
        threading.Timer(2.0, complete_serving).start()  # 2-second delay to simulate serving



    def pick_up_food(self, person_id):
        self.robot_state = 'serving'
        self.robot_holding_food = person_id

        self.get_logger().info(f"Picked up food for person {person_id} from home.")

        # Simulate a short delay at home using threading
        def complete_pickup():
            # After pickup, send a signal to continue with the next task
            self.robot_target = None
            self.robot_state = 'traveling'  # Transition back to traveling
            self.get_logger().info("Finished picking up food. Ready to deliver.")
            
            # Wrap the food ID in a String message
            msg = String()
            msg.data = str(self.robot_holding_food)
            self.holding_info_publisher.publish(msg)  # Publish the food ID
            self.publish_robot_status('serving_complete')

        # Start the timer thread for delay
        threading.Timer(2.0, complete_pickup).start()

    def plot_average_satisfaction(self):
        if not self.final_satisfactions:
            self.get_logger().info("No satisfaction data to plot.")
            return

        average_satisfactions = []
        cumulative_sum = 0
        for i, satisfaction in enumerate(self.final_satisfactions, start=1):
            cumulative_sum += satisfaction
            average_satisfactions.append(cumulative_sum / i)

        plt.figure()
        plt.plot(range(1, len(average_satisfactions) + 1), average_satisfactions, marker='o')
        plt.title('Average Customer Satisfaction Over Time')
        plt.xlabel('Number of Customers')
        plt.ylabel('Average Satisfaction')
        plt.grid()
        plt.savefig('average_customer_satisfaction2.png')
        plt.show()

    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            # Clear the screen
            self.screen.fill((0, 0, 0))

            # Draw all tables
            for table_name, table_info in self.tables.items():
                x, y = table_info['pos']
                pygame.draw.rect(self.screen, self.table_color, pygame.Rect(x, y, self.rect_width, self.rect_height))

            # Draw static areas
            pygame.draw.rect(self.screen, self.entrance_color, pygame.Rect(self.entrance_x, self.entrance_y, self.rect_width, self.rect_height))
            pygame.draw.rect(self.screen, self.bathroom_color, pygame.Rect(self.bathroom_x, self.bathroom_y, self.rect_width, self.rect_height))
            pygame.draw.rect(self.screen, self.home_color, pygame.Rect(self.home, self.home, self.box_width, self.box_height))

            # Spawn people every few seconds
            if time.time() - self.last_spawn_time > self.spawn_interval:
                self.spawn_people()
                self.last_spawn_time = time.time()
                self.update_spawn_interval()

            # Check if someone can go to the bathroom
            if time.time() - self.last_bathroom_check > 20 and not self.bathroom_in_use:
                self.send_person_to_bathroom()
                self.last_bathroom_check = time.time()

            # Move and draw people
            self.move_people()
            for person in self.people:
                pygame.draw.circle(self.screen, self.person_color, (int(person['x']), int(person['y'])), 10)

            self.move_robot()
            pygame.draw.rect(self.screen, self.robot_color, pygame.Rect(self.robot_x, self.robot_y, self.box_width, self.box_height))

            rclpy.spin_once(self, timeout_sec=0.01)
            pygame.display.flip()  # Ensure display updates
            self.clock.tick(60)

        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    node = PyGameNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_average_satisfaction()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

