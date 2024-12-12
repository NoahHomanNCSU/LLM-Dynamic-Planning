import openai
import time

# Function to interact with OpenAI API and update the task queue
def update_task_queue_with_ai(current_time, current_task_queue, people_states, current_task='None', holding="None"):
    openai.api_key = 'secret'

    #models = openai.Model.list()
    #print([model["id"] for model in models["data"]])

    system_prompt = (
        """
        Your job is to update the task plan of a robot waiter in a restaurant simulation. There are 8 tables that can have parties 1-4. 
        The robot starts in its home and stays there until there are tasks for it to perform. The tables are layed out such that table1 is the closest,
        and table8 is the farthest from the home. Patrons need the robot to order, receive their meals, and pay. The robot only needs to reach the table
        to take orders (state=waiting to order) and payment (state=waiting to pay), but in the case of serving food, it must go back to the home to pick up the person's food once it is ready and then,
        bring it to the table. 

        Currently holding refers to the id of the person whose food the robot is holding, if any.

        Above all else, prioritize making the first task in the queue the current task (if there is one), unless it is beneficial to switch tasks,
        and do not ever output any text other than the tasks in the specified format.

        Follow this procedure:
        -If a person is waiting to order, the robot goes to their table to get their order
        -If a person is waiting to eat and their food is ready, the robot needs to pick up their food from home and then go to their table
        -If a person is waiting to pay, the robot goes to their table to get their payment
        -If a multiple people at a table need to order or pay, only make a task for one of them
        -Only set home as target when no one needs service or the robot needs to pick up food that is ready

        Each patron's state will be represented like this:
        id,table,state,arrival_time,food_ready(boolean)

        Each task is represented like this:
        target,id

        Where the target is either 'home' or a table name (e.g. table1, table2, etc...). The id is the person that the task is being doing for.
        (Ex: "home,4" means that the robot is going to the home to pick up person 4's food).

        Your output should be a task queue (max length of 5) where tasks are separted by newlines and are formatted in the given target,id format.
        Do not output anything else besides these tasks. The irst line will be the first task and so on.

        When creating the task plane consider these things:
        -Minimize average task completion time
        -Maximize customer satisfaction. A patron's satisfaction score decreases over time since the time they arrived
        -The robot can take the orders and payments of everyone at a table at once, even if the task is only designating one person
        -The robot can only hold one person's meal at a time 
        """
    )

    input_message = f"Current Time: {current_time}\n\nCurrent Task: {current_task}\n\nCurrent Task Queue: {current_task_queue}\n\nPeople States: {people_states}\n\nCurrently Holding: {holding}"

    response = None

    try:
        response = openai.ChatCompletion.create(
            model='gpt-4o',
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": input_message},
            ],
            max_tokens=500,
        )
    except Exception as e:
        raise Exception("Rate limited")

    # Extract and parse the response
    raw_response = response['choices'][0]['message']['content']
    updated_task_queue = []

    # Parse the response into a list of task dictionaries
    for line in raw_response.splitlines():
        if ',' in line:
            target, person_id = line.split(',')
            updated_task_queue.append({"target": target.strip(), "person_id": int(person_id.strip())})

    return updated_task_queue
