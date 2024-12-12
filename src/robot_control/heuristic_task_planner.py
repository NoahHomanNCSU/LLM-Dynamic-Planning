def update_task_queue_with_heuristic(current_time, current_task_queue, people_states, current_task='None', holding="None"):
    # Distance mapping: closer tables have lower "distance scores"
    table_distances = {
        'table1': 1, 'table2': 2, 'table3': 3, 'table4': 4,
        'table5': 5, 'table6': 6, 'table7': 7, 'table8': 8
    }

    tasks = []
    people = []
    for person in people_states:
        person_info = {
            'id': person['id'],
            'table': person['table'],
            'state': person['state'],
            'arrival_time': person['arrival_time'],
            'food_ready': person['food_ready'],
        }
        people.append(person_info)

    people.sort(key=lambda p: (p['arrival_time'], table_distances[p['table']]))
    print(len(people), flush=True)

    # Generate tasks
    for person in people:
        if len(tasks) >= 5:  # Max queue length
            break

        if person['state'] == 'waiting_to_order':
            tasks.append({'target': person['table'], 'person_id': person['id']})
        elif person['state'] == 'waiting_to_eat' and person['food_ready']:
            if holding == str(person['id']):
                tasks.append({'target': person['table'], 'person_id': person['id']})
            elif holding == "None":
                tasks.append({'target': 'home', 'person_id': person['id']})
        elif person['state'] == 'waiting_to_pay':
            tasks.append({'target': person['table'], 'person_id': person['id']})

    # Add a "go home" task if no tasks remain and robot is idle
    if not tasks and current_task_queue == [] and holding == "None":
        tasks.append({'target': 'home', 'person_id': None})

    # Ensure tasks are in the format for the robot
    return tasks
